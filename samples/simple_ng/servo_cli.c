/** \file
 * \brief Minimal CiA402 command-line control for one EtherCAT servo.
 *
 * Usage:
 *   servo_cli IFACE status
 *   servo_cli IFACE start
 *   servo_cli IFACE stop
 *   servo_cli IFACE fault-reset
 *   servo_cli IFACE move POS
 *   servo_cli IFACE mover DELTA
 *   servo_cli IFACE vel VELOCITY
 */

#include "soem/soem.h"
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <unistd.h>

#define SERVO_SLAVE 1
#define DEFAULT_CYCLE_US 1000
#define IO_MAP_SIZE 4096
#define NSEC_PER_SEC 1000000000
#define MODE_CSP 8
#define MODE_CSV 9
#define MAX_DELTA 100000
#define MAX_VELOCITY 100000
#define DEFAULT_VELOCITY_ACCEL 10000
#define DEFAULT_SPEED 100

static volatile sig_atomic_t keep_running = 1;
static volatile int csv_pdo_remap_ok = 0;

typedef struct
{
   int present;
   int bit_offset;
   int bit_length;
} PdoEntry;

typedef struct
{
   PdoEntry controlword;          /* 0x6040:00 */
   PdoEntry error_code;           /* 0x603f:00 */
   PdoEntry statusword;           /* 0x6041:00 */
   PdoEntry modes_of_operation;   /* 0x6060:00 */
   PdoEntry mode_display;         /* 0x6061:00 */
   PdoEntry target_position;      /* 0x607a:00 */
   PdoEntry actual_position;      /* 0x6064:00 */
   PdoEntry target_velocity;      /* 0x60ff:00 */
   PdoEntry target_torque;        /* 0x6071:00 */
   PdoEntry digital_outputs;      /* 0x60fe:01 */
   PdoEntry max_profile_velocity; /* 0x607f:00 */
} ServoMap;

typedef struct
{
   ecx_contextt context;
   uint8 iomap[IO_MAP_SIZE];
   ServoMap map;
   pthread_t cyclic_thread;
   volatile int cyclic_run;
   int expected_wkc;
   int wkc;
   int cycle;
   int bad_wkc;
   uint32 cycle_us;
   uint32 cycle_ns;
   int8 requested_mode;
   int mode_requested;
} ServoBus;

static void handle_signal(int signal)
{
   (void)signal;
   keep_running = 0;
}

static void print_errors(ecx_contextt *context)
{
   while (context->ecaterror)
   {
      printf("%s", ecx_elist2string(context));
   }
}

static int roundtrip(ServoBus *bus)
{
   ecx_send_processdata(&bus->context);
   return ecx_receive_processdata(&bus->context, EC_TIMEOUTRET);
}

static void add_time_ns(ec_timet *ts, int64 addtime)
{
   ec_timet addts;

   addts.tv_nsec = addtime % NSEC_PER_SEC;
   addts.tv_sec = (addtime - addts.tv_nsec) / NSEC_PER_SEC;
   osal_timespecadd(ts, &addts, ts);
}

static OSAL_THREAD_FUNC_RT cyclic_task(void *arg)
{
   ServoBus *bus = arg;
   ec_timet ts;
   int ht;

   osal_get_monotonic_time(&ts);
   ht = (ts.tv_nsec / 1000000) + 1;
   ts.tv_nsec = ht * 1000000;

   ecx_send_processdata(&bus->context);
   while (bus->cyclic_run)
   {
      add_time_ns(&ts, bus->cycle_ns);
      osal_monotonic_sleep(&ts);
      bus->wkc = ecx_receive_processdata(&bus->context, EC_TIMEOUTRET);
      if (bus->wkc != bus->expected_wkc)
      {
         bus->bad_wkc++;
      }
      ecx_send_processdata(&bus->context);
      bus->cycle++;
   }
}

static int start_cyclic(ServoBus *bus)
{
   bus->cyclic_run = 1;
   if (!osal_thread_create_rt(&bus->cyclic_thread, 128000, &cyclic_task, bus))
   {
      printf("Warning: RT cyclic thread priority was not applied; using normal scheduling\n");
      if (!osal_thread_create(&bus->cyclic_thread, 128000, &cyclic_task, bus))
      {
         bus->cyclic_run = 0;
         return 0;
      }
   }
   return 1;
}

static void stop_cyclic(ServoBus *bus)
{
   if (bus->cyclic_run)
   {
      bus->cyclic_run = 0;
      pthread_join(bus->cyclic_thread, NULL);
   }
}

static void run_cycles(ServoBus *bus, int cycles)
{
   int i;
   for (i = 0; i < cycles; ++i)
   {
      if (!bus->cyclic_run)
      {
         roundtrip(bus);
      }
      osal_usleep(bus->cycle_us);
   }
}

static void set_entry(PdoEntry *entry, int bit_offset, int bit_length)
{
   entry->present = 1;
   entry->bit_offset = bit_offset;
   entry->bit_length = bit_length;
}

static void remember_pdo_entry(ServoMap *map, uint16 index, uint8 subindex,
                               int bit_offset, int bit_length)
{
   if (index == 0x60fe && subindex == 1)
   {
      set_entry(&map->digital_outputs, bit_offset, bit_length);
      return;
   }

   if (subindex != 0)
   {
      return;
   }

   switch (index)
   {
   case 0x603f:
      set_entry(&map->error_code, bit_offset, bit_length);
      break;
   case 0x6040:
      set_entry(&map->controlword, bit_offset, bit_length);
      break;
   case 0x6041:
      set_entry(&map->statusword, bit_offset, bit_length);
      break;
   case 0x6060:
      set_entry(&map->modes_of_operation, bit_offset, bit_length);
      break;
   case 0x6061:
      set_entry(&map->mode_display, bit_offset, bit_length);
      break;
   case 0x607a:
      set_entry(&map->target_position, bit_offset, bit_length);
      break;
   case 0x6064:
      set_entry(&map->actual_position, bit_offset, bit_length);
      break;
   case 0x60ff:
      set_entry(&map->target_velocity, bit_offset, bit_length);
      break;
   case 0x6071:
      set_entry(&map->target_torque, bit_offset, bit_length);
      break;
   case 0x607f:
      set_entry(&map->max_profile_velocity, bit_offset, bit_length);
      break;
   default:
      break;
   }
}

static int read_pdo_mapping(ecx_contextt *context, ServoMap *map,
                            uint16 assign_index, int start_bit_offset)
{
   uint16 pdo_index;
   uint32 mapping;
   uint8 pdo_count;
   uint8 entry_count;
   int bit_offset;
   int size;
   int wkc;
   int i, j;

   bit_offset = start_bit_offset;
   size = sizeof(pdo_count);
   pdo_count = 0;
   wkc = ecx_SDOread(context, SERVO_SLAVE, assign_index, 0x00, FALSE,
                     &size, &pdo_count, EC_TIMEOUTRXM);
   if (wkc <= 0)
   {
      print_errors(context);
      return 0;
   }

   for (i = 1; i <= pdo_count; ++i)
   {
      size = sizeof(pdo_index);
      pdo_index = 0;
      wkc = ecx_SDOread(context, SERVO_SLAVE, assign_index, (uint8)i, FALSE,
                        &size, &pdo_index, EC_TIMEOUTRXM);
      if (wkc <= 0)
      {
         print_errors(context);
         return 0;
      }
      pdo_index = etohs(pdo_index);

      size = sizeof(entry_count);
      entry_count = 0;
      wkc = ecx_SDOread(context, SERVO_SLAVE, pdo_index, 0x00, FALSE,
                        &size, &entry_count, EC_TIMEOUTRXM);
      if (wkc <= 0)
      {
         print_errors(context);
         return 0;
      }

      for (j = 1; j <= entry_count; ++j)
      {
         uint16 index;
         uint8 subindex;
         uint8 bit_length;

         size = sizeof(mapping);
         mapping = 0;
         wkc = ecx_SDOread(context, SERVO_SLAVE, pdo_index, (uint8)j, FALSE,
                           &size, &mapping, EC_TIMEOUTRXM);
         if (wkc <= 0)
         {
            print_errors(context);
            return 0;
         }
         mapping = etohl(mapping);
         index = (uint16)(mapping >> 16);
         subindex = (uint8)((mapping >> 8) & 0xff);
         bit_length = (uint8)(mapping & 0xff);

         printf("  %04x:%02x at bit %d len %u\n",
                index, subindex, bit_offset, bit_length);
         remember_pdo_entry(map, index, subindex, bit_offset, bit_length);
         bit_offset += bit_length;
      }
   }

   return bit_offset - start_bit_offset;
}

static uint8 *entry_ptr(ec_slavet *slave, const PdoEntry *entry, int output)
{
   uint8 *base;
   if (!entry->present || (entry->bit_offset % 8) != 0)
   {
      return NULL;
   }
   base = output ? slave->outputs : slave->inputs;
   return base + (entry->bit_offset / 8);
}

static void write_u16(ec_slavet *slave, const PdoEntry *entry, uint16 value)
{
   uint8 *p = entry_ptr(slave, entry, 1);
   if (p && entry->bit_length == 16)
   {
      uint16 v = htoes(value);
      memcpy(p, &v, sizeof(v));
   }
}

static void write_i32(ec_slavet *slave, const PdoEntry *entry, int32 value)
{
   uint8 *p = entry_ptr(slave, entry, 1);
   if (p && entry->bit_length == 32)
   {
      int32 v = htoel(value);
      memcpy(p, &v, sizeof(v));
   }
}

static void write_i8(ec_slavet *slave, const PdoEntry *entry, int8 value)
{
   uint8 *p = entry_ptr(slave, entry, 1);
   if (p && entry->bit_length == 8)
   {
      memcpy(p, &value, sizeof(value));
   }
}

static void write_u32(ec_slavet *slave, const PdoEntry *entry, uint32 value)
{
   uint8 *p = entry_ptr(slave, entry, 1);
   if (p && entry->bit_length == 32)
   {
      uint32 v = htoel(value);
      memcpy(p, &v, sizeof(v));
   }
}

static uint16 read_u16(ec_slavet *slave, const PdoEntry *entry)
{
   uint8 *p = entry_ptr(slave, entry, 0);
   uint16 value = 0;
   if (p && entry->bit_length == 16)
   {
      memcpy(&value, p, sizeof(value));
      value = etohs(value);
   }
   return value;
}

static int8 read_i8(ec_slavet *slave, const PdoEntry *entry)
{
   uint8 *p = entry_ptr(slave, entry, 0);
   int8 value = 0;
   if (p && entry->bit_length == 8)
   {
      memcpy(&value, p, sizeof(value));
   }
   return value;
}

static int32 read_i32(ec_slavet *slave, const PdoEntry *entry)
{
   uint8 *p = entry_ptr(slave, entry, 0);
   int32 value = 0;
   if (p && entry->bit_length == 32)
   {
      memcpy(&value, p, sizeof(value));
      value = etohl(value);
   }
   return value;
}

static int sdo_read_u8(ecx_contextt *context, uint16 index, uint8 subindex, uint8 *value)
{
   int size = sizeof(*value);
   *value = 0;
   return ecx_SDOread(context, SERVO_SLAVE, index, subindex, FALSE,
                      &size, value, EC_TIMEOUTRXM);
}

static int sdo_read_u16(ecx_contextt *context, uint16 index, uint8 subindex, uint16 *value)
{
   int size = sizeof(*value);
   *value = 0;
   if (ecx_SDOread(context, SERVO_SLAVE, index, subindex, FALSE,
                   &size, value, EC_TIMEOUTRXM) <= 0)
   {
      return 0;
   }
   *value = etohs(*value);
   return 1;
}

static int sdo_read_u32(ecx_contextt *context, uint16 index, uint8 subindex, uint32 *value)
{
   int size = sizeof(*value);
   *value = 0;
   if (ecx_SDOread(context, SERVO_SLAVE, index, subindex, FALSE,
                   &size, value, EC_TIMEOUTRXM) <= 0)
   {
      return 0;
   }
   *value = etohl(*value);
   return 1;
}

static int sdo_write_u8(ecx_contextt *context, uint16 index, uint8 subindex, uint8 value)
{
   return ecx_SDOwrite(context, SERVO_SLAVE, index, subindex, FALSE,
                       sizeof(value), &value, EC_TIMEOUTRXM) > 0;
}

static int sdo_write_u16(ecx_contextt *context, uint16 index, uint8 subindex, uint16 value)
{
   uint16 v = htoes(value);
   return ecx_SDOwrite(context, SERVO_SLAVE, index, subindex, FALSE,
                       sizeof(v), &v, EC_TIMEOUTRXM) > 0;
}

static int sdo_write_u32(ecx_contextt *context, uint16 index, uint8 subindex, uint32 value)
{
   uint32 v = htoel(value);
   return ecx_SDOwrite(context, SERVO_SLAVE, index, subindex, FALSE,
                       sizeof(v), &v, EC_TIMEOUTRXM) > 0;
}

static const char *cia402_state(uint16 sw)
{
   if (sw & 0x0008) return "fault";
   if ((sw & 0x004f) == 0x0040) return "switch on disabled";
   if ((sw & 0x006f) == 0x0021) return "ready to switch on";
   if ((sw & 0x006f) == 0x0023) return "switched on";
   if ((sw & 0x006f) == 0x0027) return "operation enabled";
   if ((sw & 0x006f) == 0x0007) return "quick stop active";
   return "unknown";
}

static int is_operation_enabled(uint16 sw)
{
   return (sw & 0x006f) == 0x0027;
}

static void print_pdo_status(ServoBus *bus);
static void write_do_u32(ServoBus *bus, uint32 value)
{
   ec_slavet *slave = &bus->context.slavelist[SERVO_SLAVE];
   write_u32(slave, &bus->map.digital_outputs, value);
}

static void prepare_safe_targets(ServoBus *bus)
{
   ec_slavet *slave = &bus->context.slavelist[SERVO_SLAVE];

   if (bus->map.target_position.present && bus->map.actual_position.present)
   {
      write_i32(slave, &bus->map.target_position,
                read_i32(slave, &bus->map.actual_position));
   }
   if (bus->map.target_velocity.present)
   {
      write_i32(slave, &bus->map.target_velocity, 0);
   }
   if (bus->map.target_torque.present)
   {
      write_u16(slave, &bus->map.target_torque, 0);
   }
   if (bus->mode_requested && bus->map.modes_of_operation.present)
   {
      write_i8(slave, &bus->map.modes_of_operation, bus->requested_mode);
   }
}

/*
 * Wait for statusword to match (mask & value), writing controlword each cycle.
 * When cyclic thread is running, only writes to output buffers; the cyclic
 * thread handles the actual PDO exchange.  When cyclic is not running,
 * does manual roundtrip.
 */
static int wait_status(ServoBus *bus, uint16 controlword, uint16 mask, uint16 value, int cycles)
{
   ec_slavet *slave = &bus->context.slavelist[SERVO_SLAVE];
   uint16 last_sw = 0;
   int i;
   for (i = 0; i < cycles; ++i)
   {
      uint16 sw;
      write_u16(slave, &bus->map.controlword, controlword);
      write_do_u32(bus, 0x00000001);
      prepare_safe_targets(bus);
      if (!bus->cyclic_run)
      {
         roundtrip(bus);
      }
      /* When cyclic thread is running, inputs are updated asynchronously.
         Brief sleep to let the cyclic thread do a PDO exchange. */
      osal_usleep(bus->cycle_us);
      sw = read_u16(slave, &bus->map.statusword);
      last_sw = sw;
      if ((sw & mask) == value)
      {
         return 1;
      }
   }
   printf("  wait_status: last statusword=0x%04x, expected (sw & 0x%04x) == 0x%04x\n",
          last_sw, mask, value);
   return 0;
}

static void servo_diagnose(ServoBus *bus)
{
   ecx_contextt *context = &bus->context;
   uint16 u16;
   uint32 u32;
   int8 i8;

   printf("--- Servo diagnostics ---\n");
   if (sdo_read_u8(context, 0x6060, 0x00, (uint8 *)&i8))
      printf("Mode of operation (0x6060): %d\n", i8);
   if (sdo_read_u8(context, 0x6061, 0x00, (uint8 *)&i8))
      printf("Mode display (0x6061): %d\n", i8);
   if (sdo_read_u16(context, 0x605a, 0x00, &u16))
      printf("Quick stop option (0x605A): %d\n", u16);
   if (sdo_read_u16(context, 0x605c, 0x00, &u16))
      printf("Disable operation option (0x605C): %d\n", u16);
   if (sdo_read_u16(context, 0x605d, 0x00, &u16))
      printf("Halt option (0x605D): %d\n", u16);
   if (sdo_read_u16(context, 0x605e, 0x00, &u16))
      printf("Fault reaction option (0x605E): %d\n", u16);
   if (sdo_read_u32(context, 0x60fd, 0x00, &u32))
      printf("Digital inputs (0x60FD): 0x%08x\n", u32);
   print_errors(context);
}

static int servo_enable(ServoBus *bus, int8 requested_mode)
{
   ecx_contextt *context = &bus->context;
   ec_slavet *slave = &bus->context.slavelist[SERVO_SLAVE];
   uint16 sw;
   int8 mode;
   int size;
   int wkc;
   int i;

   bus->requested_mode = requested_mode;
   bus->mode_requested = 1;

   /*
    * When the cyclic thread is already running, do NOT call roundtrip()
    * from the main thread — that would conflict on the EtherCAT socket.
    * Instead, just write to output buffers and let the cyclic thread
    * handle PDO exchange.
    */
   prepare_safe_targets(bus);
   write_do_u32(bus, 0x00000001);
   if (!bus->cyclic_run)
   {
      roundtrip(bus);
   }
   sw = read_u16(slave, &bus->map.statusword);

   if (sw & 0x0008)
   {
      printf("Fault present (statusword=0x%04x, error=0x%04x), sending fault reset\n",
             sw, read_u16(slave, &bus->map.error_code));
      write_u16(slave, &bus->map.controlword, 0x0080);
      run_cycles(bus, 200);
      write_u16(slave, &bus->map.controlword, 0x0000);
      run_cycles(bus, 200);
      /* Re-check if fault was cleared */
      sw = read_u16(slave, &bus->map.statusword);
      if (sw & 0x0008)
      {
         printf("Fault persists after reset (statusword=0x%04x, error=0x%04x)\n",
                sw, read_u16(slave, &bus->map.error_code));
         printf("This fault requires power-cycle of the drive or clearing via\n");
         printf("the drive panel / InoDriveShop. Standard CiA402 fault reset\n");
         printf("cannot clear all error types (e.g. following error, overload).\n");
         return 0;
      }
      printf("Fault cleared\n");
   }

   /* Set mode of operation BEFORE state machine transitions.
    * Many drives require the mode to be configured in switch-on-disabled
    * state and will reject the mode change once in ready-to-switch-on. */
   size = sizeof(mode);
   mode = requested_mode;
   printf("Setting mode of operation to %s (%d)\n",
          requested_mode == MODE_CSV ? "CSV" : "CSP", requested_mode);
   wkc = ecx_SDOwrite(context, SERVO_SLAVE, 0x6060, 0x00, FALSE,
                      size, &mode, EC_TIMEOUTRXM);
   if (wkc <= 0)
   {
      printf("Failed to set mode of operation via SDO\n");
      print_errors(context);
      return 0;
   }
   print_errors(context);
   run_cycles(bus, 20);

   /* Set profile velocity via SDO so the drive has a valid limit before
      entering the state machine.  Some drives refuse to enable CSV/CSP
      mode when max_profile_velocity is zero. */
   if (bus->map.max_profile_velocity.present)
   {
      uint32 profile_vel = htoel(MAX_VELOCITY);
      printf("Setting max profile velocity to %u\n", MAX_VELOCITY);
      ecx_SDOwrite(context, SERVO_SLAVE, 0x607f, 0x00, FALSE,
                   sizeof(profile_vel), &profile_vel, EC_TIMEOUTRXM);
      print_errors(context);
      write_i32(slave, &bus->map.max_profile_velocity, (int32)MAX_VELOCITY);
   }

   printf("Shutdown\n");
   if (!wait_status(bus, 0x0006, 0x006f, 0x0021, 1000))
   {
      printf("Shutdown did not reach ready-to-switch-on\n");
      return 0;
   }
   print_pdo_status(bus);

   printf("Switch on\n");
   if (!wait_status(bus, 0x0007, 0x006f, 0x0023, 1000))
   {
      printf("Switch on did not reach switched-on\n");
      return 0;
   }
   print_pdo_status(bus);

   printf("Setting digital outputs to 0x1\n");
   write_do_u32(bus, 0x00000001);
   run_cycles(bus, 10);

   printf("Enable operation\n");
   prepare_safe_targets(bus);
   for (i = 0; i < 3; i++)
   {
      if (wait_status(bus, 0x000f, 0x006f, 0x0027, 500))
      {
         print_pdo_status(bus);
         return 1;
      }
      printf("  Enable attempt %d failed, retrying...\n", i + 1);
      write_do_u32(bus, 0x00000001);
   }

   printf("Enable operation did not reach operation-enabled\n");
   print_pdo_status(bus);
   servo_diagnose(bus);
   return 0;
}


static void servo_disable(ServoBus *bus)
{
   ec_slavet *slave = &bus->context.slavelist[SERVO_SLAVE];
   uint16 sw;
   int i;

   printf("Disabling servo\n");

   /* Stop writing operation mode via PDO during disable. */
   bus->mode_requested = 0;

   sw = read_u16(slave, &bus->map.statusword);
   if ((sw & 0x006f) == 0x0040)
   {
      printf("  Already in switch-on-disabled\n");
      return;
   }

   /* Stage 1: Disable operation (0x0007), bit3 1→0.
      Operation-enabled → Switched-on. */
   if ((sw & 0x006f) == 0x0027)
   {
      printf("  Stage 1/2: disable operation → switched-on\n");
      for (i = 0; i < 500; i++)
      {
         write_u16(slave, &bus->map.controlword, 0x0007);
         osal_usleep(bus->cycle_us);
         sw = read_u16(slave, &bus->map.statusword);
         if ((sw & 0x006f) == 0x0033 || (sw & 0x006f) == 0x0021 || (sw & 0x006f) == 0x0040)
         {
            printf("  Stage 1 ok, sw=0x%04x\n", sw);
            break;
         }
      }
   }

   /* Stage 2: Disable voltage (0x0000), bit1 1→0.
      Works from Switched-on or Ready-to-switch-on.
      Forces transition to Switch-on-disabled regardless of current state. */
   if ((sw & 0x006f) != 0x0040)
   {
      printf("  Stage 2/2: disable voltage → switch-on-disabled\n");
      for (i = 0; i < 500; i++)
      {
         write_u16(slave, &bus->map.controlword, 0x0000);
         osal_usleep(bus->cycle_us);
         sw = read_u16(slave, &bus->map.statusword);
         if ((sw & 0x006f) == 0x0040)
         {
            printf("  Stage 2 ok, switch-on-disabled\n");
            return;
         }
      }
   }

   printf("Warning: servo may not have fully disabled (last sw=0x%04x)\n", sw);
}

/*
 * Smooth relative move with trajectory interpolation.
 * Instead of jumping to target in one cycle (which would cause
 * violent acceleration or drive fault), we increment target_position
 * gradually each ms by 'speed' pulses, creating a constant-velocity ramp.
 *
 * The cyclic thread handles PDO exchange, so we just write to
 * the output buffer and sleep 1ms between each step.
 */
static void motor_move_smooth(ServoBus *bus, int32 delta, int32 speed)
{
   ec_slavet *slave = &bus->context.slavelist[SERVO_SLAVE];
   int32 start_pos, target;
   int32 step;
   int total_steps, i;

   if (!bus->map.target_position.present || !bus->map.actual_position.present)
   {
      printf("Required PDO entries not mapped\n");
      return;
   }

   start_pos = read_i32(slave, &bus->map.actual_position);
   target = start_pos + delta;
   step = (delta > 0) ? speed : -speed;
   total_steps = (abs(delta) + speed - 1) / speed;  /* ceil division */
   if (total_steps == 0) total_steps = 1;

   printf("Smooth move: start=%d, delta=%d, target=%d, speed=%d/ms, steps=%d\n",
          start_pos, delta, target, speed, total_steps);

   for (i = 0; i < total_steps; i++)
   {
      /* Check for fault during movement */
      uint16 sw = read_u16(slave, &bus->map.statusword);
      if (sw & 0x0008)
      {
         printf("* FAULT during move (statusword=0x%04x, error=0x%04x) - stopping\n",
                sw, read_u16(slave, &bus->map.error_code));
         return;
      }

      /* Interpolate: step towards target */
      if (i == total_steps - 1)
      {
         /* Last step: use exact target to avoid rounding error */
         write_i32(slave, &bus->map.target_position, target);
      }
      else
      {
         start_pos += step;
         write_i32(slave, &bus->map.target_position, start_pos);
      }
      write_do_u32(bus, 0x00000001);
      write_u16(slave, &bus->map.controlword, 0x000f);
      osal_usleep(bus->cycle_us);
   }

   printf("Move complete, actual position: %d\n",
          read_i32(slave, &bus->map.actual_position));
}

static void motor_run_velocity(ServoBus *bus, int32 velocity)
{
   ec_slavet *slave = &bus->context.slavelist[SERVO_SLAVE];
   int32 command_velocity;
   int32 ramp_step;
   int32 max_velocity;

   if (!bus->map.target_velocity.present)
   {
      printf("Required PDO entry 0x60FF target velocity is not mapped\n");
      return;
   }

   command_velocity = 0;
   ramp_step = (int32)(((int64)DEFAULT_VELOCITY_ACCEL * bus->cycle_us) / 1000000);
   if (ramp_step < 1)
   {
      ramp_step = 1;
   }
   max_velocity = velocity < 0 ? -velocity : velocity;
   if (bus->map.max_profile_velocity.present)
   {
      write_i32(slave, &bus->map.max_profile_velocity, max_velocity);
   }

   printf("Velocity run: target_velocity=%d, accel=%d/s^2. Press Ctrl-C to stop.\n",
          velocity, DEFAULT_VELOCITY_ACCEL);
   signal(SIGINT, handle_signal);
   signal(SIGTERM, handle_signal);

   while (keep_running)
   {
      uint16 sw = read_u16(slave, &bus->map.statusword);
      if (sw & 0x0008)
      {
         printf("* FAULT during velocity run (statusword=0x%04x, error=0x%04x) - stopping\n",
                sw, read_u16(slave, &bus->map.error_code));
         break;
      }
      if (!is_operation_enabled(sw))
      {
         printf("Not operation-enabled (0x%04x: %s)\n", sw, cia402_state(sw));
         break;
      }

      if (command_velocity < velocity)
      {
         command_velocity += ramp_step;
         if (command_velocity > velocity)
         {
            command_velocity = velocity;
         }
      }
      else if (command_velocity > velocity)
      {
         command_velocity -= ramp_step;
         if (command_velocity < velocity)
         {
            command_velocity = velocity;
         }
      }

      if (bus->map.modes_of_operation.present)
      {
         write_i8(slave, &bus->map.modes_of_operation, MODE_CSV);
      }
      if (bus->map.max_profile_velocity.present)
      {
         write_i32(slave, &bus->map.max_profile_velocity, max_velocity);
      }
      write_i32(slave, &bus->map.target_velocity, command_velocity);
      write_u16(slave, &bus->map.controlword, 0x000f);
      write_do_u32(bus, 0x00000001);
      osal_usleep(bus->cycle_us);
   }

   printf("Stopping velocity\n");
   while (command_velocity != 0)
   {
      if (command_velocity > 0)
      {
         command_velocity -= ramp_step;
         if (command_velocity < 0)
         {
            command_velocity = 0;
         }
      }
      else
      {
         command_velocity += ramp_step;
         if (command_velocity > 0)
         {
            command_velocity = 0;
         }
      }
      if (bus->map.modes_of_operation.present)
      {
         write_i8(slave, &bus->map.modes_of_operation, MODE_CSV);
      }
      write_i32(slave, &bus->map.target_velocity, command_velocity);
      write_u16(slave, &bus->map.controlword, 0x000f);
      write_do_u32(bus, 0x00000001);
      osal_usleep(bus->cycle_us);
   }
   write_i32(slave, &bus->map.target_velocity, 0);
   write_u16(slave, &bus->map.controlword, 0x000f);
   write_do_u32(bus, 0x00000001);
   run_cycles(bus, 200);
}

static void servo_fault_reset(ServoBus *bus)
{
   ec_slavet *slave = &bus->context.slavelist[SERVO_SLAVE];
   write_u16(slave, &bus->map.controlword, 0x0080);
   run_cycles(bus, 100);
   write_u16(slave, &bus->map.controlword, 0x0000);
   run_cycles(bus, 100);
}

static void print_status(ServoBus *bus)
{
   ec_slavet *slave = &bus->context.slavelist[SERVO_SLAVE];
   ecx_contextt *context = &bus->context;
   uint16 sw = read_u16(slave, &bus->map.statusword);
   uint32 detail;
   uint32 sync_cycle;
   uint16 sync_type;
   uint8 mode;

   printf("Statusword: 0x%04x (%s)\n", sw, cia402_state(sw));
   if (bus->map.error_code.present)
   {
      printf("Error code: 0x%04x\n", read_u16(slave, &bus->map.error_code));
   }
   if (sdo_read_u32(context, 0x203f, 0x00, &detail))
   {
      printf("Manufacturer detail 0x203f: 0x%08x\n", detail);
   }
   print_errors(context);
   if (sdo_read_u8(context, 0x6060, 0x00, &mode))
   {
      printf("Mode of operation 0x6060: %d\n", (int8)mode);
   }
   print_errors(context);
   if (sdo_read_u8(context, 0x6061, 0x00, &mode))
   {
      printf("Mode display 0x6061: %d\n", (int8)mode);
   }
   print_errors(context);
   if (sdo_read_u16(context, 0x1c32, 0x01, &sync_type))
   {
      printf("SM2 sync type 0x1c32:01: %u\n", sync_type);
   }
   print_errors(context);
   if (sdo_read_u32(context, 0x1c32, 0x02, &sync_cycle))
   {
      printf("SM2 cycle 0x1c32:02: %u ns\n", sync_cycle);
   }
   print_errors(context);
   if (sdo_read_u16(context, 0x1c33, 0x01, &sync_type))
   {
      printf("SM3 sync type 0x1c33:01: %u\n", sync_type);
   }
   print_errors(context);
   if (sdo_read_u32(context, 0x1c33, 0x02, &sync_cycle))
   {
      printf("SM3 cycle 0x1c33:02: %u ns\n", sync_cycle);
   }
   print_errors(context);
   if (bus->map.actual_position.present)
   {
      printf("Actual position: %d\n", read_i32(slave, &bus->map.actual_position));
   }
   printf("Cyclic: cycle=%d wkc=%d bad_wkc=%d expected_wkc=%d\n",
          bus->cycle, bus->wkc, bus->bad_wkc, bus->expected_wkc);
}

static void print_pdo_status(ServoBus *bus)
{
   ec_slavet *slave = &bus->context.slavelist[SERVO_SLAVE];
   uint16 sw = read_u16(slave, &bus->map.statusword);

   printf("Statusword: 0x%04x (%s)", sw, cia402_state(sw));
   if (bus->map.error_code.present)
   {
      printf(" Error: 0x%04x", read_u16(slave, &bus->map.error_code));
   }
   if (bus->map.actual_position.present)
   {
      printf(" Pos: %d", read_i32(slave, &bus->map.actual_position));
   }
   if (bus->map.mode_display.present)
   {
      printf(" Mode: %d", read_i8(slave, &bus->map.mode_display));
   }
   printf(" WKC: %d bad:%d\n", bus->wkc, bus->bad_wkc);
}

static int configure_sync_mode(ecx_contextt *context, uint32 cycle_ns)
{
   uint16 sync0_mode = htoes(2);
   uint32 sync_cycle = htoel(cycle_ns);
   int ok = 1;
   int wkc;

   wkc = ecx_SDOwrite(context, SERVO_SLAVE, 0x1c32, 0x01, FALSE,
                      sizeof(sync0_mode), &sync0_mode, EC_TIMEOUTRXM);
   ok = ok && (wkc > 0);
   wkc = ecx_SDOwrite(context, SERVO_SLAVE, 0x1c33, 0x01, FALSE,
                      sizeof(sync0_mode), &sync0_mode, EC_TIMEOUTRXM);
   ok = ok && (wkc > 0);
   wkc = ecx_SDOwrite(context, SERVO_SLAVE, 0x1c32, 0x02, FALSE,
                      sizeof(sync_cycle), &sync_cycle, EC_TIMEOUTRXM);
   ok = ok && (wkc > 0);
   wkc = ecx_SDOwrite(context, SERVO_SLAVE, 0x1c33, 0x02, FALSE,
                      sizeof(sync_cycle), &sync_cycle, EC_TIMEOUTRXM);
   ok = ok && (wkc > 0);
   print_errors(context);
   return ok;
}

static void discard_errors(ecx_contextt *context)
{
   while (context->ecaterror)
   {
      (void)ecx_elist2string(context);
   }
}

static int pdo_entry_index(uint32 mapping)
{
   return (int)((mapping >> 16) & 0xffff);
}

static int pdo_entries_contain(uint32 *entries, uint8 count, uint16 index)
{
   int i;
   for (i = 0; i < count; ++i)
   {
      if (pdo_entry_index(entries[i]) == index)
      {
         return 1;
      }
   }
   return 0;
}

static int read_pdo_entries(ecx_contextt *context, uint16 pdo_index,
                            uint32 *entries, uint8 max_entries, uint8 *entry_count)
{
   int size;
   int i;

   size = sizeof(*entry_count);
   *entry_count = 0;
   if (ecx_SDOread(context, SERVO_SLAVE, pdo_index, 0x00, FALSE,
                   &size, entry_count, EC_TIMEOUTRXM) <= 0)
   {
      return 0;
   }
   if (*entry_count > max_entries)
   {
      return 0;
   }

   for (i = 0; i < *entry_count; ++i)
   {
      size = sizeof(entries[i]);
      entries[i] = 0;
      if (ecx_SDOread(context, SERVO_SLAVE, pdo_index, (uint8)(i + 1), FALSE,
                      &size, &entries[i], EC_TIMEOUTRXM) <= 0)
      {
         return 0;
      }
      entries[i] = etohl(entries[i]);
   }
   return 1;
}

static int assign_rxpdo(ecx_contextt *context, uint16 pdo_index,
                        uint16 *original_assignments, uint8 original_count)
{
   int ok;
   int i;

   ok = 1;
   if (!sdo_write_u8(context, 0x1c12, 0x00, 0)) ok = 0;
   if (ok && !sdo_write_u16(context, 0x1c12, 0x01, pdo_index)) ok = 0;
   if (ok && !sdo_write_u8(context, 0x1c12, 0x00, 1)) ok = 0;
   print_errors(context);

   if (ok)
   {
      return 1;
   }

   printf("Restoring original RxPDO assignment\n");
   sdo_write_u8(context, 0x1c12, 0x00, 0);
   for (i = 0; i < original_count; ++i)
   {
      sdo_write_u16(context, 0x1c12, (uint8)(i + 1), original_assignments[i]);
   }
   sdo_write_u8(context, 0x1c12, 0x00, original_count);
   print_errors(context);
   return 0;
}

static int select_fixed_csv_rxpdo(ecx_contextt *context,
                                  uint16 *original_assignments, uint8 original_count)
{
   uint32 entries[32];
   uint8 entry_count;
   uint16 pdo_index;

   printf("Scanning fixed RxPDOs for 0x60FF target velocity\n");
   for (pdo_index = 0x1600; pdo_index <= 0x17ff; ++pdo_index)
   {
      if (!read_pdo_entries(context, pdo_index, entries, 32, &entry_count))
      {
         discard_errors(context);
         continue;
      }
      if (pdo_entries_contain(entries, entry_count, 0x6040) &&
          pdo_entries_contain(entries, entry_count, 0x60ff))
      {
         printf("Found CSV-capable fixed RxPDO 0x%04x; trying assignment\n",
                pdo_index);
         return assign_rxpdo(context, pdo_index, original_assignments,
                             original_count);
      }
   }

   printf("No fixed RxPDO containing both 0x6040 and 0x60FF was found\n");
   return 0;
}

static int configure_csv_rxpdo(ecx_contextt *context)
{
   uint8 assign_count;
   uint8 original_entry_count;
   uint16 assignments[16];
   uint16 pdo_index;
   uint32 original_entries[32];
   int size;
   int ok;
   int assignment_disabled;
   int mapping_cleared;
   int i;

   size = sizeof(assign_count);
   assign_count = 0;
   if (ecx_SDOread(context, SERVO_SLAVE, 0x1c12, 0x00, FALSE,
                   &size, &assign_count, EC_TIMEOUTRXM) <= 0)
   {
      printf("Could not read RxPDO assignment 0x1C12:00\n");
      print_errors(context);
      return 0;
   }
   if (assign_count < 1)
   {
      printf("No RxPDO assignment found; cannot map 0x60FF\n");
      return 0;
   }
   if (assign_count > 16)
   {
      printf("Too many RxPDO assignments (%u); refusing automatic remap\n",
             assign_count);
      return 0;
   }

   for (i = 0; i < assign_count; ++i)
   {
      size = sizeof(assignments[i]);
      assignments[i] = 0;
      if (ecx_SDOread(context, SERVO_SLAVE, 0x1c12, (uint8)(i + 1), FALSE,
                      &size, &assignments[i], EC_TIMEOUTRXM) <= 0)
      {
         printf("Could not read RxPDO assignment 0x1C12:%02x\n", i + 1);
         print_errors(context);
         return 0;
      }
      assignments[i] = etohs(assignments[i]);
   }
   pdo_index = assignments[0];

   size = sizeof(original_entry_count);
   original_entry_count = 0;
   if (ecx_SDOread(context, SERVO_SLAVE, pdo_index, 0x00, FALSE,
                   &size, &original_entry_count, EC_TIMEOUTRXM) <= 0)
   {
      printf("Could not read RxPDO mapping 0x%04x:00\n", pdo_index);
      print_errors(context);
      return 0;
   }
   if (original_entry_count > 32)
   {
      printf("Too many RxPDO entries (%u); refusing automatic remap\n",
             original_entry_count);
      return 0;
   }
   for (i = 0; i < original_entry_count; ++i)
   {
      size = sizeof(original_entries[i]);
      original_entries[i] = 0;
      if (ecx_SDOread(context, SERVO_SLAVE, pdo_index, (uint8)(i + 1), FALSE,
                      &size, &original_entries[i], EC_TIMEOUTRXM) <= 0)
      {
         printf("Could not read RxPDO mapping 0x%04x:%02x\n", pdo_index, i + 1);
         print_errors(context);
         return 0;
      }
      original_entries[i] = etohl(original_entries[i]);
   }
   if (pdo_entries_contain(original_entries, original_entry_count, 0x6040) &&
       pdo_entries_contain(original_entries, original_entry_count, 0x60ff))
   {
      printf("Current RxPDO 0x%04x already contains 0x60FF\n", pdo_index);
      return 1;
   }

   printf("Trying CSV RxPDO mapping on 0x%04x: 6040, 60FF, 60B8, 60FE:01\n",
          pdo_index);

   ok = 1;
   assignment_disabled = 0;
   mapping_cleared = 0;
   if (!sdo_write_u8(context, 0x1c12, 0x00, 0)) ok = 0;
   else assignment_disabled = 1;
   if (ok && !sdo_write_u8(context, pdo_index, 0x00, 0)) ok = 0;
   else if (ok) mapping_cleared = 1;
   if (ok && !sdo_write_u32(context, pdo_index, 0x01, 0x60400010)) ok = 0;
   if (ok && !sdo_write_u32(context, pdo_index, 0x02, 0x60ff0020)) ok = 0;
   if (ok && !sdo_write_u32(context, pdo_index, 0x03, 0x60b80010)) ok = 0;
   if (ok && !sdo_write_u32(context, pdo_index, 0x04, 0x60fe0120)) ok = 0;
   if (ok && !sdo_write_u8(context, pdo_index, 0x00, 4)) ok = 0;
   if (ok && !sdo_write_u16(context, 0x1c12, 0x01, pdo_index)) ok = 0;
   if (ok && !sdo_write_u8(context, 0x1c12, 0x00, 1)) ok = 0;
   print_errors(context);

   if (!ok)
   {
      printf("CSV RxPDO remap failed; drive may reject dynamic PDO mapping.\n");
      if (mapping_cleared)
      {
         printf("Restoring original RxPDO mapping\n");
         sdo_write_u8(context, pdo_index, 0x00, 0);
         for (i = 0; i < original_entry_count; ++i)
         {
            sdo_write_u32(context, pdo_index, (uint8)(i + 1), original_entries[i]);
         }
         sdo_write_u8(context, pdo_index, 0x00, original_entry_count);
      }
      if (assignment_disabled)
      {
         for (i = 0; i < assign_count; ++i)
         {
            sdo_write_u16(context, 0x1c12, (uint8)(i + 1), assignments[i]);
         }
         sdo_write_u8(context, 0x1c12, 0x00, assign_count);
      }
      print_errors(context);
      return select_fixed_csv_rxpdo(context, assignments, assign_count);
   }
   printf("CSV RxPDO remap accepted\n");
   return 1;
}

static int csv_pdo_config_callback(ecx_contextt *context, uint16 slave)
{
   if (slave != SERVO_SLAVE)
   {
      return 0;
   }
   csv_pdo_remap_ok = configure_csv_rxpdo(context);
   return csv_pdo_remap_ok;
}

static int bus_start(ServoBus *bus, const char *iface, uint32 cycle_us, int request_csv_pdo)
{
   ecx_contextt *context = &bus->context;
   ec_groupt *group = &context->grouplist[0];
   ec_slavet *slave;
   int expected_wkc;
   int i;

   memset(bus, 0, sizeof(*bus));
   if (cycle_us == 0)
   {
      cycle_us = DEFAULT_CYCLE_US;
   }
   bus->cycle_us = cycle_us;
   bus->cycle_ns = cycle_us * 1000;

   printf("Opening %s, cycle=%u us\n", iface, bus->cycle_us);
   if (!ecx_init(context, iface))
   {
      printf("No socket connection\n");
      return 0;
   }
   if (ecx_config_init(context) <= 0)
   {
      printf("No slaves found\n");
      return 0;
   }
   if (context->slavecount < SERVO_SLAVE)
   {
      printf("Servo slave %d not found\n", SERVO_SLAVE);
      return 0;
   }

   context->manualstatechange = 1;
   csv_pdo_remap_ok = 0;
   if (request_csv_pdo)
   {
      context->slavelist[SERVO_SLAVE].PO2SOconfig = csv_pdo_config_callback;
   }

   ecx_config_map_group(context, bus->iomap, 0);
   printf("Mapped %dO+%dI bytes\n", group->Obytes, group->Ibytes);

   printf("RxPDO mapping:\n");
   read_pdo_mapping(context, &bus->map, 0x1c12, 0);
   printf("TxPDO mapping:\n");
   read_pdo_mapping(context, &bus->map, 0x1c13, 0);

   if (!bus->map.controlword.present || !bus->map.statusword.present)
   {
      printf("Required CiA402 PDO entries 0x6040/0x6041 were not found.\n");
      return 0;
   }

   if (!configure_sync_mode(context, bus->cycle_ns))
   {
      printf("Warning: could not write both sync manager mode objects\n");
   }
   ecx_configdc(context);
   for (i = 1; i <= context->slavecount; ++i)
   {
      slave = &context->slavelist[i];
      if (slave->hasdc)
      {
         ecx_dcsync0(context, i, TRUE, bus->cycle_ns, 0);
      }
   }

   context->slavelist[0].state = EC_STATE_SAFE_OP;
   ecx_writestate(context, 0);
   ecx_statecheck(context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
   if (context->slavelist[0].state != EC_STATE_SAFE_OP)
   {
      printf("Failed to reach SAFE_OP\n");
      return 0;
   }

   run_cycles(bus, 200);
   context->slavelist[0].state = EC_STATE_OPERATIONAL;
   ecx_writestate(context, 0);
   for (i = 0; i < 500; ++i)
   {
      roundtrip(bus);
      ecx_statecheck(context, 0, EC_STATE_OPERATIONAL, bus->cycle_us);
      if (context->slavelist[0].state == EC_STATE_OPERATIONAL)
      {
         bus->expected_wkc = group->outputsWKC * 2 + group->inputsWKC;
         expected_wkc = bus->expected_wkc;
         printf("OP reached, expected WKC %d\n", expected_wkc);
         /* Initialize PDO output values before starting cyclic thread.
            Without this, the first PDO frames carry controlword=0
            (Disable voltage + Quick stop active), locking the servo
            in Switch-on-disabled state. */
         {
            ec_slavet *slave = &context->slavelist[SERVO_SLAVE];
            write_u16(slave, &bus->map.controlword, 0x0006);
            if (bus->map.modes_of_operation.present)
            {
               write_i8(slave, &bus->map.modes_of_operation, 0);
            }
            if (bus->map.target_velocity.present)
            {
               write_i32(slave, &bus->map.target_velocity, 0);
            }
            if (bus->map.max_profile_velocity.present)
            {
               write_i32(slave, &bus->map.max_profile_velocity, 0);
            }
            /* Do one manual roundtrip to push safe values before cyclic takes over */
            roundtrip(bus);
         }
         if (!start_cyclic(bus))
         {
            printf("Failed to start cyclic PDO thread\n");
            return 0;
         }
         run_cycles(bus, 20);
         return 1;
      }
      osal_usleep(bus->cycle_us);
   }

   printf("Failed to reach OP\n");
   return 0;
}

static void bus_stop(ServoBus *bus)
{
   ecx_contextt *context = &bus->context;
   int i;

   servo_disable(bus);
   stop_cyclic(bus);
   for (i = 1; i <= context->slavecount; ++i)
   {
      if (context->slavelist[i].DCactive)
      {
         ecx_dcsync0(context, i, FALSE, 0, 0);
      }
   }
   context->slavelist[0].state = EC_STATE_INIT;
   ecx_writestate(context, 0);
   ecx_close(context);
}

static void usage(const char *name)
{
   printf("Usage:\n");
   printf("  %s IFACE status [cycle_us]          Show servo status\n", name);
   printf("  %s IFACE start [cycle_us]           Enable servo, interactive control\n", name);
   printf("  %s IFACE stop [cycle_us]            Disable servo\n", name);
   printf("  %s IFACE move DELTA [cycle_us]      Enable, smooth relative move by DELTA, disable\n", name);
   printf("  %s IFACE mover DELTA [cycle_us]     Same as move\n", name);
   printf("  %s IFACE vel VELOCITY [cycle_us]    Run at constant velocity until Ctrl-C\n", name);
   printf("  %s IFACE fault-reset [cycle_us]     Reset servo fault\n", name);
   printf("  cycle_us defaults to %d; try 2000 or 4000 in a VM if EE08.6 appears\n",
          DEFAULT_CYCLE_US);
   printf("\nInteractive commands (after start):\n");
   printf("  <delta>          Smooth relative move by delta (max +/-%d)\n", MAX_DELTA);
   printf("  vel VELOCITY is limited to +/-%d; if 0x60FF is not mapped, unit is position-counts/s\n",
          MAX_VELOCITY);
   printf("  a<absolute_pos>  Move to absolute position (no limit)\n");
   printf("  v<speed>         Set move speed in pulses/ms (default %d, max 1000)\n", DEFAULT_SPEED);
   printf("  s                Show status\n");
   printf("  q                Quit\n");
}

int main(int argc, char *argv[])
{
   ServoBus bus;
   const char *cmd;
   uint32 cycle_us;
   int ok = 1;

   if (argc < 3)
   {
      usage(argv[0]);
      return 1;
   }

   cmd = argv[2];
   cycle_us = DEFAULT_CYCLE_US;
   if ((strcmp(cmd, "move") == 0) || (strcmp(cmd, "mover") == 0) ||
       (strcmp(cmd, "vel") == 0))
   {
      if (argc >= 5)
      {
         cycle_us = (uint32)strtoul(argv[4], NULL, 0);
      }
   }
   else if (argc >= 4)
   {
      cycle_us = (uint32)strtoul(argv[3], NULL, 0);
   }

   if (!bus_start(&bus, argv[1], cycle_us, strcmp(cmd, "vel") == 0))
   {
      ecx_close(&bus.context);
      return 1;
   }

   if (strcmp(cmd, "status") == 0)
   {
      print_status(&bus);
   }
   else if (strcmp(cmd, "start") == 0)
   {
      ok = servo_enable(&bus, MODE_CSP);
      print_pdo_status(&bus);
      if (ok)
      {
         ec_slavet *slave = &bus.context.slavelist[SERVO_SLAVE];
         char line[256];
         int line_pos = 0;
         int32 speed = DEFAULT_SPEED;  /* configurable via 'v' command */
         printf("Motor enabled. Speed=%d pulses/ms, MAX_DELTA=%d\n", speed, MAX_DELTA);
         printf("Commands: <delta> | a<abs_pos> | v<speed> | s | q\n");
         printf("> ");
         fflush(stdout);
         signal(SIGINT, handle_signal);
         signal(SIGTERM, handle_signal);
         while (keep_running)
         {
            fd_set fds;
            struct timeval tv;
            int ret;
            uint16 sw;

            /* Check motor state from latest cyclic data */
            sw = read_u16(slave, &bus.map.statusword);
            if ((sw & 0x0008) && !is_operation_enabled(sw))
            {
               printf("\n* Motor fault (statusword=0x%04x, error=0x%04x)\n",
                      sw, read_u16(slave, &bus.map.error_code));
               break;
            }

            /* Poll stdin with 1ms timeout */
            FD_ZERO(&fds);
            FD_SET(STDIN_FILENO, &fds);
            tv.tv_sec = 0;
            tv.tv_usec = bus.cycle_us;
            ret = select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv);

            if (ret > 0 && FD_ISSET(STDIN_FILENO, &fds))
            {
               char c;
               if (read(STDIN_FILENO, &c, 1) <= 0)
               {
                  break;
               }
               if (c == '\n')
               {
                  line[line_pos] = '\0';
                  if (line[0] == 'q')
                  {
                     break;
                  }
                  if (line[0] == 's')
                  {
                     print_pdo_status(&bus);
                  }
                  else if (line[0] == 'v' && line[1] != '\0')
                  {
                     /* Set move speed (pulses per ms) */
                     speed = (int32)strtol(line + 1, NULL, 0);
                     if (speed <= 0) speed = DEFAULT_SPEED;
                     if (speed > 1000) speed = 1000;
                     printf("Speed set to %d pulses/ms\n", speed);
                  }
                  else if (line[0] == 'a')
                  {
                     /* Absolute position move (advanced, no limit check) */
                     int32 abs_pos = (int32)strtol(line + 1, NULL, 0);
                     int32 current = read_i32(slave, &bus.map.actual_position);
                     int32 delta = abs_pos - current;
                     if (is_operation_enabled(sw))
                     {
                        motor_move_smooth(&bus, delta, speed);
                     }
                     else
                     {
                        printf("Not operation-enabled (0x%04x: %s)\n",
                               sw, cia402_state(sw));
                     }
                  }
                  else if ((line[0] >= '0' && line[0] <= '9') ||
                           line[0] == '-' || line[0] == '+')
                  {
                     /* Relative move (default): each number is a delta */
                     int32 delta = (int32)strtol(line, NULL, 0);
                     if (!is_operation_enabled(sw))
                     {
                        printf("Not operation-enabled (0x%04x: %s)\n",
                               sw, cia402_state(sw));
                     }
                     else if (abs(delta) > MAX_DELTA)
                     {
                        printf("SAFETY: delta %d exceeds MAX_DELTA %d, rejected\n",
                               delta, MAX_DELTA);
                     }
                     else
                     {
                        motor_move_smooth(&bus, delta, speed);
                     }
                  }
                  else if (line[0] != '\0')
                  {
                     printf("Unknown: %s\n", line);
                  }
                  line_pos = 0;
                  printf("> ");
                  fflush(stdout);
               }
               else if (line_pos < (int)sizeof(line) - 1)
               {
                  line[line_pos++] = c;
               }
            }
         }
         printf("\nDisabling servo\n");
      }
      else
      {
         stop_cyclic(&bus);
         print_status(&bus);
      }
   }
   else if (strcmp(cmd, "move") == 0 || strcmp(cmd, "mover") == 0)
   {
      if (argc < 4)
      {
         printf("Usage: %s IFACE %s <delta>\n", argv[0], cmd);
         ok = 0;
      }
      else if (servo_enable(&bus, MODE_CSP))
      {
         int32 delta = (int32)strtol(argv[3], NULL, 0);
         if (abs(delta) > MAX_DELTA)
         {
            printf("SAFETY: delta %d exceeds MAX_DELTA %d, rejected\n",
                   delta, MAX_DELTA);
            ok = 0;
         }
         else
         {
            motor_move_smooth(&bus, delta, DEFAULT_SPEED);
         }
         print_pdo_status(&bus);
      }
      else
      {
         ok = 0;
      }
   }
   else if (strcmp(cmd, "vel") == 0)
   {
      if (argc < 4)
      {
         printf("Usage: %s IFACE vel <velocity> [cycle_us]\n", argv[0]);
         ok = 0;
      }
      else
      {
         int32 velocity = (int32)strtol(argv[3], NULL, 0);
         if (velocity > MAX_VELOCITY || velocity < -MAX_VELOCITY)
         {
            printf("SAFETY: velocity %d exceeds MAX_VELOCITY %d, rejected\n",
                   velocity, MAX_VELOCITY);
            ok = 0;
         }
         else if (bus.map.target_velocity.present)
         {
            if (servo_enable(&bus, MODE_CSV))
            {
               motor_run_velocity(&bus, velocity);
               print_pdo_status(&bus);
            }
            else
            {
               ok = 0;
            }
         }
         else
         {
            printf("Native CSV unavailable: 0x60FF target velocity is not mapped in RxPDO.\n");
            printf("Check the earlier CSV RxPDO remap messages; no CSP fallback is used for vel.\n");
            ok = 0;
         }
      }
   }
   else if (strcmp(cmd, "stop") == 0)
   {
      servo_disable(&bus);
      print_status(&bus);
   }
   else if (strcmp(cmd, "fault-reset") == 0)
   {
      servo_fault_reset(&bus);
      print_status(&bus);
   }
   else
   {
      usage(argv[0]);
      ok = 0;
   }

   bus_stop(&bus);
   return ok ? 0 : 1;
}
