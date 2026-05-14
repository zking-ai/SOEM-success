/** \file
 * \brief Simple EtherCAT servo velocity control (PV mode).
 *
 * Drive handles acceleration/deceleration internally via
 * profile acceleration (0x6083) and deceleration (0x6084).
 * Master only writes target velocity (0x60FF).
 *
 * Usage:
 *   servo_cli IFACE enable [cycle_us]    Enable servo, prompt for velocity
 *   servo_cli IFACE status [cycle_us]    Show servo diagnostics
 *   servo_cli IFACE fault-reset [cycle_us]  Reset servo fault
 *
 * Flow:
 *   1. "enable"  → bus init, OP, CiA402 enable, wait for input
 *   2. type a number (e.g. 50000) → motor runs at that velocity
 *   3. Ctrl-C    → stops motion, stays in OP, back to prompt
 *   4. Ctrl-C at prompt (or 'q') → safe shutdown
 */

#include "soem/soem.h"
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <unistd.h>

#define SERVO_SLAVE       1
#define DEFAULT_CYCLE_US  1000
#define IO_MAP_SIZE       4096

#define MODE_PV   3
#define MODE_CSV  9

#define MAX_VELOCITY  5000000
#define ACCEL_DEFAULT 8000000
#define DECEL_DEFAULT 8000000

/* ---- state shared with signal handler ---- */
static volatile sig_atomic_t g_running    = 1;  /* whole program */
static volatile sig_atomic_t g_in_motion  = 0;  /* motor is spinning */
static volatile sig_atomic_t g_stop_motion = 0; /* request to stop motor */

static void on_signal(int sig)
{
   (void)sig;
   if (g_in_motion) g_stop_motion = 1;
   else             g_running = 0;
}

/* ---- PDO layout ---- */
typedef struct {
   int present, bit_offset, bit_length;
} PdoEntry;

typedef struct {
   PdoEntry controlword, statusword, error_code;
   PdoEntry modes_of_operation, mode_display;
   PdoEntry target_velocity, target_position, actual_position;
   PdoEntry digital_outputs, max_profile_velocity;
} ServoMap;

typedef struct {
   ecx_contextt    ctx;
   uint8           iomap[IO_MAP_SIZE];
   ServoMap        map;
   pthread_t       thread;
   volatile int    thread_run;
   int             expected_wkc, wkc, cycle, bad_wkc;
   uint32          cycle_us, cycle_ns;
} ServoBus;

/* ---- tiny helpers ---- */
static int roundtrip(ServoBus *b)
   { ecx_send_processdata(&b->ctx); return ecx_receive_processdata(&b->ctx, EC_TIMEOUTRET); }

static void sleep_cyc(ServoBus *b, int n) {
   for (int i = 0; i < n; i++) { if (!b->thread_run) roundtrip(b); osal_usleep(b->cycle_us); }
}

static void drain_errs(ecx_contextt *c) { while (c->ecaterror) ecx_elist2string(c); }

/* ---- PDO access ---- */
static uint8 *e_ptr(ec_slavet *s, const PdoEntry *e, int out) {
   if (!e->present || (e->bit_offset & 7)) return NULL;
   return (out ? s->outputs : s->inputs) + e->bit_offset / 8;
}
static void pdo_w16(ec_slavet *s, const PdoEntry *e, uint16 v) {
   uint8 *p = e_ptr(s, e, 1); if (p && e->bit_length == 16) { uint16 x = htoes(v); memcpy(p, &x, 2); }
}
static void pdo_w32(ec_slavet *s, const PdoEntry *e, int32 v) {
   uint8 *p = e_ptr(s, e, 1); if (p && e->bit_length == 32) { int32 x = htoel(v); memcpy(p, &x, 4); }
}
static void pdo_w8(ec_slavet *s, const PdoEntry *e, int8 v) {
   uint8 *p = e_ptr(s, e, 1); if (p && e->bit_length == 8) memcpy(p, &v, 1);
}
static uint16 pdo_r16(ec_slavet *s, const PdoEntry *e) {
   uint8 *p = e_ptr(s, e, 0); uint16 v = 0;
   if (p && e->bit_length == 16) { memcpy(&v, p, 2); v = etohs(v); } return v;
}
static int32 pdo_r32(ec_slavet *s, const PdoEntry *e) {
   uint8 *p = e_ptr(s, e, 0); int32 v = 0;
   if (p && e->bit_length == 32) { memcpy(&v, p, 4); v = etohl(v); } return v;
}

/* ---- SDO access ---- */
static int sdo_r8 (ecx_contextt *c, uint16 i, uint8  su, uint8  *v) { int sz=1; return ecx_SDOread(c,SERVO_SLAVE,i,su,FALSE,&sz,v,EC_TIMEOUTRXM)>0; }
static int sdo_r16(ecx_contextt *c, uint16 i, uint8  su, uint16 *v) { int sz=2; if(ecx_SDOread(c,SERVO_SLAVE,i,su,FALSE,&sz,v,EC_TIMEOUTRXM)<=0)return 0; *v=etohs(*v); return 1; }
static int sdo_r32(ecx_contextt *c, uint16 i, uint8  su, uint32 *v) { int sz=4; if(ecx_SDOread(c,SERVO_SLAVE,i,su,FALSE,&sz,v,EC_TIMEOUTRXM)<=0)return 0; *v=etohl(*v); return 1; }
static int sdo_w8 (ecx_contextt *c, uint16 i, uint8  su, uint8  v) { return ecx_SDOwrite(c,SERVO_SLAVE,i,su,FALSE,1,&v,EC_TIMEOUTRXM)>0; }
static int sdo_w16(ecx_contextt *c, uint16 i, uint8  su, uint16 v) { uint16 x=htoes(v); return ecx_SDOwrite(c,SERVO_SLAVE,i,su,FALSE,2,&x,EC_TIMEOUTRXM)>0; }
static int sdo_w32(ecx_contextt *c, uint16 i, uint8  su, uint32 v) { uint32 x=htoel(v); return ecx_SDOwrite(c,SERVO_SLAVE,i,su,FALSE,4,&x,EC_TIMEOUTRXM)>0; }

/* ---- CiA402 helpers ---- */
static const char *cia402_str(uint16 sw) {
   if (sw & 0x0008) return "FAULT";
   if ((sw & 0x004f) == 0x0040) return "switch-on-disabled";
   if ((sw & 0x006f) == 0x0021) return "ready-to-switch-on";
   if ((sw & 0x006f) == 0x0023) return "switched-on";
   if ((sw & 0x006f) == 0x0027) return "operation-enabled";
   return "?";
}
static int is_oe(uint16 sw) { return (sw & 0x006f) == 0x0027; }

/* ---- PDO map discovery ---- */
static void remember(ServoMap *m, uint16 idx, uint8 sub, int off, int len) {
   if (sub != 0) return;
   PdoEntry *e = NULL;
   switch (idx) {
   case 0x603f: e = &m->error_code; break;
   case 0x6040: e = &m->controlword; break;
   case 0x6041: e = &m->statusword; break;
   case 0x6060: e = &m->modes_of_operation; break;
   case 0x6061: e = &m->mode_display; break;
   case 0x60ff: e = &m->target_velocity; break;
   case 0x607a: e = &m->target_position; break;
   case 0x6064: e = &m->actual_position; break;
   case 0x607f: e = &m->max_profile_velocity; break;
   case 0x60fe: e = &m->digital_outputs; break;
   }
   if (e) { e->present = 1; e->bit_offset = off; e->bit_length = len; }
}

static int read_map(ecx_contextt *c, ServoMap *m, uint16 assign, int boff) {
   uint8 pc = 0, ec = 0; uint16 pi = 0; uint32 mp = 0; int sz;
   sz = 1; if (ecx_SDOread(c, SERVO_SLAVE, assign, 0x00, FALSE, &sz, &pc, EC_TIMEOUTRXM) <= 0) return boff;
   for (int i = 1; i <= pc; i++) {
      sz = 2; pi = 0;
      if (ecx_SDOread(c, SERVO_SLAVE, assign, (uint8)i, FALSE, &sz, &pi, EC_TIMEOUTRXM) <= 0) return boff;
      pi = etohs(pi);
      sz = 1; ec = 0;
      if (ecx_SDOread(c, SERVO_SLAVE, pi, 0x00, FALSE, &sz, &ec, EC_TIMEOUTRXM) <= 0) return boff;
      for (int j = 1; j <= ec; j++) {
         sz = 4; mp = 0;
         if (ecx_SDOread(c, SERVO_SLAVE, pi, (uint8)j, FALSE, &sz, &mp, EC_TIMEOUTRXM) <= 0) return boff;
         mp = etohl(mp);
         uint16 idx = (uint16)(mp >> 16);
         uint8  sub = (uint8)((mp >> 8) & 0xff);
         uint8  blen = (uint8)(mp & 0xff);
         printf("  %04x:%02x bit:%d len:%u\n", idx, sub, boff, blen);
         remember(m, idx, sub, boff, blen);
         boff += blen;
      }
   }
   return boff;
}

/* ---- scan fixed PDOs for one containing 0x60FF + 0x6040 ---- */
static int find_and_assign_pv_pdo(ecx_contextt *c)
{
   int sz;
   printf("Scanning fixed RxPDOs for 0x6040+0x60FF...\n");
   for (uint16 idx = 0x1600; idx <= 0x17ff; idx++) {
      uint8 cnt = 0; sz = 1;
      if (ecx_SDOread(c, SERVO_SLAVE, idx, 0x00, FALSE, &sz, &cnt, EC_TIMEOUTRXM) <= 0) {
         drain_errs(c); continue;
      }
      int has_6040 = 0, has_60ff = 0;
      for (int j = 1; j <= cnt && !(has_6040 && has_60ff); j++) {
         uint32 m = 0; sz = 4;
         if (ecx_SDOread(c, SERVO_SLAVE, idx, (uint8)j, FALSE, &sz, &m, EC_TIMEOUTRXM) > 0) {
            m = etohl(m);
            if ((m >> 16) == 0x6040) has_6040 = 1;
            if ((m >> 16) == 0x60ff) has_60ff = 1;
         }
      }
      if (has_6040 && has_60ff) {
         printf("  Found PV-compatible PDO 0x%04x\n", idx);
         if (!sdo_w8(c, 0x1c12, 0x00, 0)) { drain_errs(c); return 0; }
         if (!sdo_w16(c, 0x1c12, 0x01, idx)) { drain_errs(c); return 0; }
         if (!sdo_w8(c, 0x1c12, 0x00, 1)) { drain_errs(c); return 0; }
         return 1;
      }
      drain_errs(c);
   }
   printf("  No fixed PDO with 0x60FF found — drive may need manual PDO config\n");
   return 0;
}

static int pv_pdo_cb(ecx_contextt *c, uint16 slave)
{
   if (slave != SERVO_SLAVE) return 0;
   return find_and_assign_pv_pdo(c);
}

/* ---- cyclic thread ---- */
static void add_ns(ec_timet *t, int64 ns) {
   ec_timet a; a.tv_nsec = ns % 1000000000; a.tv_sec = (ns - a.tv_nsec) / 1000000000;
   osal_timespecadd(t, &a, t);
}
static OSAL_THREAD_FUNC_RT cyclic_thread(void *arg) {
   ServoBus *b = arg; ec_timet ts; int ht;
   osal_get_monotonic_time(&ts);
   ht = (ts.tv_nsec / 1000000) + 1; ts.tv_nsec = ht * 1000000;
   ecx_send_processdata(&b->ctx);
   while (b->thread_run) {
      add_ns(&ts, b->cycle_ns); osal_monotonic_sleep(&ts);
      b->wkc = ecx_receive_processdata(&b->ctx, EC_TIMEOUTRET);
      if (b->wkc != b->expected_wkc) b->bad_wkc++;
      ecx_send_processdata(&b->ctx);
      b->cycle++;
   }
}
static int start_thread(ServoBus *b) {
   b->thread_run = 1;
   if (!osal_thread_create_rt(&b->thread, 128000, &cyclic_thread, b)) {
      printf("Warning: RT thread failed, using normal priority\n");
      if (!osal_thread_create(&b->thread, 128000, &cyclic_thread, b)) { b->thread_run = 0; return 0; }
   }
   return 1;
}
static void stop_thread(ServoBus *b) {
   if (!b->thread_run) return;
   b->thread_run = 0; pthread_join(b->thread, NULL);
}

/* ---- bus init ---- */
static int bus_init(ServoBus *b, const char *iface, uint32 cyc_us)
{
   ecx_contextt *c = &b->ctx;
   memset(b, 0, sizeof(*b));
   b->cycle_us = cyc_us ? cyc_us : DEFAULT_CYCLE_US;
   b->cycle_ns = b->cycle_us * 1000;

   printf("Opening %s, cycle=%u us\n", iface, b->cycle_us);
   if (!ecx_init(c, iface)) { printf("No socket\n"); return 0; }
   if (ecx_config_init(c) <= 0) { printf("No slaves\n"); return 0; }
   if (c->slavecount < SERVO_SLAVE) { printf("Slave %d missing\n", SERVO_SLAVE); return 0; }

   c->manualstatechange = 1;
   c->slavelist[SERVO_SLAVE].PO2SOconfig = pv_pdo_cb;
   ecx_config_map_group(c, b->iomap, 0);

   printf("RxPDO:\n"); read_map(c, &b->map, 0x1c12, 0);
   printf("TxPDO:\n"); read_map(c, &b->map, 0x1c13, 0);
   drain_errs(c);

   if (!b->map.controlword.present || !b->map.statusword.present) { printf("Need 0x6040/0x6041\n"); return 0; }
   if (!b->map.target_velocity.present) { printf("0x60FF not mapped — PDO scan failed?\n"); return 0; }

   /* configure DC sync mode */
   {  uint16 sm = htoes(2); uint32 cy = htoel(b->cycle_ns);
      ecx_SDOwrite(c, SERVO_SLAVE, 0x1c32, 0x01, FALSE, 2, &sm, EC_TIMEOUTRXM);
      ecx_SDOwrite(c, SERVO_SLAVE, 0x1c33, 0x01, FALSE, 2, &sm, EC_TIMEOUTRXM);
      ecx_SDOwrite(c, SERVO_SLAVE, 0x1c32, 0x02, FALSE, 4, &cy, EC_TIMEOUTRXM);
      ecx_SDOwrite(c, SERVO_SLAVE, 0x1c33, 0x02, FALSE, 4, &cy, EC_TIMEOUTRXM);
      drain_errs(c);
   }

   ecx_configdc(c);
   for (int i = 1; i <= c->slavecount; i++)
      if (c->slavelist[i].hasdc) ecx_dcsync0(c, i, TRUE, b->cycle_ns, 0);

   /* → SAFE_OP → OP */
   c->slavelist[0].state = EC_STATE_SAFE_OP; ecx_writestate(c, 0);
   ecx_statecheck(c, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
   if (c->slavelist[0].state != EC_STATE_SAFE_OP) { printf("FAIL SAFE_OP\n"); return 0; }
   sleep_cyc(b, 200);

   c->slavelist[0].state = EC_STATE_OPERATIONAL; ecx_writestate(c, 0);
   for (int i = 0; i < 500; i++) {
      roundtrip(b); ecx_statecheck(c, 0, EC_STATE_OPERATIONAL, b->cycle_us);
      if (c->slavelist[0].state == EC_STATE_OPERATIONAL) {
         b->expected_wkc = c->grouplist[0].outputsWKC * 2 + c->grouplist[0].inputsWKC;
         printf("OP OK, expected WKC=%d\n", b->expected_wkc);
         /* prime safe outputs */
         {  ec_slavet *s = &c->slavelist[SERVO_SLAVE];
            pdo_w16(s, &b->map.controlword, 0x0006);
            pdo_w8(s, &b->map.modes_of_operation, 0);
            pdo_w32(s, &b->map.target_velocity, 0);
            roundtrip(b);
         }
         if (!start_thread(b)) { printf("Thread fail\n"); return 0; }
         sleep_cyc(b, 200);
         return 1;
      }
      osal_usleep(b->cycle_us);
   }
   printf("FAIL OP\n"); return 0;
}

/* ---- CiA402 enable (PV mode) ---- */
static int servo_enable(ServoBus *b)
{
   ecx_contextt *c = &b->ctx;
   ec_slavet *s = &c->slavelist[SERVO_SLAVE];
   uint16 sw = pdo_r16(s, &b->map.statusword);

   /* fault reset if needed */
   if (sw & 0x0008) {
      printf("Fault (sw=0x%04x err=0x%04x), resetting...\n",
             sw, b->map.error_code.present ? pdo_r16(s, &b->map.error_code) : 0);
      for (int j = 0; j < 3; j++) {
         pdo_w16(s, &b->map.controlword, 0x0080); sleep_cyc(b, 100);
         pdo_w16(s, &b->map.controlword, 0x0000); sleep_cyc(b, 100);
         sw = pdo_r16(s, &b->map.statusword);
         if (!(sw & 0x0008)) { printf("Fault cleared\n"); goto ok; }
      }
      sdo_w32(c, 0x203f, 0x00, 0); sleep_cyc(b, 100);
      pdo_w16(s, &b->map.controlword, 0x0080); sleep_cyc(b, 100);
      pdo_w16(s, &b->map.controlword, 0x0000); sleep_cyc(b, 100);
      sw = pdo_r16(s, &b->map.statusword);
      if (sw & 0x0008) { printf("Fault persists — use InoDriveShop reset\n"); return 0; }
      printf("Fault cleared (SDO)\n");
   }
ok:

   /* set PV mode + profile accel/decel */
   printf("Mode → PV (3)\n"); sdo_w8(c, 0x6060, 0x00, MODE_PV);
   printf("0x6083=%u 0x6084=%u\n", ACCEL_DEFAULT, DECEL_DEFAULT);
   sdo_w32(c, 0x6083, 0x00, ACCEL_DEFAULT);
   sdo_w32(c, 0x6084, 0x00, DECEL_DEFAULT);
   drain_errs(c); sleep_cyc(b, 20);

   /* write max_profile_velocity via SDO + PDO */
   sdo_w32(c, 0x607f, 0x00, MAX_VELOCITY);
   if (b->map.max_profile_velocity.present)
      pdo_w32(s, &b->map.max_profile_velocity, MAX_VELOCITY);

   /* Start CiA402 state machine from current state.
    * Shutdown (0x0006) reaches ready-to-switch-on from any non-fault state.
    * Retry up to 5 times — drive internal init (power stage, encoder, DC lock)
    * takes variable time after OP is reached. */
   for (int attempt = 0; attempt < 10; attempt++) {
      if (attempt > 0) {
         printf("Retry %d/10, waiting 1000ms...\n", attempt + 1);
         sleep_cyc(b, 1000);
      }

      sw = pdo_r16(s, &b->map.statusword);
      printf("Attempt %d: sw=0x%04x (%s)\n", attempt + 1, sw, cia402_str(sw));
      if (sw & 0x0008) { printf("  Fault detected, aborting\n"); return 0; }

      /* state machine: shutdown → switch-on → enable-operation */
      pdo_w32(s, &b->map.target_velocity, 0);

      /* shutdown → ready-to-switch-on */
      int ok_rts = 0;
      for (int i = 0; i < 2000; i++) {
         pdo_w16(s, &b->map.controlword, 0x0006);
         osal_usleep(b->cycle_us);
         sw = pdo_r16(s, &b->map.statusword);
         if ((sw & 0x006f) == 0x0021) { ok_rts = 1; break; }
      }
      if (!ok_rts) { printf("  shutdown timeout (sw=0x%04x)\n", sw); continue; }

      /* switch-on → switched-on */
      int ok_so = 0;
      for (int i = 0; i < 2000; i++) {
         pdo_w16(s, &b->map.controlword, 0x0007);
         osal_usleep(b->cycle_us);
         sw = pdo_r16(s, &b->map.statusword);
         if ((sw & 0x006f) == 0x0023) { ok_so = 1; break; }
      }
      if (!ok_so) { printf("  switch-on timeout (sw=0x%04x)\n", sw); continue; }

      /* enable-operation → operation-enabled */
      pdo_w32(s, &b->map.target_velocity, 0);
      for (int i = 0; i < 2000; i++) {
         pdo_w16(s, &b->map.controlword, 0x000f);
         osal_usleep(b->cycle_us);
         sw = pdo_r16(s, &b->map.statusword);
         if (sw & 0x0008) {
            printf("  FAULT during enable (sw=0x%04x err=0x%04x)\n",
                   sw, b->map.error_code.present ? pdo_r16(s, &b->map.error_code) : 0);
            return 0;
         }
         if (is_oe(sw)) {
            printf("Enabled (OP), sw=0x%04x\n", sw);
            if (b->map.modes_of_operation.present)
               pdo_w8(s, &b->map.modes_of_operation, MODE_PV);
            return 1;
         }
      }
      printf("  enable timeout (sw=0x%04x)\n", sw);
   }
   printf("FAIL: could not enable after 10 attempts\n");
   return 0;
}

/* ---- PV velocity run ---- */
static void motor_run(ServoBus *b, int32 vel)
{
   ec_slavet *s = &b->ctx.slavelist[SERVO_SLAVE];

   g_in_motion  = 1;
   g_stop_motion = 0;
   printf("Running at %d (drive handles ramp). Ctrl-C to stop.\n", vel);

   if (b->map.modes_of_operation.present)
      pdo_w8(s, &b->map.modes_of_operation, MODE_PV);
   if (b->map.max_profile_velocity.present)
      pdo_w32(s, &b->map.max_profile_velocity, MAX_VELOCITY);
   pdo_w32(s, &b->map.target_velocity, vel);
   pdo_w16(s, &b->map.controlword, 0x000f);

   while (!g_stop_motion) {
      uint16 sw = pdo_r16(s, &b->map.statusword);
      if (sw & 0x0008) { printf("* FAULT 0x%04x\n", sw); break; }
      if (!is_oe(sw))  { printf("Lost OP (0x%04x)\n", sw); break; }
      if (b->map.modes_of_operation.present)
         pdo_w8(s, &b->map.modes_of_operation, MODE_PV);
      if (b->map.max_profile_velocity.present)
         pdo_w32(s, &b->map.max_profile_velocity, MAX_VELOCITY);
      pdo_w32(s, &b->map.target_velocity, vel);
      pdo_w16(s, &b->map.controlword, 0x000f);
      osal_usleep(b->cycle_us);
   }

   printf("Stopping...\n");
   if (b->map.modes_of_operation.present)
      pdo_w8(s, &b->map.modes_of_operation, MODE_PV);
   if (b->map.max_profile_velocity.present)
      pdo_w32(s, &b->map.max_profile_velocity, MAX_VELOCITY);
   pdo_w32(s, &b->map.target_velocity, 0);
   pdo_w16(s, &b->map.controlword, 0x000f);
   sleep_cyc(b, 500);
   g_in_motion = 0;
   printf("Stopped (still in OP)\n");
}

/* ---- CiA402 disable (called while cyclic thread is RUNNING) ---- */
static void servo_disable(ServoBus *b)
{
   ec_slavet *s = &b->ctx.slavelist[SERVO_SLAVE];
   uint16 sw = pdo_r16(s, &b->map.statusword);
   printf("Disabling CiA402...\n");

   if ((sw & 0x006f) == 0x0040) { printf("  Already disabled\n"); return; }
   if (sw & 0x0008) { printf("  In fault, skipping\n"); return; }

   /* Stage 1: operation-enabled → switched-on */
   if ((sw & 0x006f) == 0x0027) {
      printf("  disable-op...\n");
      for (int i = 0; i < 500; i++) {
         pdo_w16(s, &b->map.controlword, 0x0007);
         pdo_w32(s, &b->map.target_velocity, 0);
         osal_usleep(b->cycle_us);
         sw = pdo_r16(s, &b->map.statusword);
         if ((sw & 0x006f) == 0x0023 || (sw & 0x006f) == 0x0021) break;
      }
   }
   /* Stage 2: → switch-on-disabled */
   if ((sw & 0x006f) != 0x0040) {
      printf("  disable-voltage...\n");
      for (int i = 0; i < 500; i++) {
         pdo_w16(s, &b->map.controlword, 0x0000);
         pdo_w32(s, &b->map.target_velocity, 0);
         osal_usleep(b->cycle_us);
         if ((pdo_r16(s, &b->map.statusword) & 0x006f) == 0x0040) { printf("  OK\n"); return; }
      }
   }
   printf("  sw=0x%04x\n", pdo_r16(s, &b->map.statusword));
}

/* ---- safe shutdown (like InoDriveShop) ---- */
static void bus_shutdown(ServoBus *b)
{
   ecx_contextt *c = &b->ctx;
   printf("=== Safe shutdown ===\n");

   /* 1. CiA402 disable (cyclic thread still running!) */
   servo_disable(b);

   /* 2. Stop cyclic PDO */
   printf("Stop cyclic...\n"); stop_thread(b);

   /* 3. EtherCAT state: OP(8) → SAFE_OP(4) → PRE_OP(2) → INIT(1) */
   int states[] = { EC_STATE_SAFE_OP, EC_STATE_PRE_OP, EC_STATE_INIT };
   const char *names[] = { "SAFE_OP", "PRE_OP", "INIT" };
   for (int i = 0; i < 3; i++) {
      printf("  → %s\n", names[i]);
      c->slavelist[0].state = states[i]; ecx_writestate(c, 0);
      for (int j = 0; j < 200; j++) {
         roundtrip(b); ecx_statecheck(c, 0, states[i], b->cycle_us);
         if (c->slavelist[0].state == states[i]) break;
         osal_usleep(b->cycle_us);
      }
   }

   /* 4. Disable DC sync */
   for (int i = 1; i <= c->slavecount; i++)
      if (c->slavelist[i].DCactive) ecx_dcsync0(c, i, FALSE, 0, 0);

   /* 5. Close */
   ecx_close(c);
   printf("=== Done ===\n");
}

/* ---- diagnostics ---- */
static void show_status(ServoBus *b)
{
   ecx_contextt *c = &b->ctx;
   ec_slavet *s = &c->slavelist[SERVO_SLAVE];
   uint16 sw = pdo_r16(s, &b->map.statusword);
   printf("Statusword: 0x%04x (%s)\n", sw, cia402_str(sw));
   if (b->map.error_code.present) printf("Error: 0x%04x\n", pdo_r16(s, &b->map.error_code));
   uint32 d; if (sdo_r32(c, 0x203f, 0x00, &d)) printf("0x203F: %08x\n", d);
   int8 m; if (sdo_r8(c, 0x6061, 0x00, (uint8*)&m)) printf("Mode display: %d\n", m);
   if (b->map.actual_position.present) printf("Pos: %d\n", pdo_r32(s, &b->map.actual_position));
   printf("WKC=%d bad=%d cycle=%d\n", b->wkc, b->bad_wkc, b->cycle);
   drain_errs(c);
}

/* ---- main ---- */
static void usage(const char *p)
{
   printf("Usage:\n");
   printf("  %s IFACE enable [cycle_us]      Enable servo, prompt for velocity\n", p);
   printf("  %s IFACE status [cycle_us]      Show diagnostics\n", p);
   printf("  %s IFACE fault-reset [cycle_us] Reset fault\n", p);
   printf("\nFlow:\n");
   printf("  enable  → type a number (velocity) → motor runs\n");
   printf("  Ctrl-C during run  → stops motion, stays in OP\n");
   printf("  Ctrl-C at prompt   → safe shutdown\n");
   printf("  q at prompt        → safe shutdown\n");
}

int main(int argc, char **argv)
{
   ServoBus bus;
   const char *cmd;
   uint32 cyc;
   int ok = 1, started = 0;

   if (argc < 3) { usage(argv[0]); return 1; }
   cmd = argv[2];
   cyc = (argc >= 4) ? (uint32)strtoul(argv[3], NULL, 0) : DEFAULT_CYCLE_US;

   if (strcmp(cmd, "status") == 0) {
      if (!bus_init(&bus, argv[1], cyc)) { ecx_close(&bus.ctx); return 1; }
      started = 1; show_status(&bus);
   }
   else if (strcmp(cmd, "fault-reset") == 0) {
      if (!bus_init(&bus, argv[1], cyc)) { ecx_close(&bus.ctx); return 1; }
      started = 1;
      /* fault reset is embedded in servo_enable */
      servo_enable(&bus);
      show_status(&bus);
   }
   else if (strcmp(cmd, "enable") == 0) {
      if (!bus_init(&bus, argv[1], cyc)) { ecx_close(&bus.ctx); return 1; }
      started = 1;
      if (!servo_enable(&bus)) { ok = 0; goto done; }

      /* ---- prompt loop ---- */
      signal(SIGINT, on_signal);
      signal(SIGTERM, on_signal);
      g_running = 1;
      printf("Ready. Enter velocity value (e.g. 50000) to run, 'q' to quit.\n");

      while (g_running) {
         printf("> "); fflush(stdout);

         /* read one line */
         char line[64]; int pos = 0;
         while (pos < (int)sizeof(line) - 1) {
            char ch;
            if (read(STDIN_FILENO, &ch, 1) <= 0) { g_running = 0; break; }
            if (ch == '\n') break;
            line[pos++] = ch;
         }
         line[pos] = '\0';
         if (!g_running) break;

         /* check for faults */
         {
            ec_slavet *s = &bus.ctx.slavelist[SERVO_SLAVE];
            uint16 sw = pdo_r16(s, &bus.map.statusword);
            if ((sw & 0x0008) && !is_oe(sw)) {
               printf("Fault (0x%04x)\n", sw); g_running = 0; break;
            }
         }

         if (line[0] == 'q' || line[0] == 'Q') break;
         if (line[0] == '\0') continue;

         /* parse velocity */
         char *end; int32 vel = (int32)strtol(line, &end, 0);
         if (end == line) { printf("? Enter a number or 'q'\n"); continue; }
         if (abs(vel) > MAX_VELOCITY) { printf("Too large (max %d)\n", MAX_VELOCITY); continue; }

         /* run */
         {
            ec_slavet *s = &bus.ctx.slavelist[SERVO_SLAVE];
            if (!is_oe(pdo_r16(s, &bus.map.statusword))) {
               printf("Not operation-enabled\n"); continue;
            }
            motor_run(&bus, vel);
         }
      }
   }
   else { usage(argv[0]); ok = 0; }

done:
   if (started) bus_shutdown(&bus);
   return ok ? 0 : 1;
}
