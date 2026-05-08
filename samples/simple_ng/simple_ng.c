/** \file
 * \brief Example code for Simple Open EtherCAT master
 *
 * Usage: simple_ng IFNAME1
 * IFNAME1 is the NIC interface name, e.g. 'eth0'
 *
 * This is a minimal test.
 */

#include "soem/soem.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FIELD_BUS_CYCLE_NS 1000000
#define FIELD_BUS_CYCLE_US (FIELD_BUS_CYCLE_NS / 1000)
#define FIELD_BUS_OP_TRIES 500

typedef struct
{
   ecx_contextt context;
   char *iface;
   uint8 group;
   int roundtrip_time;
   uint8 map[4096];
} Fieldbus;

static void
fieldbus_initialize(Fieldbus *fieldbus, char *iface)
{
   /* Let's start by 0-filling `fieldbus` to avoid surprises */
   memset(fieldbus, 0, sizeof(*fieldbus));

   fieldbus->iface = iface;
   fieldbus->group = 0;
   fieldbus->roundtrip_time = 0;
}

static int
fieldbus_roundtrip(Fieldbus *fieldbus)
{
   ecx_contextt *context;
   ec_timet start, end, diff;
   int wkc;

   context = &fieldbus->context;

   start = osal_current_time();
   ecx_send_processdata(context);
   wkc = ecx_receive_processdata(context, EC_TIMEOUTRET);
   end = osal_current_time();
   osal_time_diff(&start, &end, &diff);
   fieldbus->roundtrip_time = (int)(diff.tv_sec * 1000000 + diff.tv_nsec / 1000);

   return wkc;
}

static void
fieldbus_print_errors(ecx_contextt *context)
{
   while (context->ecaterror)
   {
      printf("%s", ecx_elist2string(context));
   }
}

static void
fieldbus_run_cycles(Fieldbus *fieldbus, int cycles)
{
   int i;

   for (i = 0; i < cycles; ++i)
   {
      fieldbus_roundtrip(fieldbus);
      osal_usleep(FIELD_BUS_CYCLE_US);
   }
}

static void
fieldbus_configure_sync_mode(ecx_contextt *context)
{
   uint16 sync0_mode;
   uint16 value;
   int size;
   int wkc;
   int i;

   sync0_mode = htoes(2); /* SM synchronization type: DC Sync0 */
   for (i = 1; i <= context->slavecount; ++i)
   {
      ec_slavet *slave = context->slavelist + i;
      if ((slave->mbx_proto & ECT_MBXPROT_COE) == 0)
      {
         continue;
      }

      wkc = ecx_SDOwrite(context, i, 0x1c32, 0x01, FALSE,
                         sizeof(sync0_mode), &sync0_mode, EC_TIMEOUTRXM);
      printf(" slave %d SM2 write=%d", i, wkc);
      fieldbus_print_errors(context);

      wkc = ecx_SDOwrite(context, i, 0x1c33, 0x01, FALSE,
                         sizeof(sync0_mode), &sync0_mode, EC_TIMEOUTRXM);
      printf(" SM3 write=%d", wkc);
      fieldbus_print_errors(context);

      size = sizeof(value);
      value = 0;
      wkc = ecx_SDOread(context, i, 0x1c32, 0x01, FALSE, &size, &value, EC_TIMEOUTRXM);
      if (wkc > 0)
      {
         printf(" SM2=%u", etohs(value));
      }
      fieldbus_print_errors(context);

      size = sizeof(value);
      value = 0;
      wkc = ecx_SDOread(context, i, 0x1c33, 0x01, FALSE, &size, &value, EC_TIMEOUTRXM);
      if (wkc > 0)
      {
         printf(" SM3=%u", etohs(value));
      }
      fieldbus_print_errors(context);
   }
}

static boolean
fieldbus_start(Fieldbus *fieldbus)
{
   ecx_contextt *context;
   ec_groupt *grp;
   ec_slavet *slave;
   int i;

   context = &fieldbus->context;
   grp = context->grouplist + fieldbus->group;

   printf("Initializing SOEM on '%s'... ", fieldbus->iface);
   if (!ecx_init(context, fieldbus->iface))
   {
      printf("no socket connection\n");
      return FALSE;
   }
   printf("done\n");

   printf("Finding autoconfig slaves... ");
   if (ecx_config_init(context) <= 0)
   {
      printf("no slaves found\n");
      return FALSE;
   }
   printf("%d slaves found\n", context->slavecount);

   context->manualstatechange = 1;

   printf("Sequential mapping of I/O... ");
   ecx_config_map_group(context, fieldbus->map, fieldbus->group);
   printf("mapped %dO+%dI bytes from %d segments",
          grp->Obytes, grp->Ibytes, grp->nsegments);
   if (grp->nsegments > 1)
   {
      /* Show how slaves are distributed */
      for (i = 0; i < grp->nsegments; ++i)
      {
         printf("%s%d", i == 0 ? " (" : "+", grp->IOsegment[i]);
      }
      printf(" slaves)");
   }
   printf("\n");

   printf("Configuring sync manager mode...");
   fieldbus_configure_sync_mode(context);
   printf(" done\n");

   printf("Configuring distributed clock... ");
   ecx_configdc(context);
   for (i = 1; i <= context->slavecount; ++i)
   {
      slave = context->slavelist + i;
      if (slave->hasdc)
      {
         ecx_dcsync0(context, i, TRUE, FIELD_BUS_CYCLE_NS, 0);
         printf(" slave %d SYNC0=%dns", i, FIELD_BUS_CYCLE_NS);
      }
   }
   printf("done\n");

   printf("Waiting for all slaves in safe operational... ");
   context->slavelist[0].state = EC_STATE_SAFE_OP;
   ecx_writestate(context, 0);
   ecx_statecheck(context, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
   if (context->slavelist[0].state == EC_STATE_SAFE_OP)
   {
      printf("done\n");
   }
   else
   {
      printf("failed,");
      ecx_readstate(context);
      for (i = 1; i <= context->slavecount; ++i)
      {
         slave = context->slavelist + i;
         if (slave->state != EC_STATE_SAFE_OP)
         {
            printf(" slave %d is 0x%04X (AL-status=0x%04X %s)",
                   i, slave->state, slave->ALstatuscode,
                   ec_ALstatuscode2string(slave->ALstatuscode));
         }
      }
      printf("\n");
      return FALSE;
   }

   printf("Run process data before operational... ");
   fieldbus_run_cycles(fieldbus, 200);
   printf("done\n");

   printf("Setting operational state..");
   /* Act on slave 0 (a virtual slave used for broadcasting) */
   slave = context->slavelist;
   slave->state = EC_STATE_OPERATIONAL;
   ecx_writestate(context, 0);
   for (i = 0; i < FIELD_BUS_OP_TRIES; ++i)
   {
      printf(".");
      fieldbus_roundtrip(fieldbus);
      ecx_statecheck(context, 0, EC_STATE_OPERATIONAL, FIELD_BUS_CYCLE_US);
      if (slave->state == EC_STATE_OPERATIONAL)
      {
         printf(" all slaves are now operational\n");
         return TRUE;
      }
      osal_usleep(FIELD_BUS_CYCLE_US);
   }

   printf(" failed,");
   ecx_readstate(context);
   for (i = 1; i <= context->slavecount; ++i)
   {
      slave = context->slavelist + i;
      if (slave->state != EC_STATE_OPERATIONAL)
      {
         printf(" slave %d is 0x%04X (AL-status=0x%04X %s)",
                i, slave->state, slave->ALstatuscode,
                ec_ALstatuscode2string(slave->ALstatuscode));
      }
   }
   printf("\n");

   return FALSE;
}

static void
fieldbus_stop(Fieldbus *fieldbus)
{
   ecx_contextt *context;
   ec_slavet *slave;

   context = &fieldbus->context;
   /* Act on slave 0 (a virtual slave used for broadcasting) */
   slave = context->slavelist;

   printf("Requesting init state on all slaves... ");
   for (uint16 i = 1; i <= context->slavecount; ++i)
   {
      if (context->slavelist[i].DCactive)
      {
         ecx_dcsync0(context, i, FALSE, 0, 0);
      }
   }
   slave->state = EC_STATE_INIT;
   ecx_writestate(context, 0);
   printf("done\n");

   printf("Close socket... ");
   ecx_close(context);
   printf("done\n");
}

static boolean
fieldbus_dump(Fieldbus *fieldbus)
{
   ecx_contextt *context;
   ec_groupt *grp;
   uint32 n;
   int wkc, expected_wkc;

   context = &fieldbus->context;
   grp = context->grouplist + fieldbus->group;

   wkc = fieldbus_roundtrip(fieldbus);
   expected_wkc = grp->outputsWKC * 2 + grp->inputsWKC;
   printf("%6d usec  WKC %d", fieldbus->roundtrip_time, wkc);
   if (wkc < expected_wkc)
   {
      printf(" wrong (expected %d)\n", expected_wkc);
      return FALSE;
   }

   printf("  O:");
   for (n = 0; n < grp->Obytes; ++n)
   {
      printf(" %02X", grp->outputs[n]);
   }
   printf("  I:");
   for (n = 0; n < grp->Ibytes; ++n)
   {
      printf(" %02X", grp->inputs[n]);
   }
   printf("  T: %lld\r", (long long)context->DCtime);
   return TRUE;
}

static void
fieldbus_check_state(Fieldbus *fieldbus)
{
   ecx_contextt *context;
   ec_groupt *grp;
   ec_slavet *slave;
   int i;

   context = &fieldbus->context;
   grp = context->grouplist + fieldbus->group;
   grp->docheckstate = FALSE;
   ecx_readstate(context);
   for (i = 1; i <= context->slavecount; ++i)
   {
      slave = context->slavelist + i;
      if (slave->group != fieldbus->group)
      {
         /* This slave is part of another group: do nothing */
      }
      else if (slave->state != EC_STATE_OPERATIONAL)
      {
         grp->docheckstate = TRUE;
         if (slave->state == EC_STATE_SAFE_OP + EC_STATE_ERROR)
         {
            printf("* Slave %d is in SAFE_OP+ERROR, attempting ACK\n", i);
            slave->state = EC_STATE_SAFE_OP + EC_STATE_ACK;
            ecx_writestate(context, i);
         }
         else if (slave->state == EC_STATE_SAFE_OP)
         {
            printf("* Slave %d is in SAFE_OP, change to OPERATIONAL\n", i);
            slave->state = EC_STATE_OPERATIONAL;
            ecx_writestate(context, i);
         }
         else if (slave->state > EC_STATE_NONE)
         {
            if (ecx_reconfig_slave(context, i, EC_TIMEOUTRET))
            {
               slave->islost = FALSE;
               printf("* Slave %d reconfigured\n", i);
            }
         }
         else if (!slave->islost)
         {
            ecx_statecheck(context, i, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
            if (slave->state == EC_STATE_NONE)
            {
               slave->islost = TRUE;
               printf("* Slave %d lost\n", i);
            }
         }
      }
      else if (slave->islost)
      {
         if (slave->state != EC_STATE_NONE)
         {
            slave->islost = FALSE;
            printf("* Slave %d found\n", i);
         }
         else if (ecx_recover_slave(context, i, EC_TIMEOUTRET))
         {
            slave->islost = FALSE;
            printf("* Slave %d recovered\n", i);
         }
      }
   }

   if (!grp->docheckstate)
   {
      printf("All slaves resumed OPERATIONAL\n");
   }
}

int main(int argc, char *argv[])
{
   Fieldbus fieldbus;

   if (argc != 2)
   {
      ec_adaptert *adapter = NULL;
      ec_adaptert *head = NULL;
      printf("Usage: simple_ng IFNAME1\n"
             "IFNAME1 is the NIC interface name, e.g. 'eth0'\n");

      printf("\nAvailable adapters:\n");
      head = adapter = ec_find_adapters();
      while (adapter != NULL)
      {
         printf("    - %s  (%s)\n", adapter->name, adapter->desc);
         adapter = adapter->next;
      }
      ec_free_adapters(head);
      return 1;
   }

   fieldbus_initialize(&fieldbus, argv[1]);
   if (fieldbus_start(&fieldbus))
   {
      int i, min_time, max_time;
      min_time = max_time = 0;
      for (i = 1; i <= 10000; ++i)
      {
         printf("Iteration %4d:", i);
         if (!fieldbus_dump(&fieldbus))
         {
            fieldbus_check_state(&fieldbus);
         }
         else if (i == 1)
         {
            min_time = max_time = fieldbus.roundtrip_time;
         }
         else if (fieldbus.roundtrip_time < min_time)
         {
            min_time = fieldbus.roundtrip_time;
         }
         else if (fieldbus.roundtrip_time > max_time)
         {
            max_time = fieldbus.roundtrip_time;
         }
         osal_usleep(5000);
      }
      printf("\nRoundtrip time (usec): min %d max %d\n", min_time, max_time);
      fieldbus_stop(&fieldbus);
   }

   return 0;
}
