/**
\brief This project runs the full OpenWSN stack.

\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, August 2010
*/

#include "thread.h"

#include "board_ow.h"
#include "leds.h"
#include "crypto_engine.h"
#include "scheduler.h"
#include "openstack.h"
#include "opendefs.h"

#include "03oos_openwsn.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

static char openwsn_stack[KERNEL_CONF_STACKSIZE_MAIN];
int openwsn_pid = -1;

void openwsn_init(void);
void* openwsn_start(void *arg);

void openwsn_start_thread(void) {
    DEBUG("%s\n",__PRETTY_FUNCTION__);
    openwsn_pid = thread_create(openwsn_stack, KERNEL_CONF_STACKSIZE_MAIN,
                                PRIORITY_OPENWSN-2, CREATE_STACKTEST,
                                openwsn_start, NULL, "openwsn thread");
}

void* openwsn_start(void *arg) {
    DEBUG("%s\n",__PRETTY_FUNCTION__);
    (void)arg;
    leds_all_off();
    board_init_ow();
    scheduler_init();
    openstack_init();
    puts("OpenWSN thread started.");
    scheduler_start();
    return NULL;
}


int mote_main(void) {
   
   // initialize
   board_init();
   CRYPTO_ENGINE.init();
   scheduler_init();
   openstack_init();
   
   // indicate
   
   // start
   scheduler_start();
   return 0; // this line should never be reached
}
