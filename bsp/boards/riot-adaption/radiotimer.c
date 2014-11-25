/**
\brief openmoteSTM32 definition of the "radiotimer" bsp module.
On openmoteSTM32, we use RTC for the radiotimer module.
\author Thomas Watteyne <watteyne@eecs.berkeley.edu>, February 2012.
\author Chang Tengfei <tengfei.chang@gmail.com>,  July 2012.
*/

#include "stdint.h"

#include "periph/timer.h"

#include "leds.h"
#include "radiotimer.h"
#include "board_info.h"

#define ENABLE_DEBUG (0)
#include "debug.h"


//=========================== variables =======================================

enum  radiotimer_irqstatus_enum{
    RADIOTIMER_NONE     = 0x00, //alarm interrupt default status
    RADIOTIMER_OVERFLOW = 0x01, //alarm interrupt caused by overflow
    RADIOTIMER_COMPARE  = 0x02, //alarm interrupt caused by compare
};

typedef struct {
   radiotimer_compare_cbt    overflow_cb;
   radiotimer_compare_cbt    compare_cb;
   uint8_t                   overflowORcompare;//indicate RTC alarm interrupt status
   uint16_t                  currentSlotPeriod;
} radiotimer_vars_t;

volatile radiotimer_vars_t radiotimer_vars;
uint16_t current_period;

//=========================== prototypes ======================================

//=========================== public ==========================================

//===== admin

void radiotimer_init(void) {
   // clear local variables
   memset(&radiotimer_vars,0,sizeof(radiotimer_vars_t));
   current_period = 0;
}

void radiotimer_setOverflowCb(radiotimer_compare_cbt cb) {
   radiotimer_vars.overflow_cb    = cb;
}

void radiotimer_setCompareCb(radiotimer_compare_cbt cb) {
   radiotimer_vars.compare_cb     = cb;
}

void radiotimer_setStartFrameCb(radiotimer_capture_cbt cb) {
   while(1);
}

void radiotimer_setEndFrameCb(radiotimer_capture_cbt cb) {
   while(1);
}

void radiotimer_start(uint16_t period) {
    DEBUG("%s\n", __PRETTY_FUNCTION__);
    timer_init(TIMER_1, 1, &radiotimer_isr);
    timer_set(TIMER_1, 1, (0xffff)&((unsigned int)period));
    current_period = period;
   radiotimer_vars.currentSlotPeriod = period;
   radiotimer_vars.overflowORcompare = RADIOTIMER_OVERFLOW;
}

//===== direct access

uint16_t radiotimer_getValue(void) {
    return (uint16_t)((0xffff)&timer_read(TIMER_1));
}

void radiotimer_setPeriod(uint16_t period) {
    timer_set(TIMER_1, 1, (0xffff)&((unsigned int)period));
    current_period = period;
    radiotimer_vars.currentSlotPeriod = period;

    //set radiotimer irpstatus
    radiotimer_vars.overflowORcompare = RADIOTIMER_OVERFLOW;
}

uint16_t radiotimer_getPeriod(void) {
    return current_period;
}

//===== compare

void radiotimer_schedule(uint16_t offset) {
    timer_irq_disable(TIMER_1);
    timer_set(TIMER_1, 1, offset);
    current_period = offset;
    timer_irq_enable(TIMER_1);
    //set radiotimer irpstatus
    radiotimer_vars.overflowORcompare = RADIOTIMER_COMPARE;
}

void radiotimer_cancel(void) {
    timer_irq_disable(TIMER_1);
    timer_clear(TIMER_1, 1);
    current_period = 0;
    timer_irq_enable(TIMER_1);

    //set radiotimer irpstatus
    radiotimer_vars.overflowORcompare = RADIOTIMER_OVERFLOW;
}

//===== capture

inline uint16_t radiotimer_getCapturedTime(void) {
    return (uint16_t)((0xffff)&timer_read(TIMER_1));
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================

kick_scheduler_t radiotimer_isr(void) {
    uint8_t taiv_temp = radiotimer_vars.overflowORcompare;
    switch (taiv_temp) {
        case RADIOTIMER_COMPARE:
            DEBUG("%s cmp\n", __PRETTY_FUNCTION__);
            if (radiotimer_vars.compare_cb!=NULL) {
                radiotimer_vars.compare_cb();
                // kick the OS
                return KICK_SCHEDULER;
            }
            break;
        case RADIOTIMER_OVERFLOW: // timer overflows
            DEBUG("%s of\n", __PRETTY_FUNCTION__);
            if (radiotimer_vars.overflow_cb!=NULL) {
                //Wait until last write operation on RTC registers has finished
                timer_reset(TIMER_1);
                // call the callback
                radiotimer_vars.overflow_cb();
                DEBUG("returned...\n");
                // kick the OS
                return KICK_SCHEDULER;
            }
            break;
      case RADIOTIMER_NONE:                     // this should not happen
            DEBUG("%s none\n", __PRETTY_FUNCTION__);
      default:
            DEBUG("%s default\n", __PRETTY_FUNCTION__);
            // while(1);                               // this should not happen
    }
    return DO_NOT_KICK_SCHEDULER;
}