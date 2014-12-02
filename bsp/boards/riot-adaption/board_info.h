/**
\brief RIOT adaption information bsp module.

This module simply defines some strings describing the board, which CoAP uses
to return the board's description.

\author Thomas Eichinger <thomas.eichinger@fu-berlin.de>
*/

#ifndef __BOARD_INFO_H
#define __BOARD_INFO_H

#include "stdint.h"
#include "string.h"
#include "cpu.h"

//=========================== defines =========================================

#define INTERRUPT_DECLARATION(); //no declaration

#define DISABLE_INTERRUPTS()    disableIRQ();
#define ENABLE_INTERRUPTS()     enableIRQ();

//===== timer

#define PORT_TIMER_WIDTH                    uint32_t
#define PORT_RADIOTIMER_WIDTH               uint32_t

#define PORT_SIGNED_INT_WIDTH               int32_t
#define PORT_TICS_PER_MS                    1000
#define SCHEDULER_WAKEUP()
#define SCHEDULER_ENABLE_INTERRUPT()

//===== IEEE802154E timing
// time-slot related
#define PORT_TsSlotDuration                 15000

#define PORT_maxTxDataPrepare               2014
#define PORT_maxRxAckPrepare                900
#define PORT_maxRxDataPrepare               976
#define PORT_maxTxAckPrepare                900
// radio speed related
#define PORT_delayTx                        549
#define PORT_delayRx                        0

//===== adaptive_sync accuracy

#define SYNC_ACCURACY                       2     // ticks

//=========================== typedef  ========================================

//=========================== variables =======================================

static const uint8_t rreg_uriquery[]        = "h=ucb";
static const uint8_t infoBoardname[]        = "riot-os";
static const uint8_t infouCName[]           = "various";
static const uint8_t infoRadioName[]        = "various";

//=========================== prototypes ======================================

//=========================== public ==========================================

//=========================== private =========================================

#endif
