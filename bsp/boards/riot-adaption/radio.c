#include "board_ow.h"
#include "radio.h"
#include "board.h"
#include "at86rf231.h"
#include "at86rf231_spi.h"
#include "radiotimer.h"
#include "debugpins.h"
#include "leds.h"
#include "periph_conf.h"

#define ENABLE_DEBUG (0)
#include "debug.h"


//=========================== defines =========================================

//=========================== variables =======================================

typedef struct {
   radiotimer_capture_cbt    startFrame_cb;
   radiotimer_capture_cbt    endFrame_cb;
   radio_state_t             state;
} radio_vars_t;

radio_vars_t radio_vars;

//=========================== prototypes ======================================

// void    radio_spiWriteReg(uint8_t reg_addr, uint8_t reg_setting);
// uint8_t radio_spiReadReg(uint8_t reg_addr);
// void    radio_spiWriteTxFifo(uint8_t* bufToWrite, uint8_t lenToWrite);
// void    radio_spiReadRxFifo(uint8_t* pBufRead,
//                             uint8_t* pLenRead,
//                             uint8_t  maxBufLen,
//                             uint8_t* pLqi);
// uint8_t radio_spiReadRadioInfo(void);

//=========================== public ==========================================

//===== admin

void radio_init(void) {
   DEBUG("%s\n", __PRETTY_FUNCTION__);
   at86rf231_initialize(&at86rf231_netdev);
   at86rf231_set_monitor(1);

//    // clear variables
//    memset(&radio_vars,0,sizeof(radio_vars_t));

//    // change state
//    radio_vars.state          = RADIOSTATE_STOPPED;
//    DEBUG("%s\n",__PRETTY_FUNCTION__);
//    // configure the radio
//    radio_spiWriteReg(RG_TRX_STATE, CMD_FORCE_TRX_OFF);    // turn radio off

//    radio_spiWriteReg(RG_IRQ_MASK,
//                      (AT_IRQ_RX_START| AT_IRQ_TRX_END));  // tell radio to fire interrupt on TRX_END and RX_START
//    radio_spiReadReg(RG_IRQ_STATUS);                       // deassert the interrupt pin in case is high
//    radio_spiWriteReg(RG_ANT_DIV, RADIO_CHIP_ANTENNA);     // use chip antenna
// #define RG_TRX_CTRL_1 0x04
//    radio_spiWriteReg(RG_TRX_CTRL_1, 0x20);                // have the radio calculate CRC
//    //busy wait until radio status is TRX_OFF
//    uint16_t c = 0;
//    while((radio_spiReadReg(RG_TRX_STATUS) & 0x1F) != TRX_OFF)
//        if (c++ == 10000) {
//            DEBUG("radio_spiReadReg timeout\n");
//            break;
//        }
//    DEBUG("%s\n",__PRETTY_FUNCTION__);
//    // change state
//    radio_vars.state          = RADIOSTATE_RFOFF;
}

void radio_setOverflowCb(radiotimer_compare_cbt cb) {
   radiotimer_setOverflowCb(cb);
}

void radio_setCompareCb(radiotimer_compare_cbt cb) {
   radiotimer_setCompareCb(cb);
}

void radio_setStartFrameCb(radiotimer_capture_cbt cb) {
   radio_vars.startFrame_cb  = cb;
}

void radio_setEndFrameCb(radiotimer_capture_cbt cb) {
   radio_vars.endFrame_cb    = cb;
}

//===== reset

void radio_reset(void) {
   at86rf231_off();
   at86rf231_on();
}

//===== timer

void radio_startTimer(PORT_TIMER_WIDTH period) {
   radiotimer_start(period);
}

PORT_TIMER_WIDTH radio_getTimerValue(void) {
   return radiotimer_getValue();
}

void radio_setTimerPeriod(PORT_TIMER_WIDTH period) {
   radiotimer_setPeriod(period);
}

PORT_TIMER_WIDTH radio_getTimerPeriod(void) {
   return radiotimer_getPeriod();
}

//===== RF admin

void radio_setFrequency(uint8_t frequency) {
   // change state
   radio_vars.state = RADIOSTATE_SETTING_FREQUENCY;

   // configure the radio to the right frequecy
   at86rf231_set_channel(frequency);

   // change state
   radio_vars.state = RADIOSTATE_FREQUENCY_SET;
}

void radio_rfOn(void) {
   at86rf231_on();
}

void radio_rfOff(void) {
    // DEBUG("%s\n",__PRETTY_FUNCTION__);
//    // change state
//    radio_vars.state = RADIOSTATE_TURNING_OFF;
//    radio_spiReadReg(RG_TRX_STATUS);
//    DEBUG("step 1\n");
//    // turn radio off
//    radio_spiWriteReg(RG_TRX_STATE, CMD_FORCE_TRX_OFF);
//    DEBUG("step 2\n");
//    radio_spiWriteReg(RG_TRX_STATE, CMD_TRX_OFF);

//    // busy wait until done
//    uint16_t c = 0;
//    while((radio_spiReadReg(RG_TRX_STATUS) & 0x1F) != TRX_OFF)
//        ;// if (c++ == 100000) {
// //            DEBUG("%s: radio_spiReadReg timeout\n", __PRETTY_FUNCTION__);
// //            break;
// //        }

//    DEBUG("step 3\n");
//    // wiggle debug pin
//    debugpins_radio_clr();
   leds_radio_off();
//    DEBUG("step 4\n");
//    // change state
//    radio_vars.state = RADIOSTATE_RFOFF;
//    DEBUG("step 5\n");
}

//===== TX

void radio_loadPacket(uint8_t* packet, uint8_t len) {
   DEBUG("rf load\n");
   // change state
   radio_vars.state = RADIOSTATE_LOADING_PACKET;

   // load packet in TXFIFO
   at86rf231_write_fifo(packet, len);

   // change state
   radio_vars.state = RADIOSTATE_PACKET_LOADED;
}

void radio_txEnable(void) {
   // change state
   radio_vars.state = RADIOSTATE_ENABLING_TX;
   DEBUG("rf tx en\n");
//    // wiggle debug pin
//    debugpins_radio_set();
   leds_radio_on();

   /* Go to state PLL_ON */
    at86rf231_reg_write(AT86RF231_REG__TRX_STATE, AT86RF231_TRX_STATE__PLL_ON);

    /* wait until it is on PLL_ON state */
    do {
        int max_wait = 100;
        if (!--max_wait) {
            DEBUG("at86rf231 : ERROR : could not enter PLL_ON mode\n");
            break;
        }
    } while ((at86rf231_get_status() & AT86RF231_TRX_STATUS_MASK__TRX_STATUS)
             != AT86RF231_TRX_STATUS__PLL_ON);

    /* change into TX_ARET_ON state */
    at86rf231_reg_write(AT86RF231_REG__TRX_STATE, AT86RF231_TRX_STATE__TX_ARET_ON);

    do {
        int max_wait = 100;
        if (!--max_wait) {
            DEBUG("at86rf231 : ERROR : could not enter TX_ARET_ON mode\n");
            break;
        }
    } while (at86rf231_get_status() != AT86RF231_TRX_STATUS__TX_ARET_ON);

//    // turn on radio's PLL
//    radio_spiWriteReg(RG_TRX_STATE, CMD_PLL_ON);
//    uint16_t c = 0;
//    while((radio_spiReadReg(RG_TRX_STATUS) & 0x1F) != PLL_ON) // busy wait until done
//        ;// if (c++ == 100000) {
// //            DEBUG("%s: radio_spiReadReg timeout\n", __PRETTY_FUNCTION__);
// //            break;
// //        }

   // change state
   radio_vars.state = RADIOSTATE_TX_ENABLED;
}

void radio_txNow(void) {
   PORT_TIMER_WIDTH val;
   // change state
   radio_vars.state = RADIOSTATE_TRANSMITTING;
   leds_radio_toggle();
   // send packet by pulsing the SLP_TR_CNTL pin
   // PORT_PIN_RADIO_SLP_TR_CNTL_HIGH();
   // PORT_PIN_RADIO_SLP_TR_CNTL_LOW();

   at86rf231_transmit_tx_buf(&at86rf231_netdev);

   leds_radio_toggle();
   // The AT86RF231 does not generate an interrupt when the radio transmits the
   // SFD, which messes up the MAC state machine. The danger is that, if we leave
   // this funtion like this, any radio watchdog timer will expire.
   // Instead, we cheat an mimick a start of frame event by calling
   // ieee154e_startOfFrame from here. This also means that software can never catch
   // a radio glitch by which #radio_txEnable would not be followed by a packet being
   // transmitted (I've never seen that).
   if (radio_vars.startFrame_cb!=NULL) {
      // call the callback
      val=radiotimer_getCapturedTime();
      radio_vars.startFrame_cb(val);
   }
   DEBUG("SENT");
}

//===== RX

void radio_rxEnable(void) {
   // change state
   radio_vars.state = RADIOSTATE_ENABLING_RX;

   // put radio in reception mode
   // radio_spiWriteReg(RG_TRX_STATE, CMD_RX_ON);

   at86rf231_switch_to_rx();

//    // wiggle debug pin
//    debugpins_radio_set();
   leds_radio_on();

//    // busy wait until radio really listening
//    uint16_t c = 0;
//    while((radio_spiReadReg(RG_TRX_STATUS) & 0x1F) != RX_ON)
//        ;// if (c++ == 100000) {
// //            DEBUG("%s: radio_spiReadReg timeout\n",__PRETTY_FUNCTION__);
// //            break;
// //        }

   // change state
   radio_vars.state = RADIOSTATE_LISTENING;
}

void radio_rxNow(void) {
   // nothing to do

}

void radio_getReceivedFrame(uint8_t* pBufRead,
                            uint8_t* pLenRead,
                            uint8_t  maxBufLen,
                             int8_t* pRssi,
                            uint8_t* pLqi,
                               bool* pCrc) {
   uint8_t temp_reg_value;

   //===== crc
   temp_reg_value  = at86rf231_reg_read(AT86RF231_REG__PHY_RSSI);
   *pCrc           = (temp_reg_value & 0x80)>>7;  // msb is whether packet passed CRC
   *pRssi          = (temp_reg_value & 0x0f);

   // //===== rssi
   // // as per section 8.4.3 of the AT86RF231, the RSSI is calculate as:
   // // -91 + ED [dBm]
   // temp_reg_value  = radio_spiReadReg(RG_PHY_ED_LEVEL);
   // *pRssi          = -91 + temp_reg_value;

   // //===== packet
   // radio_spiReadRxFifo(pBufRead,
   //                     pLenRead,
   //                     maxBufLen,
   //                     pLqi);
   at86rf231_read_fifo(pLenRead, 1);
   at86rf231_read_fifo(pBufRead, *pLenRead);
   *pLqi = pBufRead[(*pLenRead)-1];
}

//=========================== private =========================================

// static inline void CSn_SET(void)
// {
//     SPI_0_CS_PORT->BSRR = (1 << SPI_0_CS_PIN);
// }
// static inline void CSn_CLR(void)
// {
//     SPI_0_CS_PORT->BRR = (1 << SPI_0_CS_PIN);
// }

// uint8_t radio_spiReadRadioInfo(void){
//    uint8_t              spi_tx_buffer[3];
//    uint8_t              spi_rx_buffer[3];
//    DEBUG("%s\n",__PRETTY_FUNCTION__);
//    // prepare buffer to send over SPI
//    spi_tx_buffer[0]     =  (0x80 | 0x1E);        // [b7]    Read/Write:    1    (read)
//    // [b6]    RAM/Register : 1    (register)
//    // [b5-0]  address:       0x1E (Manufacturer ID, Lower 16 Bit)
//    spi_tx_buffer[1]     =  0x00;                 // send a SNOP strobe just to get the reg value
//    spi_tx_buffer[2]     =  0x00;                 // send a SNOP strobe just to get the reg value

//    // retrieve radio manufacturer ID over SPI
//    // spi_txrx(spi_tx_buffer,
//    //       sizeof(spi_tx_buffer),
//    //       SPI_BUFFER,
//    //       spi_rx_buffer,
//    //       sizeof(spi_rx_buffer),
//    //       SPI_FIRST,
//    //       SPI_LAST);
//    CSn_CLR();
//    spi_transfer_bytes(SPI_0, spi_tx_buffer, spi_rx_buffer, 3);
//    CSn_SET();
//    return spi_rx_buffer[2];
// }

// void radio_spiWriteReg(uint8_t reg_addr, uint8_t reg_setting) {
//    uint8_t spi_tx_buffer[2];
//    uint8_t spi_rx_buffer[2];
//    spi_tx_buffer[0] = (0xC0 | reg_addr);        // turn addess in a 'reg write' address
//    spi_tx_buffer[1] = reg_setting;

//    // spi_txrx(spi_tx_buffer,
//    //          sizeof(spi_tx_buffer),
//    //          SPI_BUFFER,
//    //          (uint8_t*)spi_rx_buffer,
//    //          sizeof(spi_rx_buffer),
//    //          SPI_FIRST,
//    //          SPI_LAST);
//    CSn_CLR();
//    spi_transfer_byte(SPI_0, spi_tx_buffer[0], NULL);
//    spi_transfer_byte(SPI_0, spi_tx_buffer[1], NULL);
//    CSn_SET();
// }

// uint8_t radio_spiReadReg(uint8_t reg_addr) {
//    uint8_t spi_tx_buffer[2];
//    uint8_t spi_rx_buffer[2];
//    spi_tx_buffer[0] = (0x80 | reg_addr);        // turn addess in a 'reg read' address
//    spi_tx_buffer[1] = 0x00;                     // send a no_operation command just to get the reg value

//    // spi_txrx(spi_tx_buffer,
//    //          sizeof(spi_tx_buffer),
//    //          SPI_BUFFER,
//    //          (uint8_t*)spi_rx_buffer,
//    //          sizeof(spi_rx_buffer),
//    //          SPI_FIRST,
//    //          SPI_LAST);
//    CSn_CLR();
//    spi_transfer_byte(SPI_0, spi_tx_buffer[0], NULL);
//    spi_transfer_byte(SPI_0, 0, spi_rx_buffer);
//    CSn_SET();

//   return spi_rx_buffer[0];
// }

// /** for testing purposes, remove if not needed anymore**/

// void radio_spiWriteTxFifo(uint8_t* bufToWrite, uint8_t  lenToWrite) {
//    uint8_t spi_tx_buffer[2];
//    uint8_t spi_rx_buffer[1+1+127];               // 1B SPI address, 1B length, max. 127B data
//    DEBUG("%s\n",__PRETTY_FUNCTION__);
//    spi_tx_buffer[0] = 0x60;                      // SPI destination address for TXFIFO
//    spi_tx_buffer[1] = lenToWrite;                // length byte

//    CSn_CLR();
//    spi_transfer_byte(SPI_0, spi_tx_buffer[0], NULL);
//    spi_transfer_byte(SPI_0, spi_tx_buffer[1], NULL);
//    spi_transfer_bytes(SPI_0, bufToWrite, NULL, lenToWrite);
//    CSn_SET();

//    // spi_txrx(spi_tx_buffer,
//    //          sizeof(spi_tx_buffer),
//    //          SPI_BUFFER,
//    //          spi_rx_buffer,
//    //          sizeof(spi_rx_buffer),
//    //          SPI_FIRST,
//    //          SPI_NOTLAST);

//    // spi_txrx(bufToWrite,
//    //          lenToWrite,
//    //          SPI_BUFFER,
//    //          spi_rx_buffer,
//    //          sizeof(spi_rx_buffer),
//    //          SPI_NOTFIRST,
//    //          SPI_LAST);
// }



// void radio_spiReadRxFifo(uint8_t* pBufRead,
//                          uint8_t* pLenRead,
//                          uint8_t  maxBufLen,
//                          uint8_t* pLqi) {
//    // when reading the packet over SPI from the RX buffer, you get the following:
//    // - *[1B]     dummy byte because of SPI
//    // - *[1B]     length byte
//    // -  [0-125B] packet (excluding CRC)
//    // - *[2B]     CRC
//    // - *[1B]     LQI
//    uint8_t spi_tx_buffer[125];
//    uint8_t spi_rx_buffer[3];
//    DEBUG("%s\n",__PRETTY_FUNCTION__);
//    spi_tx_buffer[0] = 0x20;

//    CSn_CLR();
//    spi_transfer_byte(SPI_0, spi_tx_buffer[0], NULL);
//    spi_transfer_byte(SPI_0, 0, spi_rx_buffer);
//    // 2 first bytes
//    // spi_txrx(spi_tx_buffer,
//    //          2,
//    //          SPI_BUFFER,
//    //          spi_rx_buffer,
//    //          sizeof(spi_rx_buffer),
//    //          SPI_FIRST,
//    //          SPI_NOTLAST);

//    *pLenRead  = spi_rx_buffer[0];

//    if (*pLenRead>2 && *pLenRead<=127) {
//       // valid length
//       spi_transfer_byte(SPI_0, spi_tx_buffer[0], NULL);
//       spi_transfer_bytes(SPI_0, NULL, pBufRead, *pLenRead);

//       // //read packet
//       // spi_txrx(spi_tx_buffer,
//       //          *pLenRead,
//       //          SPI_BUFFER,
//       //          pBufRead,
//       //          125,
//       //          SPI_NOTFIRST,
//       //          SPI_NOTLAST);
//       spi_transfer_byte(SPI_0, 0, 0);
//       spi_transfer_byte(SPI_0, 0, 0);
//       spi_transfer_byte(SPI_0, 0, pLqi);
//       // CRC (2B) and LQI (1B)
//       // spi_txrx(spi_tx_buffer,
//       //          2+1,
//       //          SPI_BUFFER,
//       //          spi_rx_buffer,
//       //          3,
//       //          SPI_NOTFIRST,
//       //          SPI_LAST);

//       // *pLqi   = spi_rx_buffer[2];

//    } else {
//       // invalid length
//       spi_transfer_byte(SPI_0, 0, 0);
//       // read a just byte to close spi
//       // spi_txrx(spi_tx_buffer,
//       //          1,
//       //          SPI_BUFFER,
//       //          spi_rx_buffer,
//       //          sizeof(spi_rx_buffer),
//       //          SPI_NOTFIRST,
//       //          SPI_LAST);
//    }
//    CSn_SET();
// }

//=========================== callbacks =======================================

//=========================== interrupt handlers ==============================

void at86rf231_rx_irq(void) {
   PORT_TIMER_WIDTH capturedTime;
   uint8_t  irq_status;

   // capture the time
   capturedTime = radiotimer_getCapturedTime();

   // reading IRQ_STATUS causes radio's IRQ pin to go low
   irq_status = at86rf231_reg_read(AT86RF231_REG__IRQ_STATUS);

   // start of frame event
   if (irq_status & AT86RF231_IRQ_STATUS_MASK__RX_START) {
       DEBUG("Start of frame.\n");
      // change state
      radio_vars.state = RADIOSTATE_RECEIVING;
      if (radio_vars.startFrame_cb!=NULL) {
         // call the callback
         radio_vars.startFrame_cb(capturedTime);
         // kick the OS
         return;
      } else {
         while(1);
      }
   }
   // end of frame event
   if (irq_status & AT86RF231_IRQ_STATUS_MASK__TRX_END) {
       DEBUG("End of Frame.\n");
      // change state
      radio_vars.state = RADIOSTATE_TXRX_DONE;
      if (radio_vars.endFrame_cb!=NULL) {
         // call the callback
         radio_vars.endFrame_cb(capturedTime);
         // kick the OS
         return;
      } else {
         while(1);
      }
   }
}