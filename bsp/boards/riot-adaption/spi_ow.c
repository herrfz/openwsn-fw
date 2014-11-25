#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include "spi_ow.h"
#include "spi.h"
#include "leds.h"
#include "board.h"
#include "radio.h"
#include "periph/gpio.h"
#include "periph_conf.h"
#include "at86rf231.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

//=========================== defines =========================================

//=========================== variables =======================================

typedef struct {
   // information about the current transaction
   uint8_t*        pNextTxByte;
   uint8_t         numTxedBytes;
   uint8_t         txBytesLeft;
   spi_return_t    returnType;
   uint8_t*        pNextRxByte;
   uint8_t         maxRxBytes;
   spi_first_t     isFirst;
   spi_last_t      isLast;
   // state of the module
   uint8_t         busy;
#ifdef SPI_IN_INTERRUPT_MODE
   // callback when module done
   spi_cbt         callback;
#endif
} spi_vars_t;

volatile spi_vars_t spi_vars;

//=========================== prototypes ======================================
// inline static void RESET_CLR(void) { GPIOC->BRR = 1<<1; }
// inline static void RESET_SET(void) { GPIOC->BSRR = 1<<1; }
// inline static void CSn_SET(void) { GPIOA->BSRR = 1<<4; }
// inline static void CSn_CLR(void) { GPIOA->BRR = 1<<4; }
// inline static void SLEEP_CLR(void) { GPIOA->BRR = 1<<2; }
static inline void RESET_CLR(void)
{
    SPI_0_RESET_PORT->BRR = (1 << SPI_0_RESET_PIN);
}
static inline void RESET_SET(void)
{
    SPI_0_RESET_PORT->BSRR = (1 << SPI_0_RESET_PIN);
}
static inline void CSn_SET(void)
{
    SPI_0_CS_PORT->BSRR = (1 << SPI_0_CS_PIN);
}
static inline void CSn_CLR(void)
{
    SPI_0_CS_PORT->BRR = (1 << SPI_0_CS_PIN);
}
static inline void SLEEP_CLR(void)
{
    SPI_0_SLEEP_PORT->BRR = (1 << SPI_0_SLEEP_PIN);
}

//=========================== public ==========================================

void spi_init_ow(void) {
   // clear variables
    memset(&spi_vars,0,sizeof(spi_vars_t));

    /* set up GPIO pins */
    /* SCLK and MOSI*/
    GPIOA->CRL &= ~(0xf << (5 * 4));
    GPIOA->CRL |= (0xb << (5 * 4));
    GPIOA->CRL &= ~(0xf << (7 * 4));
    GPIOA->CRL |= (0xb << (7 * 4));
    /* MISO */
    gpio_init_in(SPI_0_MISO_GPIO, GPIO_NOPULL);

    /* SPI init */
    spi_init_master(SPI_0, SPI_CONF_FIRST_RISING, 4500000);

    spi_poweron(SPI_0);

    /* IRQ0 */
    gpio_init_in(SPI_0_IRQ0_GPIO, GPIO_NOPULL);
    gpio_init_int(SPI_0_IRQ0_GPIO, GPIO_NOPULL, GPIO_RISING, radio_isr);

    /* Connect EXTI4 Line to PC4 pin */
    gpio_irq_enable(SPI_0_IRQ0_GPIO);

    /* CS */
    gpio_init_out(SPI_0_CS_GPIO, GPIO_NOPULL);
    /* SLEEP */
    gpio_init_out(SPI_0_SLEEP_GPIO, GPIO_NOPULL);
    /* RESET */
    gpio_init_out(SPI_0_RESET_GPIO, GPIO_NOPULL);

    // force reset
    RESET_CLR();
    CSn_SET();
    SLEEP_CLR();

    for (uint16_t j=0;j<0xFFFF;j++); //small wait

    RESET_SET();
    // /* set up GPIO pins */
    // /* SCLK and MOSI*/
    // GPIOA->CRL &= ~(0xf << (5 * 4));
    // GPIOA->CRL |= (0xb << (5 * 4));
    // GPIOA->CRL &= ~(0xf << (7 * 4));
    // GPIOA->CRL |= (0xb << (7 * 4));
    // /* MISO */
    // gpio_init_in(SPI_0_MISO_GPIO, GPIO_NOPULL);

    // /* SPI init */
    // spi_init_master(SPI_0, SPI_CONF_FIRST_RISING, 4500000);

    // spi_poweron(SPI_0);

    // /* IRQ0 */
    // gpio_init_in(SPI_0_IRQ0_GPIO, GPIO_NOPULL);
    // gpio_init_int(SPI_0_IRQ0_GPIO, GPIO_NOPULL, GPIO_RISING, radio_isr);

    // /* Connect EXTI4 Line to PC4 pin */
    // gpio_irq_enable(SPI_0_IRQ0_GPIO);

    // /* CS */
    // gpio_init_out(SPI_0_CS_GPIO, GPIO_NOPULL);
    // /* SLEEP */
    // gpio_init_out(SPI_0_SLEEP_GPIO, GPIO_NOPULL);
    // /* RESET */
    // gpio_init_out(SPI_0_RESET_GPIO, GPIO_NOPULL);

    // /* force reset */
    // RESET_CLR();
    // CSn_SET();
    // SLEEP_CLR();

    // vtimer_usleep(AT86RF231_TIMING__RESET);

    // RESET_SET();

    // /* Wait until TRX_OFF is entered */
    // vtimer_usleep(AT86RF231_TIMING__RESET_TO_TRX_OFF);

    // /* Send a FORCE TRX OFF command */
    // at86rf231_reg_write(AT86RF231_REG__TRX_STATE, AT86RF231_TRX_STATE__FORCE_TRX_OFF);

    // /* Wait until TRX_OFF state is entered from P_ON */
    // vtimer_usleep(AT86RF231_TIMING__SLEEP_TO_TRX_OFF);

    // /* busy wait for TRX_OFF state */
    // uint8_t status;
    // uint8_t max_wait = 100;   // TODO : move elsewhere, this is in 10us

    // do {
    //     status = at86rf231_get_status();

    //     vtimer_usleep(10);

    //     if (!--max_wait) {
    //         printf("at86rf231 : ERROR : could not enter TRX_OFF mode\n");
    //         break;
    //     }
    // } while ((status & AT86RF231_TRX_STATUS_MASK__TRX_STATUS)
    //          != AT86RF231_TRX_STATUS__TRX_OFF);

}

#ifdef SPI_IN_INTERRUPT_MODE
void spi_setCallback(spi_cbt cb) {
   spi_vars.callback = cb;
}
#endif

void spi_txrx(uint8_t*     bufTx,
              uint8_t      lenbufTx,
              spi_return_t returnType,
              uint8_t*     bufRx,
              uint8_t      maxLenBufRx,
              spi_first_t  isFirst,
              spi_last_t   isLast) {

#ifdef SPI_IN_INTERRUPT_MODE
   // disable interrupts
   NVIC_RESETPRIMASK();
#endif

   // register spi frame to send
   spi_vars.pNextTxByte      =  bufTx;
   spi_vars.numTxedBytes     =  0;
   spi_vars.txBytesLeft      =  lenbufTx;
   spi_vars.returnType       =  returnType;
   spi_vars.pNextRxByte      =  bufRx;
   spi_vars.maxRxBytes       =  maxLenBufRx;
   spi_vars.isFirst          =  isFirst;
   spi_vars.isLast           =  isLast;

   // SPI is now busy
   spi_vars.busy             =  1;


   // lower CS signal to have slave listening
   if (spi_vars.isFirst==SPI_FIRST) {
        CSn_CLR();
   }

#ifdef SPI_IN_INTERRUPT_MODE
   // implementation 1. use a callback function when transaction finishes

   // write first byte to TX buffer
   SPI_I2S_SendData(SPI1,*spi_vars.pNextTxByte);

   // re-enable interrupts
   NVIC_SETPRIMASK();
#else
   // implementation 2. busy wait for each byte to be sent
   // send all bytes
   while (spi_vars.txBytesLeft>0) {
      // write next byte to TX buffer
   // SPI_I2S_SendData(SPI1,*spi_vars.pNextTxByte);
        spi_transfer_byte(SPI_0, *((char*)spi_vars.pNextTxByte), NULL);

      // busy wait on the interrupt flag
//       uint16_t c = 0;
//       while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE) == RESET)
//           ;// if (c++ == 10000) {
// //               //DEBUG("spi_txrx timeout\n");
// //               break;
// //           }

//       // clear the interrupt flag
//       SPI_I2S_ClearFlag(SPI1, SPI_I2S_FLAG_RXNE);
      // save the byte just received in the RX buffer
      switch (spi_vars.returnType) {
         case SPI_FIRSTBYTE:
            if (spi_vars.numTxedBytes==0) {
                spi_transfer_byte(SPI_0, 0, (char*)spi_vars.pNextRxByte);
               // *spi_vars.pNextRxByte   = SPI_I2S_ReceiveData(SPI1);
            }
            break;
         case SPI_BUFFER:
            spi_transfer_byte(SPI_0, 0, (char*)spi_vars.pNextRxByte);
            // *spi_vars.pNextRxByte      = SPI_I2S_ReceiveData(SPI1);
            spi_vars.pNextRxByte++;
            break;
         case SPI_LASTBYTE:
            spi_transfer_byte(SPI_0, 0, (char*)spi_vars.pNextRxByte);
            // *spi_vars.pNextRxByte      = SPI_I2S_ReceiveData(SPI1);
            break;
      }
      // one byte less to go
      spi_vars.pNextTxByte++;
      spi_vars.numTxedBytes++;
      spi_vars.txBytesLeft--;
   }

   // put CS signal high to signal end of transmission to slave
   if (spi_vars.isLast==SPI_LAST) {
        CSn_SET();
   }

   // SPI is not busy anymore
   spi_vars.busy             =  0;
#endif
}

//=========================== private =========================================

//=========================== interrupt handlers ==============================

kick_scheduler_t spi_isr(void) {
#ifdef SPI_IN_INTERRUPT_MODE
   // save the byte just received in the RX buffer
   switch (spi_vars.returnType) {
      case SPI_FIRSTBYTE:
         if (spi_vars.numTxedBytes==0) {
            *spi_vars.pNextRxByte = SPI_I2S_ReceiveData(SPI1);
         }
         break;
      case SPI_BUFFER:
         *spi_vars.pNextRxByte    = SPI_I2S_ReceiveData(SPI1);
         spi_vars.pNextRxByte++;
         break;
      case SPI_LASTBYTE:
         *spi_vars.pNextRxByte    = SPI_I2S_ReceiveData(SPI1);
         break;
   }

   // one byte less to go
   spi_vars.pNextTxByte++;
   spi_vars.numTxedBytes++;
   spi_vars.txBytesLeft--;

   if (spi_vars.txBytesLeft>0) {
      // write next byte to TX buffer
   SPI_SendData(SPI1,*spi_vars.pNextTxByte);
   } else {
      // put CS signal high to signal end of transmission to slave
      if (spi_vars.isLast==SPI_LAST) {
   GPIO_SetBits(GPIOA, GPIO_Pin_4);
      }
      // SPI is not busy anymore
      spi_vars.busy          =  0;

      // SPI is done!
      if (spi_vars.callback!=NULL) {
         // call the callback
         spi_vars.callback();
         // kick the OS
         return 1;
      }
   }
#else
   while(1);// this should never happen
   return 1;
#endif
}