#include "board_ow.h"
#include "radiotimer.h"
#include "radio.h"
#include "debugpins.h"
#include "spi_ow.h"

#define ENABLE_DEBUG (1)
#include "debug.h"

void board_init_ow(void)
{
    DEBUG("%s\n",__PRETTY_FUNCTION__);
    spi_init_ow();
    radio_init();
    DEBUG("%s\n",__PRETTY_FUNCTION__);
    radiotimer_init();
    DEBUG("%s\n",__PRETTY_FUNCTION__);
    debugpins_init();
    DEBUG("%s\n",__PRETTY_FUNCTION__);
}

void board_sleep(void)
{
}

void board_reset(void)
{
}