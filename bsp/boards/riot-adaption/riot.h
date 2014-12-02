/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @{
 *
 * @file
 * @brief           RIOT specific definitions for OpenWSN
 *
 * @author          Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 */

#ifndef __RIOT_H
#define __RIOT_H

#include "thread.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief The peripheral timer to use with OpenWSN
 */
#define OWSN_TIMER  TIMER_1

#ifdef __cplusplus
}
#endif
#endif /* __RIOT_H */
/** @} */