#ifndef CONFIG_H
#define CONFIG_H

/**
 * @brief Положение, указывающее на полностью открытое состояние шторы.
 */
const int position_open = 100;

/**
 * @brief Положение, указывающее на полностью закрытое состояние шторы.
 */
const int position_closed = 0;

#include "config/pins.h"
#include "config/mqtt_topics.h"
#include "config/memory.h"

#define DEBUG_SERIAL_OUT

#endif