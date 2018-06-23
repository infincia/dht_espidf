/* ****************************************************************************
 *
 * ESP32 platform interface for DHT temperature & humidity sensors
 *
 * Copyright (c) 2017, Arnim Laeuger
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * ****************************************************************************/


#ifndef __DHT_H
#define __DHT_H

// ---------------------------------------------------------------------------
// INCLUDES

#include <stdint.h>

#include "sdkconfig.h"

#include <esp_err.h>
#include <driver/gpio.h>

/*** 
 * 
 * Types
 **/

struct dht_reading {
    double humidity;
    double temperature;
};

typedef enum {
  DHT_OK = 0,
  DHT_ERROR_CHECKSUM = -1,
  DHT_ERROR_TIMEOUT = -2,
  DHT_INVALID_VALUE = -999
} dht_result_t;

typedef enum {
  DHT11,
  DHT2X
} dht_type_t;



// ---------------------------------------------------------------------------
// FUNCTION PROTOTYPES

//
// external

dht_result_t read_dht_sensor_data(const gpio_num_t gpio_num, dht_type_t type, struct dht_reading *sensor_data);

#endif
