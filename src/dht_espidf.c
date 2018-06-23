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


#include "dht_espidf.h"

#include "driver/gpio.h"
#include "driver/rmt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"


#undef DHT_DEBUG

// RX idle threshold [us]
// needs to be larger than any duration occurring during bit slots
// datasheet specs up to 200us for "Bus master has released time"
#define DHT_DURATION_RX_IDLE (250)

// zero bit duration threshold [us]
// a high phase
// * shorter than this is detected as a zero bit
// * longer than this is detected as a one bit
#define DHT_DURATION_ZERO (50)


#define DHT11_WAKEUP_MS 22
#define DHT2X_WAKEUP_MS 2

// grouped information for RMT management
static struct {
    int channel;
    RingbufHandle_t rb;
    gpio_num_t gpio;
} dht_rmt = {-1, NULL, -1};


static void dht_deinit(void) {
    // drive idle level 1
    gpio_set_level(dht_rmt.gpio, 1);

    rmt_rx_stop(dht_rmt.channel);
    rmt_driver_uninstall(dht_rmt.channel);

    // invalidate channel and gpio assignments
    dht_rmt.channel = -1;
    dht_rmt.gpio = -1;
}


static esp_err_t dht_init(gpio_num_t gpio_num) {
    dht_rmt.channel = RMT_CHANNEL_1;
    dht_rmt.gpio = gpio_num;

#ifdef DHT_DEBUG
    ESP_LOGI("dht", "RMT RX channel: %d", dht_rmt.channel);
#endif

    rmt_config_t rmt_rx;
    rmt_rx.channel = dht_rmt.channel;
    rmt_rx.gpio_num = dht_rmt.gpio;
    rmt_rx.clk_div = 80; // base period is 1us
    rmt_rx.mem_block_num = 1;
    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.rx_config.filter_en = true;
    rmt_rx.rx_config.filter_ticks_thresh = 30;
    rmt_rx.rx_config.idle_threshold = DHT_DURATION_RX_IDLE;

    ESP_ERROR_CHECK(rmt_config(&rmt_rx));
    ESP_ERROR_CHECK(rmt_driver_install(
        rmt_rx.channel, 512, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_SHARED));


    rmt_get_ringbuf_handle(dht_rmt.channel, &dht_rmt.rb);


    // use gpio for TX direction
    // drive idle level 1
    gpio_set_level(dht_rmt.gpio, 1);
    gpio_pullup_dis(dht_rmt.gpio);
    gpio_pulldown_dis(dht_rmt.gpio);
    gpio_set_direction(dht_rmt.gpio, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_intr_type(dht_rmt.gpio, GPIO_INTR_DISABLE);

    return ESP_OK;
}


static esp_err_t _dht_read(gpio_num_t gpio_num, uint8_t wakeup_ms, uint8_t *data) {
    if (dht_init(gpio_num) != ESP_OK) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    // send start signal and arm RX channel
    TickType_t xDelay = pdMS_TO_TICKS(wakeup_ms) > 0 ? pdMS_TO_TICKS(wakeup_ms) : 1;
    gpio_set_level(gpio_num, 0); // pull wire low
    vTaskDelay(xDelay); // time low phase
    rmt_rx_start(dht_rmt.channel, true); // arm RX channel
    gpio_set_level(gpio_num, 1); // release wire

    // wait for incoming bit stream
    size_t rx_size;
    rmt_item32_t *rx_items = (rmt_item32_t *)xRingbufferReceive(dht_rmt.rb, &rx_size, pdMS_TO_TICKS(100));

    // default is "no error"
    // error conditions have to overwrite this with ESP_ERR
    int res = ESP_OK;

    if (rx_items) {

#ifdef DHT_DEBUG
        ESP_LOGI("dht", "rx_items received: %d", rx_size);
        for (size_t i = 0; i < rx_size / 4; i++) {
            ESP_LOGI("dht", "level: %d, duration %d", rx_items[i].level0, rx_items[i].duration0);
            ESP_LOGI("dht", "level: %d, duration %d", rx_items[i].level1, rx_items[i].duration1);
        }
#endif

        // we expect 40 bits of payload plus a response bit and two edges for start and stop signals
        // each bit on the wire consumes 2 rmt samples (and 2 rmt samples stretch over 4 bytes)
        if (rx_size >= (5 * 8 + 1 + 1) * 4) {

            // check framing
            if (rx_items[0].level0 == 1 && // rising edge of the start signal
                rx_items[0].level1 == 0 && rx_items[1].level0 == 1 && // response signal
                rx_items[41].level1 == 0) { // falling edge of stop signal

                // run through the bytes
                for (size_t byte = 0; byte < 5 && res == ESP_OK; byte++) {
                    size_t bit_pos = 1 + byte * 8;
                    data[byte] = 0;

                    // decode the bits inside a byte
                    for (size_t bit = 0; bit < 8; bit++, bit_pos++) {
                        if (rx_items[bit_pos].level1 != 0) {
                            // not a falling edge, terminate decoding
                            res = ESP_ERR_INVALID_RESPONSE;
                            break;
                        }
                        // ignore duration of low level

                        // data is sent MSB first
                        data[byte] <<= 1;
                        if (rx_items[bit_pos + 1].level0 == 1 &&
                            rx_items[bit_pos + 1].duration0 > DHT_DURATION_ZERO)
                            data[byte] |= 1;
                    }

#ifdef DHT_DEBUG
                    ESP_LOGI("dht", "data 0x%02x", data[byte]);
#endif
                }

                // all done

            } else {
                // framing mismatch on start, response, or stop signals
                res = ESP_ERR_INVALID_RESPONSE;
            }

        } else {
            // too few bits received
            res = ESP_ERR_INVALID_RESPONSE;
        }

        vRingbufferReturnItem(dht_rmt.rb, (void *)rx_items);
    } else {
        // time out occurred, this indicates an unconnected / misconfigured bus
        res = ESP_ERR_TIMEOUT;
    }

    dht_deinit();

    return res;
}


static dht_result_t dht_compute_data11(uint8_t *data, double *temp, double *humi) {
    *humi = data[0];
    *temp = data[2];

    uint8_t sum = data[0] + data[1] + data[2] + data[3];
    return sum == data[4] ? DHT_OK : DHT_ERROR_CHECKSUM;
}

static dht_result_t dht_compute_data2x(uint8_t *data, double *temp, double *humi) {
    *humi = (data[0] * 256 + data[1]) * 0.1;
    *temp = ((data[2] & 0x7f) * 256 + data[3]) * 0.1;

    if (data[2] & 0x80)
        *temp = -*temp;

    uint8_t sum = data[0] + data[1] + data[2] + data[3];
    return sum == data[4] ? DHT_OK : DHT_ERROR_CHECKSUM;
}

dht_result_t read_dht_sensor_data(const gpio_num_t gpio_num, dht_type_t type, struct dht_reading *sensor_data) {
    uint8_t data[5];
    esp_err_t err;

    err = _dht_read(gpio_num, type == DHT11 ? DHT11_WAKEUP_MS : DHT2X_WAKEUP_MS, data);

    switch (err) {
        case ESP_ERR_INVALID_RESPONSE:
            return DHT_INVALID_VALUE;
        case ESP_ERR_TIMEOUT:
            return DHT_ERROR_TIMEOUT;
        default:
            break;
    }

    double temperature, humidity;
    int res;
    switch (type) {
        case DHT11:
            res = dht_compute_data11(data, &temperature, &humidity);
            break;
        case DHT2X:
            res = dht_compute_data2x(data, &temperature, &humidity);
            break;
        default:
            res = DHT_INVALID_VALUE;
            temperature = humidity = 0;
            break;
    }

    sensor_data->temperature = temperature;
    sensor_data->humidity = humidity;

    return res;
}
