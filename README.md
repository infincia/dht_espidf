# dht_espidf

An [ESP-IDF](https://github.com/espressif/esp-idf/) component for the DHT11/22 humidity and 
temperature sensor family.

Derived from the [NodeMCU firmware driver](https://github.com/nodemcu/nodemcu-firmware) for DHT11/22 sensors.

### Installation

#### ESP-IDF build system

Clone this repository into your `components` directory inside the project:

    cd components
    git clone https://github.com/infincia/dht_espidf.git


#### PlatformIO

Add this repository as a library in your `platformio.ini` file in the root of 
your project:

    [env:development]
    platform = espressif32
    board = esp32dev
    framework = espidf
    lib_deps =
      https://github.com/infincia/dht_espidf.git#v0.1.0

### Full example of usage

This is a bare minimum `main.cpp` file you can refer to when using this library. 

    #include <esp_err.h>
    #include <esp_log.h>
    static const char *TAG = "[MyProject]";

    #include <dht_espidf.h>

    #define DHT_IO 27


    extern "C" {
    void app_main();
    }

    int app_main() {

        struct dht_reading dht_data{};

        dht_result_t res = read_dht_sensor_data((gpio_num_t)DHT_IO, DHT11, &dht_data);

        if (res != DHT_OK) {
            ESP_LOGW(TAG, "DHT sensor reading failed");
        } else {
            double fahrenheit = (dht_data.temperature * 1.8f) + 32.0f;
            double humidity = dht_data.humidity;
            ESP_LOGI(TAG, "DHT sensor reading: %f° / %f", fahrenheit, humidity);
        }

        return 0;
    }


