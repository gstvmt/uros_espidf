#include <Arduino.h>
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO    22    // GPIO number for I2C master clock
#define I2C_MASTER_SDA_IO    21    // GPIO number for I2C master data
#define I2C_MASTER_NUM       I2C_NUM_0 // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ   100000    // I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE   0   // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0   // I2C master doesn't need buffer
#define HMC6343_SENSOR_ADDR  0x19 // Slave address of the HMC6343 sensor

static const char *TAG = "HMC6343";

static void i2c_master_init()
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static void i2c_scan()
{
    Serial.println("Scanning I2C bus...");
    for (uint8_t i = 1; i < 127; i++)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK)
        {
            Serial.print("Found I2C device at address 0x");
            Serial.println(i, HEX);
        }
    }
    Serial.println("I2C scan complete.");
}

static esp_err_t hmc6343_write_register(uint8_t register_address, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (HMC6343_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, register_address, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t hmc6343_read_register(uint8_t register_address, uint8_t *data, size_t length)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (HMC6343_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, register_address, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (HMC6343_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    if (length > 1) {
        i2c_master_read(cmd, data, length - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + length - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void setup()
{
    Serial.begin(115200);
    i2c_master_init();
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Scan for I2C devices
    i2c_scan();
}

int16_t combineBytes(uint8_t msb, uint8_t lsb) {
    return (int16_t)((msb << 8) | lsb);
}

void loop()
{
    uint8_t heading_data[6]; // Buffer to hold the heading, pitch, and roll data
    esp_err_t ret = hmc6343_read_register(0x50, heading_data, 6);
    if (ret == ESP_OK)
    {
        int16_t heading_raw = combineBytes(heading_data[0], heading_data[1]);
        int16_t pitch_raw = combineBytes(heading_data[2], heading_data[3]);
        int16_t roll_raw = combineBytes(heading_data[4], heading_data[5]);

        float heading = heading_raw / 10.0; // Convert to degrees
        float pitch = pitch_raw / 10.0; // Convert to degrees
        float roll = roll_raw / 10.0; // Convert to degrees

        Serial.print("Heading: ");
        Serial.print(heading);
        Serial.print(" degrees, Pitch: ");
        Serial.print(pitch);
        Serial.print(" degrees, Roll: ");
        Serial.println(roll);
        Serial.println(" degrees");
    }
    else
    {
        Serial.println("Failed to read from HMC6343 sensor");
        ESP_LOGE(TAG, "Error: %s", esp_err_to_name(ret));
    }

    delay(2000); // Wait for 2 seconds
}
