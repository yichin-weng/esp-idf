/* main.c - Application main entry point */

/*
 * Copyright (c) 2017 Intel Corporation
 * Additional Copyright (c) 2018 Espressif Systems (Shanghai) PTE LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * I2C protocol api procedure.
 *  Procedure:
 *      i2c_driver_install() -> i2c_cmd_link_create() -> i2c_master_start() -> i2c_master_write()
 * -> i2c_master_cmd_begin() ->i2c_cmd_link_delete()
 *
 * i2c_driver_install : ensure/enable the i2c interface is useful for esp32.
 *
 *
 * (Optional: if there is any problem with i2c, check i2c_driver_install FLAG:ESP_INTR_FLAG_IRAM)
 *
 */

/*
 * CO2 model S-300E (S-300) . If we read the data from CO2 model right after power on, the data will be default value
 * 500ppm. If the data sometimes shows 500ppm, it means there may be some problems with power.
 */

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_sensor_model_api.h"

#include "ble_mesh_example_init.h"
#include "board.h"
#include "driver/i2c.h"

#define TAG "EXAMPLE"

#define CID_ESP     0x02E5
#define BME280_ADDR 0x76
#define S300_ADDR   0x31

/*
 Macro of operation code of CO2 sensor
 */

#define SLEEPING_MODE   0x53
#define WAKE_UP_MODE    0x57
#define READ_DATA       0x52
#define MCDL_MODE       0x4D  // start manual calibration
#define ACDL_MODE       0x41  // start automatic calibration
#define MCDL_END        0x45  // end of calibration
#define CLEAR           0x43  // clear calibration data
#define UART_CHANGE     0x55  // change uart data rate
#define TARGET_PPM      0x54  // recalibration by target PPM

/*
 Macro for I2C setting:
 */

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ    /*!< I2C master read */
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */

/* Sensor Property ID */
#define SENSOR_PROPERTY_ID_0        0x0056  /* Present Indoor Ambient Temperature */
#define SENSOR_PROPERTY_ID_1        0x005B  /* Present Outdoor Ambient Temperature */
#define SENSOR_PROPERTY_ID_2        0x0060  /* Present Indoor CO2 concentration */
#define SENSOR_PROPERTY_ID_3        0x0061  /* Present Indoor O2 concentration */
#define SENSOR_PROPERTY_ID_4        0x0062  /* Present Inddor humidity */

unsigned long int hum_raw,temp_raw,pres_raw;
signed long int t_fine;

#define SENSOR_POSITIVE_TOLERANCE   ESP_BLE_MESH_SENSOR_UNSPECIFIED_POS_TOLERANCE
#define SENSOR_NEGATIVE_TOLERANCE   ESP_BLE_MESH_SENSOR_UNSPECIFIED_NEG_TOLERANCE
#define SENSOR_SAMPLE_FUNCTION      ESP_BLE_MESH_SAMPLE_FUNC_UNSPECIFIED
#define SENSOR_MEASURE_PERIOD       ESP_BLE_MESH_SENSOR_NOT_APPL_MEASURE_PERIOD
#define SENSOR_UPDATE_INTERVAL      ESP_BLE_MESH_SENSOR_NOT_APPL_UPDATE_INTERVAL

static uint8_t dev_uuid[ESP_BLE_MESH_OCTET16_LEN] = { 0x32, 0x10 };

/*
 * calibration parameter for temperature , humidity, pressure
 */
static uint16_t dig_T1;
static int16_t dig_T2;
static int16_t dig_T3;
static uint16_t dig_P1;
static int16_t dig_P2;
static int16_t dig_P3;
static int16_t dig_P4;
static int16_t dig_P5;
static int16_t dig_P6;
static int16_t dig_P7;
static int16_t dig_P8;
static int16_t dig_P9;
static int8_t  dig_H1;
static int16_t dig_H2;
static int8_t  dig_H3;
static int16_t dig_H4;
static int16_t dig_H5;
static int8_t  dig_H6;

/*
 * Final data for temperature , pressure , humidity
 */

static union {
    uint8_t co2_data[2];
    uint16_t co2_act;
} my_co2;

static union {
    double  temp_act;
    uint8_t temp_data[8];
} my_temp;
static union {
    double  press_act;
    uint8_t press_data[8];
} my_press;
static union {
    double  hum_act;
    uint8_t hum_data[8];
} my_hum;

/*
 * By doing the idf.py menuconfig, we enable the Macro
 */
static esp_ble_mesh_cfg_srv_t config_server = {
    .relay = ESP_BLE_MESH_RELAY_ENABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

NET_BUF_SIMPLE_DEFINE_STATIC(sensor_data_0, 8);
NET_BUF_SIMPLE_DEFINE_STATIC(sensor_data_1, 8);
NET_BUF_SIMPLE_DEFINE_STATIC(sensor_data_2, 8);
NET_BUF_SIMPLE_DEFINE_STATIC(sensor_data_3, 2);


static esp_ble_mesh_sensor_state_t sensor_states[4] = {
    /* Mesh Model Spec:
     * Multiple instances of the Sensor states may be present within the same model,
     * provided that each instance has a unique value of the Sensor Property ID to
     * allow the instances to be differentiated. Such sensors are known as multisensors.
     * In this example, two instances of the Sensor states within the same model are
     * provided.
     */
    [0] = {
        /* Mesh Model Spec:
         * Sensor Property ID is a 2-octet value referencing a device property
         * that describes the meaning and format of data reported by a sensor.
         * 0x0000 is prohibited.
         */
        .sensor_property_id = SENSOR_PROPERTY_ID_0,
        /* Mesh Model Spec:
         * Sensor Descriptor state represents the attributes describing the sensor
         * data. This state does not change throughout the lifetime of an element.
         */
        .descriptor.positive_tolerance = SENSOR_POSITIVE_TOLERANCE,
        .descriptor.negative_tolerance = SENSOR_NEGATIVE_TOLERANCE,
        .descriptor.sampling_function = SENSOR_SAMPLE_FUNCTION,
        .descriptor.measure_period = SENSOR_MEASURE_PERIOD,
        .descriptor.update_interval = SENSOR_UPDATE_INTERVAL,
        .sensor_data.format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
        .sensor_data.length = 7, /* 0 represents the length is 1 */
        .sensor_data.raw_value = &sensor_data_0,
    },
    [1] = {
        .sensor_property_id = SENSOR_PROPERTY_ID_1,
        .descriptor.positive_tolerance = SENSOR_POSITIVE_TOLERANCE,
        .descriptor.negative_tolerance = SENSOR_NEGATIVE_TOLERANCE,
        .descriptor.sampling_function = SENSOR_SAMPLE_FUNCTION,
        .descriptor.measure_period = SENSOR_MEASURE_PERIOD,
        .descriptor.update_interval = SENSOR_UPDATE_INTERVAL,
        .sensor_data.format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
        .sensor_data.length = 7, /* 0 represents the length is 1 */
        .sensor_data.raw_value = &sensor_data_1,
    },
    [2] = {
        .sensor_property_id = SENSOR_PROPERTY_ID_2,
        .descriptor.positive_tolerance = SENSOR_POSITIVE_TOLERANCE,
        .descriptor.negative_tolerance = SENSOR_NEGATIVE_TOLERANCE,
        .descriptor.sampling_function = SENSOR_SAMPLE_FUNCTION,
        .descriptor.measure_period = SENSOR_MEASURE_PERIOD,
        .descriptor.update_interval = SENSOR_UPDATE_INTERVAL,
        .sensor_data.format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
        .sensor_data.length = 7, /* 0 represents the length is 1 */
        .sensor_data.raw_value = &sensor_data_2,
    },
    [3] = {
        .sensor_property_id = SENSOR_PROPERTY_ID_2,
        .descriptor.positive_tolerance = SENSOR_POSITIVE_TOLERANCE,
        .descriptor.negative_tolerance = SENSOR_NEGATIVE_TOLERANCE,
        .descriptor.sampling_function = SENSOR_SAMPLE_FUNCTION,
        .descriptor.measure_period = SENSOR_MEASURE_PERIOD,
        .descriptor.update_interval = SENSOR_UPDATE_INTERVAL,
        .sensor_data.format = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A,
        .sensor_data.length = 2, /* 0 represents the length is 1 */
        .sensor_data.raw_value = &sensor_data_3,
    },
};

/* 20 octets is large enough to hold two Sensor Descriptor state values. */
ESP_BLE_MESH_MODEL_PUB_DEFINE(sensor_pub, 20, ROLE_NODE);
static esp_ble_mesh_sensor_srv_t sensor_server = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    .state_count = ARRAY_SIZE(sensor_states),
    .states = sensor_states,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(sensor_setup_pub, 20, ROLE_NODE);
static esp_ble_mesh_sensor_setup_srv_t sensor_setup_server = {
    .rsp_ctrl.get_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    .rsp_ctrl.set_auto_rsp = ESP_BLE_MESH_SERVER_RSP_BY_APP,
    .state_count = ARRAY_SIZE(sensor_states),
    .states = sensor_states,
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_SENSOR_SRV(&sensor_pub, &sensor_server),
    ESP_BLE_MESH_MODEL_SENSOR_SETUP_SRV(&sensor_setup_pub, &sensor_setup_server),
};

/*
 * this structure describe the elements we can edit in mesh interface.
 */
static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
};

/*
 * initialize i2c master mode with default parameters
 * sda_io_num = 18 (pin number)
 * scl_io_num = 19 (pin number)
 * i2c_frequency = 100,000hz
 */

static gpio_num_t i2c_gpio_sda = 21;
static gpio_num_t i2c_gpio_scl = 22;
static uint32_t i2c_frequency = 100000;
static i2c_port_t i2c_port = I2C_NUM_0;

/*
 * i2c_master_driver_initialize()
 * this function is used for initialize the i2c.
 */

static esp_err_t i2c_master_driver_initialize(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c_gpio_sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = i2c_gpio_scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = i2c_frequency
    };
    return i2c_param_config(i2c_port, &conf);
}

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .elements = elements,
    .element_count = ARRAY_SIZE(elements),
};

static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
};

static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "net_idx 0x%03x, addr 0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags 0x%02x, iv_index 0x%08x", flags, iv_index);
    board_led_operation(LED_G, LED_OFF);
}

static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
            param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
            param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
            param->node_prov_complete.flags, param->node_prov_complete.iv_index);
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
        break;
    default:
        break;
    }
}

/*
 *  @brief
 */
static void i2c_send_to_S300(uint8_t* data, size_t data_len, bool read_bit) {
    uint8_t index = 0;
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (S300_ADDR << 1) | read_bit, I2C_MASTER_ACK);  // depend on read_bit
    if (read_bit) {
        while ( index < data_len ) {
            if (index == data_len - 1) {
                i2c_master_read_byte(cmd_handle, data + index -1 , I2C_MASTER_LAST_NACK);  // The first byte should be data_addr
            }
            else {
                i2c_master_read_byte(cmd_handle, data + index -1 , I2C_MASTER_LAST_ACK);
                index++;
            }
        }
    } else {
        i2c_master_write(cmd_handle, data, data_len - 1, I2C_MASTER_ACK); // The first byte should be data_addr
    }
    i2c_master_stop(cmd_handle);
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd_handle, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd_handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "i2c Success");
    } else if (err == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "Bus is busy");
    } else {
        ESP_LOGE(TAG, "Failed");
    }
}


/* @brief   Combine master mode i2c write and i2c read into single api.
 *          @note
 *          The handle which deals with i2c interface will be created and destroyed when this function is called.
 *          When using read mode, the last byte will not return ack signal
 * @param
 *          read_bit : True: read mode; False: write mode
 *          data:
 *                  data[0]: sensor register address we are interested in,
 *                  data[1 : data_len + 1]: it is used to store the data we want to write into register or
 *                  read from register.
 * data_len : The length of data we get from register or write in register
 */

static void i2c_send_to_BME280(uint8_t* data, size_t data_len,  bool read_bit) {
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    i2c_master_start(cmd_handle);
    i2c_master_write_byte(cmd_handle, (BME280_ADDR << 1) | WRITE_BIT, I2C_MASTER_ACK);  // depend on read_bit
    i2c_master_write_byte(cmd_handle, data[0], I2C_MASTER_ACK);
    if (read_bit) {
        i2c_master_start(cmd_handle);
        i2c_master_write_byte(cmd_handle, (BME280_ADDR << 1) | read_bit, I2C_MASTER_ACK);
        if (data_len > 1)
            i2c_master_read(cmd_handle, data + 1, data_len - 1, I2C_MASTER_LAST_NACK);  // The first byte should be data_addr
        else
            i2c_master_read_byte(cmd_handle, data + data_len -1 , I2C_MASTER_LAST_NACK);
    } else {
        i2c_master_write(cmd_handle, data + 1, data_len - 1, I2C_MASTER_ACK); // The first byte should be data_addr
    }
    i2c_master_stop(cmd_handle);
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd_handle, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd_handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "i2c Success");
    } else if (err == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "Bus is busy");
    } else {
        ESP_LOGE(TAG, "Failed");
    }
}

/*
 *  @brief  Initialize BME280 with structure.
 */
static void BME280_initialize() {
  uint8_t osrs_t = 1;             //Temperature oversampling x 1
  uint8_t osrs_p = 1;             //Pressure oversampling x 1
  uint8_t osrs_h = 1;             //Humidity oversampling x 1
  uint8_t mode = 3;               //Normal mode
  uint8_t t_sb = 5;               //Tstandby 1000ms
  uint8_t filter = 0;             //Filter off
  uint8_t spi3w_en = 0;           //3-wire SPI Disable

  uint8_t ctrl_meas_reg[2] = { 0xF4, (osrs_t << 5) | (osrs_p << 2) | mode};
  uint8_t config_reg[2]    = { 0xF5, (t_sb << 5) | (filter << 2) | spi3w_en};
  uint8_t ctrl_hum_reg[2]  = { 0xF2, osrs_h};

  /*
   * write register
   */
  i2c_send_to_BME280(ctrl_meas_reg, 2, false);
  i2c_send_to_BME280(config_reg, 2, false);
  i2c_send_to_BME280(ctrl_hum_reg, 2, false);
  ESP_LOGI(TAG, "BME280 initialized");
}

/*
 * @brief   read raw data from S300
 */

static void S300_readData()
{
    uint8_t opcode = READ_DATA;
    uint8_t data[7] = {0};
    i2c_send_to_S300(&opcode, 1, false);
    i2c_send_to_S300(data, 7, true);
    my_co2.co2_act = (data[1] << 8) | data[2];
}

/*
 *  @brief  send commend to S300
 */

static void S300_send_commend(uint8_t opcode)
{
    i2c_send_to_S300(&opcode, 1, false);
}

/*
 * @brief read raw data from BME280
 */

static void BME280_readData()
{
    int i = 0;
    uint8_t data[9] = { 0xF7, 0,0,0,0,0,0,0,0};    // read raw data
    i2c_send_to_BME280( data, 8, true);            // i2c read mode
    ESP_LOGI(TAG, "data[0]: %02x", data[0]);
    pres_raw = (data[1] << 12) | (data[2] << 4) | (data[3] >> 4);
    temp_raw = (data[4] << 12) | (data[5] << 4) | (data[6] >> 4);
    hum_raw  = (data[7] << 8) | data[8];
}

/* @brief read calibration data
 *
 */
static void BME280_read_calibration() {
    uint8_t data[33] = {0};
    data[0] = 0x88;
    i2c_send_to_BME280(data ,24 ,true );
    dig_T1 = (data[2] << 8) | data[1];
    dig_T2 = (data[4] << 8) | data[3];
    dig_T3 = (data[6] << 8) | data[5];
    dig_P1 = (data[8] << 8) | data[7];
    dig_P2 = (data[10] << 8) | data[9];
    dig_P3 = (data[12]<< 8) | data[11];
    dig_P4 = (data[14]<< 8) | data[13];
    dig_P5 = (data[16]<< 8) | data[15];
    dig_P6 = (data[18]<< 8) | data[17];
    dig_P7 = (data[20]<< 8) | data[19];
    dig_P8 = (data[22]<< 8) | data[21];
    dig_P9 = (data[24]<< 8) | data[23];
    ESP_LOGI(TAG, "dig_T1: %u, data[4]: %02x, data[3]: %01x, dig_T2: %u, dig_T3: %u",dig_T1, data[4], data[3], dig_T2, dig_T3 );
    data[24] = 0xA1;
    i2c_send_to_BME280(data+24, 1, true);
    dig_H1 = data[24];
    ESP_LOGI(TAG, "dig_H1: %d, data[24]: %02x", dig_H1, data[24]);
    data[25] = 0xE1;
    i2c_send_to_BME280(data+25, 7, true);
    dig_H2 = (data[27]<< 8) | data[26];
    dig_H3 = data[28];
    dig_H4 = (data[29]<< 4) | (0x0F & data[30]);
    dig_H5 = (data[31] << 4) | ((data[30] >> 4) & 0x0F);
    dig_H6 = data[32];
}

static signed long int calibration_T(signed long int adc_T)
{
    signed long int var1, var2, T;
    var1 = ((((adc_T >> 3) - ((signed long int)dig_T1<<1))) * ((signed long int)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T>>4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

static unsigned long int calibration_P(signed long int adc_P)
{
    signed long int var1, var2;
    unsigned long int P;
    var1 = (((signed long int)t_fine)>>1) - (signed long int)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int)dig_P6);
    var2 = var2 + ((var1*((signed long int)dig_P5))<<1);
    var2 = (var2>>2)+(((signed long int)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((signed long int)dig_P2) * var1)>>1))>>18;
    var1 = ((((32768+var1))*((signed long int)dig_P1))>>15);
    if (var1 == 0)
    {
        return 0;
    }
    P = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125;
    if(P<0x80000000)
    {
       P = (P << 1) / ((unsigned long int) var1);
    }
    else
    {
        P = (P / (unsigned long int)var1) * 2;
    }
    var1 = (((signed long int)dig_P9) * ((signed long int)(((P>>3) * (P>>3))>>13)))>>12;
    var2 = (((signed long int)(P>>2)) * ((signed long int)dig_P8))>>13;
    P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
    return P;
}

static unsigned long int calibration_H(signed long int adc_H)
{
    signed long int v_x1;

    v_x1 = (t_fine - ((signed long int)76800));
    v_x1 = (((((adc_H << 14) -(((signed long int)dig_H4) << 20) - (((signed long int)dig_H5) * v_x1)) +
              ((signed long int)16384)) >> 15) * (((((((v_x1 * ((signed long int)dig_H6)) >> 10) *
              (((v_x1 * ((signed long int)dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) *
              ((signed long int) dig_H2) + 8192) >> 14));
   v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)dig_H1)) >> 4));
   v_x1 = (v_x1 < 0 ? 0 : v_x1);
   v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
   return (unsigned long int)(v_x1 >> 12);
}

/*
 * check write register is correct or not
 */

static void BME280_read_configuration() {
    uint8_t ctrl_meas_reg[2] = { 0xF4, 0};
    uint8_t config_reg[2]    = { 0xF5, 0};
    uint8_t ctrl_hum_reg[2]  = { 0xF2, 0};
    i2c_send_to_BME280(ctrl_meas_reg, 2, true);
    ESP_LOGI(TAG, "ctrl_meas_reg[0]: %02x, ctrl_meas_reg[1]: %02x", ctrl_meas_reg[0], ctrl_meas_reg[1]);
    i2c_send_to_BME280(config_reg, 2, true);
    ESP_LOGI(TAG, "config_reg[0]: %02x, config_reg[1]: %02x", config_reg[0], config_reg[1]);
    i2c_send_to_BME280(ctrl_hum_reg, 2, true);
    ESP_LOGI(TAG, "ctrl_hum_reg[0]: %02x, ctrl_hum_reg[1]: %02x", ctrl_hum_reg[0], ctrl_hum_reg[1]);
}

/*
 * BME280 final data
 */

static void BME280_data() {
    signed long int temp_cal;
    unsigned long int press_cal,hum_cal;
    temp_cal = calibration_T(temp_raw);
    press_cal = calibration_P(pres_raw);
    hum_cal = calibration_H(hum_raw);
    my_temp.temp_act = (double) temp_cal / 100.0;
    my_press.press_act = (double) press_cal / 100.0;
    my_hum.hum_act = (double) hum_cal / 1024.0;
    ESP_LOGI(TAG, "temperature: %lf, pressure: %lf, humidity: %lf", my_temp.temp_act, my_press.press_act, my_hum.hum_act);
}

static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                              esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                param->value.state_change.appkey_add.net_idx,
                param->value.state_change.appkey_add.app_idx);
            ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
            ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_app_bind.element_addr,
                param->value.state_change.mod_app_bind.app_idx,
                param->value.state_change.mod_app_bind.company_id,
                param->value.state_change.mod_app_bind.model_id);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD");
            ESP_LOGI(TAG, "elem_addr 0x%04x, sub_addr 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_sub_add.element_addr,
                param->value.state_change.mod_sub_add.sub_addr,
                param->value.state_change.mod_sub_add.company_id,
                param->value.state_change.mod_sub_add.model_id);
            break;
        default:
            break;
        }
    }
}

struct example_sensor_descriptor {
    uint16_t sensor_prop_id;
    uint32_t pos_tolerance:12,
             neg_tolerance:12,
             sample_func:8;
    uint8_t  measure_period;
    uint8_t  update_interval;
} __attribute__((packed));

static void example_ble_mesh_send_sensor_descriptor_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    struct example_sensor_descriptor descriptor = {0};
    uint8_t *status = NULL;
    uint16_t length = 0;
    esp_err_t err;
    int i;

    status = calloc(1, ARRAY_SIZE(sensor_states) * ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN);
    if (!status) {
        ESP_LOGE(TAG, "No memory for sensor descriptor status!");
        return;
    }

    if (param->value.get.sensor_descriptor.op_en == false) {
        /* Mesh Model Spec:
         * Upon receiving a Sensor Descriptor Get message with the Property ID field
         * omitted, the Sensor Server shall respond with a Sensor Descriptor Status
         * message containing the Sensor Descriptor states for all sensors within the
         * Sensor Server.
         */
        for (i = 0; i < ARRAY_SIZE(sensor_states); i++) {
            descriptor.sensor_prop_id = sensor_states[i].sensor_property_id;
            descriptor.pos_tolerance = sensor_states[i].descriptor.positive_tolerance;
            descriptor.neg_tolerance = sensor_states[i].descriptor.negative_tolerance;
            descriptor.sample_func = sensor_states[i].descriptor.sampling_function;
            descriptor.measure_period = sensor_states[i].descriptor.measure_period;
            descriptor.update_interval = sensor_states[i].descriptor.update_interval;
            memcpy(status + length, &descriptor, ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN);
            length += ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN;
        }
        goto send;
    }

    for (i = 0; i < ARRAY_SIZE(sensor_states); i++) {
        if (param->value.get.sensor_descriptor.property_id == sensor_states[i].sensor_property_id) {
            descriptor.sensor_prop_id = sensor_states[i].sensor_property_id;
            descriptor.pos_tolerance = sensor_states[i].descriptor.positive_tolerance;
            descriptor.neg_tolerance = sensor_states[i].descriptor.negative_tolerance;
            descriptor.sample_func = sensor_states[i].descriptor.sampling_function;
            descriptor.measure_period = sensor_states[i].descriptor.measure_period;
            descriptor.update_interval = sensor_states[i].descriptor.update_interval;
            memcpy(status, &descriptor, ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN);
            length = ESP_BLE_MESH_SENSOR_DESCRIPTOR_LEN;
            goto send;
        }
    }

    /* Mesh Model Spec:
     * When a Sensor Descriptor Get message that identifies a sensor descriptor
     * property that does not exist on the element, the Descriptor field shall
     * contain the requested Property ID value and the other fields of the Sensor
     * Descriptor state shall be omitted.
     */
    memcpy(status, &param->value.get.sensor_descriptor.property_id, ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN);
    length = ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN;

send:
    ESP_LOG_BUFFER_HEX("Sensor Descriptor", status, length);

    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
            ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_STATUS, length, status);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Sensor Descriptor Status");
    }
    free(status);
}

static void example_ble_mesh_send_sensor_cadence_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    esp_err_t err;

    /* Sensor Cadence state is not supported currently. */
    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
            ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_STATUS,
            ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN,
            (uint8_t *)&param->value.get.sensor_cadence.property_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Sensor Cadence Status");
    }
}

static void example_ble_mesh_send_sensor_settings_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    esp_err_t err;

    /* Sensor Setting state is not supported currently. */
    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
            ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_STATUS,
            ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN,
            (uint8_t *)&param->value.get.sensor_settings.property_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Sensor Settings Status");
    }
}

struct example_sensor_setting {
    uint16_t sensor_prop_id;
    uint16_t sensor_setting_prop_id;
} __attribute__((packed));

static void example_ble_mesh_send_sensor_setting_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    struct example_sensor_setting setting = {0};
    esp_err_t err;

    /* Mesh Model Spec:
     * If the message is sent as a response to the Sensor Setting Get message or
     * a Sensor Setting Set message with an unknown Sensor Property ID field or
     * an unknown Sensor Setting Property ID field, the Sensor Setting Access
     * field and the Sensor Setting Raw field shall be omitted.
     */

    setting.sensor_prop_id = param->value.get.sensor_setting.property_id;
    setting.sensor_setting_prop_id = param->value.get.sensor_setting.setting_property_id;

    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
            ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_STATUS,
            sizeof(setting), (uint8_t *)&setting);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Sensor Setting Status");
    }
}

/*
 * get data from sensors
 */

static uint16_t example_ble_mesh_get_sensor_data(esp_ble_mesh_sensor_state_t *state, uint8_t *data)
{
    uint8_t mpid_len = 0, data_len = 0;
    uint32_t mpid = 0;

    if (state == NULL || data == NULL) {
        ESP_LOGE(TAG, "%s, Invalid parameter", __func__);
        return 0;
    }

    if (state->sensor_data.length == ESP_BLE_MESH_SENSOR_DATA_ZERO_LEN) {
        /* For zero-length sensor data, the length is 0x7F, and the format is Format B. */
        mpid = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID(state->sensor_data.length, state->sensor_property_id);
        mpid_len = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;
        data_len = 0;
    } else {
        if (state->sensor_data.format == ESP_BLE_MESH_SENSOR_DATA_FORMAT_A) {
            mpid = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A_MPID(state->sensor_data.length, state->sensor_property_id);
            mpid_len = ESP_BLE_MESH_SENSOR_DATA_FORMAT_A_MPID_LEN;
        } else {
            mpid = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID(state->sensor_data.length, state->sensor_property_id);
            mpid_len = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;
        }
        /* Use "state->sensor_data.length + 1" because the length of sensor data is zero-based. */
        data_len = state->sensor_data.length + 1;
    }


    memcpy(data, &mpid, mpid_len);
    memcpy(data + mpid_len, state->sensor_data.raw_value->data, data_len);

    return (mpid_len + data_len);
}

static void example_ble_mesh_send_sensor_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    uint8_t *status = NULL;
    uint16_t buf_size = 0;
    uint16_t length = 0;
    uint32_t mpid = 0;
    esp_err_t err;
    int i;
    BME280_readData();  // read raw data from BME280
    BME280_data();      // get data after calibration
    S300_readData();    // read raw data from S300

    memcpy(sensor_states[0].sensor_data.raw_value->data, my_temp.temp_data, 8);
    memcpy(sensor_states[1].sensor_data.raw_value->data, my_hum.hum_data, 8);
    memcpy(sensor_states[2].sensor_data.raw_value->data, my_press.press_data, 8);
    memcpy(sensor_states[3].sensor_data.raw_value->data, my_co2.co2_data, 2);
    /**
     * Sensor Data state from Mesh Model Spec
     * |--------Field--------|-Size (octets)-|------------------------Notes-------------------------|
     * |----Property ID 1----|-------2-------|--ID of the 1st device property of the sensor---------|
     * |-----Raw Value 1-----|----variable---|--Raw Value field defined by the 1st device property--|
     * |----Property ID 2----|-------2-------|--ID of the 2nd device property of the sensor---------|
     * |-----Raw Value 2-----|----variable---|--Raw Value field defined by the 2nd device property--|
     * | ...... |
     * |----Property ID n----|-------2-------|--ID of the nth device property of the sensor---------|
     * |-----Raw Value n-----|----variable---|--Raw Value field defined by the nth device property--|
     */
    for (i = 0; i < ARRAY_SIZE(sensor_states); i++) {
        esp_ble_mesh_sensor_state_t *state = &sensor_states[i];
        if (state->sensor_data.length == ESP_BLE_MESH_SENSOR_DATA_ZERO_LEN) {
            buf_size += ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;
        } else {
            /* Use "state->sensor_data.length + 1" because the length of sensor data is zero-based. */
            if (state->sensor_data.format == ESP_BLE_MESH_SENSOR_DATA_FORMAT_A) {
                buf_size += ESP_BLE_MESH_SENSOR_DATA_FORMAT_A_MPID_LEN + state->sensor_data.length + 1;
            } else {
                buf_size += ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN + state->sensor_data.length + 1;
            }
        }
    }

    status = calloc(1, buf_size);
    if (!status) {
        ESP_LOGE(TAG, "No memory for sensor status!");
        return;
    }

    if (param->value.get.sensor_data.op_en == false) {
        /* Mesh Model Spec:
         * If the message is sent as a response to the Sensor Get message, and if the
         * Property ID field of the incoming message is omitted, the Marshalled Sensor
         * Data field shall contain data for all device properties within a sensor.
         */
        for (i = 0; i < ARRAY_SIZE(sensor_states); i++) {
            length += example_ble_mesh_get_sensor_data(&sensor_states[i], status + length);
        }
        goto send;
    }

    /* Mesh Model Spec:
     * Otherwise, the Marshalled Sensor Data field shall contain data for the requested
     * device property only.
     */
    for (i = 0; i < ARRAY_SIZE(sensor_states); i++) {
        if (param->value.get.sensor_data.property_id == sensor_states[i].sensor_property_id) {
            length = example_ble_mesh_get_sensor_data(&sensor_states[i], status);
            goto send;
        }
    }

    /* Mesh Model Spec:
     * Or the Length shall represent the value of zero and the Raw Value field shall
     * contain only the Property ID if the requested device property is not recognized
     * by the Sensor Server.
     */
    mpid = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID(ESP_BLE_MESH_SENSOR_DATA_ZERO_LEN,
            param->value.get.sensor_data.property_id);
    memcpy(status, &mpid, ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN);
    length = ESP_BLE_MESH_SENSOR_DATA_FORMAT_B_MPID_LEN;

send:
    ESP_LOG_BUFFER_HEX("Sensor Data", status, length);

    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
            ESP_BLE_MESH_MODEL_OP_SENSOR_STATUS, length, status);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Sensor Status");
    }
    free(status);
}

static void example_ble_mesh_send_sensor_column_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    uint8_t *status = NULL;
    uint16_t length = 0;
    esp_err_t err;

    length = ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN +param->value.get.sensor_column.raw_value_x->len;

    status = calloc(1, length);
    if (!status) {
        ESP_LOGE(TAG, "No memory for sensor column status!");
        return;
    }

    memcpy(status, &param->value.get.sensor_column.property_id, ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN);
    memcpy(status + ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN, param->value.get.sensor_column.raw_value_x->data,
        param->value.get.sensor_column.raw_value_x->len);

    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
            ESP_BLE_MESH_MODEL_OP_SENSOR_COLUMN_STATUS, length, status);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Sensor Column Status");
    }
    free(status);
}

static void example_ble_mesh_send_sensor_series_status(esp_ble_mesh_sensor_server_cb_param_t *param)
{
    esp_err_t err;

    err = esp_ble_mesh_server_model_send_msg(param->model, &param->ctx,
            ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_STATUS,
            ESP_BLE_MESH_SENSOR_PROPERTY_ID_LEN,
            (uint8_t *)&param->value.get.sensor_series.property_id);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Sensor Column Status");
    }
}

static void example_ble_mesh_sensor_server_cb(esp_ble_mesh_sensor_server_cb_event_t event,
                                              esp_ble_mesh_sensor_server_cb_param_t *param)
{
    ESP_LOGI(TAG, "Sensor server, event %d, src 0x%04x, dst 0x%04x, model_id 0x%04x",
        event, param->ctx.addr, param->ctx.recv_dst, param->model->model_id);

    switch (event) {
    case ESP_BLE_MESH_SENSOR_SERVER_RECV_GET_MSG_EVT:
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_DESCRIPTOR_GET");
            example_ble_mesh_send_sensor_descriptor_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_GET");
            example_ble_mesh_send_sensor_cadence_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET");
            example_ble_mesh_send_sensor_settings_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_SETTINGS_GET");
            example_ble_mesh_send_sensor_setting_status(param);
            break;
            /* edit this part to fit multiple sensors */
        case ESP_BLE_MESH_MODEL_OP_SENSOR_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_GET");
            example_ble_mesh_send_sensor_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_COLUMN_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_COLUMN_GET");
            example_ble_mesh_send_sensor_column_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_SERIES_GET");
            example_ble_mesh_send_sensor_series_status(param);
            break;
        default:
            ESP_LOGE(TAG, "Unknown Sensor Get opcode 0x%04x", param->ctx.recv_op);
            return;
        }
        break;
    case ESP_BLE_MESH_SENSOR_SERVER_RECV_SET_MSG_EVT:
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET");
            example_ble_mesh_send_sensor_cadence_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET_UNACK:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_CADENCE_SET_UNACK");
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET");
            example_ble_mesh_send_sensor_setting_status(param);
            break;
        case ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET_UNACK:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_SENSOR_SETTING_SET_UNACK");
            break;
        default:
            ESP_LOGE(TAG, "Unknown Sensor Set opcode 0x%04x", param->ctx.recv_op);
            break;
        }
        break;
    default:
        ESP_LOGE(TAG, "Unknown Sensor Server event %d", event);
        break;
    }
}

static esp_err_t ble_mesh_init(void)
{
    esp_err_t err;

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);
    esp_ble_mesh_register_sensor_server_callback(example_ble_mesh_sensor_server_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mesh stack");
        return err;
    }
    /* I2C master mode does not need any buffer */
    err = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to use i2c driver");
        return err;
    }

    err = i2c_master_driver_initialize();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C with these parameters");
        return err;
    }

    err = esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable mesh node");
        return err;
    }
    /* create i2c handle, and test i2c interface */
    i2c_cmd_handle_t cmd_handle = i2c_cmd_link_create();
    err = i2c_master_start(cmd_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2C");
        return err;
    }
    i2c_master_write_byte(cmd_handle, (BME280_ADDR << 1) | WRITE_BIT, I2C_MASTER_ACK);
    i2c_master_stop(cmd_handle);
    err = i2c_master_cmd_begin(0, cmd_handle, 1000 / portTICK_RATE_MS); // 50ms
    if (err == ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "I2C timeout");
    } else if (err == ESP_FAIL) {
        ESP_LOGE(TAG, "I2C failed");
    } else if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to begin I2C");
    } else if (err == ESP_ERR_INVALID_ARG) {
        ESP_LOGE(TAG, "I2C invalid argument");
    } else {
        ESP_LOGI(TAG, "SUCCESS");
    }
    i2c_cmd_link_delete(cmd_handle);

    /*
     * Initialize BME280
     */
    BME280_initialize();

    BME280_read_configuration();

    BME280_read_calibration();

    board_led_operation(LED_G, LED_ON); // this part is not needed

    ESP_LOGI(TAG, "BLE Mesh sensor server initialized");

    return ESP_OK;
}

void app_main(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    board_init();

    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }
}

