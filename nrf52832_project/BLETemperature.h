#ifndef BLE_TEMPERATURE_H
#define BLE_TEMPERATURE_H

#include "driver/i2c.h"

#define BME280_ADDR 0x76
#define BME280_ADDR_ALTER 0x77


/*!
 *  @brief Register addresses
 */
enum {
  BME280_REGISTER_DIG_T1 = 0x88,
  BME280_REGISTER_DIG_T2 = 0x8A,
  BME280_REGISTER_DIG_T3 = 0x8C,

  BME280_REGISTER_DIG_P1 = 0x8E,
  BME280_REGISTER_DIG_P2 = 0x90,
  BME280_REGISTER_DIG_P3 = 0x92,
  BME280_REGISTER_DIG_P4 = 0x94,
  BME280_REGISTER_DIG_P5 = 0x96,
  BME280_REGISTER_DIG_P6 = 0x98,
  BME280_REGISTER_DIG_P7 = 0x9A,
  BME280_REGISTER_DIG_P8 = 0x9C,
  BME280_REGISTER_DIG_P9 = 0x9E,

  BME280_REGISTER_DIG_H1 = 0xA1,
  BME280_REGISTER_DIG_H2 = 0xE1,
  BME280_REGISTER_DIG_H3 = 0xE3,
  BME280_REGISTER_DIG_H4 = 0xE4,
  BME280_REGISTER_DIG_H5 = 0xE5,
  BME280_REGISTER_DIG_H6 = 0xE7,

  BME280_REGISTER_CHIPID = 0xD0,
  BME280_REGISTER_VERSION = 0xD1,
  BME280_REGISTER_SOFTRESET = 0xE0,

  BME280_REGISTER_CAL26 = 0xE1, // R calibration stored in 0xE1-0xF0

  BME280_REGISTER_CONTROLHUMID = 0xF2,
  BME280_REGISTER_STATUS = 0XF3,
  BME280_REGISTER_CONTROL = 0xF4,
  BME280_REGISTER_CONFIG = 0xF5,
  BME280_REGISTER_PRESSUREDATA = 0xF7,
  BME280_REGISTER_TEMPDATA = 0xFA,
  BME280_REGISTER_HUMIDDATA = 0xFD
};


struct calib_param_temp // calibration parameter of temperature
{
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
} CALIB_TEMP = {0};

struct calib_param_pres // calibration parameter of atmospheric pressure
{
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} CALIB_PRES = {0};

struct calib_param_hum // calibration parameter of humidity
{
    int8_t  dig_H1;
    int16_t dig_H2;
    int8_t  dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t  dig_H6;
} CALIB_HUM = {0};

struct setting
{
    uint8_t osrs_t;       // Temperature oversampling
    uint8_t osrs_p;       // atmospheric pressure oversampling
    uint8_t osrs_h;       // humidity oversampling
    uint8_t mode;         // Normal mode -> need to implement different modes
    uint8_t t_sb;         // standby time: 1000ms
    bool    filter;       // disable filter
    bool    spi3w_en;     // disable 3-wire SPI interface
} INSTRUCTION = {1, 1, 1, 3, 5, false, false};

/*
 This struct represents raw data from temperature sensor
 */

struct raw_result
{
    unsigned long int hum_raw;
    unsigned long int temp_raw;
    unsigned long int pres_raw;
    unsigned char current_addr;
    long int t_fine;
} raw_data = {0};

struct reg
{
    uint8_t ctrl_meas_reg;          //
    uint8_t config_reg;             //
    uint8_t ctrl_hum_reg;           //
} bme280_r = {0};

static bool BME280_init(){
    bme280_r.ctrl_meas_reg = (INSTRUCTION.osrs_t << 5) | (INSTRUCTION.osrs_p << 2) | INSTRUCTION.mode;
    bme280_r.config_reg    = (INSTRUCTION.t_sb << 5) | (INSTRUCTION.filter << 2) | INSTRUCTION.spi3w_en;
    bme280_r.ctrl_hum_reg  = INSTRUCTION.osrs_h;

    writeReg(0xF2,register.ctrl_hum_reg);
    writeReg(0xF4,register.ctrl_meas_reg);
    writeReg(0xF5,register.config_reg);
    readTrim();
}

static void readTrim()
{
    uint8_t data[32] = {0};
    uint8_t i = 0;
}

#endif
