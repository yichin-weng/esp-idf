#ifndef BLE_TEMPERATURE_H
#define BLE_TEMPERATURE_H

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

class BME280
{
    public:
        struct calib_param_temp // calibration parameter of temperature
        {
            uint16_t dig_T1;
            int16_t dig_T2;
            int16_t dig_T3;
        };
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
        };
        struct calib_param_hum // calibration parameter of humidity
        {
            int8_t  dig_H1;
            int16_t dig_H2;
            int8_t  dig_H3;
            int16_t dig_H4;
            int16_t dig_H5;
            int8_t  dig_H6;
        };
        uint8_t osrs_t = 0;       // Temperature oversampling
        uint8_t osrs_p = 0;       // atmospheric pressure oversampling
        uint8_t osrs_h = 0;       // humidity oversampling
        uint8_t mode = 3;         // Normal mode -> need to implement different modes
        uint8_t t_sb = 5;         // standby time: 1000ms
        bool    filter = false;   // disable filter
        bool    spi3w_en = false; // disable 3-wire SPI interface
        unsigned long int hum_raw = 0;
        unsigned long int temp_raw = 0;
        unsigned long int pres_raw = 0;
        unsigned char current_addr = 0x00;
        long int t_fine = 0;

        BME280();                                               // constructor
        bool init();                                            // initialization
        unsigned char read_address();                           // check current address
        signed long int calibration_T(signed long int adc_T);   // calibrate the temperature data
        signed long int calibration_P(signed long int adc_P);   // calibrate the atmospheric pressure data
        signed long int calibration_H(signed long int adc_H);   // calibrate the humidity data
        void readData();                                        // read the raw data from sensor
        void readTrim();                                        // read parameter from
        void writeReg(uint8_t address, uint8_t regData);        // write register of address

    private:
        unsigned char BME280_addr = 0x76;
        unsigned char BME280_addr_alter = 0x77;
        uint8_t ctrl_meas_reg = 0;          //
        uint8_t config_reg = 0;             //
        uint8_t ctrl_hun_reg = 0;           //
}

#endif