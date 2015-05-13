#ifndef _MS5607_02BA03_H
#define _MS5607_02BA03_H
#include "Arduino.h"
//#include <Wire.h>

//#define __I2C__
#define __SPI__

#define MS5607_Addr  0x77

#define MS5607_RESET        0x1E
#define MS5607_ADC_READ     0x00
#define MS5607_CONVD1_256   0x40
#define MS5607_CONVD1_512   0x42
#define MS5607_CONVD1_1024  0x44
#define MS5607_CONVD1_2048  0x46
#define MS5607_CONVD1_4096  0x48
#define MS5607_CONVD2_256   0x50
#define MS5607_CONVD2_512   0x52
#define MS5607_CONVD2_1024  0x54
#define MS5607_CONVD2_2048  0x56
#define MS5607_CONVD2_4096  0x58
#define MS5607_PROM_READ    0xA0

union calibration_t
{
    uint16_t coeff[8];
    uint8_t  bytes[16];
    struct
    {
        uint16_t RESERVE;
        uint16_t C1;
        uint16_t C2;
        uint16_t C3;
        uint16_t C4;
        uint16_t C5;
        uint16_t C6;
        uint16_t CRC4;
    } x;
};

enum Resolution { OSR_256, OSR_512, OSR_1024, OSR_2048, OSR_4096 };
static calibration_t calibration;

class  MS5607_02BA03_Altimeter
{
public:
    MS5607_02BA03_Altimeter();
 #ifdef __SPI__
    bool begin( uint8_t cspin );
#endif // __SPI__
    bool begin(void);
    void ResetSensor(void);
    void ReadCoefficients(void);
    void Set_Resolution( Resolution res );
    void ReadTempAndPress(int32_t &temperature, int32_t &pressure);

private:
    uint32_t GetRawData( uint8_t cmd );
    void ReadADC( uint32_t &temp_raw, uint32_t &press_raw );
    void DoSoftwareCompensation(int32_t &temperature, int32_t &pressure);
    //uint8_t crc4(calibration_t *p );
    Resolution current_res = OSR_4096;
#ifdef __SPI__
     uint8_t chipSelectPin;
#endif // __SPI__

};




#endif // _MS5607_02BA03_H
