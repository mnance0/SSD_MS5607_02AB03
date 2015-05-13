/********************************************************************************
   Saturn Star Drive
   Measurement Specialties MS5607-02BA03 Pressure Sensor
   Ardino Library.

   v0.0  4/22/2015 -  Initial code



*********************************************************************************/
#include "Arduino.h"
#include "MS5607_02BA03.h"

//#define __I2C__   // Set interface
#define __SPI__

#if not( defined(__I2C__) || defined(__SPI__) )
#error "__I2C__ or __SPI__ must defined!"
#endif
#if ( defined(__I2C__) && defined (__SPI__) )
#error "Please select either __I2C__ or __SPI__!"
#endif // defined

#ifdef __I2C__
#include <Wire.h>
#endif // __I2C__

#ifdef __SPI__
#include <SPI.h>
#endif // __SPI__

////////////////////// Public Methods //////////////////////////

//Constructor
MS5607_02BA03_Altimeter::MS5607_02BA03_Altimeter()
{
}

// Initialize sensor
#ifdef __SPI__
bool MS5607_02BA03_Altimeter::begin( uint8_t cspin)
#endif // __SPI__
#ifdef __I2C__
bool MS5607_02BA03_Altimeter::begin(void)
#endif // __I2C__
{
#ifdef __I2C__
    Wire.begin();
#endif // __I2C__

#ifdef __SPI__
    MS5607_02BA03_Altimeter::chipSelectPin = cspin; //Set chip select pin
    SPI.begin();
    pinMode(chipSelectPin,OUTPUT);  // Set chip select as output
    digitalWrite(chipSelectPin,HIGH);  // Deselect sensor
#endif // __SPI__
    MS5607_02BA03_Altimeter::ResetSensor();      // Reset sensor
    MS5607_02BA03_Altimeter::ReadCoefficients();  // Read sensor compensation values
    return true;
}

#ifdef __SPI__
bool MS5607_02BA03_Altimeter::begin(void)
{
    begin(0);
    return true;
}
#endif // __SPI__

/**/
void MS5607_02BA03_Altimeter::ResetSensor(void)
{
#ifdef __I2C__
    Wire.beginTransmission(MS5607_Addr);  // Begin Transmission
    Wire.write(MS5607_RESET);             // Send RESET cmd
    Wire.endTransmission();               // End Transmission
#endif // __I2C__
#ifdef __SPI__
    digitalWrite(chipSelectPin,LOW);      // Select sensor
    SPI.transfer(MS5607_RESET);           // Send RESET cmd
    digitalWrite(chipSelectPin,HIGH);     // Deselect sensor
#endif // __SPI__
    delay(3);
    Serial.println("Sensor Reset Sent");
}

/**/
void MS5607_02BA03_Altimeter::ReadCoefficients(void)
{
    uint8_t i;

    for(i=0; i<8; ++i)
    {
#ifdef __I2C__
        Wire.beginTransmission(MS5607_Addr);     // Start transmission
        Wire.write(MS5607_PROM_READ + (i << 1)); // send PROM READ command
        Wire.endTransmission();                  // End Transmission
        Wire.requestFrom(MS5607_Addr, 2);

        if(Wire.available() >= 2)                // Check to see if two bytes are available
        {
            calibration.coeff[i] |= Wire.read() & 0x00FF;   // read MSB and acknowledge
            calibration.coeff[i] = (calibration.coeff[i] << 8) & 0xFF00;  // Shift data to MSB
            calibration.coeff[i] |=  (Wire.read() & 0x00FF);        // read LSB and not acknowledge
        }
#endif // __I2C__
#ifdef __SPI__
        digitalWrite(chipSelectPin,LOW);                        // Select pressure sensor
        SPI.transfer(MS5607_PROM_READ + (i << 1));              // send PROM READ cmd
        calibration.coeff[i] |= SPI.transfer(0x00) & 0x00FF;    // read MSB
        calibration.coeff[i] = (calibration.coeff[i] << 8) & 0xFF00;  // Shift data
        calibration.coeff[i] |= (SPI.transfer(0x00) & 0x00FF);  // read LSB
        digitalWrite(chipSelectPin,HIGH);                       // Deselect device
#endif // __SPI__
#ifdef __I2C__
        else
        {
            Serial.println("No data available in ReadCoefficient()");  // I2C error
        }
#endif // __I2C__
    }
    for(uint8_t i=0; i<8; ++i)    // Print out coefficient
    {
        Serial.print("Coefficient        ");
        Serial.print(i , DEC);
        Serial.print(" : ");
        Serial.println(calibration.coeff[i], DEC);
    }
}

/*
*/
void MS5607_02BA03_Altimeter::Set_Resolution( Resolution res )
{
    current_res = res;
}


/*
*/
void MS5607_02BA03_Altimeter::ReadTempAndPress(int32_t &temperature, int32_t &pressure)
{
    uint32_t tt, tp;
    ReadADC(tt, tp); // temp_r : typical 8077636 press_r : typical 6465444
    temperature = static_cast<int32_t>(tt);  //cast to signed long
    pressure = static_cast<int32_t>(tp);
    DoSoftwareCompensation(temperature, pressure);  // Do software compensation.
}




///////////////////////////  Private methods  ///////////////////////////
uint32_t MS5607_02BA03_Altimeter::GetRawData( uint8_t cmd )
{
    uint32_t t=0;
#ifdef __I2C__
    Wire.beginTransmission(MS5607_Addr);
    Wire.write( cmd ); // send conversion command
    Wire.endTransmission();
#endif // __I2C__
#ifdef __SPI__
    digitalWrite(chipSelectPin,LOW);  // Send conversion command
    SPI.transfer( cmd );
#endif // __SPI__
    // wait necessary conversion time
    switch(current_res)
    {
    case OSR_256:
        delayMicroseconds(600);
        break;
    case OSR_512:
        delayMicroseconds(1170);
        break;
    case OSR_1024:
        delayMicroseconds(2280);
        break;
    case OSR_2048:
        delayMicroseconds(4540);
        break;
    case OSR_4096:
        delayMicroseconds(9040);
        break;
    }
#ifdef __I2C__
    Wire.beginTransmission(MS5607_Addr);
    Wire.write(MS5607_ADC_READ);
    Wire.endTransmission();

    Wire.requestFrom(MS5607_Addr, 3);
    if(Wire.available() >= 3)
    {
        t  = Wire.read(); // read MSB and acknowledge
        t <<= 8;
        t |= Wire.read();      // read byte and acknowledge
        t <<= 8;
        t |=  Wire.read();  // read LSB and not acknowledge
        return t;
    }
#endif // __I2C__
#ifdef __SPI__
    digitalWrite(chipSelectPin,HIGH);    // Raise CS to finish conversion
    digitalWrite(chipSelectPin,LOW);    //  Select sensor
    SPI.transfer(MS5607_ADC_READ);      // Send ADC read cmd
    t = SPI.transfer(0x00);             // Get reading
    t <<= 8;
    t |= SPI.transfer(0x00);
    t <<= 8;
    t |= SPI.transfer(0x00);
    digitalWrite(chipSelectPin,HIGH);  // End transfer
    return t;
#endif // __SPI__
#ifdef __I2C__
    else
    {
        Serial.println(Wire.available());
    }
    return -1L;
#endif // __I2C__
}

/*
*/
void MS5607_02BA03_Altimeter::ReadADC( uint32_t &temp_raw, uint32_t &press_raw )
{
    press_raw  = GetRawData( MS5607_CONVD1_256 + (current_res << 1)); // Get raw pressure data
    temp_raw = GetRawData( MS5607_CONVD2_256 + (current_res << 1));   // Get raw temperature data
}


/*
*/
void MS5607_02BA03_Altimeter::DoSoftwareCompensation(int32_t &temperature, int32_t &pressure)
{
    const uint64_t C1 = static_cast<uint64_t>(calibration.coeff[1]);
    const uint64_t C2 = static_cast<uint64_t>(calibration.coeff[2]);
    const uint64_t C3 = static_cast<uint64_t>(calibration.coeff[3]);
    const uint64_t C4 = static_cast<uint64_t>(calibration.coeff[4]);
    const uint64_t C5 = static_cast<uint64_t>(calibration.coeff[5]);
    const uint64_t C6 = static_cast<uint64_t>(calibration.coeff[6]);
    int32_t dT;
    int64_t dT2, OFF, SENS, OFF2, SENS2, t2, tt2;

    // calculate 1st order pressure and temperature (MS5607 1st order algorithm)
    dT = temperature - (C5 << 8) ;           // difference between actual and reference temperature
    temperature = 2000L + ((dT * C6) >> 23); // actual temperature
    OFF   = (C2 << 17 ) + ((C4 * dT) >> 6 ); // offset at actual temperature

    SENS  = (C1 << 16 ) + ((C3 * dT) >> 7 ); // sensitivity at actual temperature
    dT2 = OFF2 = SENS2 = 0LL;  // Zero 2nd order coefficients
    // Calculate 2nd order compensation.
    if ( temperature < 2000L )    // Do 2nd order compensation if temperature is below 20°C
    {
        if ( temperature > -1500L )   // Is temp above -15°C?
        {

            t2 = static_cast<int64_t>(dT);
            dT2  = (t2 * t2) >> 31;
            t2 = static_cast<int64_t>(temperature - 2000LL);
            t2 *= t2;
            OFF2  = 61LL * ( t2 >> 4);
            SENS2 = t2 << 1;
        }
        else
        {
            t2 = static_cast<int64_t>(temperature + 1500LL);
            t2 *= t2;
            OFF2 = OFF2 + ( 15LL *  t2 );
            SENS2 = SENS2 + ( t2 << 3 );
        }

    }
    temperature = temperature - dT2;
    OFF = OFF - OFF2;
    SENS = SENS - SENS2;
    pressure = (((pressure * SENS ) >> 21) - OFF) >> 15 ; // / 100;      // temperature compensated pressure
}

//********************************************************
//! @brief calculate the CRC code for details look into CRC CODE NOTES
//!
//! @return crc code
//********************************************************
//uint8_t MS5607_02BA03_Altimeter::crc4(calibration_t *p )
//{
//    uint8_t cnt; // simple counter
//    uint16_t n_rem; // crc reminder
//    uint16_t crc_read; // original value of the crc
//    uint8_t n_bit;
//    n_rem = 0x00;
//    crc_read=p->coeff[7]; //save read CRC
//    p->coeff[7]=(0xFF00 & (p->coeff[7])); //CRC byte is replaced by 0
//    for (cnt = 0; cnt < 16; cnt++) // operation is performed on bytes
//    {
//        n_rem  ^= p->bytes[cnt];
//        // choose LSB or MSB
//        //if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
//        //else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
//        for (n_bit = 8; n_bit > 0; n_bit--)
//        {
//            if (n_rem & (0x8000))
//            {
//                n_rem = (n_rem << 1) ^ 0x3000;
//            }
//  μ          else
//            {
//                n_rem = (n_rem << 1);
//            }
//        }
//    }
//    n_rem= (0x000F & (n_rem >> 12)); // // final 4-bit reminder is CRC code
//    p->coeff[7]=crc_read; // restore the crc_read to its original place
//    return (n_rem ^ 0x00);
//}

