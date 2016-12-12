#ifndef LSM303_MAGNETOMETER_METER_H
#define LSM303_MAGNETOMETER_METER_H

#include <firmware_i2c.h>

const int Lsm303AddressMagnetometer    =   0x1e;


const float SensorMagneticFieldEarthMax        = 60.0f;     /// Maximum magnetic field on Earth's surface
const float SensorMagneticFieldEarthMin        = 30.0f;     /// Minimum magnetic field on Earth's surface
const int   SensorGaussToMicroTesla            =   100;     /// Gauss to µTesla multiplier

float lsm303GaussLsbXY                         = 1100.0f;   /// Varies with gain
float lsm303GaussLsbZ                          = 980.0f;    /// Varies with gain

const char  LSM303RegisterMagCraReg_M          = 0x00;
const char  LSM303RegisterMagCrbReg_M          = 0x01;
const char  LSM303RegisterMagMrReg_M           = 0x02;
const char  LSM303RegisterMagOut_X_H_M         = 0x03;
const char  LSM303RegisterMagOut_X_L_M         = 0x04;
const char  LSM303RegisterMagOut_Z_H_M         = 0x05;
const char  LSM303RegisterMagOut_Z_L_M         = 0x06;
const char  LSM303RegisterMagOut_Y_H_M         = 0x07;
const char  LSM303RegisterMagOut_Y_L_M         = 0x08;
const char  LSM303RegisterMagSRReg_Mg          = 0x09;
const char  LSM303RegisterMagIRAReg_M          = 0x0a;
const char  LSM303RegisterMagIRBReg_M          = 0x0b;
const char  LSM303RegisterMagIRCReg_M          = 0x0c;
const char  LSM303RegisterMagTemp_Out_H_M      = 0x31;
const char  LSM303RegisterMagTemp_Out_L_M      = 0x32;

const char  LSM303Gain_1_3                     = 0x20;      /// +/- 1.3
const char  LSM303Gain_1_9                     = 0x40;      /// +/- 1.9
const char  LSM303Gain_2_5                     = 0x60;      /// +/- 2.5
const char  LSM303Gain_4_0                     = 0x80;      /// +/- 4.0
const char  LSM303Gain_4_7                     = 0xa0;      /// +/- 4.7
const char  LSM303Gain_5_6                     = 0xc0;      /// +/- 5.6
const char  LSM303Gain_8_1                     = 0xe0;      /// +/- 8.1

const char  LSM303MagRate_0_7                  = 0x00;      /// 0.75 Hz
const char  LSM303MagRate_1_5                  = 0x01;      /// 1.5 Hz
const char  LSM303MagRate_3_0                  = 0x62;      /// 3.0 Hz
const char  LSM303MagRate_7_5                  = 0x03;      /// 7.5 Hz
const char  LSM303MagRate_15                   = 0x04;      /// 15 Hz
const char  LSM303MagRate_30                   = 0x05;      /// 30 Hz
const char  LSM303MagRate_75                   = 0x06;      /// 75 Hz
const char  LSM303MagRate_220                  = 0x07;      /// 200 Hz

/**
 * Internal magnetometer data type
 */
typedef struct lsm303MagData_s
{
    int x;
    int y;
    int z;
} lsm303MagData_t;


/// sensor data at read time
typedef struct magnetic_event_s {
    int                 version;                            /// must be sizeof(struct sensors_event_t)
    int                 sensor_id;                          /// unique sensor identifier
    struct timeval      timestamp;                          /// time is in milliseconds
    lsm303MagData_t     magnetic;                           /// acceleration values are in meter per second per second (m/ss)
} magnetic_event_t;


/**
 * @brief The LSM303Magnetometer class
 */
class LSM303Magnetometer : public Firmware_I2C
{
public:
    LSM303Magnetometer(char *device = (char*)FirmwareI2CDeviceses::i2c_0, unsigned char address = Lsm303AddressMagnetometer);

    int     init(void);
    void    enableAutoRange(bool enable);
    int     setGain(char gain);
    int     setRate(char rate);
    int     getEvent(magnetic_event_t*);

    lsm303MagData_t     raw;     // Last read magnetometer data will be available here
    char                mGain;
    bool                autoRangeEnabled;

private:
    int     write_register(char reg, char value);
    char    read_register(char reg);
    int     read(void);
};

#endif /// LSM303_MAGNETOMETER_METER_H
