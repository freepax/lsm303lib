#ifndef LSM303_ACCELEROMETER_H
#define LSM303_ACCELEROMETER_H

#include <sys/time.h>
#include <firmware_i2c.h>


const int Lsm303AddressAccelerometer    =   0x19;

/// Pysical constants
const float SensorGravityEarth                          =   9.80665f;
const float SensorGravityMoon                           =   1.6f;                       /// The moon's gravity in m/ss
const float SensorGravitySun                            =   275.0f;                     /// The sun's gravity in m/ss
const float SensorPressureSeaLevelHpa                   =   1013.25f;                   /// Average sea level pressure is 1013.25 hPa
const float SensorDpsToRadians                          =   0.017453293f;               /// Degrees/s to rad/s multiplier

/// multiplication constants
static const float lsm303Accel_MG_LSB                   = 0.001f;                       /// 1, 2, 4 or 12 mg per lsb

/// lsm303 accelerometer register chip addresses
const char LSM303RegisterAccelerometerCtrlReg1_A        = 0x20;
const char LSM303RegisterAccelerometerCtrlReg2_A        = 0x21;
const char LSM303RegisterAccelerometerCtrlReg3_A        = 0x22;
const char LSM303RegisterAccelerometerCtrlReg4_A        = 0x23;
const char LSM303RegisterAccelerometerCtrlReg5_A        = 0x24;
const char LSM303RegisterAccelerometerCtrlReg6_A        = 0x25;
const char LSM303RegisterAccelerometerReference_A       = 0x26;
const char LSM303RegisterAccelerometerStatusReg_A       = 0x27;
const char LSM303RegisterAccelerometerOut_X_L_A         = 0x28;
const char LSM303RegisterAccelerometerOut_X_H_A         = 0x29;
const char LSM303RegisterAccelerometerOut_Y_L_A         = 0x2a;
const char LSM303RegisterAccelerometerOut_Y_H_A         = 0x2b;
const char LSM303RegisterAccelerometerOut_Z_L_A         = 0x2c;
const char LSM303RegisterAccelerometerOut_Z_H_A         = 0x2d;
const char LSM303RegisterAccelerometerFifoCtrlReg_A     = 0x2e;
const char LSM303RegisterAccelerometerFifoSRCReg_A      = 0x2f;
const char LSM303RegisterAccelerometerInt1CFG_A         = 0x30;
const char LSM303RegisterAccelerometerInt1Source_A      = 0x31;
const char LSM303RegisterAccelerometerInt1THS_A         = 0x32;
const char LSM303RegisterAccelerometerInt1Duration_A    = 0x33;
const char LSM303RegisterAccelerometerInt2CFG_A         = 0x34;
const char LSM303RegisterAccelerometerInt2SOURCE_A      = 0x35;
const char LSM303RegisterAccelerometerInt2THS_A         = 0x36;
const char LSM303RegisterAccelerometerInt2Duration_A    = 0x37;
const char LSM303RegisterAccelerometerClickCFG_A        = 0x38;
const char LSM303RegisterAccelerometerClickSRC_A        = 0x39;
const char LSM303RegisterAccelerometerClickTHS_A        = 0x3a;
const char LSM303RegisterAccelerometerTimeLimit_A       = 0x3b;
const char LSM303RegisterAccelerometerTimeLatency_A     = 0x3c;
const char LSM303RegisterAccelerometerTimeWindow_A      = 0x3d;

/// raw data from accelerator sensor
typedef struct lsm303RawData_s
{
    unsigned int x;
    unsigned int y;
    unsigned int z;
} lsm303RawData_t;

/// calculated acceleration
typedef struct accel_vector_s {
    float x;
    float y;
    float z;
} accel_vector_t;

/// sensor data at read time
typedef struct accel_event_s {
    int                 version;                            /**< must be sizeof(struct sensors_event_t) */
    int                 sensor_id;                          /**< unique sensor identifier */
    struct timeval      timestamp;                          /**< time is in milliseconds */
    accel_vector_t      acceleration;                       /**< acceleration values are in meter per second per second (m/s^2) */
} accel_event_t;


class LSM303Accelerometer : public Firmware_I2C
{
public:
    LSM303Accelerometer(char *device = (char*)FirmwareI2CDeviceses::i2c_0, unsigned char address = Lsm303AddressAccelerometer);

    int                 init(void);
    int                 getRegister(char reg, char *value);
    int                 getEvent(accel_event_t*);

private:
    int                 write_byte(char reg, char value);
    int                 read(void);

    lsm303RawData_t     raw;   // Last read accelerometer data will be available here
};


#endif /// LSM303_ACCELEROMETER_H
