#ifndef _LSM303_H
#define _LSM303_H

#include <firmware_i2c.h>

/* i2c Addresses */
const int Lsm303AddressAccelerometer    =   0x19;
const int Lsm303AddressMagnetometer     =   0x1e;

/* constants */
const int SENSORS_GRAVITY_EARTH         =   9.80665f;
const int SENSORS_GRAVITY_MOON          =   1.6f;                       /**< The moon's gravity in m/s^2 */
const int SENSORS_GRAVITY_SUN           =   275.0f;                     /**< The sun's gravity in m/s^2 */
const int SENSORS_GRAVITY_STANDARD      =   SENSORS_GRAVITY_EARTH;
const int SENSORS_MAGFIELD_EARTH_MAX    =   60.0f;                      /**< Maximum magnetic field on Earth's surface */
const int SENSORS_MAGFIELD_EARTH_MIN    =   30.0f;                      /**< Minimum magnetic field on Earth's surface */
const int SENSORS_PRESSURE_SEALEVELHPA  =   1013.25f;                   /**< Average sea level pressure is 1013.25 hPa */
const int SENSORS_DPS_TO_RADS           =   0.017453293f;               /**< Degrees/s to rad/s multiplier */
const int SENSORS_GAUSS_TO_MICROTESLA   =   100;                        /**< Gauss to micro-Tesla multiplier */

typedef enum
{
  SENSOR_TYPE_ACCELEROMETER         = (1),   /**< Gravity + linear acceleration */
  SENSOR_TYPE_MAGNETIC_FIELD        = (2),
  SENSOR_TYPE_ORIENTATION           = (3),
  SENSOR_TYPE_GYROSCOPE             = (4),
  SENSOR_TYPE_LIGHT                 = (5),
  SENSOR_TYPE_PRESSURE              = (6),
  SENSOR_TYPE_PROXIMITY             = (8),
  SENSOR_TYPE_GRAVITY               = (9),
  SENSOR_TYPE_LINEAR_ACCELERATION   = (10),  /**< Acceleration not including gravity */
  SENSOR_TYPE_ROTATION_VECTOR       = (11),
  SENSOR_TYPE_RELATIVE_HUMIDITY     = (12),
  SENSOR_TYPE_AMBIENT_TEMPERATURE   = (13),
  SENSOR_TYPE_VOLTAGE               = (15),
  SENSOR_TYPE_CURRENT               = (16),
  SENSOR_TYPE_COLOR                 = (17)
} sensors_type_t;

/* accelerometer register addresses */
typedef enum
{
    LSM303_REGISTER_ACCEL_CTRL_REG1_A = 0x00,
    LSM303_REGISTER_ACCEL_CTRL_REG2_A,
    LSM303_REGISTER_ACCEL_CTRL_REG3_A,
    LSM303_REGISTER_ACCEL_CTRL_REG4_A,
    LSM303_REGISTER_ACCEL_CTRL_REG5_A,
    LSM303_REGISTER_ACCEL_CTRL_REG6_A,
    LSM303_REGISTER_ACCEL_REFERENCE_A,
    LSM303_REGISTER_ACCEL_STATUS_REG_A,
    LSM303_REGISTER_ACCEL_OUT_X_L_A,
    LSM303_REGISTER_ACCEL_OUT_X_H_A,
    LSM303_REGISTER_ACCEL_OUT_Y_L_A,
    LSM303_REGISTER_ACCEL_OUT_Y_H_A,
    LSM303_REGISTER_ACCEL_OUT_Z_L_A,
    LSM303_REGISTER_ACCEL_OUT_Z_H_A,
    LSM303_REGISTER_ACCEL_FIFO_CTRL_REG_A,
    LSM303_REGISTER_ACCEL_FIFO_SRC_REG_A,
    LSM303_REGISTER_ACCEL_INT1_CFG_A,
    LSM303_REGISTER_ACCEL_INT1_SOURCE_A,
    LSM303_REGISTER_ACCEL_INT1_THS_A,
    LSM303_REGISTER_ACCEL_INT1_DURATION_A,
    LSM303_REGISTER_ACCEL_INT2_CFG_A,
    LSM303_REGISTER_ACCEL_INT2_SOURCE_A,
    LSM303_REGISTER_ACCEL_INT2_THS_A,
    LSM303_REGISTER_ACCEL_INT2_DURATION_A,
    LSM303_REGISTER_ACCEL_CLICK_CFG_A,
    LSM303_REGISTER_ACCEL_CLICK_SRC_A,
    LSM303_REGISTER_ACCEL_CLICK_THS_A,
    LSM303_REGISTER_ACCEL_TIME_LIMIT_A,
    LSM303_REGISTER_ACCEL_TIME_LATENCY_A,
    LSM303_REGISTER_ACCEL_TIME_WINDOW_A
} lsm303AccelRegisters_t;

/* magnetometer register addresses */
typedef enum
{
    LSM303_REGISTER_MAG_CRA_REG_M,
    LSM303_REGISTER_MAG_CRB_REG_M,
    LSM303_REGISTER_MAG_MR_REG_M,
    LSM303_REGISTER_MAG_OUT_X_H_M,
    LSM303_REGISTER_MAG_OUT_X_L_M,
    LSM303_REGISTER_MAG_OUT_Z_H_M,
    LSM303_REGISTER_MAG_OUT_Z_L_M,
    LSM303_REGISTER_MAG_OUT_Y_H_M,
    LSM303_REGISTER_MAG_OUT_Y_L_M,
    LSM303_REGISTER_MAG_SR_REG_Mg,
    LSM303_REGISTER_MAG_IRA_REG_M,
    LSM303_REGISTER_MAG_IRB_REG_M,
    LSM303_REGISTER_MAG_IRC_REG_M,
    LSM303_REGISTER_MAG_TEMP_OUT_H_M          = 0x31,
    LSM303_REGISTER_MAG_TEMP_OUT_L_M          = 0x32
} lsm303MagRegisters_t;

/* MAGNETOMETER GAIN SETTINGS */
typedef enum
{
    LSM303_MAGGAIN_1_3                        = 0x20,  // +/- 1.3
    LSM303_MAGGAIN_1_9                        = 0x40,  // +/- 1.9
    LSM303_MAGGAIN_2_5                        = 0x60,  // +/- 2.5
    LSM303_MAGGAIN_4_0                        = 0x80,  // +/- 4.0
    LSM303_MAGGAIN_4_7                        = 0xA0,  // +/- 4.7
    LSM303_MAGGAIN_5_6                        = 0xC0,  // +/- 5.6
    LSM303_MAGGAIN_8_1                        = 0xE0   // +/- 8.1
} lsm303MagGain_t;

/* MAGNETOMETER UPDATE RATE SETTINGS */
typedef enum
{
    LSM303_MAGRATE_0_7                        = 0x00,  // 0.75 Hz
    LSM303_MAGRATE_1_5                        = 0x01,  // 1.5 Hz
    LSM303_MAGRATE_3_0                        = 0x62,  // 3.0 Hz
    LSM303_MAGRATE_7_5                        = 0x03,  // 7.5 Hz
    LSM303_MAGRATE_15                         = 0x04,  // 15 Hz
    LSM303_MAGRATE_30                         = 0x05,  // 30 Hz
    LSM303_MAGRATE_75                         = 0x06,  // 75 Hz
    LSM303_MAGRATE_220                        = 0x07   // 200 Hz
} lsm303MagRate_t;

/**
 * Internal magnetometer data type
 */
typedef struct lsm303MagData_s
{
    int x;
    int y;
    int z;
} lsm303MagData_t;

/**
 * INTERNAL ACCELERATION DATA TYPE
 */
typedef struct lsm303AccelData_s
{
    unsigned int x;
    unsigned int y;
    unsigned int z;
} lsm303AccelData_t;

/**
 * CHIP ID
 */
#define LSM303_ID                     (0b11010100)

typedef struct {
    union {
        float           v[3];
        struct {
            float       x;
            float       y;
            float       z;
        };
        /* Orientation sensors */
        struct {
            float       roll;    /**< Rotation around the longitudinal axis (the plane body, 'X axis'). Roll is positive and increasing when moving downward. -90°<=roll<=90° */
            float       pitch;   /**< Rotation around the lateral axis (the wing span, 'Y axis'). Pitch is positive and increasing when moving upwards. -180°<=pitch<=180°) */
            float       heading; /**< Angle between the longitudinal axis (the plane body) and magnetic north, measured clockwise when viewing from the top of the device. 0-359° */
        };
    };
    char                status;
    char                reserved[3];
} sensors_vec_t;

typedef struct {
    union {
        float           c[3];
        /* RGB color space */
        struct {
            float       r;       /**< Red component */
            float       g;       /**< Green component */
            float       b;       /**< Blue component */
        };
    };
    unsigned int        rgba;         /**< 24-bit RGBA value */
} sensors_color_t;


/* Unified sensor driver for the accelerometer */
typedef struct sensor_s
{
    char                name[12];                        /**< sensor name */
    int                 version;                         /**< version of the hardware + driver */
    int                 sensor_id;                       /**< unique sensor identifier */
    int                 type;                            /**< this sensor's type (ex. SENSOR_TYPE_LIGHT) */
    float               max_value;                       /**< maximum value of this sensor's value in SI units */
    float               min_value;                       /**< minimum value of this sensor's value in SI units */
    float               resolution;                      /**< smallest difference between two values reported by this sensor */
    int                 min_delay;                       /**< min delay in microseconds between events. zero = not a constant rate */
} sensor_t;

typedef struct
{
    int                 version;                          /**< must be sizeof(struct sensors_event_t) */
    int                 sensor_id;                        /**< unique sensor identifier */
    int                 type;                             /**< sensor type */
    int                 reserved0;                        /**< reserved */
    int                 timestamp;                        /**< time is in milliseconds */
    union
    {
        float           data[4];
        sensors_vec_t   acceleration;         /**< acceleration values are in meter per second per second (m/s^2) */
        sensors_vec_t   magnetic;             /**< magnetic vector values are in micro-Tesla (uT) */
        sensors_vec_t   orientation;          /**< orientation values are in degrees */
        sensors_vec_t   gyro;                 /**< gyroscope values are in rad/s */
        float           temperature;          /**< temperature is in degrees centigrade (Celsius) */
        float           distance;             /**< distance in centimeters */
        float           light;                /**< light in SI lux units */
        float           pressure;             /**< pressure in hectopascal (hPa) */
        float           relative_humidity;    /**< relative humidity in percent */
        float           current;              /**< current in milliamps (mA) */
        float           voltage;              /**< voltage in volts (V) */
        sensors_color_t color;                /**< color in RGB component values */
    };
} sensors_event_t;


//dafruit_LSM303_Accel_Unified : public Adafruit_Sensor
class LSM303Accelerometer : Firmware_I2C
{
public:
    LSM303Accelerometer(char *device = (char*)FirmwareI2CDeviceses::i2c_0, unsigned char address = Lsm303AddressAccelerometer, int sensorID = -1);

    int                 init(void);
    int                 getEvent(sensors_event_t*);
    void                getSensor(sensor_t*);

    lsm303AccelData_t   raw;   // Last read accelerometer data will be available here

private:
    int                 write_byte(char reg, char value);
    int                 read(void);

    int                 _sensorID;
    lsm303AccelData_t   accelData;      /// Last read accelerometer data will be available here

};

/* Unified sensor driver for the magnetometer */
class LSM303Magnetometer : public Firmware_I2C
{
public:
    LSM303Magnetometer(char *device = (char*)FirmwareI2CDeviceses::i2c_0, unsigned char address = Lsm303AddressAccelerometer, int sensorID = -1);

    int     init(void);
    void    enableAutoRange(bool enable);
    int     setMagGain(lsm303MagGain_t gain);
    int     setMagRate(lsm303MagRate_t rate);
    int     getEvent(sensors_event_t*);
    void    getSensor(sensor_t*);

    lsm303MagData_t     raw;     // Last read magnetometer data will be available here
    lsm303MagGain_t     magGain;
    bool                autoRangeEnabled;

private:
    int     _sensorID;

    int     write_byte(char reg, char value);
    char    read_byte(char reg);
    int     read(void);
};

#if 0
/* Non Unified (old) driver for compatibility reasons */
class Adafruit_LSM303
{
public:
    int begin(void);
    void read(void);
    void setMagGain(lsm303MagGain_t gain);

    lsm303MagData_t     magData;        /// Last read magnetometer data will be available here

    int write_byte(char reg, char value);
};
#endif
#endif
