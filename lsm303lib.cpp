//#include <limits.h>
#include <iostream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

#include "lsm303lib.h"


static float lsm303Accel_MG_LSB     = 0.001f;   // 1, 2, 4 or 12 mg per lsb
static float lsm303Mag_Gauss_LSB_XY = 1100.0f;  // Varies with gain
static float lsm303Mag_Gauss_LSB_Z  = 980.0f;   // Varies with gain

/**
 * @brief LSM303Accelerometer::LSM303Accelerometer
 *
 * ctor
 *
 * @param device                    device to open, eg. /dev/i2c-x
 * @param address                   i2c chip address
 * @param sensorID                  chip id
 */
LSM303Accelerometer::LSM303Accelerometer(char *device, unsigned char address, int sensorID) : Firmware_I2C(device, address) {
    _sensorID = sensorID;

    /// openDevice in firmware_i2c open's the i2c device and set's the slave address (ioctl)
    int status = Firmware_I2C::openDevice();
    if (status < 0)
        std::cerr << __func__ << " openDevice failed with error " << status << std::endl;

    // Clear the raw accel data
    raw.x = 0;
    raw.y = 0;
    raw.z = 0;
}

/**
 * @brief LSM303Accelerometer::begin
 *
 * open i2c device and read ID
 *
 * @return                          zero on success, negative error value on failure
 */
int LSM303Accelerometer::init()
{
    /// write controll reg A to address 0x57
    unsigned char buffer[2] = { 0x57, LSM303_REGISTER_ACCEL_CTRL_REG1_A };

    /// write start address to device
    int status = writeData(buffer, 2);
    if (status < 0) {
        std::cerr << __func__ << " writeData failed with error " << status << std::endl;
        return -1;
    }

    memset(buffer, 0, 2);

    /// read back controll register back to verify that we're connected
    status = readData(buffer, 1);
    if (status < 0) {
        std::cerr << __func__ << " readData failed with error " << status << std::endl;
        return -2;
    }

    printf("Register A: 0x%02x\n", buffer[0]);

    if (buffer[0] != 0x57) {
        return -3;
    }

    return 0;
}


/**
 * @brief LSM303Accelerometer::write_byte
 * @param reg
 * @param value
 * @return
 */
int LSM303Accelerometer::write_byte(char reg, char value)
{
    /// write controll reg A to address 0x57
    unsigned char buffer[2];
    memset(buffer, 0, 2);

    buffer[0] = reg;
    buffer[1] = value;

    /// write start address to device
    int status = writeData(buffer, 2);
    if (status < 0) {
        std::cerr << __func__ << " writeData failed with error " << status << std::endl;
        return -1;
    }

    return 0;
}


/**
 * @brief LSM303Accelerometer::read
 */
int LSM303Accelerometer::read()
{
    unsigned char buffer[6];
    memset(buffer, 0, 6);

    buffer[0] = LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80;

    /// write data start address to device
    int status = writeData(buffer, 1);
    if (status < 0) {
        std::cerr << __func__ << " writeData failed with error " << status << std::endl;
        return -1;
    }
    memset(buffer, 0, 6);

    /// read 6 bytes of data from device
    status = readData(buffer, 6);
    if (status < 0) {
        std::cerr << __func__ << " readData failed with error " << status << std::endl;
        return -2;
    }

    char xlo = buffer[0];
    char xhi = buffer[1];
    char ylo = buffer[2];
    char yhi = buffer[3];
    char zlo = buffer[4];
    char zhi = buffer[5];

    raw.x = (unsigned int)(xlo | (xhi << 8)) >> 4;
    raw.y = (unsigned int)(ylo | (yhi << 8)) >> 4;
    raw.z = (unsigned int)(zlo | (zhi << 8)) >> 4;

    return 0;
}


/**
 * @brief LSM303Accelerometer::getEvent
 * @param event
 *
 * @return                          zero on success, negative error value on failure
 */
int LSM303Accelerometer::getEvent(sensors_event_t *event)
{
    /* Clear the event */
    memset(event, 0, sizeof(sensors_event_t));
    unsigned char buffer[6];
    memset(buffer, 0, 6);

    int status = readData(buffer, 6);
    if (status < 0) {
        std::cerr << __func__ << " readData failed with error " << status << std::endl;
        return -1;
    }

    float SENSORS_GRAVITY_STANDARD = 9.81f;

    char xlo = buffer[0];
    char xhi = buffer[1];
    char ylo = buffer[2];
    char yhi = buffer[3];
    char zlo = buffer[4];
    char zhi = buffer[5];

    accelData.x = (short)((unsigned int)xlo | ((unsigned int)xhi << 8)) >> 4;
    accelData.y = (short)((unsigned int)ylo | ((unsigned int)yhi << 8)) >> 4;
    accelData.z = (short)((unsigned int)zlo | ((unsigned int)zhi << 8)) >> 4;

    event->version          = sizeof(sensors_event_t);
    event->sensor_id        = _sensorID;
    event->type             = SENSOR_TYPE_ACCELEROMETER;
    event->timestamp        = 0;        //millis();
    event->acceleration.x   = (float)raw.x * lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
    event->acceleration.y   = (float)raw.y * lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;
    event->acceleration.z   = (float)raw.z * lsm303Accel_MG_LSB * SENSORS_GRAVITY_STANDARD;

    return 0;
}


/**
 * @brief LSM303Accelerometer::getSensor
 * @param sensor
 */
void LSM303Accelerometer::getSensor(sensor_t *sensor) {
    memset(sensor, 0, sizeof(sensor_t));

    /* Insert the sensor name in the fixed length char array */
    strncpy (sensor->name, "LSM303", sizeof(sensor->name) - 1);

    sensor->name[sizeof(sensor->name)- 1] = 0;
    sensor->version     = 1;
    sensor->sensor_id   = _sensorID;
    sensor->type        = SENSOR_TYPE_ACCELEROMETER;
    sensor->min_delay   = 0;
    sensor->max_value   = 0.0F; // TBD
    sensor->min_value   = 0.0F; // TBD
    sensor->resolution  = 0.0F; // TBD
}



/**
 * @brief LSM303Magnetometer::LSM303Magnetometer
 * @param sensorID                  sensor id...
 */
LSM303Magnetometer::LSM303Magnetometer(char *device, unsigned char address, int sensorID) : Firmware_I2C(device, address) {
    _sensorID = sensorID;
    autoRangeEnabled = false;

    raw.x = 0;
    raw.y = 0;
    raw.z = 0;
}

/**
 * @brief LSM303Magnetometer::write_byte
 * @param reg                       register to write
 * @param value                     value to be written to above register
 * @return                          zero on success, negative error value on failure
 */
int LSM303Magnetometer::write_byte(char reg, char value)
{
    /// write controll reg A to address 0x57
    unsigned char buffer[2];
    memset(buffer, 0, 2);

    buffer[0] = reg;
    buffer[1] = value;

    /// write start address to device
    int status = writeData(buffer, 2);
    if (status < 0) {
        std::cerr << __func__ << " writeData failed with error " << status << std::endl;
        return -1;
    }

    return 0;
}


/**
 * @brief LSM303Magnetometer::read8
 * @param reg                       register to be read
 * @return                          zero on success, negative error value on failure
 */
char LSM303Magnetometer::read_byte(char reg)
{
    /// write controll reg A to address 0x57
    unsigned char buffer[1];
    memset(buffer, 0, 1);

    buffer[0] = reg;

    /// write start address to device
    int status = writeData(buffer, 1);
    if (status < 0) {
        std::cerr << __func__ << " writeData failed with error " << status << std::endl;
        return -1;
    }

    memset(buffer, 0, 1);

    status = readData(buffer, 1);
    if (status < 0) {
        std::cerr << __func__ << " readData failed with error " << status << std::endl;
        return -2;
    }

    return buffer[0];
}


/**
 * @brief LSM303Magnetometer::read
 */
int LSM303Magnetometer::read()
{
    unsigned char buffer[6];
    memset(buffer, 0, 6);

    buffer[0] = LSM303_REGISTER_MAG_OUT_X_H_M;

    /// write data start address to device
    int status = writeData(buffer, 1);
    if (status < 0) {
        std::cerr << __func__ << " writeData failed with error " << status << std::endl;
        return -1;
    }
    memset(buffer, 0, 6);

    /// read 6 bytes of data from device
    status = readData(buffer, 6);
    if (status < 0) {
        std::cerr << __func__ << " readData failed with error " << status << std::endl;
        return -2;
    }

    char xhi = buffer[0];
    char xlo = buffer[1];
    char zhi = buffer[2];
    char zlo = buffer[3];
    char yhi = buffer[4];
    char ylo = buffer[5];

    raw.x = (int)(xlo | ((int)xhi << 8));
    raw.y = (int)(ylo | ((int)yhi << 8));
    raw.z = (int)(zlo | ((int)zhi << 8));

    return 0;
}


/**
 * @brief LSM303Magnetometer::begin
 * @return                          zero on success, negative error value on failure
 */
int LSM303Magnetometer::init()
{
    /// Enable the magnetometer
    int status = write_byte(LSM303_REGISTER_MAG_MR_REG_M, 0x00);
    if (status < 0) {
        std::cerr << __func__ << " write_byte failed with error " << status << std::endl;
        return -1;
    }

    /// LSM303DLHC has no "whoami" so read register and check the default value
    char value = read_byte(LSM303_REGISTER_MAG_CRA_REG_M);
    if (value != 0x10) {
        std::cerr << __func__ << " value read is not equal value written" << std::endl;
        return -2;
    }

    /// Set the gain to a known level
    setMagGain(LSM303_MAGGAIN_1_3);

    return 0;
}


/**
 * @brief LSM303Magnetometer::enableAutoRange
 *
 * set autoRangeEnabled
 *
 * @param enabled
 */
void LSM303Magnetometer::enableAutoRange(bool enabled)
{
    autoRangeEnabled = enabled;
}


/**
 * @brief LSM303Magnetometer::setMagGain
 *
 * set gain on magnetometer
 *
 * @param gain
 */
int LSM303Magnetometer::setMagGain(lsm303MagGain_t gain)
{
    int status = write_byte(LSM303_REGISTER_MAG_CRB_REG_M, (char)gain);
    if (status < 0) {
        std::cerr << __func__ << " write_byte failed with error " << status << std::endl;
        return -1;
    }

    magGain = gain;

    switch(gain) {
    case LSM303_MAGGAIN_1_3:
        lsm303Mag_Gauss_LSB_XY = 1100;
        lsm303Mag_Gauss_LSB_Z  = 980;
        break;
    case LSM303_MAGGAIN_1_9:
        lsm303Mag_Gauss_LSB_XY = 855;
        lsm303Mag_Gauss_LSB_Z  = 760;
        break;
    case LSM303_MAGGAIN_2_5:
        lsm303Mag_Gauss_LSB_XY = 670;
        lsm303Mag_Gauss_LSB_Z  = 600;
        break;
    case LSM303_MAGGAIN_4_0:
        lsm303Mag_Gauss_LSB_XY = 450;
        lsm303Mag_Gauss_LSB_Z  = 400;
        break;
    case LSM303_MAGGAIN_4_7:
        lsm303Mag_Gauss_LSB_XY = 400;
        lsm303Mag_Gauss_LSB_Z  = 355;
        break;
    case LSM303_MAGGAIN_5_6:
        lsm303Mag_Gauss_LSB_XY = 330;
        lsm303Mag_Gauss_LSB_Z  = 295;
        break;
    case LSM303_MAGGAIN_8_1:
        lsm303Mag_Gauss_LSB_XY = 230;
        lsm303Mag_Gauss_LSB_Z  = 205;
        break;
    }

    return 0;
}


/**
 * @brief LSM303Magnetometer::setMagRate
 *
 * set update rate on device
 *
 * @param rate                      rate to be set on device
 * @return                          zero on success, negative error value on failure
 */
int LSM303Magnetometer::setMagRate(lsm303MagRate_t rate)
{
    char reg = ((char)rate & 0x07) << 2;
    int status = write_byte(LSM303_REGISTER_MAG_CRA_REG_M, reg);
    if (status < 0) {
        std::cerr << __func__ << " write_byte failed with error " << status << std::endl;
        return -1;
    }

    return 0;
}


/**
 * @brief LSM303Magnetometer::getEvent
 * @param event
 * @return
 */
int LSM303Magnetometer::getEvent(sensors_event_t *event) {
    bool readingValid = false;

    /* Clear the event */
    memset(event, 0, sizeof(sensors_event_t));

    while(!readingValid) {

        char reg_mg = read_byte(LSM303_REGISTER_MAG_SR_REG_Mg);
        if (!(reg_mg & 0x1)) {
            return -1;
        }

        /* Read new data */
        int status = read();
        if (status < 0) {
            std::cerr << __func__ << " read failed with error " << status << std::endl;
            return -2;
        }

        /* Make sure the sensor isn't saturating if auto-ranging is enabled */
        if (!autoRangeEnabled) {
            readingValid = true;
        }
        else {
            printf("x : %d y : %d z : %d\n", raw.x, raw.y, raw.z);

            /* Check if the sensor is saturating or not */
            if ( (raw.x >= 2040) | (raw.x <= -2040) | (raw.y >= 2040) | (raw.y <= -2040) | (raw.z >= 2040) | (raw.z <= -2040) ) {
                /* saturated - increase the range if possible */
                switch(magGain) {
                case LSM303_MAGGAIN_5_6:
                    setMagGain(LSM303_MAGGAIN_8_1);
                    readingValid = false;
                    printf("changing range to +/- 8.1\n");
                    break;

                case LSM303_MAGGAIN_4_7:
                    setMagGain(LSM303_MAGGAIN_5_6);
                    readingValid = false;
                    printf("changing range to +/- 5.6\n");
                    break;

                case LSM303_MAGGAIN_4_0:
                    setMagGain(LSM303_MAGGAIN_4_7);
                    readingValid = false;
                    printf("changing range to +/- 4.7\n");
                    break;

                case LSM303_MAGGAIN_2_5:
                    setMagGain(LSM303_MAGGAIN_4_0);
                    readingValid = false;
                    printf("Changing range to +/- 4.0\n");
                    break;

                case LSM303_MAGGAIN_1_9:
                    setMagGain(LSM303_MAGGAIN_2_5);
                    readingValid = false;
                    printf("Changing range to +/- 2.5\n");
                    break;

                case LSM303_MAGGAIN_1_3:
                    setMagGain(LSM303_MAGGAIN_1_9);
                    readingValid = false;
                    printf("Changing range to +/- 1.9\n");
                    break;

                default:
                    readingValid = true;
                    break;
                }
            }
            else
            {
                /* All values are withing range */
                readingValid = true;
            }
        }
    }

    event->version      = sizeof(sensors_event_t);
    event->sensor_id    = _sensorID;
    event->type         = SENSOR_TYPE_MAGNETIC_FIELD;
    event->timestamp    = 0;        //millis();
    event->magnetic.x   = (float)raw.x / lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
    event->magnetic.y   = (float)raw.y / lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
    event->magnetic.z   = (float)raw.z / lsm303Mag_Gauss_LSB_Z  * SENSORS_GAUSS_TO_MICROTESLA;

    return 0;
}


/**
 * @brief LSM303Magnetometer::getSensor
 *
 * Get sensor_t data
 *
 * @param sensor                    pointer to sensor_t structure
 */
void LSM303Magnetometer::getSensor(sensor_t *sensor) {
    /// clean struct
    memset(sensor, 0, sizeof(sensor_t));

    /* Insert the sensor name in the fixed length char array */
    strncpy(sensor->name, "LSM303", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name)- 1] = 0;
    sensor->version     = 1;
    sensor->sensor_id   = _sensorID;
    sensor->type        = SENSOR_TYPE_MAGNETIC_FIELD;
    sensor->min_delay   = 0;
    sensor->max_value   = 0.0F; // TBD
    sensor->min_value   = 0.0F; // TBD
    sensor->resolution  = 0.0F; // TBD
}


#if 0
int Adafruit_LSM303::begin()
{
    // Enable the accelerometer
    int status = write_byte(LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x27);
    if (status < 0) {
        std::cerr << __func__ << " write_byte failed with error " << status << std::endl;
        return -1;
    }

    // Enable the magnetometer
    status = write_byte(LSM303_REGISTER_MAG_MR_REG_M, 0x00);
    if (status < 0) {
        std::cerr << __func__ << " write_byte failed with error " << status << std::endl;
        return -1;
    }

    return 0;
}


/**
 * @brief Adafruit_LSM303::read
 */
void Adafruit_LSM303::read()
{
    unsigned char buffer[6];
    memset(buffer, 0, 6);

    buffer[0] = LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80;

    /// write data start address to device
    int status = writeData(buffer, 1);
    if (status < 0) {
        std::cerr << __func__ << " writeData failed with error " << status << std::endl;
        return -1;
    }
    memset(buffer, 0, 6);

    /// read 6 bytes of data from device
    status = readData(buffer, 6);
    if (status < 0) {
        std::cerr << __func__ << " readData failed with error " << status << std::endl;
        return -2;
    }

    char xlo = buffer[0];
    char xhi = buffer[1];
    char ylo = buffer[2];
    char yhi = buffer[3];
    char zlo = buffer[4];
    char zhi = buffer[5];

    // Shift values to create properly formed integer (low byte first)
    magData.x = (unsigned int)((unsigned int)xlo | ((unsigned int)xhi << 8));
    magData.y = (unsigned int)((unsigned int)ylo | ((unsigned int)yhi << 8));
    magData.z = (unsigned int)((unsigned int)zlo | ((unsigned int)zhi << 8));
}

void Adafruit_LSM303::setMagGain(lsm303MagGain_t gain)
{
    write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRB_REG_M, (byte)gain);
}

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/
void Adafruit_LSM303::write8(byte address, byte reg, byte value)
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

byte Adafruit_LSM303::read8(byte address, byte reg)
{
    byte value;

    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(address, (byte)1);
    value = Wire.read();
    Wire.endTransmission();

    return value;
}
#endif
