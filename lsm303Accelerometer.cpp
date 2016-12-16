#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

#include "lsm303Accelerometer.h"

/**
 * @brief LSM303Accelerometer::LSM303Accelerometer
 *
 * ctor
 *
 * @param device                    device to open, eg. /dev/i2c-x
 * @param address                   i2c chip address
 * @param sensorID                  chip id
 */
LSM303Accelerometer::LSM303Accelerometer(char *device, unsigned char address) : Firmware_I2C(device, address) {
    printf("device %s address 0x%0x\n", device, address);

    /// openDevice in firmware_i2c open's the i2c device and set's the slave address (ioctl)
    int status = Firmware_I2C::openDevice();
    if (status < 0)
        std::cerr << __func__ << " openDevice failed with error " << status << std::endl;

    /// Clear the raw accel data
    raw.x = 0;
    raw.y = 0;
    raw.z = 0;
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
 * @brief LSM303Accelerometer::begin
 *
 * open i2c device and read ID
 *
 * @return                          zero on success, negative error value on failure
 */
int LSM303Accelerometer::init()
{
    int status = -1;
    char value = 0x57;

    status = write_byte(LSM303RegisterAccelerometerCtrlReg1_A, value);
    if (status < 0) {
        std::cerr << __func__ << " write_byte failed with error " << status << std::endl;
        return -1;
    }

    unsigned char buffer[1];
    memset(buffer, 0, 1);

    /// read back controll register back to verify that we're connected
    status = readData(buffer, 1);
    if (status < 0) {
        std::cerr << __func__ << " readData failed with error " << status << std::endl;
        return -2;
    }

    if (buffer[0] != value) {
        return -3;
    }

    return 0;
}


int LSM303Accelerometer::getRegister(char reg, char *value)
{
    /// set value to zero - by default
    *value = 0x00;

    int status = readRegister(reg, value);
    if (status < 0) {
        std::cerr << __func__ << ":" << __LINE__ << " Firmware_I2C::readRegister failed with error " << status << std::endl;
        return -1;
    }

    *value = (char)status;

    return 0;

}

/**
 * @brief LSM303Accelerometer::read
 */
int LSM303Accelerometer::read()
{
    unsigned char buffer[6];
    memset(buffer, 0, 6);

    buffer[0] = LSM303RegisterAccelerometerOut_X_L_A | 0x80;

    /// write data start address to device
    int status = writeData(buffer, 1);
    if (status < 0) {
        std::cerr << __func__ << " writeData failed with error " << status << std::endl;
        return -1;
    }

    usleep(100);

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

    //printf("Six bytes: %d %d %d %d %d %d\n", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);

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
int LSM303Accelerometer::getEvent(accel_event_t *event)
{
    unsigned char buffer[6];

    /// clear data buffers
    memset(event, 0, sizeof(accel_event_t));
    memset(buffer, 0, 6);

    /// read device
    int status = read();
    if (status < 0) {
        std::cerr << __func__ << ": read failed with error " << status << std::endl;
        return -1;
    }

    status = gettimeofday(&event->timestamp, NULL);
    if (status < 0) {
        std::cerr << __func__ << ": gettimeofday failed with error " << status << std::endl;
        return -2;
    }

    event->version          = sizeof(accel_event_t);
    event->acceleration.x   = (float)raw.x * lsm303Accel_MG_LSB * SensorGravityEarth;
    event->acceleration.y   = (float)raw.y * lsm303Accel_MG_LSB * SensorGravityEarth;
    event->acceleration.z   = (float)raw.z * lsm303Accel_MG_LSB * SensorGravityEarth;

    return 0;
}
