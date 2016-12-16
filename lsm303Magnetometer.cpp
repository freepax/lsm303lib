#include <stdio.h>
#include <iostream>
#include <string.h>
#include <sys/time.h>

#include "lsm303Magnetometer.h"

/**
 * @brief LSM303Magnetometer::LSM303Magnetometer
 * @param sensorID                  sensor id...
 */
LSM303Magnetometer::LSM303Magnetometer(char *device, unsigned char address) : Firmware_I2C(device, address) {
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
int LSM303Magnetometer::write_register(char reg, char value)
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
char LSM303Magnetometer::read_register(char reg)
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

    buffer[0] = LSM303RegisterMagOut_X_H_M;

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
    int status = write_register(LSM303RegisterMagMrReg_M, 0x00);
    if (status < 0) {
        std::cerr << __func__ << " write_byte failed with error " << status << std::endl;
        return -1;
    }

    /// LSM303DLHC has no "whoami" so read register and check the default value
    char value = read_register(LSM303RegisterMagCraReg_M);
    if (value != 0x10) {
        std::cerr << __func__ << " value read is not equal value written" << std::endl;
        return -2;
    }

    /// Set the gain to a known level
    setGain(LSM303Gain_1_3);

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
int LSM303Magnetometer::setGain(char gain)
{
    int status = write_register(LSM303RegisterMagCrbReg_M, (char)gain);
    if (status < 0) {
        std::cerr << __func__ << " write_byte failed with error " << status << std::endl;
        return -1;
    }

    mGain = gain;

    switch(gain) {
    case LSM303Gain_1_3:
        lsm303GaussLsbXY = 1100;
        lsm303GaussLsbZ  = 980;
        break;
    case LSM303Gain_1_9:
        lsm303GaussLsbXY = 855;
        lsm303GaussLsbZ  = 760;
        break;
    case LSM303Gain_2_5:
        lsm303GaussLsbXY = 670;
        lsm303GaussLsbZ  = 600;
        break;
    case LSM303Gain_4_0:
        lsm303GaussLsbXY = 450;
        lsm303GaussLsbZ  = 400;
        break;
    case LSM303Gain_4_7:
        lsm303GaussLsbXY = 400;
        lsm303GaussLsbZ  = 355;
        break;
    case LSM303Gain_5_6:
        lsm303GaussLsbXY = 330;
        lsm303GaussLsbZ  = 295;
        break;
    case LSM303Gain_8_1:
        lsm303GaussLsbXY = 230;
        lsm303GaussLsbZ  = 205;
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
int LSM303Magnetometer::setRate(char rate)
{
    char reg = ((char)rate & 0x07) << 2;
    int status = write_register(LSM303RegisterMagCraReg_M, reg);
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
int LSM303Magnetometer::getEvent(magnetic_event_t *event) {
    bool readingValid = false;

    /* Clear the event */
    memset(event, 0, sizeof(magnetic_event_t));

    while(!readingValid) {

        char reg_mg = read_register(LSM303RegisterMagSRReg_Mg);
        if (!(reg_mg & 0x1)) {
            printf("%s read_register did not return expected value 0x1 (0x%x)\n", __func__, reg_mg);
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
            printf("%s: x : %d y : %d z : %d gain %d\n", __func__, raw.x, raw.y, raw.z, mGain);

            /* Check if the sensor is saturating or not */
            if ((raw.x >= 2040) | (raw.x <= -2040) | (raw.y >= 2040) | (raw.y <= -2040) | (raw.z >= 2040) | (raw.z <= -2040)) {
                /* saturated - increase the range if possible */
                switch(mGain) {
                case LSM303Gain_5_6:
                    setGain(LSM303Gain_8_1);
                    readingValid = false;
                    printf("changing range to +/- 8.1\n");
                    break;

                case LSM303Gain_4_7:
                    setGain(LSM303Gain_5_6);
                    readingValid = false;
                    printf("changing range to +/- 5.6\n");
                    break;

                case LSM303Gain_4_0:
                    setGain(LSM303Gain_4_7);
                    readingValid = false;
                    printf("changing range to +/- 4.7\n");
                    break;

                case LSM303Gain_2_5:
                    setGain(LSM303Gain_4_0);
                    readingValid = false;
                    printf("Changing range to +/- 4.0\n");
                    break;

                case LSM303Gain_1_9:
                    setGain(LSM303Gain_2_5);
                    readingValid = false;
                    printf("Changing range to +/- 2.5\n");
                    break;

                case LSM303Gain_1_3:
                    setGain(LSM303Gain_1_9);
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


    int status = gettimeofday(&event->timestamp, NULL);
    if (status < 0) {
        std::cerr << __func__ << ": gettimeofday failed with error " << status << std::endl;
        return -2;
    }

    event->magnetic.x   = (float)raw.x / lsm303GaussLsbXY * SensorGaussToMicroTesla;
    event->magnetic.y   = (float)raw.y / lsm303GaussLsbXY * SensorGaussToMicroTesla;
    event->magnetic.z   = (float)raw.z / lsm303GaussLsbZ  * SensorGaussToMicroTesla;

    return 0;
}
