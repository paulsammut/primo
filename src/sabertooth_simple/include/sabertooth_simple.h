#pragma once

#include "serial/serial.h"
#include <iostream>

#define STOOTH_BAUD 9600

/**
 * \class SabertoothSimple
 *
 * \brief Simple driver for the sabertooth 2x** modules using Plain Text Serial. This
 * node uses the wjwood serial ros library.
 *
 * \author Paul Sammut
 *
 * Contact: paul@sammut-tech.com
 *
 */
class SabertoothSimple
{
    public:
        /**
         * @brief Sets the power output for either motor 1 or motor 2
         *
         * @param motorNum The number of the motor, either 1 or 2
         * @param powerVal Power value which goes from -2047 to 2047
         *
         * @return Returns 1 if successful, 0 if fail.
         */
        int setM(int motorNum, int powerVal);

        /**
         * @brief Gets the current power setting for either motor 1 or motor 2
         *
         * @param motorNum The number of the motor, either 1 or 2
         * @param powerVal Pointer to the power value which goes from -2047 to 2047
         *
         * @return Returns 1 if successful, 0 if fail.
         */
        int getM(int motorNum, int &powerVal);

        /**
         * @brief Default constructor that may or may not do anything
         */
        SabertoothSimple(void);

        /**
         * @brief Destructor that may or may not do anything
         */
        ~SabertoothSimple(void);

        /**
         * @brief Opens the port and connects to the sabertooth. Automatically looks
         * for the sabertooth driver :)
         *
         * @return 1 if success, 0 if failed
         */
        int connect(void);

        bool estop;

    private:
        /**
         * @brief This is the serial object that handles connection with the
         * sabertooth module
         */
        serial::Serial stoothSerPort;
};
