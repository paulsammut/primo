#pragma once

#include "serial/serial.h"
#include <iostream>
#include <vector>

#define UIMU_BAUD 57600

/**
 * \class UIMU Class
 *
 * \brief This driver reads the serial port and decodes uimu packets
 *
 * \author Paul Sammut
 *
 * Contact: paul@sammut-tech.com
 *
 */
class UimuClass
{
    public:

        /**
         * @brief Opens the port and connects to the sabertooth. Automatically looks
         * for the sabertooth driver :)
         *
         * @return 1 if success, 0 if failed
         */
        int connect(void);

        /**
         * @brief Don't think we do anything here
         */
        UimuClass(void);

        
        /**
         * @brief Deconstructor in which we close the port
         */
        ~UimuClass(void);

        void readPort(void);

    private:
        /**
         * @brief This is the serial object that handles connection with the
         * sabertooth module
         */
        serial::Serial uimuSerPort;
        
        std::vector<uint8_t> readBuffer;

        bool validPacket;
        bool foundSync;

};
