#pragma once

#include "serial/serial.h"
#include "uimu_packet.h"
#include <iostream>
#include <vector>
#include "sensor_msgs/Imu.h"
#include <sensor_msgs/MagneticField.h>
#include <ros/ros.h>

#define UIMU_BAUD 57600

const float G = 9.81;

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


        std::string frame_id_;
        double angular_velocity_stdev_;
        double linear_acceleration_stdev_;
        double magnetic_field_stdev_;

        ros::Publisher  imu_publisher;
        ros::Publisher  mag_publisher;

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

        UimuPacket uimuTempPacket;
        sensor_msgs::Imu imu_msg;
        sensor_msgs::MagneticField mag_msg;
        
        std::vector<uint8_t> readBuffer;
        std::vector<uint8_t> rawPacket;

        bool validPacket;

        /**
         * @brief Decodes the packet
         */
        void decodePacket(void);

        void processPacket(void);

        void setRawPacket(std::vector<uint8_t> &p_vect);
};
