#include "serial/serial.h"
#include "sabertooth_simple.h"
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <cstdio>
#include <sstream>

using std::vector;
using std::cout;

int SabertoothSimple::setM(int motorNum, int powerVal)
{
    // First make sure serial port is open
    if(stoothSerPort.isOpen()) {
        // Formulate our command string
        std::string cmdString;
        if(motorNum==1)
            cmdString = "M1:";
        else if(motorNum==2)
            cmdString = "M2:";
        else
            return 0;

        // Estop test
        if(estop)
           powerVal = 0;

        std::ostringstream cmdNum;
        cmdNum << powerVal;
        cmdString += cmdNum.str();
        cmdString += "\r\n";
        stoothSerPort.write(cmdString);
        return 1;
    } else
        return 0;
}


int SabertoothSimple::getM(int motorNum, int &powerVal)
{
    // Not yet implemented
    powerVal = 69;
    return 1;
}

int SabertoothSimple::connect(void)
{
    // These are used to iterate through the serial port and look for the Sabertooth
    // module. Taken from serial examples.
    vector<serial::PortInfo> devices_found = serial::list_ports();
    vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while(iter != devices_found.end())
    {
        serial::PortInfo device = *iter++;

        // Look for the sabertooth module.
        if(strstr(device.description.c_str(), "Sabertooth") != NULL)
        {
            ROS_INFO( "Found: (%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
                      device.hardware_id.c_str() );

            stoothSerPort.setPort(device.port.c_str());
            stoothSerPort.setBaudrate(STOOTH_BAUD);

            // This gave me loads of trouble. You have to set a sensible timeout or
            // else the port won't work.
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            stoothSerPort.setTimeout(timeout);

            // Open the serial port
            stoothSerPort.open();
            if(stoothSerPort.isOpen())
            {
                ROS_INFO("Sabertooth port opened! %d baud, %s port",
                         stoothSerPort.getBaudrate(), stoothSerPort.getPort().c_str()) ;
            }
            else
            {
                ROS_ERROR("Port was not able to opened");
                return 0;
            }

            std::string daters = "M1:0\r\nM2:0\r\n";
            size_t writtenBees = stoothSerPort.write(daters);
            // ROS_INFO("Wrote %lu bytes", writtenBees);
        }
    }
    return 1;
}

SabertoothSimple::SabertoothSimple(void)
{
    estop = false;
}

SabertoothSimple::~SabertoothSimple(void)
{
    setM(1,0);
    setM(2,0);
    stoothSerPort.close();
}
