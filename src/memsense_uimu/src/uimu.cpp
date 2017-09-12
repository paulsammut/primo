#include "serial/serial.h"
#include "uimu.h"
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <cstdio>
#include <sstream>

using std::vector;
using std::cout;

int UimuClass::connect(void)
{
    // These are used to iterate through the serial port and look for the Sabertooth
    // module. Taken from serial examples.
    vector<serial::PortInfo> devices_found = serial::list_ports();
    vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while(iter != devices_found.end())
    {
        serial::PortInfo device = *iter++;

        // Look for the sabertooth module.
        if(strstr(device.description.c_str(), "Silicon") != NULL)
        {
            ROS_INFO( "Found: (%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
                      device.hardware_id.c_str() );

            uimuSerPort.setPort(device.port.c_str());
            uimuSerPort.setBaudrate(UIMU_BAUD);

            // This gave me loads of trouble. You have to set a sensible timeout or
            // else the port won't work.
            serial::Timeout timeout = serial::Timeout::simpleTimeout(10);
            uimuSerPort.setTimeout(timeout);

            // Open the serial port
            uimuSerPort.open();
            if(uimuSerPort.isOpen())
            {
                ROS_INFO("UIMU port opened! %d baud, %s port",
                         uimuSerPort.getBaudrate(), uimuSerPort.getPort().c_str()) ;
            }
            else
            {
                ROS_ERROR("Port was not able to opened");
                return 0;
            }
        }
    }
    return 1;
}

UimuClass::UimuClass(void)
{
    validPacket = false;
}

UimuClass::~UimuClass(void)
{
    uimuSerPort.close();
}

void UimuClass::readPort(void)
{
    std::string tempData = uimuSerPort.read(5);

    std::vector<uint8_t> tempDataV(tempData.begin(), tempData.end());

    readBuffer.insert(readBuffer.end(), tempDataV.begin(), tempDataV.end()); 

    // ROS_INFO("Read: %lu, total size is: %lu", tempData.length(), readBuffer.size());

    std::vector<uint8_t>::iterator it;

    int ffCounter = 0;
    int startIndex = 0;
    bool foundSync = false;

    int i = 0;
    
    // ROS_INFO("Sweep");
    for(it=readBuffer.begin() ; it < readBuffer.end(); it++, i++)
    {
        // We have found the 4 sync bytes
        if(ffCounter == 4)
        {
            // foundSync = true;
            // startIndex = i-3;
            if(*it == 0x24)
            {
                foundSync = true;
                startIndex = i - 4;
            }
        } 

        // ROS_INFO("%x, %d, %d", static_cast<int>(*it), i, startIndex);
        if(*it == 0xff)
        {
            ffCounter += 1;
            // ROS_INFO("Found %d", ffCounter);
        }
        else
        {
            // ROS_INFO("Reset");
            ffCounter = 0;
        }
        
        if(foundSync)
        {
            // check to see if we have enough bytes to complete a packet
            if(readBuffer.size() >= (36 + startIndex))
            {

                // std::stringstream tempPacket;
                // int j = startIndex;
                // for(j=startIndex ; j < (startIndex+36) ; j++
                // {
                //     tempPacket << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(readBuffer[j]); 
                //     ROS_INFO("%d", j);
                // }
                // ROS_INFO("Last element: %x, %d, %d", static_cast<int>(readBuffer[j-1]), startIndex, j);
                // ROS_INFO("%s",tempPacket.str().c_str());

                rawPacket.assign(readBuffer[startIndex], readBuffer[startIndex+36]);
                foundSync = false;
                validPacket = true;
                readBuffer.erase(readBuffer.begin(), readBuffer.begin()+36);
                break;
            }
        }
    }

    if(validPacket)
    {
        decodePacket();
    }
}


void UimuClass::decodePacket(void)
{
}
