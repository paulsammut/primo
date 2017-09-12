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

    validPacket = false;
    
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
                std::vector<uint8_t>::const_iterator first = readBuffer.begin()+startIndex;
                std::vector<uint8_t>::const_iterator last = readBuffer.begin()+startIndex+36;
                std::vector<uint8_t> tempV(first,last);
                setRawPacket(tempV);
                break;
            }
        }
    }

    if(validPacket)
    {
        validPacket = false;
        decodePacket();
    }
}


void UimuClass::decodePacket(void)
{
    // std::stringstream tempPacket;
    // for(int j=0; j < rawPacket.size(); j++)
    // {
    //     tempPacket << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(rawPacket[j]); 
    // }
    // ROS_INFO("%s",tempPacket.str().c_str());

    short int temp_val;

    // Gyro X
    temp_val = static_cast<short int>(rawPacket[13])<<8;
    temp_val += rawPacket[14];
    uimuTempPacket.gyro_X = (static_cast<float>(temp_val)) * (600/2*1.5/32768);
    
    // Gyro Y
    temp_val = static_cast<short int>(rawPacket[15])<<8;
    temp_val += rawPacket[16];
    uimuTempPacket.gyro_Y = (static_cast<float>(temp_val)) * (600/2*1.5/32768);
    
    // Gyro Z
    temp_val = static_cast<short int>(rawPacket[17])<<8;
    temp_val += rawPacket[18];
    uimuTempPacket.gyro_Z = (static_cast<float>(temp_val)) * (600/2*1.5/32768);

    // Acc X
    temp_val = static_cast<short int>(rawPacket[19])<<8;
    temp_val += rawPacket[20];
    uimuTempPacket.acc_X = (static_cast<float>(temp_val)) * (10/2*1.5/32768);

    // Acc Y
    temp_val = static_cast<short int>(rawPacket[21])<<8;
    temp_val += rawPacket[22];
    uimuTempPacket.acc_Y = (static_cast<float>(temp_val)) * (10/2*1.5/32768);

    // Acc Z
    temp_val = static_cast<short int>(rawPacket[23])<<8;
    temp_val += rawPacket[24];
    uimuTempPacket.acc_Z = (static_cast<float>(temp_val)) * (10/2*1.5/32768);
    
    // Mag X
    temp_val = static_cast<short int>(rawPacket[25])<<8;
    temp_val += rawPacket[26];
    uimuTempPacket.mag_X = (static_cast<float>(temp_val)) * (2.8/2*1.5/32768);

    // Mag Y
    temp_val = static_cast<short int>(rawPacket[27])<<8;
    temp_val += rawPacket[28];
    uimuTempPacket.mag_Y = (static_cast<float>(temp_val)) * (2.8/2*1.5/32768);

    // Mag Z
    temp_val = static_cast<short int>(rawPacket[29])<<8;
    temp_val += rawPacket[30];
    uimuTempPacket.mag_Z = (static_cast<float>(temp_val)) * (2.8/2*1.5/32768);

    ROS_INFO("%f, %f, %f, %f, %f, %f, %f, %f, %f",
            uimuTempPacket.gyro_X,
            uimuTempPacket.gyro_Y,
            uimuTempPacket.gyro_Z,
            uimuTempPacket.acc_X,
            uimuTempPacket.acc_Y,
            uimuTempPacket.acc_Z,
            uimuTempPacket.mag_X,
            uimuTempPacket.mag_Y,
            uimuTempPacket.mag_Z
            );
}

void UimuClass::setRawPacket(std::vector<uint8_t> &p_vect)
{
    rawPacket = p_vect;

}


