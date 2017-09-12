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
            ROS_INFO( "Found: (%s, %s, %s)\n", 
                    device.port.c_str(), device.description.c_str(),

            device.hardware_id.c_str() );

            uimuSerPort.setPort(device.port.c_str());
            uimuSerPort.setBaudrate(UIMU_BAUD);

            // This gave me loads of trouble. You have to set a sensible timeout or
            // else the port won't work.
            serial::Timeout timeout = serial::Timeout::simpleTimeout(50);
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
    numBytes = 1;

    frame_id_ = "imu";
    angular_velocity_stdev_ = 0.5 * (M_PI / 180.0); // 0.5 deg/s noise
    linear_acceleration_stdev_ = 5.0 * 1e-3 * G; // 5 mg
    magnetic_field_stdev_ = 5.6 * 1e-3 * 1e-4; // 5.6 milli gauss converted to teslas

    // ---- imu message

    imu_msg.header.frame_id = frame_id_;

    // build covariance matrices
   
    double ang_vel_var = angular_velocity_stdev_ * angular_velocity_stdev_;
    double lin_acc_var = linear_acceleration_stdev_ * linear_acceleration_stdev_;
    for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
    {
      int idx = j*3 +i;
  
      if (i == j)
      {
        imu_msg.angular_velocity_covariance[idx]    = ang_vel_var;
        imu_msg.linear_acceleration_covariance[idx] = lin_acc_var;
      }
      else
      {
        imu_msg.angular_velocity_covariance[idx]    = 0.0;
        imu_msg.linear_acceleration_covariance[idx] = 0.0;
      }
    }
  
    // ---- magnetic field message
  
    mag_msg.header.frame_id = frame_id_;
  
    // build covariance matrix
  
    double mag_field_var = magnetic_field_stdev_ * magnetic_field_stdev_;
  
    for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
    {
      int idx = j * 3 + i;
  
      if (i == j)
      {
        mag_msg.magnetic_field_covariance[idx] = mag_field_var;
      }
      else
      {
        mag_msg.magnetic_field_covariance[idx] = 0.0;
      }
    }
}

UimuClass::~UimuClass(void)
{
    uimuSerPort.close();
}

void UimuClass::readPort(void)
{
    std::string tempData = uimuSerPort.read(numBytes);

    std::vector<uint8_t> tempDataV(tempData.begin(), tempData.end());

    readBuffer.insert(readBuffer.end(), tempDataV.begin(), tempDataV.end()); 

    // ROS_INFO("Read: %lu, total size is: %lu", tempData.length(), readBuffer.size());

    std::vector<uint8_t>::iterator it;

    int ffCounter = 0;
    int startIndex = 0;
    bool foundSync = false;

    int i = 0;

    validPacket = false;
    
    // ROS_INFO("Sweep with buffer length: %lu", readBuffer.size());
    for(it=readBuffer.begin() ; it < readBuffer.end(); it++, i++)
    {
        // We have found the 4 sync bytes
        if(ffCounter == 4)
        {
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
                rawPacket.assign(readBuffer[startIndex], readBuffer[startIndex+36]);
                foundSync = false;
                validPacket = true;
                readBuffer.erase(readBuffer.begin(), readBuffer.begin()+36);
                std::vector<uint8_t>::const_iterator first = readBuffer.begin()+startIndex;
                std::vector<uint8_t>::const_iterator last = readBuffer.begin()+startIndex+36;
                std::vector<uint8_t> tempV(first,last);
                setRawPacket(tempV);
                // set the read bytes to 36 now
                if(numBytes == 1)
                    readBuffer.clear();
                numBytes = 36;
                break;
            }
        }
    }

    if(validPacket)
    {
        validPacket = false;
        decodePacket();
        processPacket();
    }
}


void UimuClass::decodePacket(void)
{
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
    uimuTempPacket.mag_X = (static_cast<float>(temp_val)) * (3.8/2*1.5/32768);

    // Mag Y
    temp_val = static_cast<short int>(rawPacket[27])<<8;
    temp_val += rawPacket[28];
    uimuTempPacket.mag_Y = (static_cast<float>(temp_val)) * (3.8/2*1.5/32768);

    // Mag Z
    temp_val = static_cast<short int>(rawPacket[29])<<8;
    temp_val += rawPacket[30];
    uimuTempPacket.mag_Z = (static_cast<float>(temp_val)) * (3.8/2*1.5/32768);

    // Print out the packet
    // ROS_INFO("%f, %f, %f, %f, %f, %f, %f, %f, %f",
    //         uimuTempPacket.gyro_X,
    //         uimuTempPacket.gyro_Y,
    //         uimuTempPacket.gyro_Z,
    //         uimuTempPacket.acc_X,
    //         uimuTempPacket.acc_Y,
    //         uimuTempPacket.acc_Z,
    //         uimuTempPacket.mag_X,
    //         uimuTempPacket.mag_Y,
    //         uimuTempPacket.mag_Z
    //         );
}

void UimuClass::setRawPacket(std::vector<uint8_t> &p_vect)
{
    rawPacket = p_vect;
}

void UimuClass::processPacket(void)
{
    // set the times
    ros::Time time_now = ros::Time::now();
    imu_msg.header.stamp = time_now;
    mag_msg.header.stamp = time_now;
  
    
    // set linear acceleration
    imu_msg.linear_acceleration.x = - uimuTempPacket.acc_X * G;
    imu_msg.linear_acceleration.y = - uimuTempPacket.acc_Y * G;
    imu_msg.linear_acceleration.z = - uimuTempPacket.acc_Z * G;
  
    // set angular velocities
    imu_msg.angular_velocity.x = uimuTempPacket.gyro_X  * (M_PI / 180.0);
    imu_msg.angular_velocity.y = uimuTempPacket.gyro_Y  * (M_PI / 180.0);
    imu_msg.angular_velocity.z = uimuTempPacket.gyro_Z  * (M_PI / 180.0);
  
    // mag message
    // device reports data in Gauss, multiply by 1e-4 to convert to Tesla
    mag_msg.magnetic_field.x = uimuTempPacket.mag_X * 1e-4;
    mag_msg.magnetic_field.y = uimuTempPacket.mag_Y * 1e-4;
    mag_msg.magnetic_field.z = uimuTempPacket.mag_Z * 1e-4;

    imu_publisher.publish(imu_msg);
    mag_publisher.publish(mag_msg);
}
