#pragma once

#include <iostream>

/**
 * @brief This is the data that comes out of the uimu
 */
class UimuPacket 
{
    public:
        float gyro_X;
        float gyro_Y;
        float gyro_Z;
        float acc_X;
        float acc_Y;
        float acc_Z;
        float mag_X;
        float mag_Y;
        float mag_Z;
}
       
