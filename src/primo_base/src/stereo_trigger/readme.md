# Stereo Trigger

This is an arduino device that generates a 12V trigger for the stereo cameras.
It does so by using a ULN2308 with 12V connected to its common line.

## Software Notes

Didn't use parameters before they were proving to be a PITA with the arduino. 
Used the hardware timer 1 on the arduino. This may cause interference with
sycing to ros, but it seems to work OK when used under 20 Hz.

## Schematic


                                                                 +12V
                                                                 |
                                                                 /
                                                                 \
                                                                 /   
            Arduino                                              \    
         +----------------+               ULN2803                |
         |                |            +------------------+      |
        -|-USB       Pin2-+------------+-1(1b)   18(1c)  -+------+------Trigger
         |                |            |                  |
         |           Gnd -+------------+-9(Gnd)  10(Com) -+--------------+12V
         |                |            |                  |
         |                |            |                  |
         |                |            +------------------+
         +----------------+
