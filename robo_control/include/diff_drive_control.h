#ifndef DIFF_DRIVE_CONTROL_H
#define DIFF_DRIVE_CONTROL_H

#include "joystick.h"
#include "j1939_generate.h"
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <math.h>


namespace CanTransmission{
#define BYTE unsigned char

class DiffDriveControl{
    private:
        int test_;
        JOYSTICK_VALUE joystick_value_;

    public:
        void joystick_value_set_default();
        void print_test();
        void twist_to_joystick(double v_x, double yaw, double max_twist_speed);
        STICK_VALUE get_left_stick();
        STICK_VALUE get_right_stick();
        void int_to_hex(int value, BYTE* value_h,int size_byte);

};

}

#endif