#include "diff_drive_control.h"


template <typename T>
T clamp(const T& n, const T& lower, const T& upper) {
  return std::max(lower, std::min(n, upper));
}




namespace CanTransmission{

    
    void DiffDriveControl::print_test(){
        test_ = 1;
        printf(" Diff Drive control lib, test is %d \n", test_);
    }

    void DiffDriveControl::joystick_value_set_default(){
        joystick_value_.left_stick.x = NEUTRAL_VALUE;
        joystick_value_.left_stick.y = NEUTRAL_VALUE;
        joystick_value_.right_stick.x = NEUTRAL_VALUE;
        joystick_value_.right_stick.y = NEUTRAL_VALUE;
    }

    void DiffDriveControl::int_to_hex(int value, BYTE* value_h,int size_byte)
    {
                memcpy(value_h,&value,size_byte);//size is the byte size
                // printf("int size %d, value size %d \n",sizeof(int),sizeof(value));
                // printf("int to hex %02x, %02x, %02x, %02x,%02x, %02x, %02x, %02x\n",value_h[0],value_h[1], value_h[2],value_h[3],value_h[4],value_h[5],value_h[6],value_h[7]);
    }
    
    void DiffDriveControl::twist_to_joystick(double v_x, double yaw, double max_twist_speed){

        double scale = v_x / max_twist_speed;
        int x = (int)(cos(yaw) * NEUTRAL_VALUE * scale); 
        int y = (int)(sin(yaw) * NEUTRAL_VALUE * scale); 
        
        x = x + NEUTRAL_VALUE;
        y = y + NEUTRAL_VALUE;

        x = clamp(x, MIN_VALUE, MAX_VALUE);
        y = clamp(y, MIN_VALUE, MAX_VALUE);

        std::cout << "linear speed x is " << v_x << std::endl;
        std::cout << "angular speed z is " <<yaw << std::endl;
        std::cout << "joystick x value is " <<x << std::endl;
        std::cout << "joystick y value is " <<y << std::endl;


        joystick_value_.left_stick.x = x;
        joystick_value_.left_stick.y = y;
    }




    STICK_VALUE DiffDriveControl::get_left_stick(){
        return joystick_value_.left_stick;
    }
    
    STICK_VALUE DiffDriveControl::get_right_stick(){
        return joystick_value_.right_stick;
    }


}