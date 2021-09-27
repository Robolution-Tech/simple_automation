#ifndef J1939_GENERATE_H
#define J1939_GENERATE_H

#   include <stdio.h>
#   include <stdlib.h>
#   include <string.h>
#   include <strings.h>
#   include <sstream>
#   include <fstream>
#   include <unistd.h>
#   include <sys/types.h>
#   include <sys/stat.h>
#   include <fcntl.h>
#   include <pthread.h>
#   include "controlcan.h"
#   include "joystick.h"
#   include "json.hpp"



#   include "ros/ros.h"
#   include "std_msgs/String.h"


using json = nlohmann::json; 


namespace CanTransmission{

#   define msleep(ms)  usleep((ms)*1000)

#   define CANControlInputId 0x0CFF2A80 //todo: add to config




const std::string config_path = "/home/robo-dell/catkin_ws/src/simple_automation/robo_control/src/config/CAN_config.json";
// const std::string config_path = __FILE__;

inline struct CONTROL_INPUT_TYPE{
    std::string keyboard = "keyboard";
    std::string joystick = "joystick";
    std::string twist = "twist";
} control_input_type;


inline struct KEYBOARD_MODE{
    std::string continues = "continues";
    std::string discrete = "discrete";
} keyboard_mode;


struct STICK_VALUE{
    unsigned x;
    unsigned y;
};

struct JOYSTICK_VALUE{
    STICK_VALUE left_stick;
    STICK_VALUE right_stick;
};

class CAN_Dev{
        unsigned gDevType_; // 4-usbcan-ii, 5-pci9820, 14-pci9840, 16-pci9820i, ....
        unsigned gDevIdx_; //device core, always 0 
        unsigned gChMask_; //3=> 11 means both channel are enabled
        unsigned gBaud_; // 0x1400-1M, 0x1c03-125K, ....
        unsigned gTxType_; //0-normal, 1-single, 2-self_test, 3-single_self_test, 4-single_no_wait....
        unsigned gTxSleep_; // sleep in ms
        unsigned gTxFrames_; // number of frames once
        unsigned gTxCount_; // how many times
        std::string ControlInputType_;
        std::string keyboard_mode_;
        double twist_max_speed_;
        bool joystick_pub_flag_;
        VCI_INIT_CONFIG config_;
    public:
        json configJson_;
        CAN_Dev();
        CAN_Dev(const std::string);
        CAN_Dev(unsigned gDevType, unsigned gDevIdx, unsigned gBaud);

        //~CAN_Dev();
        //{
        //
        //    VCI_CloseDevice(gDevType_, gDevIdx_);
        //}

        void show_param();

        void generate_frame(VCI_CAN_OBJ *can,unsigned ID, BYTE* value_h);

        // void int_to_hex(int value, BYTE* value_h,int size_byte);

        void j1939_send(int channel_num, unsigned ID, BYTE* value);

        void CloseDevice();
            
        void KeyboardCallback(const std_msgs::String::ConstPtr& msg);

        void Remote_joystick_read();

        void Ignition();

        void Remote_Init();

        void config_parse(const std::string);
        
        std::string get_control_input_type(){
            return ControlInputType_;
        }

        std::string get_keyboard_mode(){
            return keyboard_mode_;
        }
        double get_twist_max_speed(){
            return twist_max_speed_;
        }
        double get_joystick_pub_flag(){
            return joystick_pub_flag_;
        }
        
};

}
#endif
