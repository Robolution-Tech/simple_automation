#include "ros/ros.h"
//#include "std_msgs/String.h"
#include "j1939_generate.h"
#include "diff_drive_control.h"
#include <chrono>
#include <mutex>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Bool.h"

//basic file operations
#include <iostream>
#include <fstream>


namespace CanTransmission{

#define NUM_THREADS 3
#define JOYSTICK_EM_STOP 6

enum Keyboard{
    l_left = 'a',
    l_right = 'd', 
    l_forward = 'w',
    l_back = 's',

    r_left = 'j',
    r_right = 'l', 
    r_forward = 'i',
    r_back = 'k',

    em_stop = 'q'
};



class CAN_Node{
    private:
        ros::NodeHandle nl_;
        ros::Subscriber keyboard_sub_;
        ros::Subscriber twist_sub_;
        ros::Subscriber hardcode_loading_sub_;
        ros::Publisher loading_flag_pub_;
        CAN_Dev device_;
        Joystick js_;
        DiffDriveControl diff_drive_control_;
        BYTE joystick_value_[8]; //mimic equipment joystic value
        js_state js_state_;
        std::mutex js_state_mtx_;

        std::ofstream js_record_file_write_; //loading record file, write
        std::ifstream js_record_file_read_; //loading record file, read
        std::string joystick_record_file_dir_; //file dir
        std::string hardcode_flag_topic_; //loading flag //TODO: use action lib for loading operation
        bool hardcode_flag_;
        std::string loading_complete_flag_;
        


    public:

        std::string get_control_input_type(){
            return device_.get_control_input_type();
        }

        void joystick_set_default(){
            joystick_value_[0] = 0x00;
            joystick_value_[1] = 0x02;
            joystick_value_[2] = 0x00;
            joystick_value_[3] = 0x02;
            joystick_value_[4] = 0x00;
            joystick_value_[5] = 0x02;
            joystick_value_[6] = 0x00;
            joystick_value_[7] = 0x02;
        }

        void js_set_default(){
            js_state_.axis[0].x = NEUTRAL_VALUE;
            js_state_.axis[0].y = NEUTRAL_VALUE;
            js_state_.axis[1].x = NEUTRAL_VALUE;
            js_state_.axis[1].y = NEUTRAL_VALUE;
            js_state_.axis[2].x = NEUTRAL_VALUE;
            js_state_.axis[2].y = NEUTRAL_VALUE;
        }

        void int_to_hex(int value, BYTE* value_h,int size_byte)
        {
            memcpy(value_h,&value,size_byte);//size is the byte size
            // printf("int size %d, value size %d \n",sizeof(int),sizeof(value));
            // printf("int to hex %02x, %02x, %02x, %02x,%02x, %02x, %02x, %02x\n",value_h[0],value_h[1], value_h[2],value_h[3],value_h[4],value_h[5],value_h[6],value_h[7]);
        }

        bool split(const std::string& string_in, char delim, std::vector<std::string>& tokens_out) 
        { 
            /**
             * @brief 
             * 
             * @return std::istringstream , if the string exist return true, else return false
             */
            std::istringstream iss(string_in); 
            std::string token; 
            if (string_in.length() == 0){
                return false;
            }else{
                while (std::getline(iss, token, delim)) { 
                    tokens_out.push_back(token); 
                    } 
                return true;
            }
        }

        void TwistSubscribe(){
            keyboard_sub_ = nl_.subscribe("cmd_vel", 1000, &CAN_Node::TwistCallback,this); //TODO: add topic name to config
            
            js_record_file_read_.open(joystick_record_file_dir_);

            while(1){ //TODO: after the boom control is added, this while(1) need to be moved to somewhere else

                if (hardcode_flag_ == true){
                    if (js_record_file_read_.is_open())
                    {
                        std::string line;
                        getline (js_record_file_read_,line);
                        
                        std::vector<std::string> vec_out;
                        if (split(line,' ',vec_out)){
                            //l l-r
                            int_to_hex(atoi( (vec_out.at(0).c_str()) ),&joystick_value_[4],2);
                            //l f-b
                            int_to_hex(atoi( (vec_out.at(1).c_str()) ),&joystick_value_[6],2);
                            //r l-r
                            int_to_hex(atoi( (vec_out.at(2).c_str()) ),&joystick_value_[0],2);
                            //r f-b
                            int_to_hex(atoi( (vec_out.at(3).c_str()) ),&joystick_value_[2],2);
                        }else{
                            hardcode_flag_ = false;
                            std_msgs::Bool loading_complete;
                            loading_complete.data = hardcode_flag_;
                            loading_flag_pub_.publish(true);
                        }
                    }
                }

                device_.j1939_send(0,CANControlInputId,joystick_value_);     
                msleep(40);
            }
            printf("automation done");
            int ret0 = 0;
            pthread_exit(&ret0);
        }

        void TwistCallback(const geometry_msgs::Twist::ConstPtr& msg){
            /**
             * @brief takse twist msg to generate equipment joystick control signals
             * 
             */
            float v_x = msg->linear.x;
            float yaw = msg->angular.z;
            float twist_max_speed = device_.get_twist_max_speed();
            diff_drive_control_.twist_to_joystick(v_x, yaw , twist_max_speed);
            STICK_VALUE left_stick = diff_drive_control_.get_left_stick();

            //forward & back -- x-axis
            int_to_hex(left_stick.x,&joystick_value_[6],2);

            //left & right -- y-axis
            int_to_hex(left_stick.y,&joystick_value_[4],2);
        }


        void KeyboardSubscribe(){
            keyboard_sub_ = nl_.subscribe("keys", 1000, &CAN_Node::KeyboardCallback,this);

            while(1){
            //for(int i = 0; i < 3000; i++){ 
                device_.j1939_send(0,CANControlInputId,joystick_value_);
                if(device_.get_keyboard_mode() == keyboard_mode.discrete){
                    joystick_set_default();
                }
                
                msleep(40);
            }
            printf("keyboard done");
            int ret0 = 0;
            pthread_exit(&ret0);
        }


        void KeyboardCallback(const std_msgs::String::ConstPtr& msg){
            /****
             *Left joystick:  "a" left, "d" right, "w" forward, "s" back
             *right joystick: "j" left, "l" right, "i" forward, "k" back
             *
             * ****/

            if(device_.get_keyboard_mode() == keyboard_mode.continues){
                joystick_set_default();
            }

            ROS_INFO("I heard:[%s]",msg->data.c_str());

            //add speed pid for keyboard control input
            //Joystick

            char key = msg->data.c_str()[0];

            switch(key){
                case l_left:
                    int_to_hex(4,&joystick_value_[4],2);
                    break;

                case l_right:
                    int_to_hex(1020,&joystick_value_[4],2);
                    break;
                case l_forward:
                    int_to_hex(1020,&joystick_value_[6],2);
                    break;

                case l_back:
                    int_to_hex(4,&joystick_value_[6],2);
                    break;

                case r_left:
                    int_to_hex(4,&joystick_value_[0],2);
                    break;

                case r_right:
                    int_to_hex(1020,&joystick_value_[0],2);
                    break;
                case r_forward:
                    int_to_hex(4,&joystick_value_[2],2);
                    break;

                case r_back:
                    int_to_hex(1020,&joystick_value_[2],2);
                    break;
                case em_stop:
                    printf("em pressed");
                    exit(1);
                default:
                    joystick_set_default();
                    break;

            
            }
        }

        void Remote_joystick_read(){
            if (device_.get_joystick_record_flag() == true){
                js_record_file_write_.open(joystick_record_file_dir_, std::ios_base::app);
            }
            while(js_.device_plugged()){
                js_state_mtx_.lock();
                //joystick_set_default();
                js_set_default();
                js_state_ = js_.device_read_value();
                
                std::cout << "-----" << device_.get_joystick_record_flag()<<" -----" << std::endl;
                //record joystick value
                if (device_.get_joystick_record_flag() == true){
                    js_record_file_write_ << js_state_.axis[0].x << " " << js_state_.axis[0].y << " " <<
                    js_state_.axis[1].x << " " << js_state_.axis[1].y << "\n";
                }

                //check em stop
                // printf("Button %u %d\n", js_state_.button.number, js_state_.button.pressed );
                if(js_state_.button.number == JOYSTICK_EM_STOP && js_state_.button.pressed == 1){
                    printf("em pressed");
                    
                    exit(1);
                }
                //l l-r
                int_to_hex(js_state_.axis[0].x,&joystick_value_[4],2);
                //l f-b
                int_to_hex(js_state_.axis[0].y,&joystick_value_[6],2);
                //r l-r
                int_to_hex(js_state_.axis[1].x,&joystick_value_[0],2);
                //r f-b
                int_to_hex(js_state_.axis[1].y,&joystick_value_[2],2);

                js_state_mtx_.unlock();
                printf("Axis %d at (%6d, %6d)\n", 0, js_state_.axis[0].x, js_state_.axis[0].y);
                printf("Axis %d at (%6d, %6d)\n", 1, js_state_.axis[1].x, js_state_.axis[1].y);
                printf("Axis %d at (%6d, %6d)\n", 2, js_state_.axis[2].x, js_state_.axis[2].y);
            }

        }

        void control_send(){
            while(1){          
                js_state_mtx_.lock(); 
                device_.j1939_send(0,CANControlInputId,joystick_value_);
                js_state_mtx_.unlock();

                msleep(40);
            }
        }

        void engine_start_run(){
            device_.Remote_Init();
            device_.Ignition();
            int ret1 = 1;
            pthread_exit(&ret1);
        }
        

        static void *Twist_thread_init(void *CAN_Node_c){
            ((CAN_Node *)CAN_Node_c)->TwistSubscribe();
        
        }
        static void *Keyboard_thread_init(void *CAN_Node_c){
            ((CAN_Node *)CAN_Node_c)->KeyboardSubscribe();
        
        }
        static void *Joystick_thread_init(void *CAN_Node_c){
            ((CAN_Node *)CAN_Node_c)->Remote_joystick_read();
        
        }
        static void *control_send_thread_init(void *CAN_Node_c){
            ((CAN_Node *)CAN_Node_c)->control_send();
        
        }
        static void *Engine_thread_init(void *CAN_Node_c){
            ((CAN_Node *)CAN_Node_c)->engine_start_run();
        
        }

        
        void get_param(){
            nl_.param<std::string>("/robo_param/flag_topic/hardcode_loading_flag", hardcode_flag_topic_, "hardcode_loading_flag");
            nl_.param<std::string>("/robo_param/flag_topic/loading_complete_flag", loading_complete_flag_, "loading_complete_flag");
            nl_.param<std::string>("/robo_param/file_dir/joystick_record_file_dir", joystick_record_file_dir_,"/home/robo-dell/Downloads/js_record.txt");
            loading_flag_pub_ = nl_.advertise<std_msgs::Bool>(loading_complete_flag_,10,true);
            HardcodeFlagSubscribe();
        }

                
        void HardcodeFlagSubscribe(){
            hardcode_loading_sub_ = nl_.subscribe(hardcode_flag_topic_, 1000, &CAN_Node::hardcode_flag_topic_callback,this); 
        }

        void hardcode_flag_topic_callback(const std_msgs::Bool::ConstPtr& msg){
            hardcode_flag_ = msg->data;
            std::cout << "hardcode flag is: " << hardcode_flag_ << std::endl;
        }

        CAN_Node(): device_(config_path){
            get_param();
            joystick_set_default();
            js_set_default();
            device_.show_param();
            diff_drive_control_.print_test();

        }

        ~CAN_Node(){
            device_.CloseDevice();
            js_record_file_write_.close();
            js_record_file_read_.close();
        }

};
}

using namespace CanTransmission;
int main(int argc, char **argv){
    // msleep(10000); // wait until everyone leaves the equipment 10sec
    pthread_t threads[NUM_THREADS];

    ros::init(argc, argv,"CAN_node");
    CAN_Node can_node;
    int rc;

    ///thread 0 init for machine ingnition 
    rc = pthread_create(&threads[0],NULL,&CAN_Node::Engine_thread_init,&can_node); 
    if(rc){
        printf("error: unable to create thread 0\n");
        exit(-1);
    }

    if (can_node.get_control_input_type() == control_input_type.joystick){
        printf("Accept joystick control input");
        //thread: init for joystick remote controller
        rc = pthread_create(&threads[1],NULL,&CAN_Node::Joystick_thread_init,&can_node); 
        if(rc){
            printf("error: unable to create thread 1\n");
            exit(-1);
        }

    }else if (can_node.get_control_input_type() == control_input_type.keyboard)
    {
        printf("Accept keyboard control input \n");
        //thread 1 init for keyboard input
        rc = pthread_create(&threads[1],NULL,&CAN_Node::Keyboard_thread_init,&can_node); 
        if(rc){
            printf("error: unable to create thread 1\n");
            exit(-1);
        }
    }else if (can_node.get_control_input_type() == control_input_type.twist)
    {
        printf("Accept Twist control input \n");
        //thread 1 init for keyboard input
        rc = pthread_create(&threads[1],NULL,&CAN_Node::Twist_thread_init,&can_node); 
        if(rc){
            printf("error: unable to create thread 1\n");
            exit(-1);
        }
    }else{
        std::cout<< "the control input is " << can_node.get_control_input_type() <<std::endl;
        printf("invalid control input parameter, please use: keyboard, joystick or twist\n");
    }
    
        //thread: send control signal
    rc = pthread_create(&threads[2],NULL,&CAN_Node::control_send_thread_init,&can_node); 
    if(rc){
        printf("error: unable to create thread 2\n");
        exit(-1);
    }



    /////thread 1 init for keyboard input
    //rc = pthread_create(&threads[1],NULL,&CAN_Node::Keyboard_thread_init,&can_node); 
    //if(rc){
    //    printf("error: unable to create thread 1\n");
    //    exit(-1);
    //}


    
    //pthread_join(threads[0],NULL);
    //pthread_join(threads[1],NULL);
    //printf("thread join");

    //can_node.engine_start_run(); 
    //can_node.KeyboardSubscribe();
    ros::spin();

    return 0;
}
