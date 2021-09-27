/**
 * Author: Jason White
 *
 * Description:
 * Reads joystick/gamepad events and displays them.
 *
 * Compile:
 * gcc joystick.c -o joystick
 *
 * Run:
 * ./joystick [/dev/input/jsX]
 *
 * See also:
 * https://www.kernel.org/doc/Documentation/input/joystick-api.txt
 */
#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/joystick.h>


namespace CanTransmission{

#define MAX_VALUE 1020
#define MIN_VALUE 4 
#define NEUTRAL_VALUE 512


inline int read_event(int fd, struct js_event *event)
{
    ssize_t bytes;

    bytes = read(fd, event, sizeof(*event));

    if (bytes == sizeof(*event))
        return 0;

    /* Error, could not read full event. */
    return -1;
}

/**
 * Returns the number of axes on the controller or 0 if an error occurs.
 */
inline size_t get_axis_count(int fd)
{
    __u8 axes;

    if (ioctl(fd, JSIOCGAXES, &axes) == -1)
        return 0;

    return axes;
}

/**
 * Returns the number of buttons on the controller or 0 if an error occurs.
 */
inline size_t get_button_count(int fd)
{
    __u8 buttons;
    if (ioctl(fd, JSIOCGBUTTONS, &buttons) == -1)
        return 0;

    return buttons;
}

/**
 * Current state of an axis.
 */
struct axis_state {
    short x, y;
};

struct button_state{
    unsigned number;
    int pressed;
};

struct js_state{
    axis_state axis[3];
    button_state button;

};
/**
 * Keeps track of the current axis state.
 *
 * NOTE: This function assumes that axes are numbered starting from 0, and that
 * the X axis is an even number, and the Y axis is an odd number. However, this
 * is usually a safe assumption.
 *
 * Returns the axis that the event indicated.
 */
inline size_t get_axis_state(struct js_event *event, struct axis_state axes[3])
{
    size_t axis = event->number / 2;

    if (axis < 3)
    {
        if (event->number % 2 == 0)
            axes[axis].x = event->value;
        else
            axes[axis].y = event->value;
    }

    return axis;
}

class Joystick{
    private:

        const char *device_;
        int js_;
        struct js_event event_;
        struct axis_state axes_[3];
        struct js_state js_state_;
        size_t axis_;

    public:
        Joystick():device_("/dev/input/js0"){

            axes_[0].x = NEUTRAL_VALUE;
            axes_[0].y = NEUTRAL_VALUE;
            axes_[1].x = NEUTRAL_VALUE;
            axes_[1].y = NEUTRAL_VALUE;
            axes_[2].x = NEUTRAL_VALUE;
            axes_[2].y = NEUTRAL_VALUE;

            js_state_.axis[0].x = NEUTRAL_VALUE;
            js_state_.axis[0].y = NEUTRAL_VALUE;
            js_state_.axis[1].x = NEUTRAL_VALUE;
            js_state_.axis[1].y = NEUTRAL_VALUE;
            js_state_.axis[2].x = NEUTRAL_VALUE;
            js_state_.axis[2].y = NEUTRAL_VALUE;

            js_state_.button.number = 0;
            js_state_.button.pressed = 0;

        
            js_ = open(device_, O_RDONLY);

            if (js_ == -1)
                perror("Could not open joystick \n");
            if (!device_plugged())
                printf("device not plugged \n");
        }

        ~Joystick(){
            close(js_);
        }

        void device_close(){
             close(js_);
        }

        bool device_open(){
            js_ = open(device_, O_RDONLY);

            if (js_ == -1)
                perror("Could not open joystick \n");
                return 0;
            if (!device_plugged())
                printf("device not plugged \n");
                return 0;
            return 1;
        }

        bool device_plugged(){
            return (read_event(js_,&event_) == 0);
        }

        int clamp(int v, int lo, int hi){
             int out = (v<lo) ? lo : (hi <v) ? hi : v;
             if ( out < 525 && out > 500)
                 out = NEUTRAL_VALUE;
             return out;
        }

        int value_mapping(int value){

            unsigned int i = (value+32768);
            //printf("value is %u\n",i);
            i = i/64;
            i = clamp(i,MIN_VALUE,MAX_VALUE);
            //printf("value is %u\n",i);
            return i;
        
        }

        js_state device_read_value(){
        
                switch (event_.type)
                {
                    case JS_EVENT_BUTTON:
                        js_state_.button.number = event_.number;
                        js_state_.button.pressed = event_.value;
                        //printf("Button %u %u\n", event_.number, event_.value );
                        //printf("Button %u %s\n", event_.number, event_.value ? "pressed" : "released");
                        return js_state_;
                        break;
                    case JS_EVENT_AXIS:
                        axis_ = get_axis_state(&event_, axes_);

                        if (axis_ < 3){
                            //axes_[axis_].x = value_mapping(axes_[axis_].x);
                            //axes_[axis_].y = value_mapping(axes_[axis_].y);

                            js_state_.axis[0].x = value_mapping(axes_[0].x);
                            js_state_.axis[0].y = 1024 - value_mapping(axes_[0].y);
                            
                            js_state_.axis[1].x = value_mapping(axes_[1].y);
                            js_state_.axis[1].y = value_mapping(axes_[2].x);
                            //js_state_.axis[1].y = 1024 - value_mapping(axes_[2].x);

                            js_state_.axis[2].x = value_mapping(axes_[1].x);
                            js_state_.axis[2].y = value_mapping(axes_[2].y);
                            //printf("Axis %d at (%6d, %6d)\n", 0, js_state_.axis[0].x, js_state_.axis[0].y);
                            //printf("Axis %d at (%6d, %6d)\n", 1, js_state_.axis[1].x, js_state_.axis[1].y);
                            //printf("Axis %d at (%6d, %6d)\n", 2, js_state_.axis[2].x, js_state_.axis[2].y);


                            return js_state_;
                        }
                        break;
                    default:
                        /* Ignore init events. */
                        break;
                }
                return js_state_;
                
                fflush(stdout);
            
        }


        


};
}

#endif
/**
 * Reads a joystick event from the joystick device.
 *
 * Returns 0 on success. Otherwise -1 is returned.
 */

//int main(int argc, char *argv[])
//{
//    const char *device;
//    int js;
//    struct js_event event;
//    struct axis_state axes[3] = {0};
//    size_t axis;
//
//    if (argc > 1)
//        device = argv[1];
//    else
//        device = "/dev/input/js0";
//
//    js = open(device, O_RDONLY);
//
//    if (js == -1)
//        perror("Could not open joystick");
//
//    /* This loop will exit if the controller is unplugged. */
//    while (read_event(js, &event) == 0)
//    {
//        switch (event.type)
//        {
//            case JS_EVENT_BUTTON:
//                printf("Button %u %s\n", event.number, event.value ? "pressed" : "released");
//                break;
//            case JS_EVENT_AXIS:
//                axis = get_axis_state(&event, axes);
//                if (axis < 3)
//                    printf("Axis %zu at (%6d, %6d)\n", axis, axes[axis].x, axes[axis].y);
//                break;
//            default:
//                /* Ignore init events. */
//                break;
//        }
//        
//        fflush(stdout);
//    }
//
//    close(js);
//    return 0;
//}
