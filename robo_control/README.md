# j1939_transmition

J1939 transimit and reciving program


1. use calls CAN_Dev to initialize and open device
2. use function int_to_hex to convert integer to hex, for data preperation
3. use j1939_send to generate CAN data and send




TODO:
1. add geometry_msgs/Twist msg, potentially called cmd_vel as input
2. add configuration file for: CANbus config, input config