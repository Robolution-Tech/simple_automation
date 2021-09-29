from target_pose_process import target_pose_process
import rospy
import pytest
import tf
import random



def test_tf_transformation():
    pose_p = target_pose_process()
    rate = rospy.Rate(10.0)
    br = tf.TransformBroadcaster()
    
    for i in range(10):
        # x = i+ random.random()
        # y = i + random.random()
        x = 1
        y = 2
        br.sendTransform((x, y, 0.0),
                            (0.0,0.0,0.0,1.0),
                            rospy.Time.now(),
                            "base_link",
                            "obj1")
        
        
        if (pose_p.send_pose() == True):
            pose = pose_p.get_processed_pose()
            print("x, y is:", x, "  " ,y)
            print(pose)
            assert x == pose.pose.position.x - pose_p.offset_x
            assert y == pose.pose.position.y - pose_p.offset_y
        
        rate.sleep()

