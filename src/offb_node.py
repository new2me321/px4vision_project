#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()
drone_pose = PoseStamped()
pose = PoseStamped()
cmd_vel = TwistStamped()

alt_tolerance = 0.02

def state_cb(msg):
    global current_state
    current_state = msg

def drone_pose_cb(msg):
    global drone_pose
    drone_pose = msg

def hover():
    global pose
    local_pos_pub.publish(pose)

def update(update_z=False):
    global pose
    global drone_pose

    pose.pose.position.x = drone_pose.pose.position.x
    pose.pose.position.y = drone_pose.pose.position.y
    
    if update_z:
        pose.pose.position.z = drone_pose.pose.position.z




if __name__ == "__main__":
    rospy.init_node("offb_node_py")
    rospy.loginfo_once("offb_node started!")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    drone_pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=drone_pose_cb)
    vel_pub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()


    # pose.pose.position.x = 0
    # pose.pose.position.y = 0
    pose.pose.position.z = 1.2

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()
    hold = False
    hold_time = rospy.Time.now() 
    to_update = False
    c= 0
    while(not rospy.is_shutdown()):
        if not hold:
            if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(set_mode_client.call(offb_set_mode).mode_sent == True):
                    rospy.loginfo("OFFBOARD enabled")

                last_req = rospy.Time.now()
            else:
                if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                    if(arming_client.call(arm_cmd).success == True):
                        rospy.loginfo("Vehicle armed")

                    last_req = rospy.Time.now()
            
                local_pos_pub.publish(pose)

            if (drone_pose.pose.position.z >= (pose.pose.position.z-alt_tolerance) and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                rospy.loginfo_once("Vehicle in the air. Ready to control")
                hold = True

        if hold:
            hover() # Keep hovering forever
            while (rospy.Time.now() - last_req) > rospy.Duration(5.0) and  c < 100:
                rospy.loginfo_once("forward ")
                cmd_vel.twist.linear.x = 0.5
                vel_pub.publish(cmd_vel)
                c+=1
                rate.sleep()
                to_update = True

            
            while (rospy.Time.now() - last_req) > rospy.Duration(20.0) and  c < 200:
                rospy.loginfo_once("right ")
                cmd_vel.twist.linear.y = -1.0
                vel_pub.publish(cmd_vel)
                c+=1
                rate.sleep()
                to_update = True

            if to_update:
                update()
                to_update = False
                last_req = rospy.Time.now()

            if (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                rospy.loginfo_once("return to home")
                pose.pose.position.x = 0
                pose.pose.position.y = 0

                local_pos_pub.publish(pose)

        rate.sleep()
