#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from mavros_msgs.msg import PositionTarget
import math
from tf.transformations import quaternion_from_euler

current_state = State()
desired_Pose = PoseStamped()
current_Pose = PoseStamped()

def state_cb(msg):
    global current_state
    current_state = msg

def goto_goal(x, y, z, q, pose_pub):
	rate = rospy.Rate(20);
	global desired_Pose
	global current_Pose
	distThreshold = 0.7;
	curr_x = current_Pose.pose.position.x
	curr_y = current_Pose.pose.position.y
	curr_z = current_Pose.pose.position.z
	desired_Pose.pose.position.x = x
	desired_Pose.pose.position.y = y
	desired_Pose.pose.position.z = z
	desired_Pose.pose.orientation.x = q[0]
	desired_Pose.pose.orientation.y = q[1]
	desired_Pose.pose.orientation.z = q[2]
	desired_Pose.pose.orientation.w = q[3]
	rospy.loginfo("Desired position: x={}, y={}, z={}".format(x, y, z))
	# print(self.curr)
	#d = math.dist([curr_x, curr_y, curr_z],[x,y,z]);
	d = math.sqrt(pow(curr_x - x, 2) + pow(curr_y - y, 2) + pow(curr_z - z, 2))
	while d > distThreshold and not rospy.is_shutdown():
		pose_pub.publish(desired_Pose)
		rate.sleep()
		curr_x = current_Pose.pose.position.x
		curr_y = current_Pose.pose.position.y
		curr_z = current_Pose.pose.position.z
		d = math.sqrt(pow(curr_x - x, 2) + pow(curr_y - y, 2) + pow(curr_z - z, 2))
		#print("curr_x :", curr_x ,"  curr_y : ", curr_y, " curr_z : ", curr_z, " x : ", x , " y : ", y , " z : ", z ,  "  anddd d : ", d)
		if d <= distThreshold:
			break

def circle_around(x,y,z,pose_pub):
	curr_x = current_Pose.pose.position.x
	curr_y = current_Pose.pose.position.y
	curr_z = current_Pose.pose.position.z
	
	n = 16;
	zh = 0;
	r = 3;
	start_ang = math.atan2(y - curr_y, x - curr_x)
	print("start_ang :", start_ang);
	#start_ang = (start_ang-2*math.pi) if start_ang > math.pi else (start_ang+2*math.pi);
	print("start_ang after qartering:", start_ang);
	for i in range(n):
		desx = x-r*math.cos(i*2*math.pi/n - start_ang);
		desy = y-r*math.sin(i*2*math.pi/n - start_ang);
		desz = z + zh;
		z_ang = math.atan2(y - desired_Pose.pose.position.y, x - desired_Pose.pose.position.x)
		z_ang = (z_ang-2*math.pi) if z_ang > math.pi else (z_ang+2*math.pi);
		q = quaternion_from_euler(0, 0, z_ang)
		goto_goal(desx, desy, desz, q, pose_pub)



def sub_pose_cb(msg):
	global current_Pose
	current_Pose = msg
	
def execute_navigation():
    global current_state
    global desired_State
    pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback = sub_pose_cb)
    pose_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10);

    # Check point 1 :   Got to sample_probe
    goto_goal(40.8, 3.5, 20,[0,0,0,1], pose_pub)
    
    # Check point 2 :   Pick sample probe
    goto_goal(40.8, 3.5, 11.5,[0,0,0,1], pose_pub)
    goto_goal(40.8, 3.5, 20,[0,0,0,1], pose_pub)
    
    # Check point 3 :   go to the rock 
    goto_goal(58.2, -12.5, 19,[0,0,0,1], pose_pub)
    goto_goal(56.2, -12.5, 20,[0,0,0,1], pose_pub)

    # Check point 4 :   circle around rock 
    circle_around(60.2, -12.5, 18.7, pose_pub)

    # Check point 5 :   go to rover 
    #goto_goal(0, 0, 5,[0,0,0,1], pose_pub)
    goto_goal(12.621, -64.5, 1,[0,0,0,1], pose_pub)
    goto_goal(12.621, -64.5, 0.5,[0,0,0,1], pose_pub)

    local_pos_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

    landing_position = PositionTarget()
    landing_position.position.x = 12.621
    landing_position.position.y = -64.55
    landing_position.position.z = -3.5
    landing_position.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    landing_position.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                             + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                             + PositionTarget.IGNORE_YAW + PositionTarget.IGNORE_YAW_RATE

    # Publish the landing position command
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        local_pos_pub.publish(landing_position)
        rate.sleep()
        if(current_Pose.pose.position.z < -3.2):
            print("Landed =========")
            arm_service(False);


if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    print("Before a while : ",current_state)
    while(not rospy.is_shutdown() and not current_state.connected):
        print("IS it connected??? \n: ",current_state)
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

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
    print("Before a while : ",current_state)
    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(2.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(2.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()


        local_pos_pub.publish(pose)

        rate.sleep()

        if(current_state.mode == "OFFBOARD" and current_state.armed):
            break;

    execute_navigation();

