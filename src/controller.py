#!/usr/bin/env python

import wiringpi as wp
import rospy
import math
from geometry_msgs.msg import Twist
from threading import Lock
import tf
import tf2_ros
import geometry_msgs.msg
import nav_msgs.msg

maxvel=0.04#m/s
maxrot=math.pi/10#rad/s

def init_wp():
    wp.wiringPiSetup()
    #right left
    #45    67
    #01    23
    
    wp.pinMode(0,wp.OUTPUT)
    wp.pinMode(1,wp.OUTPUT)
    wp.pinMode(2,wp.OUTPUT)
    wp.pinMode(3,wp.OUTPUT)
    wp.pinMode(4,wp.OUTPUT)
    wp.pinMode(5,wp.OUTPUT)
    wp.pinMode(6,wp.OUTPUT)
    wp.pinMode(7,wp.OUTPUT)
    
    wp.pinMode(11,wp.INPUT)#CE1
    wp.pullUpDnControl(11,wp.PUD_DOWN)
    
    wp.softPwmCreate(4,0,100)
    wp.softPwmCreate(5,0,100)
    wp.softPwmCreate(6,0,100)
    wp.softPwmCreate(7,0,100)
    
def go(lspeed,rspeed):
    if(rspeed>0):
        wp.softPwmWrite(5,0)
        wp.digitalWrite(0,0)
        wp.softPwmWrite(4,rspeed)
        wp.digitalWrite(1,1)
    else:
        wp.softPwmWrite(4,0)
        wp.digitalWrite(1,0)
        wp.softPwmWrite(5,-rspeed)
        wp.digitalWrite(0,1)

    if(lspeed>0):
        wp.softPwmWrite(7,0)
        wp.digitalWrite(2,0)
        wp.softPwmWrite(6,lspeed)
        wp.digitalWrite(3,1)
    else:
        wp.softPwmWrite(6,0)
        wp.digitalWrite(3,0)
        wp.softPwmWrite(7,-lspeed)
        wp.digitalWrite(2,1)
    
def stop():
    wp.softPwmWrite(5,0)
    wp.digitalWrite(1,0)
    wp.softPwmWrite(4,0)
    wp.digitalWrite(0,0)

    wp.softPwmWrite(7,0)
    wp.digitalWrite(3,0)
    wp.softPwmWrite(6,0)
    wp.digitalWrite(2,0)

def brake():
    wp.softPwmWrite(5,0)
    wp.softPwmWrite(4,0)
    wp.digitalWrite(1,1)
    wp.digitalWrite(0,1)

    wp.softPwmWrite(7,0)
    wp.softPwmWrite(6,0)
    wp.digitalWrite(3,1)
    wp.digitalWrite(2,1)

move=True
move_lock=Lock()

odom_x=0.0
odom_y=0.0
odom_th=0.0
odom_vx=0.0
odom_vy=0.0
odom_vth=0.0

odom_last_time=0

def cmd_vel_cb(msg):
    global odom_x,odom_y,odom_th,odom_vx,odom_vy,odom_vth,odom_last_time
    with move_lock:
        if move:
            vx=min(max(msg.linear.x/maxvel*100,-100),100)
            vz=min(max(msg.angular.z/maxrot*100,-100),100)
            absxz=abs(vx)+abs(vz)
            if absxz>100:
                vx/=(absxz/100)
                vz/=(absxz/100)
            print "L",int(vx-vz),"R",int(vx+vz) 
            go(int(vx-vz),int(vx+vz))

            cur_time=rospy.Time.now()
            dt=(cur_time-odom_last_time).to_sec()
            odom_x+=(odom_vx*math.cos(odom_th)-odom_vy*math.sin(odom_th))*dt
            odom_y+=(odom_vx*math.sin(odom_th)+odom_vy*math.cos(odom_th))*dt
            odom_th+=odom_vth*dt
            odom_vx=vx/100*maxvel
            odom_vy=0.0
            odom_vth=vz/100*maxrot
            odom_last_time=cur_time
                        
if __name__=="__main__":
    init_wp()
    rospy.init_node("move_base")
    odom_pub=rospy.Publisher("odom",nav_msgs.msg.Odometry,1)
    odom_bc=tf2_ros.TransformBroadcaster()
    odom_last_time=rospy.Time.now()
    sub=rospy.Subscriber("cmd_vel",geometry_msgs.msg.Twist,cmd_vel_cb)
    rate=rospy.Rate(10)
    pre_switch=0
    while not rospy.is_shutdown():
        now_switch=wp.digitalRead(11)
        if (not pre_switch) and now_switch:
            with move_lock:
                move=not move
                print "move", move 
        if not move:
            stop()
            cur_time=rospy.Time.now()
            dt=(cur_time-odom_last_time).to_sec()
            odom_x+=(odom_vx*math.cos(odom_th)-odom_vy*math.sin(odom_th))*dt
            odom_y+=(odom_vx*math.sin(odom_th)+odom_vy*math.cos(odom_th))*dt
            odom_th+=odom_vth*dt
            odom_vx=0.0
            odom_vy=0.0
            odom_vth=0.0
            odom_last_time=cur_time
                    
        pre_switch=now_switch

        quat=tf.transformations.quaternion_from_euler(0,0,odom_th)
        
        odom_tf=geometry_msgs.msg.TransformStamped()
        odom_tf.header.stamp=rospy.Time.now()
        odom_tf.header.frame_id = "odom"
        odom_tf.child_frame_id="base_link"
        odom_tf.transform.translation.x=odom_x
        odom_tf.transform.translation.y=odom_y
        odom_tf.transform.translation.z=0.0
        odom_tf.transform.rotation.x=quat[0]
        odom_tf.transform.rotation.y=quat[1]
        odom_tf.transform.rotation.z=quat[2]
        odom_tf.transform.rotation.w=quat[3]
        odom_bc.sendTransform(odom_tf)
        
        odom_msg=nav_msgs.msg.Odometry()
        odom_msg.header.stamp=rospy.Time.now()
        odom_msg.header.frame_id="odom"
        odom_msg.child_frame_id="base_link"
        odom_msg.pose.pose.position.x=odom_x
        odom_msg.pose.pose.position.y=odom_y
        odom_msg.pose.pose.position.z=0.0
        odom_msg.pose.pose.orientation=quat
        odom_msg.twist.twist.linear.x=odom_vx
        odom_msg.twist.twist.linear.y=odom_vy
        odom_msg.twist.twist.angular=odom_vth
        odom_pub.publish(odom_msg)

        rate.sleep()

    stop()
