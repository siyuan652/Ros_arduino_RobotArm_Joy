#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt16

base_angle = 75
shoulder_angle = 90
forearm_angle = 90
gripper_angle = 45

angle_step = 2
angle_step_gripper = 5
angle_step_base = 2

def control(data):
    global base_angle, shoulder_angle, forearm_angle, gripper_angle

    left_stick_x = data.axes[0]
    left_stick_y = data.axes[1]
    right_stick_x = data.axes[2]
    right_stick_y = data.axes[5]

    if left_stick_x > 0.5:
       base_angle = min(base_angle + angle_step_base, 180)
    elif left_stick_x < -0.5:
       base_angle = max(base_angle - angle_step_base, 0)

    if left_stick_y > 0.5:
       shoulder_angle = min(shoulder_angle + angle_step, 140)
    elif left_stick_y < -0.5:
       shoulder_angle = max(shoulder_angle - angle_step, 45)

    if right_stick_y > 0.5:
       forearm_angle = min(forearm_angle + angle_step, 140)
    elif right_stick_y < -0.5:
       forearm_angle = max(forearm_angle - angle_step, 45)

    if right_stick_x > 0.5:
       gripper_angle = min(gripper_angle + angle_step_gripper, 135)
    elif right_stick_x < -0.5:
       gripper_angle = max(gripper_angle - angle_step_gripper, 0)

    base_servo_pub.publish(UInt16(base_angle))
    shoulder_servo_pub.publish(UInt16(shoulder_angle))
    forearm_servo_pub.publish(UInt16(forearm_angle))
    gripper_servo_pub.publish(UInt16(gripper_angle))

if __name__ == '__main__':

  rospy.init_node('ps4arduino', anonymous=True)
  base_servo_pub = rospy.Publisher('/servo_1', UInt16, queue_size=10) #pin 9
  shoulder_servo_pub = rospy.Publisher('/servo_2', UInt16, queue_size=10) #pin 10
  forearm_servo_pub = rospy.Publisher('/servo_3', UInt16, queue_size=10) #pin 11
  gripper_servo_pub = rospy.Publisher('/servo_4', UInt16, queue_size=10) #pin 12

  base_servo_pub.publish(UInt16(base_angle))
  shoulder_servo_pub.publish(UInt16(shoulder_angle))
  forearm_servo_pub.publish(UInt16(forearm_angle))
  gripper_servo_pub.publish(UInt16(gripper_angle))

  rospy.Subscriber('/j2/joy', Joy, control)  
  rospy.spin()
