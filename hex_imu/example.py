#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import MagneticField 

def magnetic_callback(data):
    x = data.magnetic_field.x
    y = data.magnetic_field.y
    if x == 0 and y == 0:
        heading_deg = 0
    else:
        heading_rad = math.atan2(y, x)
        heading_deg = (360 - math.degrees(heading_rad)) % 360

        if heading_deg < 0:
            heading_deg += 360
    print(f"Azimuth: {heading_deg:.2f}°")

def main():
    rospy.init_node('magnetic_heading_listener', anonymous=True)
    
    rospy.Subscriber("/magnetic_data", MagneticField, magnetic_callback) 
    
    print("magnetic data listener started")
    rospy.spin()  

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
