from site import venv
import dronekit_sitl
from dronekit import Command,connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

import cv2
import rospy
from geometry_msgs.msg  import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg
import numpy as np
from std_srvs.srv import *

MODE_NAVIGATION = 0


sitl = dronekit_sitl.start_default()
connection_string = "127.0.0.1:14550"
print("Connecting to vehicle on: %s" % (connection_string))

vehicle = connect(connection_string, wait_ready=True)
cmds = vehicle.commands
redx = False
bluex = False

def imageCallback(data):
    if redx and bluex:
        print("renkler bulundu")
        rate = rospy.Rate(0.2) 
        #rospy.Subscriber("/webcam/image_raw", numpy_msg(Image), null)
        return 
    
    imageFrame = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)	
    rate = rospy.Rate(10)

    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_RGB2HSV)
  
    # Kırmızı renk için aralığı ayarlama

    red_lower1 = np.array([0, 100, 20], np.uint8)
    red_upper1 = np.array([10, 255, 255], np.uint8)
    red_lower2 = np.array([160, 100, 20], np.uint8)
    red_upper2 = np.array([179, 255, 255], np.uint8)
    red_mask1 = cv2.inRange(hsvFrame, red_lower1, red_upper1)
    red_mask2 = cv2.inRange(hsvFrame, red_lower2, red_upper2)
    red_mask = red_mask1 + red_mask2
  
    # Mavi renk için aralığı ayarlama

    blue_lower = np.array([94, 80, 2], np.uint8)
    blue_upper = np.array([120, 255, 255], np.uint8)
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
		
    kernal = np.ones((5, 5), "uint8")
      
    # Kırmızı
    red_mask = cv2.dilate(red_mask, kernal)
    res_red = cv2.bitwise_and(imageFrame, imageFrame, 
                              mask = red_mask)
      
    # Mavi 
    blue_mask = cv2.dilate(blue_mask, kernal)
    res_blue = cv2.bitwise_and(imageFrame, imageFrame,
                               mask = blue_mask)
   
    # Kırmizi rengi izlemek için kontur oluşturma
    contours, hierarchy = cv2.findContours(red_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
      
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 300):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y), 
                                       (x + w, y + h), 
                                       (255, 0, 0), 2)
              
            cv2.putText(imageFrame, "Red", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (255, 0, 0))    
            print("kırmızı bulundu")
            red =True
            
  
    # Mavi rengi izlemek için kontur oluşturma
    contours, hierarchy = cv2.findContours(blue_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 300):
            x, y, w, h = cv2.boundingRect(contour)
            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h),
                                       (0, 0, 255), 2)
              
            cv2.putText(imageFrame, "Blue", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (0, 0, 255))
            print("mavi bulundu")
            blue = True
            
              
    # Program Termination
    cv2.imshow("Renk Tanima", cv2.cvtColor(imageFrame, cv2.COLOR_RGB2BGR))
    cv2.waitKey(3)
    rate.sleep()
    
           

def kamera():
    rospy.init_node('drone_control', anonymous=True)
    rospy.Subscriber("/webcam/image_raw", numpy_msg(Image), imageCallback)
    rospy.spin()

def arm_and_takeoff(aTargetAltitude):
  
    while not vehicle.is_armable:
        print (" iha bekleniyor ")
        time.sleep(1)

    print ("Arming motors")
    
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    
    while not vehicle.armed:
        print (" arm için bekleniyor")
        time.sleep(1)

    print ("Kalkış Başlıyor")
    vehicle.simple_takeoff(aTargetAltitude) 

    
    while True:
        print (" Yükseklik: ",vehicle.location.global_relative_frame.alt) 
        
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print ("hedef yüksekliğe ulaşıldı")
            break
        time.sleep(1)

def gorev(x1,y1,x2,y2,alt):
    if vehicle.mode is not VehicleMode("GUIDED"):
        vehicle.mode = VehicleMode("GUIDED")
    cmds.clear()
    time.sleep(1)
    if (x2<x1): 
        x1,x2 = x2,x1
        y1,y2 = y2,y1

    x3 = x1
    y3 = y2
    x4 = x2
    y4 = y1
    mesafe = 0.00002
    nx = x3 
    ny = y3

    # 1 3 2 4

    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, alt))

    cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,x1 ,y1,alt))
    cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,x3 ,y3,alt))
    while(nx < x2):
        nx += mesafe
        cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,nx ,y3,alt))
        cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,nx ,y1,alt))
        nx += mesafe
        cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,nx ,y1,alt))
        cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,nx ,y3,alt))
    cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,nx ,y3,alt))

    

    cmds.upload()
    time.sleep(1)
    vehicle.mode = VehicleMode("AUTO")
    if vehicle.mode is VehicleMode("AUTO"):
        print("auto")
    #kamera()
    while (True):
        if vehicle.commands.next == len(cmds):
            break
        
        print(vehicle.commands.next,"hedefe gidiliyor")
        print(len(cmds))
        time.sleep(1)

    print("tarama tamamlandı")
     
    time.sleep(2)
    vehicle.mode = VehicleMode("RTL")
    print("eve dönülüyor")



arm_and_takeoff(20)
gorev(-35.36311117 ,149.16577231,-35.36349584 ,149.16614279,20)


#0.00001140 yaklaşık 1 metre
#0.00002 denenecek değer  1.7 metre


		



#print(" Groundspeed: %s" % vehicle.groundspeed)    # settable
#print(" Airspeed: %s" % vehicle.airspeed)		
        
       
#def myhook():
#  print "shutdown time!"
#
#rospy.on_shutdown(myhook)	