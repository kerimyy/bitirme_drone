from site import venv
import dronekit_sitl
from dronekit import Command,connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import cv2
import rospy
from rospy.numpy_msg import numpy_msg
import numpy as np
from std_srvs.srv import *
from sensor_msgs.msg import Image
import imutils
from collections import deque
import math
from colors import color

MODE_NAVIGATION = 0
font = cv2.FONT_HERSHEY_SIMPLEX
points = deque(maxlen=32)
counter = 0
(dX, dY) = (0, 0)
raduis = 0
CircleLostCount = 0
InsideCircle = False
TargetCircleraduis = 0
DisplayDx = 0.0
DisplayDy = 0.0
DisplayTreshhold = 0
enkUzunluk = 100

red = color()
blue = color()
sitl = dronekit_sitl.start_default()
connection_string = "127.0.0.1:14550"
print("Connecting to vehicle on: %s" % (connection_string))
vehicle = connect(connection_string, wait_ready=True)
cmds = vehicle.commands




def imageCallback(data):
    rate = rospy.Rate(5)

    # if red.isFind and blue.isFind:
    #     print("iki renkte bulundu")
    #     rate.sleep()
    #     return

    if MODE_NAVIGATION == 3:
        print("Tarama modu")
    elif MODE_NAVIGATION == 1:
        print("su taşıma modu")
    elif MODE_NAVIGATION == 2:
        print("eve dön modu")


    frame = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)	
    

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    redMask = cv2.inRange(hsv, (94, 80, 2), (126, 255, 255))
    redMask = cv2.erode(redMask, None, iterations=2)
    redMask = cv2.dilate(redMask, None, iterations=2)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blue_lower = np.array([20, 60, 50], np.uint8)
    blue_upper = np.array([130, 150, 255], np.uint8)
    blueMask = cv2.inRange(hsv, blue_lower, blue_upper)
    blueMask = cv2.erode(blueMask, None, iterations=2)
    blueMask = cv2.dilate(blueMask, None, iterations=2)
    
    RedFoundedContours = cv2.findContours(redMask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    RedFoundedContours = imutils.grab_contours(RedFoundedContours)
    BlueFoundedContours = cv2.findContours(blueMask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    BlueFoundedContours = imutils.grab_contours(BlueFoundedContours)
    center = None

    if len(RedFoundedContours) > 0:
        Circle = max(RedFoundedContours, key=cv2.contourArea)
        ((x,y),raduis) = cv2.minEnclosingCircle(Circle)
        Middle = cv2.moments(Circle)

        center = (int(Middle["m10"] / Middle["m00"]), int(Middle["m01"] / Middle["m00"]))

        if raduis > 10:
            cv2.circle(frame,(int (x),int (y)),int(raduis),(0,0,255),2)
            cv2.circle(frame,center,5,(0,255,255),-1)
            points.appendleft(center)
            red.isFind = True
        else:
            cv2.putText(frame,"Error no circel Found",(10,700), font, 1,(255,255,255),2,cv2.LINE_AA)
            #CircleLostCount+= 1

    else:
        cv2.putText(frame,"Error no circel Found",(10,700), font, 1,(255,255,255),2,cv2.LINE_AA)
        #CircleLostCount+= 1

    DxCount = 0.0
    DyCount = 0.0

    for i in np.arange(1, len(points)):
        if points[i - 1] is None or points[i] is None:
            continue
        
        if i == 1:
            DxCount = float(points[i][0])-320.0 
            DyCount = 170 - float(points[i][1])
            cv2.line(frame, points[i - 1], (320,250), (0, 0, 255), 5)

    points.clear()
    if len(BlueFoundedContours) > 0:
        Circle = max(BlueFoundedContours, key=cv2.contourArea)
        ((x,y),raduis) = cv2.minEnclosingCircle(Circle)
        Middle = cv2.moments(Circle)

        center = (int(Middle["m10"] / Middle["m00"]), int(Middle["m01"] / Middle["m00"]))

        if raduis > 10:
            cv2.circle(frame,(int (x),int (y)),int(raduis),(255,0,0),2)
            cv2.circle(frame,center,5,(0,255,255),-1)
            points.appendleft(center)
            blue.isFind = True
        else:
            cv2.putText(frame,"daire bulunamadı",(10,700), font, 1,(255,255,255),2,cv2.LINE_AA)
            #CircleLostCount+= 1

    else:
        cv2.putText(frame,"daire bulunamadı",(10,700), font, 1,(255,255,255),2,cv2.LINE_AA)
        #CircleLostCount+= 1

    DxCount = 0.0
    DyCount = 0.0

    for i in np.arange(1, len(points)):
        if points[i - 1] is None or points[i] is None:
            continue
        
        if i == 1:
            DxCount = float(points[i][0])-320.0 
            DyCount = 170 - float(points[i][1])
            cv2.line(frame, points[i - 1], (320,250), (0, 0, 255), 5)

    cv2.circle(frame,(320,250),10,(255,0,0),-1)
    cv2.imshow("Renk Tanima", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    cv2.waitKey(3)
    rate.sleep()
    
           

# def kamera():
#     rospy.init_node('drone_control', anonymous=True)
#     sub = rospy.Subscriber("/webcam/image_raw", numpy_msg(Image), imageCallback)
#     rospy.spin()

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







if __name__ == "__main__":
    arm_and_takeoff(20)
    #gorev(-35.36311117 ,149.16577231,-35.36349584 ,149.16614279,20)
    rospy.init_node('drone_control', anonymous=True)
    rospy.Subscriber("/webcam/image_raw", numpy_msg(Image), imageCallback)   
    rospy.spin()
    print("bitti")





#0.00001140 yaklaşık 1 metre
#0.00002 denenecek değer  1.7 metre


		



#print(" Groundspeed: %s" % vehicle.groundspeed)    # settable
#print(" Airspeed: %s" % vehicle.airspeed)		
        
       
#def myhook():
#  print "shutdown time!"
#
#rospy.on_shutdown(myhook)	