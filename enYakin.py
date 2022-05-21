from curses import KEY_B2
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
import imutils
from collections import deque
import time
import math
from std_srvs.srv import *

MODE_NAVIGATION = 0
MODE_RTL = False





Altitude = 1300

#Controller config
UpdateRate = 1
MaxMovementRatePositive = 0.020
MaxMovementRateNegative = -0.020
DivisionValueX = 14400
DivisionValueY = 9450
TargetCircleMultiplayer = 3

font = cv2.FONT_HERSHEY_SIMPLEX

points = deque(maxlen=32)
counter = 0
(dX, dY) = (0, 0)
direction = ""
Run = True
Direction1 = "error"
Direction2 = "error"
raduis = 0
CircleLostCount = 0
InsideCircle = False
TargetCircleRaduis = 0

#Display variables
DisplayDx = 0.0
DisplayDy = 0.0
DisplayTreshhold = 0

enkUzunluk = 100
kx=0
ky=0

sitl = dronekit_sitl.start_default()
connection_string = "127.0.0.1:14550"
print("Connecting to vehicle on: %s" % (connection_string))

vehicle = connect(connection_string, wait_ready=True)
cmds = vehicle.commands
redx = False
bluex = False
DxCount = 0.0
DyCount = 0.0



def closeCamera(data):
  print ("kamera kapatıldı")
def imageCallback(data):
    global enkUzunluk
    global InsideCircle 
    global TargetCircleRaduis 
    global DisplayDx
    global Altitude
    global UpdateRate 
    global MaxMovementRatePositive 
    global MaxMovementRateNegative 
    global DivisionValueX 
    global DivisionValueY 
    global TargetCircleMultiplayer 
    global font 
    global points 
    global counter 
    global dX, dY
    global direction 
    global Run 
    global kirmizi_bulundu 
    global DxCount 
    global DyCount
    global MODE_RTL
    
    rate = rospy.Rate(1)
    if MODE_RTL:
        print("eve dönülüyor")
        rate.sleep()
        return

    frame = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)	
    

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    Greenmask = cv2.inRange(hsv, (94, 80, 2), (126, 255, 255))
    Greenmask = cv2.erode(Greenmask, None, iterations=2)
    Greenmask = cv2.dilate(Greenmask, None, iterations=2)
    
    FoundedContours = cv2.findContours(Greenmask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    FoundedContours = imutils.grab_contours(FoundedContours)
    center = None

    if len(FoundedContours) > 0:
        Circle = max(FoundedContours, key=cv2.contourArea)
        ((x,y),raduis) = cv2.minEnclosingCircle(Circle)
        Middle = cv2.moments(Circle)

        center = (int(Middle["m10"] / Middle["m00"]), int(Middle["m01"] / Middle["m00"]))

        if raduis > 10:
            cv2.circle(frame,(int (x),int (y)),int(raduis),(0,0,255),2)
            cv2.circle(frame,center,5,(0,255,255),-1)
            points.appendleft(center)
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
        #check if 400,400 is in surface of circle ((x,y),raduis) if case then no need to do calculations

        if i == 1:
            
            #print("Y points: " + str(points[i][1]))

            DxCount = float(points[i][0])-320.0 
            DyCount = 170 - float(points[i][1])

            cv2.line(frame, points[i - 1], (320,250), (0, 0, 255), 5)

            #cv2.putText(frame,str(DxCount),(10,100), font, 1,(255,0,0),2,cv2.LINE_AA)
            #cv2.putText(frame,str(DyCount),(10,150), font, 1,(255,0,0),2,cv2.LINE_AA)

            if (int(points[i][0]) - 320)**2 + (int(points[i][1]) - 250)**2 < (raduis/2)**2:
                InsideCircle = True

    #cv2.putText(frame,"Inside circel" + str(InsideCircle),(10,120), font, 1,(255,0,0),2,cv2.LINE_AA)

    

    
    
    if(counter % UpdateRate) == 0:

        DisplayDx = DxCount
        DisplayDy = DyCount

        Xmovement = (DxCount / DivisionValueX) #6400
        Ymovement = (DyCount / DivisionValueY) #4200

        if Xmovement > MaxMovementRatePositive:
            Xmovement = MaxMovementRatePositive
        elif Xmovement < MaxMovementRateNegative:
            Xmovement = MaxMovementRateNegative
        if Ymovement > MaxMovementRatePositive:
            Ymovement = MaxMovementRatePositive
        elif Ymovement < MaxMovementRateNegative:
            Ymovement = MaxMovementRateNegative

        AltitudeCommand = 0.0
    global kx
    global ky
    hipo = math.sqrt(Xmovement * Xmovement + Ymovement * Ymovement)
    if hipo != 0.0:
        if hipo < enkUzunluk:
            print("daha yakın nokta bulundu")
            enkUzunluk = hipo
            kx=vehicle.location.global_relative_frame.lat
            ky=vehicle.location.global_relative_frame.lon
            kirmizi_bulundu = True
        
        print(kx)
        print(ky)
        
   

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



def goto_position_target_local_ned(north, east, down):
    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    location in the North, East, Down frame.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down,
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)




if __name__ == "__main__":
    arm_and_takeoff(20)
    rospy.init_node('drone_control', anonymous=True)
    sub = rospy.Subscriber("/webcam/image_raw", numpy_msg(Image), imageCallback)
    rospy.spin()
    print("bitti")
# kamera()


#0.00001140 yaklaşık 1 metre
#0.00002 denenecek değer  1.7 metre