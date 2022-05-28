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

enkUzunlukRed = 100
enkUzunlukBlue = 100
kxRed=0
kyRed=0
kxBlue=0
kyBlue=0

red = color()
blue = color()
sitl = dronekit_sitl.start_default()
connection_string = "127.0.0.1:14550"
print("Connecting to vehicle on: %s" % (connection_string))
vehicle = connect(connection_string, wait_ready=True)
cmds = vehicle.commands




def imageCallback(data):
    global enkUzunlukRed
    global enkUzunlukBlue
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
    global mavi_bulundu 
    global DxCount 
    global DyCount
    global MODE_RTL
    
    rate = rospy.Rate(1)
 

    frame = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)	
    

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

  

    
        

    Bluemask = cv2.inRange(hsv, (20,115,70), (255,145,120))
    Bluemask = cv2.erode(Bluemask, None, iterations=2)
    Bluemask = cv2.dilate(Bluemask, None, iterations=2)
    
    FoundedContours = cv2.findContours(Bluemask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
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
          

    else:
        cv2.putText(frame,"Error no circel Found",(10,700), font, 1,(255,255,255),2,cv2.LINE_AA)
        

    DxCount = 0.0
    DyCount = 0.0

    for i in np.arange(1, len(points)):
        if points[i - 1] is None or points[i] is None:
            continue
        if i == 1:
            DxCount = float(points[i][0])-320.0 
            DyCount = 170 - float(points[i][1])
            cv2.line(frame, points[i - 1], (320,250), (0, 0, 255), 5)

            

    
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
    global kxBlue
    global kyBlue
    hipo = math.sqrt(Xmovement * Xmovement + Ymovement * Ymovement)
    if hipo != 0.0:
        if hipo < enkUzunlukBlue:
            print("daha yakýn nokta bulundu")
            enkUzunlukBlue = hipo
            kxBlue=vehicle.location.global_relative_frame.lat
            kyBlue=vehicle.location.global_relative_frame.lon
            mavi_bulundu = True
        
        print(kxBlue)
        print(kyBlue)

   

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