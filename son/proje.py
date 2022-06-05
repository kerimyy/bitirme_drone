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
enkUzunlukRed = 100
enkUzunlukBlue = 100

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
Xmovement = 0
Ymovement = 0
red = color()
blue = color()
sitl = dronekit_sitl.start_default()
connection_string = "127.0.0.1:14550"
print("Connecting to vehicle on: %s" % (connection_string))
vehicle = connect(connection_string, wait_ready=True)
cmds = vehicle.commands


MODE_NAVIGATION = 0
MODE_RED_BLUE_FOUND = 1
MODE_RED_ALIGN = 2
MODE_BLUE_ALIGN = 3
MODE_RTL = 4
MODE_GO_BLUE = 5
MODE_GO_RED = 6
MODE_GO_HOME = 7
MODE_IRIS = MODE_NAVIGATION
LAST_MODE = -1

def imageCallback(data):
    

    # if MODE_IRIS == MODE_NAVIGATION:
    #     rate = rospy.Rate(2)
    # else:
    #     rate = rospy.Rate(0.1)
    #rate = rospy.Rate(5)

    frame = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)	
    

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    # redMask = cv2.inRange(hsv, (94, 80, 2), (126, 255, 255))
    # redMask = cv2.erode(redMask, None, iterations=2)
    # redMask = cv2.dilate(redMask, None, iterations=2)

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
    res_red = cv2.bitwise_and(frame, frame, 
                              mask = red_mask)
      
    # Mavi 
    blue_mask = cv2.dilate(blue_mask, kernal)
    res_blue = cv2.bitwise_and(frame, frame,
                               mask = blue_mask)
   
    # Kırmizi rengi izlemek için kontur oluşturma
    contours, hierarchy = cv2.findContours(red_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
      
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 300):
            Circle = max(contours, key=cv2.contourArea)
            global raduis
            ((x,y),raduis) = cv2.minEnclosingCircle(Circle)            
            Middle = cv2.moments(Circle)

            center = (int(Middle["m10"] / Middle["m00"]), int(Middle["m01"] / Middle["m00"]))

            if raduis > 50:
                cv2.circle(frame,(int (x),int (y)),int(raduis),(0,0,0),2)
                cv2.circle(frame,center,5,(0,0,0),-1)
                points.appendleft(center)
            
           
                
                red.isFind =True
                print("kırmızı bulundu",red.isFind)

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

            cv2.line(frame, points[i - 1], (320,250), (0, 255, 0), 5)

            #cv2.putText(frame,str(DxCount),(10,100), font, 1,(255,0,0),2,cv2.LINE_AA)
            #cv2.putText(frame,str(DyCount),(10,150), font, 1,(255,0,0),2,cv2.LINE_AA)

            if (MODE_IRIS == MODE_RED_ALIGN and int(points[i][0]) - 320)**2 + (int(points[i][1]) - 250)**2 < (raduis/2)**2:
                        InsideCircle = True

            if(counter % UpdateRate ) == 0 and (MODE_IRIS == MODE_RED_ALIGN or MODE_IRIS == MODE_NAVIGATION):

                DisplayDx = DxCount
                DisplayDy = DyCount

                Xmovement = (DxCount / DivisionValueX)
                Ymovement = (DyCount / DivisionValueY)

                if Xmovement > MaxMovementRatePositive:
                    Xmovement = MaxMovementRatePositive
                elif Xmovement < MaxMovementRateNegative:
                    Xmovement = MaxMovementRateNegative
                if Ymovement > MaxMovementRatePositive:
                    Ymovement = MaxMovementRatePositive
                elif Ymovement < MaxMovementRateNegative:
                    Ymovement = MaxMovementRateNegative

                AltitudeCommand = 0.0
            
                global enkUzunlukRed
                hipo = math.sqrt(Xmovement * Xmovement + Ymovement * Ymovement)
                if hipo != 0.0:
                    if hipo < enkUzunlukRed:
                        print("daha yakın nokta bulundu")
                        enkUzunlukRed = hipo
                        red.x=vehicle.location.global_relative_frame.lat
                        red.y=vehicle.location.global_relative_frame.lon
                    
                        print("kırmızı x:",red.x)
                        print("kırmızı y:",red.y)

    
    contours, hierarchy = cv2.findContours(blue_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if(area > 300):
            Circle = max(contours, key=cv2.contourArea)
            ((x,y),raduis) = cv2.minEnclosingCircle(Circle)
            Middle = cv2.moments(Circle)

            center = (int(Middle["m10"] / Middle["m00"]), int(Middle["m01"] / Middle["m00"]))

            if raduis > 50:
                cv2.circle(frame,(int (x),int (y)),int(raduis),(0,0,0),2)
                cv2.circle(frame,center,5,(0,0,0),-1)
                points.appendleft(center)
           
                print("mavi bulundu")
                blue.isFind = True
                print("mavi bulundu",blue.isFind)

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

            cv2.line(frame, points[i - 1], (320,250), (0, 255, 0), 5)
            if (MODE_IRIS == MODE_BLUE_ALIGN and int(points[i][0]) - 320)**2 + (int(points[i][1]) - 250)**2 < (raduis/2)**2:
                InsideCircle = True

            if(counter % UpdateRate ) == 0 and (MODE_IRIS == MODE_BLUE_ALIGN or MODE_IRIS == MODE_NAVIGATION):

                DisplayDx = DxCount
                DisplayDy = DyCount

                Xmovement = (DxCount / DivisionValueX)
                Ymovement = (DyCount / DivisionValueY)

                if Xmovement > MaxMovementRatePositive:
                    Xmovement = MaxMovementRatePositive
                elif Xmovement < MaxMovementRateNegative:
                    Xmovement = MaxMovementRateNegative
                if Ymovement > MaxMovementRatePositive:
                    Ymovement = MaxMovementRatePositive
                elif Ymovement < MaxMovementRateNegative:
                    Ymovement = MaxMovementRateNegative

                AltitudeCommand = 0.0
            
                global enkUzunlukBlue
                hipo = math.sqrt(Xmovement * Xmovement + Ymovement * Ymovement)
                if hipo != 0.0:
                    if hipo < enkUzunlukBlue:
                        print("daha yakın nokta bulundu")
                        enkUzunlukBlue = hipo
                        blue.x=vehicle.location.global_relative_frame.lat
                        blue.y=vehicle.location.global_relative_frame.lon
                    
                        print("mavi x:",blue.x)
                        print("mavi y:",blue.y)
    
    
            
    cv2.circle(frame,(320,250),10,(255,0,0),-1)
    cv2.imshow("Renk Tanima", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    cv2.waitKey(3)
    #rate.sleep()
    
           

# def kamera():
#     rospy.init_node('drone_control', anonymous=True)
#     sub = rospy.Subscriber("/webcam/image_raw", numpy_msg(Image), imageCallback)
#     rospy.spin()
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
    MODE = MODE_NAVIGATION

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
    mesafe = 0.000035
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
           
       

     





if __name__ == "__main__":
    arm_and_takeoff(20)
    gorev(-35.36311117 ,149.16577231,-35.36349584 ,149.16614279,20)
    rospy.init_node('drone_control', anonymous=True)
    #rospy.Subscriber("/webcam/image_raw", numpy_msg(Image), imageCallback)
    while True:
        msg = rospy.wait_for_message("/webcam/image_raw", numpy_msg(Image),timeout=None)
        imageCallback(msg)
        if MODE_IRIS == MODE_NAVIGATION:
            if vehicle.mode != VehicleMode("AUTO"):
                vehicle.mode = VehicleMode("AUTO")
                print("mode navigation")
                LAST_MODE = MODE_NAVIGATION

            elif red.isFind and blue.isFind:
                vehicle.mode = VehicleMode("GUIDED")
                time.sleep(1)
                MODE_IRIS = MODE_GO_BLUE
                
            elif vehicle.commands.next == len(cmds):
                print("havuzlar bulunamadı eve dönülüyor")
                MODE_IRIS = MODE_GO_HOME
                
            print(vehicle.commands.next,"/",len(cmds),"alan taranıyor")
    
        elif MODE_IRIS == MODE_GO_BLUE:           
            if LAST_MODE == MODE_NAVIGATION:
                print("mode go blue")            
                cmds.clear()
                time.sleep(2)
                cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 20))
                cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,blue.x ,blue.y,20))
                cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,blue.x ,blue.y,20))
                cmds.upload()               
                vehicle.mode = VehicleMode("AUTO")
                time.sleep(2)
                LAST_MODE = MODE_GO_BLUE
            else:
                print("mavi havuza gidiliyor...")
                if vehicle.commands.next == len(cmds):
                    print("mavi havuza ulaşıldı")
                    InsideCircle = False
                    MODE_IRIS = MODE_BLUE_ALIGN

        elif MODE_IRIS == MODE_BLUE_ALIGN:
            if LAST_MODE == MODE_GO_BLUE:
                print("mode blue align")
                vehicle.mode = VehicleMode("GUIDED")
                time.sleep(2)
                LAST_MODE = MODE_BLUE_ALIGN

            Ymovement = Ymovement * 30
            Xmovement = Xmovement * 30
            altsinir = 0.2
            if Ymovement < altsinir and Ymovement > -altsinir:
                if Ymovement < 0:
                    Ymovement = -altsinir
                else:
                    Ymovement = altsinir
            if Xmovement < altsinir and Xmovement > -altsinir:
                if Xmovement < 0:
                    Xmovement = -altsinir
                else:
                    Xmovement = altsinir

            if(InsideCircle == False):
                if Xmovement != 0.0:
                    print("X: " + str(Xmovement) + "      Y:" + str(Ymovement))
                    goto_position_target_local_ned(Ymovement, Xmovement, 0)

            else:
                if vehicle.location.global_relative_frame.alt < 5:
                    print("iniş tamamlandı su çekiliyor")
                    time.sleep(10)
                    print("su çekildi")
                    time.sleep(2)
                    InsideCircle = False
                    MODE_IRIS = MODE_GO_RED 
                else:
                    print("iniş")
                    goto_position_target_local_ned(Ymovement, Xmovement, 0.5)
                    print(" Attitude: %s" % vehicle.location.global_relative_frame.alt)

        elif MODE_IRIS == MODE_GO_RED:           
            if LAST_MODE == MODE_BLUE_ALIGN:
                print("MODE go red")            
                cmds.clear()
                time.sleep(2)
                cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 20))
                cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,red.x ,red.y,20))
                cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,red.x ,red.y,20))
                cmds.upload()                
                vehicle.mode = VehicleMode("AUTO")
                time.sleep(2)
                LAST_MODE == MODE_GO_RED
            else:
                print("kırmızı havuza gidiliyor...")
                if vehicle.commands.next == len(cmds):
                    print("kırmızı havuza ulaşıldı")
                    InsideCircle = False
                    MODE_IRIS = MODE_RED_ALIGN
                    vehicle.mode = VehicleMode("GUIDED")

        elif MODE_IRIS == MODE_RED_ALIGN:
            print("mode red align")
            LAST_MODE = MODE_RED_ALIGN
            time.sleep(2)
            Ymovement = Ymovement * 30
            Xmovement = Xmovement * 30
            altsinir = 0.2
            if Ymovement < altsinir and Ymovement > -altsinir:
                if Ymovement < 0:
                    Ymovement = -altsinir
                else:
                    Ymovement = altsinir
            if Xmovement < altsinir and Xmovement > -altsinir:
                if Xmovement < 0:
                    Xmovement = -altsinir
                else:
                    Xmovement = altsinir

            if(InsideCircle == False):
                if Xmovement != 0.0:
                    print("X: " + str(Xmovement) + "      Y:" + str(Ymovement))
                    goto_position_target_local_ned(Ymovement, Xmovement, 0)

            else:
                if vehicle.location.global_relative_frame.alt < 10:
                    print("iniş tamamlandı su bırakılıyor...")
                    time.sleep(10)
                    print("su bırakıldı")
                    time.sleep(2)
                    InsideCircle = False
                    MODE_IRIS = MODE_GO_HOME 
                else:
                    print("iniş")
                    goto_position_target_local_ned(Ymovement, Xmovement, 0.5)
                    print(" Attitude: %s" % vehicle.location.global_relative_frame.alt)
        elif MODE_IRIS == MODE_GO_HOME:
            if LAST_MODE == MODE_RED_ALIGN and LAST_MODE == MODE_NAVIGATION:
                vehicle.mode = VehicleMode("RTL")
                LAST_MODE = MODE_GO_HOME
            print("eve dönülüyor")
            time.sleep(1)
        
               
    
    print("bitti")





#0.00001140 yaklaşık 1 metre
#0.00002 denenecek değer  1.7 metre


		



#print(" Groundspeed: %s" % vehicle.groundspeed)    # settable
#print(" Airspeed: %s" % vehicle.airspeed)		
        
       
#def myhook():
#  print "shutdown time!"
#
#rospy.on_shutdown(myhook)	