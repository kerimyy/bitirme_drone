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
MODE = MODE_NAVIGATION
def imageCallback(data):
    rate = rospy.Rate(2)
    global MODE
    if MODE == MODE_GO_HOME:
        return
    frame = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)	
    

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    redMask = cv2.inRange(hsv, (20, 150, 150), (190, 255, 255))
    redMask = cv2.erode(redMask, None, iterations=2)
    redMask = cv2.dilate(redMask, None, iterations=2)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blue_lower = np.array([94, 80, 2], np.uint8)
    blue_upper = np.array([126, 255, 255], np.uint8)
    blueMask = cv2.inRange(hsv, blue_lower, blue_upper)
    blueMask = cv2.erode(blueMask, None, iterations=2)
    
   
    RedFoundedContours = cv2.findContours(redMask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    RedFoundedContours = imutils.grab_contours(RedFoundedContours)
    BlueFoundedContours = cv2.findContours(blueMask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    BlueFoundedContours = imutils.grab_contours(BlueFoundedContours)
    center = None

    redv = False
    bluev = False
    Xmovement = 0
    Ymovement = 0

    if len(RedFoundedContours) > 0:
        Circle = max(RedFoundedContours, key=cv2.contourArea)
        ((x,y),raduis) = cv2.minEnclosingCircle(Circle)
        Middle = cv2.moments(Circle)

        center = (int(Middle["m10"] / Middle["m00"]), int(Middle["m01"] / Middle["m00"]))

        if raduis > 10:
            cv2.circle(frame,(int (x),int (y)),int(raduis),(0,255,0),2)
            cv2.circle(frame,center,5,(0,255,255),-1)
            points.appendleft(center)
            red.isFind = True
            redv = True
            print("kırmızı göründü")
            DxCount = 0.0
            DyCount = 0.0

            for i in np.arange(1, len(points)):
                if points[i - 1] is None or points[i] is None:
                    continue
                
                if i == 1:
                    DxCount = float(points[i][0])-320.0 
                    DyCount = 170 - float(points[i][1])
                    cv2.line(frame, points[i - 1], (320,250), (0, 0, 0), 5)
                    if (MODE == MODE_RED_ALIGN and int(points[i][0]) - 320)**2 + (int(points[i][1]) - 250)**2 < (raduis/2)**2:
                        InsideCircle = True

            if(counter % UpdateRate ) == 0 and (MODE == MODE_RED_ALIGN or MODE == MODE_NAVIGATION):

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

    
    if len(BlueFoundedContours) > 0:
        Circle = max(BlueFoundedContours, key=cv2.contourArea)
        ((x,y),raduis) = cv2.minEnclosingCircle(Circle)
        Middle = cv2.moments(Circle)

        center = (int(Middle["m10"] / Middle["m00"]), int(Middle["m01"] / Middle["m00"]))

        if raduis > 10:
            cv2.circle(frame,(int (x),int (y)),int(raduis),(0,255,0),2)
            cv2.circle(frame,center,5,(0,255,255),-1)
            points.appendleft(center)
            blue.isFind = True
            bluev = True
            print("mavi göründü")
            DxCount = 0.0
            DyCount = 0.0

            for i in np.arange(1, len(points)):
                if points[i - 1] is None or points[i] is None:
                    continue
                
                if i == 1:
                    DxCount = float(points[i][0])-320.0 
                    DyCount = 170 - float(points[i][1])
                    cv2.line(frame, points[i - 1], (320,250), (0, 0, 0), 5)
                    if (MODE == MODE_BLUE_ALIGN and int(points[i][0]) - 320)**2 + (int(points[i][1]) - 250)**2 < (raduis/2)**2:
                        InsideCircle = True

            if(counter % UpdateRate ) == 0 and (MODE == MODE_BLUE_ALIGN or MODE == MODE_NAVIGATION):

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
    
    if MODE == MODE_NAVIGATION:
        if vehicle.mode != VehicleMode("AUTO"):
            vehicle.mode = VehicleMode("AUTO")
            print("mode navigation")

        elif red.isFind and blue.isFind:
            MODE = MODE_GO_BLUE
            return
        elif vehicle.commands.next == len(cmds):
            print("havuzlar bulunamadı eve dönülüyor")
            MODE = MODE_RTL
            return
        print(vehicle.commands.next,"/",len(cmds),"alan taranıyor")
    
    elif MODE == MODE_GO_BLUE:
        print("mode go blue")
        if vehicle.mode is not VehicleMode("GUIDED"):            
            vehicle.mode = VehicleMode("GUIDED")
            cmds.clear()
            time.sleep(2)
            cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 20))
            cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,blue.x ,blue.y,20))
            cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,blue.x ,blue.y,20))
            cmds.upload()
            time.sleep(1)
            vehicle.mode = VehicleMode("AUTO")
        else:
            print("mavi havuza gidiliyor...")
            if vehicle.commands.next == len(cmds):
                print("mavi havuza ulaşıldı")
                InsideCircle = False
                MODE = MODE_BLUE_ALIGN

    elif MODE == MODE_BLUE_ALIGN:
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
                InsideCircle = False
                vehicle.mode = VehicleMode("LOITER")
                MODE = MODE_GO_RED 
            else:
                print("iniş")
                goto_position_target_local_ned(Ymovement, Xmovement, 0.5)
                print(" Attitude: %s" % vehicle.location.global_relative_frame.alt)

    elif MODE == MODE_GO_RED:
        print("MODE go red")
        if vehicle.mode is not VehicleMode("GUIDED"):            
            vehicle.mode = VehicleMode("GUIDED")
            cmds.clear()
            time.sleep(2)
            cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 20))
            cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,red.x ,red.y,20))
            cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,red.x ,red.y,20))
            cmds.upload()
            time.sleep(1)
            vehicle.mode = VehicleMode("AUTO")
        else:
            print("kırmızı havuza gidiliyor...")
            if vehicle.commands.next == len(cmds):
                print("kırmızı havuza ulaşıldı")
                InsideCircle = False
                MODE = MODE_RED_ALIGN

    elif MODE == MODE_RED_ALIGN:
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
                InsideCircle = False
                vehicle.mode = VehicleMode("RTL")
                MODE = MODE_GO_HOME 
            else:
                print("iniş")
                goto_position_target_local_ned(Ymovement, Xmovement, 0.5)
                print(" Attitude: %s" % vehicle.location.global_relative_frame.alt)
            
    cv2.circle(frame,(320,250),10,(255,0,0),-1)
    cv2.imshow("Renk Tanima", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    cv2.waitKey(3)
    rate.sleep()
    
           

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