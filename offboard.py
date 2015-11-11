#!/usr/bin/env python3
# vim:set ts=4sw=4 et:
#
# Copyright 2015 UAVenture AG.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
# for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
# Updated: Tarek Taha : tarek.taha@kustar.ac.ae
#    - Changed topic names after re-factoring : https://github.com/mavlink/mavros/issues/233
 
import rospy
import thread
from threading import Thread
import time
 
from geometry_msgs.msg import PoseStamped, Quaternion
from math import *
from mavros.srv import CommandBool
from mavros.utils import *

#from mavros.msg import ActuatorControl
from mavros_msgs.msg import ActuatorControl

from mavros.msg import VFR_HUD

from sensor_msgs.msg import NavSatFix

from std_msgs.msg import Header
from std_msgs.msg import String
import socket
import math
import Queue
import LatLon

#UDP_IP = "192.168.43.218"
UDP_IP = "10.42.1.1"
UDP_SEND_PORT = 5006
UDP_RECV_PORT = 5005

sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sock_recv.bind(("", UDP_RECV_PORT))

sock_recv.settimeout(0.1)



com_thread = Queue.Queue()
servo_queue = Queue.Queue()



class Heading_PID():

    def __init__(self):

        self.heading = 0
        self.desired_heading = 0
        self.servo_motor_value = 0
        self.offset = 0
        t1 = Thread(target = self.main_pid)
        t1.start()


    def update_heading(self, message):
	print "heading" + str(message.heading)
        self.heading = (message.heading + 180 - self.offset)%360

    def update_setpoint(self, desired_heading):

        desired_heading = desired_heading*-1 + 180

        self.offset = 180 - desired_heading

        desired_heading = desired_heading -180

        
        self.desired_heading =  180



    def main_pid(self):
        self.sub_hud = rospy.Subscriber('/mavros/vfr_hud', VFR_HUD, self.update_heading)

            
        dt = 0.15     
        Kp = 45
        Ki = 0
        Kd = 0

        previous_error = 0
        integral = 0


        while not rospy.is_shutdown():

           # if self.desired_heading - self.heading >

            error = (self.desired_heading - self.heading)
            
            integral = integral + error *  dt
            derivative = (error-previous_error)/dt
            output = Kp*error + Ki*integral + Kd*derivative
            previous_error = error
            



            if output > 360:
                output = 360
            elif output < -360:
                output = -360


            #print "Out" + str(output/360) + " Heading:" + str(self.heading) +" Desired:" + str(self.offset)
            #servo_queue.put(output/360)
            self.servo_motor_value = output/360



            time.sleep(dt)


    def getServo(self):
        return self.servo_motor_value


class Comunication():

    def __init__(self):
        
        #Swift Position
        self.swift_lat = 0
        self.swift_lon = 0   
        self.swift_rot = 0

        #Mission Waypoint details
        self.mode_waypoint = 0
        self.mission_start = 0
        self.mission_lat = []
        self.mission_lon = []
        self.mission_velocity = 0

        #Thread
        t1  =  Thread(target = self.communication_thread)       
        t1.start()


    def get_rotation(self, message):
        self.swift_rot = message.heading
	print "heading" + str(message.heading)

    def communication_thread(self):
        self.sub_pos = rospy.Subscriber('/mavros/vfr_hud', VFR_HUD,  self.get_rotation)

        while not rospy.is_shutdown():



        #receive Part

            try:
                data_rcv, addr = sock_recv.recvfrom(1024) # buffer size is 1024 bytes

                info = data_rcv.split(";")
                
                try:

                    index = info.index("Mission")
                #    self.mode_waypoint = 1
                    number_of_waypoints = int(info[2])


                    self.mission_velocity = float(info[3])

                    self.mission_lat = []
                    self.mission_lon = []

                    for x in range(0,  number_of_waypoints):
                        self.mission_lat.append(info[4+x*2])
                        self.mission_lon.append(info[5+x*2])



                    print(self.mission_lat)
                    print(self.mission_lon)

                except ValueError:
                    #print("coords not available")
                    pass



                try:
                    index = info.index("STOPMISSION")
                    self.mode_waypoint = 0
                except ValueError:
                    pass



                try:
                    index = info.index("STARTMISSION")
                    self.mode_waypoint = 1
                except ValueError:
                    pass

            except socket.timeout:
                pass


        #send Part
            try:
                
                #heartbeat message
                message_to_send = ";swift_alive"

                #swift position
                message_to_send = message_to_send + ";s_pos;"+str(self.swift_lat)+";"+str(self.swift_lon)+";"+str(self.swift_rot)

                sock_send.sendto(message_to_send, (UDP_IP, UDP_SEND_PORT))

            except IOError, e:
                print "Network unreachabel"


    def updateSwiftPos(self, message):
        self.swift_lat = message.latitude;
        self.swift_lon = message.longitude;

    def missionStatus(self):
        if self.mode_waypoint == 1:
            return 1
        else:
            return 0





class Mission_mode():

    def __init__(self):

        #Swift Position
        self.swift_lat = 0
        self.swift_lon = 0   

        self.waypoints_lat = [] 
        self.waypoints_lon = []
        self.waypoints_vel = 0


        self.mission_running = 0


        self.motor_direction = 0;


    def startMission(self):
        self.mission_running = 1
        t1 = Thread(target = self.mission_Thread)
        t1.start()

    def mission_Thread(self):

        waypoint_number = 0

        heading_pid = Heading_PID()

        while self.mission_running == 1:

            print "cooords = " + str(self.waypoints_lat[waypoint_number]) +"   "+ str(self.waypoints_lon[waypoint_number])


            swift_current_pos = LatLon.LatLon(LatLon.Latitude(self.swift_lat), LatLon.Longitude(self.swift_lon))
            next_waypoint = LatLon.LatLon(LatLon.Latitude(self.waypoints_lat[waypoint_number]), LatLon.Longitude(self.waypoints_lon[waypoint_number]))
            
            if swift_current_pos.distance(next_waypoint)*1000 < 1:
                print("Waypoint Reached")
                if waypoint_number >= len(self.waypoints_lat) -1:
                    self.mission_running = 0
                    waypoint_number = 0
                else:
                    waypoint_number  = waypoint_number +1
                    continue

            else:



                #calcular heading

                heading = swift_current_pos.heading_initial(next_waypoint)

                print "heading "+ str(heading)

                heading_pid.update_setpoint(heading)

                self.motor_direction = heading_pid.getServo()


            time.sleep(0.1)


    def waypoint_reached(self):
        pass



    def getMotorDirection(self):
        return self.motor_direction


    def getMotorThrotle(self):
        return self.waypoints_vel



    def setMissionWaypoints(self, lat, lon, velocity):
        self.waypoints_lat = lat
        self.waypoints_lon = lon
        self.waypoints_vel = velocity

    def setSwiftPosition(self, lat,lon):
        self.swift_lat = lat
        self.swift_lon = lon 




    def missionStop(self):
        self.mission_running = 0
        #parar a  se o utilizar quiser
        pass

    def isMissionRunning(self):
        return self.mission_running






class Main_app():

    def __init__(self):

        #self.main_pid = Heading_PID()
        self.comm = Comunication()
        self.mission_running = 0


        #self.main_pid.update_setpoint(270)
        self.main_ros_node()


    def swiftPosition(self,message):
        self.comm.updateSwiftPos(message)
        if self.mission_running == 1: #Mission Running
            self.mission.setSwiftPosition(message.latitude, message.longitude)



    def main_ros_node(self):
        self.sub_pos = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.swiftPosition)

        self.pub = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=10)
        

        rospy.init_node('Offboard', anonymous=True)
        rate = rospy.Rate(10) 

        msg = ActuatorControl()

        motor_direction = 0.0
        motor_throtle = 0.0


        mission_already_started = 0

        while not rospy.is_shutdown():

            self.pub.publish(msg)

            #Update Mission


            status = self.comm.missionStatus()
            
            if status == 1 and self.mission_running == 0:     #Waypoints mission
           
                self.mission = Mission_mode()
                self.mission_running = 1
                self.mission.setMissionWaypoints(self.comm.mission_lat, self.comm.mission_lon, self.comm.mission_velocity  )
                
          


            elif status == 1 and self.mission_running == 1:

                if mission_already_started == 0:
                    mission_already_started = 1
                    self.mission.startMission()

                elif self.comm.missionStatus() == 0:
                    self.mission.missionStop()
                    mission_already_started = 0

                self.mission_running = self.mission.isMissionRunning()


                motor_direction = self.mission.getMotorDirection()
                motor_throtle = self.mission.getMotorThrotle()

            elif self.comm.missionStatus() == 0 and mission_already_started == 1:
                self.mission.missionStop()
                mission_already_started =0

            else:
                self.mission_running = 0 
                motor_direction = 0.0
                motor_throtle = 0.0
    


            print str(motor_direction)
            print "mission_running" + str(self.mission_running)
            print "already_started" + str(mission_already_started)
            print "status" + str(status)
            

            

            #Fill message Header
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.group_mix = 1
            


            #Update Motors
            #msg.controls = [0,0,0,self.comm.mission_velocity, self.main_pid.getServo(),0.0,1,1]
            msg.controls = [0.43,0,0,float(motor_throtle), float(motor_direction),0,1,1]
            #msg.controls = [0,0,0,0, 0,0,1,1]
            self.pub.publish(msg)





            
            rate.sleep()



if __name__ == '__main__':

    try:
        app = Main_app() 
    except rospy.ROSInterruptException:
        pass
