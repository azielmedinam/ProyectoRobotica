#!/usr/bin/env python

import rospy
import time
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32 
from std_msgs.msg import String 
import cv2

class LaserSubClass():
    def __init__(self):
        rospy.init_node("select_mode_color_detector", anonymous=True)
        rospy.on_shutdown(self.cleanup)
        
        ##### Subscribers #####
        rospy.Subscriber("scan", LaserScan, self.lidar_cb)
        rospy.Subscriber('color', String, self.color_cb)
        rospy.Subscriber('modo', Int32, self.modo_cb)

        ##### Publisher #####
        self.pub_message = rospy.Publisher('message', String, queue_size=1) 
        self.move_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pub_sound = rospy.Publisher('sound', Int32, queue_size=1)

        ##### Constant and variables #####
        self.color_received_flag =0 #This flag will tell us when at least one color has been received. 
        self.message = String()
        self.lidar = LaserScan()
        self.vel = Twist()
        self.min_range = 0.0
        self.angle_min_range = 0.0
        self.modo = 1
        self.sound = 0
        kw = 0.55
        kv = 0.33
        v = 0.0
        w = 0.0
        d_stop = 0.9

        self.video_capture = cv2.VideoCapture(0)

        r = rospy.Rate(10)  # 10Hz
        print("Node initialized 10Hz")
        """while not rospy.is_shutdown():"""
        print("Ingrese el modo de operación al tópico /modo: 1. S-Tracker 2.Follower")
       
        while not rospy.is_shutdown():   
            
            ret, frame = self.video_capture.read()
            cv2.imshow("Original", frame)

            # Romper el bucle si se presiona la tecla 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            if self.modo == 1:
                print("Hola este es el S-Tracker")
                self.message.data = "Modo S-Tracker"
                self.pub_message.publish(self.message) #publish the message
                if self.video_capture.isOpened():
                    ret, frame = self.video_capture.read()
                    cv2.imshow("Original", frame)
                    if ret:
                        if self.color_received_flag: #If the flag is 1, then publish the message
                            if self.color == "naranja":
                                if self.detect_orange_color(frame):
                                    self.stop()
                                    self.message.data = "naranja encontrado"
                                    self.pub_message.publish(self.message) #publish the message
                                    print("naranja encontrado")
                                    self.sound = 1
                                else:
                                    self.message.data = "Buscando naranja"
                                    self.obstacle_avoidance()
                                    self.pub_message.publish(self.message) #publish the message
                                    print(self.message)
                                    self.sound = 0
                            elif self.color == "amarillo":
                                if self.detect_yellow_color(frame):
                                    self.stop()
                                    self.message.data = "amarillo encontrado"
                                    self.pub_message.publish(self.message) #publish the message
                                    print("amarillo encontrado")
                                    self.sound = 1
                                else:
                                    self.message.data = "Buscando amarillo"
                                    self.obstacle_avoidance()
                                    self.pub_message.publish(self.message) #publish the message
                                    print(self.message)
                                    self.sound = 0
                            elif self.color == "verde":
                                if self.detect_green_color(frame):
                                    self.stop()
                                    self.message.data = "verde encontrado"
                                    self.pub_message.publish(self.message) #publish the message
                                    print("verde encontrado")
                                    self.sound = 1
                                else:
                                    self.message.data = "Buscando verde"
                                    self.obstacle_avoidance()
                                    self.pub_message.publish(self.message) #publish the message
                                    print(self.message)
                                    self.sound = 0
                            elif self.color == "azul":
                                if self.detect_blue_color(frame):
                                    self.stop()
                                    self.message.data = "azul encontrado"
                                    self.pub_message.publish(self.message) #publish the message
                                    print("azul encontrado")
                                    self.sound = 1
                                else:
                                    self.message.data = "Buscando azul"
                                    self.obstacle_avoidance()
                                    self.pub_message.publish(self.message) #publish the message
                                    print(self.message)
                                    self.sound = 0
                            elif self.color == "rosa":
                                if self.detect_pink_color(frame):
                                    self.stop()
                                    self.message.data = "rosa encontrado"
                                    self.pub_message.publish(self.message) #publish the message
                                    print("rosa encontrado")
                                    self.sound = 1
                                else:
                                    self.message.data = "Buscando rosa"
                                    self.obstacle_avoidance()
                                    self.pub_message.publish(self.message) #publish the message
                                    print(self.message)
                                    self.sound = 0
                            self.pub_sound.publish(self.sound) #publish the message
            elif self.modo == 2:
                print("Hola, este es el Follower")
                self.message.data = "Modo Follower"
                self.pub_message.publish(self.message) #publish the message
                if self.lidar.ranges:
                    closest_range = min(self.lidar.ranges)
                    self.min_range_index = self.lidar.ranges.index(closest_range)
                    self.min_range = self.lidar.ranges[self.min_range_index]
                    self.angle_min_range = self.lidar.angle_min + self.min_range_index * self.lidar.angle_increment
                    print(closest_range)
                    print("Angle:" + str(self.angle_min_range))

                    if self.min_range > 0.8:
                        self.vel.linear.x = 0
                        self.vel.angular.z = 1.0
                        print("Buscar")
                        self.message.data = "Modo Follower: buscando"
                        self.pub_message.publish(self.message) #publish the message
                    elif self.min_range > 0.35 and (self.angle_min_range > 0.80 or self.angle_min_range < -0.80):
                        w = kw * self.angle_min_range
                        self.vel.linear.x = 0
                        self.vel.angular.z = w
                        print("Giro")
                        self.message.data = "Modo Follower: giro"
                        self.pub_message.publish(self.message) #publish the message
                    elif self.min_range > 0.35 and -1.0 < self.angle_min_range < 1.0:
                        v = kv * closest_range
                        w = kw * self.angle_min_range
                        self.vel.linear.x = v
                        self.vel.angular.z = w
                        print("Avanzo")
                        self.message.data = "Modo Follower: avanzo"
                        self.pub_message.publish(self.message) #publish the message
                    elif closest_range < 0.35:
                        w = kw * self.angle_min_range
                        self.vel.linear.x = 0
                        self.vel.angular.z = w
                        self.message.data = "Modo Follower: acomodo"
                        self.pub_message.publish(self.message) #publish the message
                        print("acomodo")
                    else:
                        self.stop()
                        print("stop")
                        self.message.data = "Modo Follower: stop"
                        self.pub_message.publish(self.message) #publish the message
                    self.move_pub.publish(self.vel)
            r.sleep()

    def detect_orange_color(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_orange = (5, 150, 150)
        upper_orange = (15, 255, 255)
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            return True
        else:
            return False

    def detect_yellow_color(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = (17, 37, 164)
        upper = (25, 255, 255)
        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            return True
        else:
            return False

    def detect_green_color(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = (32, 39, 109)
        upper = (81, 152, 255)
        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            return True
        else:
            return False

    def detect_blue_color(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = (80, 39, 147)
        upper = (114, 152, 255)
        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            return True
        else:
            return False

    def detect_pink_color(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = (162, 52, 118)
        upper = (179, 255, 255)
        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            return True
        else:
            return False

    def color_cb(self, color): 
        ## This function receives a number  
        print("I'm in the color callback")
        self.color =  color.data #number.data is the integer we want to use. 
        #print(self.number)
        self.color_received_flag = 1 # This flag is set to 1 the first time we receive a message in the /number topic.

    def modo_cb(self, modo): 
        ## This function receives a number  
        print("Modo de operación")
        self.modo =  modo.data #number.data is the integer we want to use. 
        #print(self.number)

    def lidar_cb(self, lidar_msg):
        self.lidar = lidar_msg

    def obstacle_avoidance(self):
        # self.lidar.ranges -> vector con las distancias de todos los objetos alrededor del robot
        if self.lidar.ranges: # We have received a valid laser Scan

            closest_range = min(self.lidar.ranges) # The distance to the closest object
            index = self.lidar.ranges.index(closest_range)
            closest_angle = self.lidar.angle_min + index * self.lidar.angle_increment

            print("closest_range: ")
            print(closest_range)
            print("closest_angle: ")
            print(closest_angle)

            # Apertura de visión de -1.2 a 1.2
            if closest_angle > -1.2 and closest_angle < 1.2 and closest_range < 0.9: # If the object is in front of the object, the closest angle is between a range of -1.1 to 1.1
                print("Object detected")
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.4
                self.message.data = "Modo S-Tracker: Objeto detectado"
                self.pub_message.publish(self.message) #publish the message
            elif closest_range < 0.5:
                print("Object too close detected. Stop until someone move")
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.0
                self.message.data = "Modo S-Tracker: Objeto muy cerca detectado. No avanzo hasta que alguien se mueva"
                self.pub_message.publish(self.message) #publish the message
            else:
                print("No object is detected")
                self.vel.linear.x = 0.4
                self.vel.angular.z = 0.0
                self.message.data = "Modo S-Tracker: No objeto detectado"
                self.pub_message.publish(self.message) #publish the message
            self.move_pub.publish(self.vel)
            
    def stop(self):
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.move_pub.publish(self.vel)

    def adelante(self):
        self.vel.linear.x = 0.2
        self.vel.angular.z = 0.0
        self.move_pub.publish(self.vel)

    def cleanup(self):
        self.stop()
        if self.video_capture.isOpened():
            self.video_capture.release()
            cv2.destroyAllWindows()
        print("I'm dying, bye bye!!!")


if __name__ == "__main__": 
    LaserSubClass()
    rospy.spin()