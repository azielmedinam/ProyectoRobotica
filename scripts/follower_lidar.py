#!/usr/bin/env python 
import rospy 
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# This class will subscribe to the /base_scan topic and print some data
class LaserSubClass(): 
    def __init__(self): 
        rospy.init_node("closest_object_detector", anonymous=True) #Para que todos los nombres sean diferentes
        rospy.on_shutdown(self.cleanup) 

       #Se suscribe al topico scan
        rospy.Subscriber("scan", LaserScan, self.lidar_cb)

        #Publica al topico cmd_vel
        self.move_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        #Constantes y variables
        self.lidar = LaserScan() #Aquí se almacenan los datos del lidar
        self.vel = Twist() #Almacena la velocidad que se manda a los motores
        self.min_range = 0.0  # Variable para almacenar la distancia física más cercana
        self.angle_min_range = 0.0  # Variable para almacenar el ángulo al objeto más cercana
        kw = 0.55 #Proportional constant for the angular speed controller
        kv = 0.33 #Proportional constant for the linear speed controller
        v = 0.0 #Robot linear speed [m/s]
        w = 0.0 #Robot angular speed [rad/s]
        
        r = rospy.Rate(10) #1Hz 
        print("Node initialized 10hz")  
        while not rospy.is_shutdown(): 
            #### ADD YOUR CODE ###
            if self.lidar.ranges: # We have received a valid Laser Scan
                
                closest_range = min(self.lidar.ranges) #Distance to the closest object
                self.min_range_index = self.lidar.ranges.index(closest_range)  # índice de la distancia más cercana
                self.min_range = self.lidar.ranges[self.min_range_index]  # Distancia física más cercana
                self.angle_min_range = self.lidar.angle_min + self.min_range_index * self.lidar.angle_increment
                print(closest_range)
                print("Angle:" + str(self.angle_min_range))

                if  self.min_range > 0.8:
                    self.vel.linear.x = 0
                    self.vel.angular.z = 1.0
                    print("Buscar")
                elif  self.min_range > 0.35 and self.angle_min_range > 0.80 or self.angle_min_range < -0.80:
                    w=kw*self.angle_min_range
                    self.vel.linear.x = 0
                    self.vel.angular.z = w
                    print("Giro")
                elif  self.min_range > 0.35 and self.angle_min_range < 1.0 and self.angle_min_range > -1.0:
                    v=kv*closest_range
                    w=kw*self.angle_min_range
                    self.vel.linear.x = v
                    self.vel.angular.z = w
                    print("Avanzo")
                elif closest_range < 0.35:
                    w=kw*self.angle_min_range
                    self.vel.linear.x = 0
                    self.vel.angular.z = w
                    print("acomodo")
                else:
                    self.stop()
                    print("stop")
                self.move_pub.publish(self.vel)
            r.sleep()  #It is very important that the r.sleep function is called at least once every cycle. 
    def lidar_cb(self, lidar_msg): 
        ## This function receives the lidar message and copies this message to a member of the class 
        self.lidar = lidar_msg

    def stop(self):
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.move_pub.publish(self.vel)
        
    def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot be0fore finishing a node.   
        self.stop()
        print("I'm dying, bye bye!!!") 

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    LaserSubClass() 
    rospy.spin()