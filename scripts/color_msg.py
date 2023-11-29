bin/env python 
import rospy 
from std_msgs.msg import Int32 
from std_msgs.msg import String 
#This class will subscribe to the /number topic and display the received number as a string message
class PubSubClass(): 
    def _init_(self): 
        rospy.init_node("color_message", anonymous=True) 
        rospy.on_shutdown(self.cleanup) 
        ###******* INIT PUBLISHERS *******### 
        self.pub_message = rospy.Publisher('message', String, queue_size=1) 
        ############################### SUBSCRIBERS ##################################### 
        rospy.Subscriber('color', String, self.color_cb)
        ############ CONSTANTS AND VARIABLES ################ 
        self.color_received_flag =0 #This flag will tell us when at least one number has been received. 
        self.message = String()
        #********** INIT NODE **********### 
        r = rospy.Rate(1) #1Hz 
        print("Node initialized 1hz")
        while not rospy.is_shutdown(): 
            if self.color_received_flag: #If the flag is 1, then publish the message
                if self.color == 'naranja':
                    self.message.data = "Buscando naranja"
                self.pub_message.publish(self.message) #publish the message
                print(self.message)
            #else: #We haven't received any number
                #print("I haven't received any number yet!")
            r.sleep()  #It is very important that the r.sleep function is called at least onece every cycle. 
    
    def color_cb(self, color): 
        ## This function receives a number  
        print("I'm in the color callback")
        self.color =  color.data #number.data is the integer we want to use. 
        #print(self.number)
        self.color_received_flag = 1 # This flag is set to 1 the first time we receive a message in the /number topic.
        
    def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.   
        print("I'm dying, bye bye!!!") 

############################### MAIN PROGRAM #################################### 
if _name_ == "_main_": 
    PubSubClass()