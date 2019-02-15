#!/usr/bin/env python
import rospy
import rospkg
from xbee import XBee
import serial
import tf
import math # for trig functions

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

rospack = rospkg.RosPack()

class botControl:
    """A class for controlling a two wheeled robot with three IR sensors and three encoders.
    This class communicates with the robot using an xbee enabling the robot to move, rotate
    and view the environemnt with IR sensors."""

    def __init__(self):
        "Initializes communication over the xbee and inital hardware values."
        # create vars for hardware vs simulation
        self.robot_mode = "HARDWARE_MODE"#"SIMULATION_MODE"
        #self.control_mode = "MANUAL_CONTROL_MODE"

        # Measured bot parameters
        self.wheel_radius = 0.035
        self.bot_radius = 0.07

        # setup xbee communication, change ttyUSB0 to the USB port dongle is in
        if (self.robot_mode == "HARDWARE_MODE"):
            self.serial_port = serial.Serial('/dev/ttyUSB0', 9600)
            print(" Setting up serial port")
            try:
                self.xbee = XBee(self.serial_port)
            except:
                print("Couldn't find the serial port")

        print("Xbee setup successful")
        self.address = '\x00\x0C'#you may use this to communicate with multiple bots

        #init an odometry instance, and configure odometry info
        self.odom_init()

        #init log file, "False" indicate no log will be made, log will be in e190_bot/data folder
        self.log_init(data_logging=True,file_name="log.txt")

        # Creates ROS nodes and a topic to control movement
        rospy.init_node('botControl', anonymous=True)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        self.pubOdom = rospy.Publisher('/odom', Odometry, queue_size=10)

        #self.pubDistL = rospy.Publisher('/distL', ir_sensor, queue_size=10)
        #self.pubDistC = rospy.Publisher('/distC', ir_sensor, queue_size=10)
        #self.pubDistR = rospy.Publisher('/distR', ir_sensor, queue_size=10)
        self.time = rospy.Time.now()
        self.count = 0;

        # Variables for calibration
        self.cal = 0
        self.cal_accumulate_l = 0
        self.cal_accumulate_r = 0
        rospy.Subscriber("/calibrate", Bool, self.calibrate_callback)
        rospy.Subscriber("/cmd_pwm", Vector2, self.cmd_pwm_callback)

        # Sets publishing rate
        self.rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            self.odom_pub();
            self.rate.sleep();

    # def ir_init(self):
    #     self.ir_L = ir_sensor()
    #     self.ir_C = ir_sensor()
    #     self.ir_R = ir_sensor()

    def odom_init(self):
        """Initializes the odometer variables for distance measurements."""
        self.Odom = Odometry()
        self.Odom.header.frame_id = "/odom"
        self.Odom.child_frame_id = "/base_link"
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.encoder_resolution = 1.0/1440.0
        self.last_encoder_measurementL = 0
        self.last_encoder_measurementR = 0
        self.diffEncoderL = 0
        self.diffEncoderR = 0
        self.bot_angle = 0

    def log_init(self,data_logging=False,file_name="log.txt"):
        """Initializes logging of key events."""
        self.data_logging=data_logging
        if(data_logging):
            self.file_name = file_name
            self.make_headers();

    def calibrate(self,LAvel,RAvel):
        """Takes in left and right angular velocities of wheels and outputs
        left and right PWM values."""
        # Force angular velocities to ints and scale to 0-255
        # TODO: calibrate this; currently set to "don't make scary noises from 0-1"
        LPWM = int(11.608*abs(LAvel)+3.2461)
        RPWM = int(11.134*abs(RAvel)+6.983)

        if(LPWM < 15):
            LPWM = 0 # Floor left
        elif(LPWM > 255):
            LPWM = 255 # Ceiling left
        if(RPWM < 15):
            RPWM = 0 # Floor right
        elif(RPWM > 255):
            RPWM = 255 # Ceiling right

        return LPWM, RPWM

    def cmd_vel_callback(self,CmdVel):
        """Converts input: CmdVel a ROS message composed of two 3-vectors named
                linear and angular
            to values that the robot understands and sends them to the robot
            over the xbee.
            """
        if(self.robot_mode == "HARDWARE_MODE"):
            # Keep as floats for now
            LAvel = (CmdVel.linear.x + CmdVel.angular.z * self.bot_radius) / self.wheel_radius
            RAvel = (CmdVel.linear.x - CmdVel.angular.z * self.bot_radius) / self.wheel_radius

            LPWM, RPWM = self.calibrate(LAvel, RAvel)

            # Invert direction so +x is forward
            LDIR = int(LAvel < 0)
            RDIR = int(RAvel < 0)

            # Assemble command and send to terminal and robot
            command = '$M ' + str(LDIR) + ' ' + str(LPWM) + ' ' + str(RDIR) + ' ' + str(RPWM) + '@'
            print(command)
            self.xbee.tx(dest_addr = self.address, data = command)

    def calibrate_callback(self, Calibrate):
        """Puts the robot in or out of calibration mode."""
        self.cal = Calibrate

    def cmd_pwm_callback(self.CmdPwm):
        """Responds to commands phrased as a vector of left, right PWM."""
        if(self.robot_mode == "HARDWARE_MODE"):
            # TODO do we want normal non-calibrate cmd_pwm? 
            # Assemble command and send to terminal and robot
            command = '$M ' + str(int(CmdPwm[0]<0)) + ' ' + str(abs(CmdPwm[0])) + ' ' + str(int(CmdPwm[1]<0)) + ' ' + str(abs(CmdPwm[1])) + '@'
            print("Calibrating with: "+command)
            self.xbee.tx(dest_addr = self.address, data = command)
            self.cal_accumulate_l = 0
            self.cal_accumulate_r = 0
            cal_time = rospy.Time.now()
            rospy.sleep(5) # Wait for 5 seconds as you calibrate
            cal_time = rospy.Time.now() - cal_time
            cal_angle_l = cal_accumulate_l
            cal_angle_r = cal_accumulate_r
            if(self.calibrate): 
                self.log_cal(CmdPwm[0], CmdPwm[1], cal_angle_l, cal_angle_r)

    def odom_pub(self):
        """Handles publishing of robot sensor data: sensor measurements and
        encoder measurements."""
        if(self.robot_mode == "HARDWARE_MODE"):
            self.count = self.count + 1
            print(self.count)

            # Save last encoder measurements
            self.last_encoder_measurementL += self.diffEncoderL
            self.last_encoder_measurementR += self.diffEncoderR

            command = '$S @'
            self.xbee.tx(dest_addr = self.address, data = command)
            try:
                update = self.xbee.wait_read_frame()
            except:
                pass

            data = update['rf_data'].decode().split(' ')[:-1]
            data = [int(x) for x in data]
            encoder_measurements = [x * math.pi / 720 for x in data[-2:]] #encoder readings as radians, 2d array

            #how about velocity?
            time_diff = rospy.Time.now() - self.time
            self.diffEncoderL = encoder_measurements[0] - self.last_encoder_measurementL
            self.diffEncoderR = encoder_measurements[1] - self.last_encoder_measurementR

            # If either measurement is old, reset encoder differences
            if(self.diffEncoderL > 1000 or self.diffEncoderR > 1000):
                self.diffEncoderL = 0
                self.diffEncoderR = 0

            self.cal_accumulate_l += diffEncoderL
            self.cal_accumulate_r += diffEncoderR

            del_theta = ((self.diffEncoderR - self.diffEncoderL) * self.wheel_radius)/(2 * self.bot_radius)
            del_s = ((self.diffEncoderR + self.diffEncoderL) * self.wheel_radius)/2

            # Update x and y with deltas 
            self.Odom.pose.pose.position.x += del_s * math.cos(del_theta/2)
            self.Odom.pose.pose.position.y += del_s * math.sin(del_theta/2)

            # self.Odom.pose.pose.position.x = encoder_measurements[0]/10000.0 #this won't work for sure
            # self.Odom.pose.pose.position.y = encoder_measurements[1]/10000.0
            self.Odom.pose.pose.position.z = .0
            self.bot_angle += del_theta
            self.bot_angle = self.bot_angle % (2*math.pi) # Loop
            quat = quaternion_from_euler(.0, .0, self.bot_angle)
            self.Odom.pose.pose.orientation.x = quat[0]
            self.Odom.pose.pose.orientation.y = quat[1]
            self.Odom.pose.pose.orientation.z = quat[2]
            self.Odom.pose.pose.orientation.w = quat[3]

            # #https://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29
            self.odom_broadcaster.sendTransform(
                (self.Odom.pose.pose.position.x, self.Odom.pose.pose.position.y, .0),
                tf.transformations.quaternion_from_euler(.0, .0, self.bot_angle),
                rospy.Time.now(),
                self.Odom.child_frame_id,
                self.Odom.header.frame_id,
            )

            self.pubOdom.publish(self.Odom) #we publish in /odom topic

            #about range sensors, update here
            range_measurements = data[:-2] #range readings are here, 3d array
            #self.pubRangeSensor(range_measurements)

        if(self.data_logging):
            self.log_data();

        self.time = rospy.Time.now()

    # def pubRangeSensor(self,ranges):

    #     #May be you want to calibrate them now? Make a new function called "ir_cal"
    #     self.ir_L.distance = ir_cal(ranges[0])
    #     self.ir_C.distance = ir_cal(ranges[1])
    #     self.ir_R.distance = ir_cal(ranges[2])

    #     self.pubDistL.publish(self.ir_L)
    #     self.pubDistC.publish(self.ir_C)
    #     self.pubDistR.publish(self.ir_R)

    def make_headers(self):
        """Makes necesary headers for log file."""
        f = open(rospack.get_path('e190_bot')+"/data/"+self.file_name, 'a+')
        # f.write('{0} {1:^1} {2:^1} {3:^1} {4:^1} \n'.format('R1', 'R2', 'R3', 'RW', 'LW'))
        f.write('{0} {1:^1} {2:^1} \n'.format('TIME','ENCL','ENCR'))
        f.close()

    def log_cal(self, LPWM, RPWM, LAng, RAng):
        """Log PWMs and angular velocities for calibration. Sorry, no headers for now."""
        f = open(rospack.get_path('e190_bot')+"/data/cal"+self.file_name, 'a+')

        data = [str(x) for x in [LPWM,RPWM,LAng,RAng]]

        f.write(' '.join(data) + '\n')
        f.close()

    def log_pwm(self, LPWM, RPWM):
        """Logs PWM changes for calibration."""
        f = open(rospack.get_path('e190_bot')+"/data/"+self.file_name, 'a+')

        data = [str(x) for x in ["------------------PWM-IS-NOW",LPWM,RPWM]]

        f.write(' '.join(data) + '\n')
        f.close()

    def log_data(self):
        """Logs data for debugging reference."""
        f = open(rospack.get_path('e190_bot')+"/data/"+self.file_name, 'a+')

        # data = [str(x) for x in [1,2,3,self.Odom.pose.pose.position.x,self.Odom.pose.pose.position.y]]
        data = [str(x) for x in [self.time,self.diffEncoderL,self.diffEncoderR]]

        f.write(' '.join(data) + '\n')
        f.close()

if __name__ == '__main__':
    try:
        bot = botControl()

    except rospy.ROSInterruptException:
        pass
