#!/usr/bin/python3

#--- Import packages
from motor import Motor
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy, Range


class Robot(Node):
    '''
    Elaborate later on but for now we'll add functionality for:
    - Controlling the robot via typed commands
    - Controlling the robot via the keyboard
    - Controlling the robot via a gamepad
    - Incorporate rplidar for obstacle avoidance
    
    '''

    def __init__(self, name, wheel_diameter=0.065, wheel_base=0.14, left_max_rpm=200.0, 
            right_max_rpm=200.0):
        '''
        Elaborate on any parameters called in the class constructor
        '''

        super().__init__(name)
        
        self._left_max_rpm = left_max_rpm
        self._right_max_rpm = right_max_rpm
        self._wheel_diameter = wheel_diameter
        self._wheel_base = wheel_base
        self.motor = Motor() # Initialize motor driver

        self.speed = 0.0
        self.spin = 0.0
        self.close = 0.30 # start slowing down when this close (adapt as needed)
        self.tooclose = 0.10 # no forward motion when this close (adapt as needed)
        self.distance = 100.0 # might need to play with this to suit the resolution of the rplidar a1

        self._command_subscription = self.create_subscription(
                String, 
                'command', 
                self._command_callback, 
                10)

        self._cmd_vel_subscription = self.create_subscription(
                Twist,
                'cmd_vel',
                self._cmd_vel_callback,
                2)
        
        self._joy_subscription = self.create_subscription(
                Joy,
                'joy',
                self._joy_callback,
                5)

        self._range_subscription = self.create_subscription(
                Range,
                'range',
                self._range_callback,
                5)

    # Stop motors
    '''
    0 - for left motor
    1 - for right motor
    '''
    def stop(self):
        self.motor.MotorStop(0)
        self.motor.MotorStop(1)

    # Maximum speed in metres per second (mps) at maximum revolutions per minute (rpm)
    def max_speed(self):
        rpm = (self._left_max_rpm + self_right_max_rpm) / 2.0
        mps = rpm * math.pi * self._wheel_diameter / 60.0
        return mps
        
    # Rotation in radians per second at max rpm
    def max_twist(self):
        return self.max_speed() / self._wheel_diameter
        
    #
    def _command_callback(self, msg):
        command = msg.data
        if command == 'forward':
            print('Moving forward...')
            self.motor.MotorRun(0, 'forward', 60)
            self.motor.MotorRun(1, 'forward', 60)
        elif command == 'backward':
            print('Moving backward...')
            self.motor.MotorRun(0, 'backward', 50)
            self.motor.MotorRun(1, 'backward', 50)
        elif command == 'left':
            print('Turning left...')
            self.motor.MotorRun(0, 'forward', 10)
            self.motor.MotorRun(1, 'forward', 60)
        elif command == 'right':
            print('Turning right...')
            self.motor.MotorRun(0, 'forward', 60)
            self.motor.MotorRun(1, 'forward', 10)
        elif command == 'stop':
            print('Stopping...')
            self.stop()
        else:
            print('Unknown command, stopping motors instead')
            self.stop()

        
    #
    def _joy_callback(self, msg):
        '''
        This translates buttons on a generic game controller into speed and spin

        Using:

        right joystick for left and right motion
        left joystick for forward and backward motion 
        R2 for emergency stop

        Rstick left/right         axes[2]    +1 (left)    to -1 (right)
        Lstick forward/backward   axes[1]    +1 (forward) to -1 (backward)
        R2                        buttons[7]  1 pressed, 0 otherwise
        '''

        if abs(msg.axes[2]) > 0.10:
            self.spin = msg.axes[2]
        else:
            self.spin = 0.0

        if abs(msg.axes[1]) > 0.10:
            self.speed = msg.axes[1]
        else:
            self.speed = 0.0

        if msg.buttons[7] == 1:
            self.speed = 0.0
            self.spin = 0.0

        self._set_motor_speeds()
        
    #
    def _cmd_vel_callback(self, msg):
        '''
        '''
        self.speed = msg.linear.x
        self.spin = msg.angular.z
        self._set_motor_speeds()

    #
    def _range_callback(self, msg):
        pass

    #
    def _set_motor_speeds(self):
        # TODO: inject a stop() if no speeds seen for a while
        #
        # First figure out the speed of each wheel based on spin: each wheel
        # covers self._wheel_base meters in one radian, so the target speed
        # for each wheel in meters per sec is spin (radians/sec) times
        # wheel_base divided by wheel_diameter
        #
        right_twist_mps = self.spin * self._wheel_base / self._wheel_diameter
        left_twist_mps = -1.0 * self.spin * self._wheel_base / self._wheel_diameter
        #
        # Now add in forward motion.
        #
        left_mps = self.speed + left_twist_mps
        right_mps = self.speed + right_twist_mps
        #
        # Convert meters/sec into RPM: for each revolution, a wheel travels
        # pi * diameter meters, and each minute has 60 seconds.
        #
        left_target_rpm = (left_mps * 60.0) / (math.pi * self._wheel_diameter)
        right_target_rpm = (right_mps * 60.0) /  (math.pi * self._wheel_diameter)
        #
        left_percentage = (left_target_rpm / self._left_max_rpm) * 100.0
        right_percentage = (right_target_rpm / self._right_max_rpm) * 100.0
        #
        # clip to +- 100%
        #left_percentage = max(min(left_percentage, 100.0), -100.0)
        #right_percentage = max(min(right_percentage, 100.0), -100.0)
        # clip to +- 60%
        left_percentage = max(min(left_percentage, 60.0), -60.0)
        right_percentage = max(min(right_percentage, 60.0), -60.0)
        #
        # Add in a governor to cap forward motion when we're about
        # to collide with something (but still backwards motion)
        governor = 1.0
        if self.distance < self.tooclose:
            governor = 0.0
        elif self.distance < self.close:
            governor = (self.distance - self.tooclose) / (self.close - self.tooclose)
        if right_percentage > 0:
            right_percentage *= governor
            #index_r = 'forward'
            index_r = 'backward'
        else:
            #index_r = 'backward'
            index_r = 'forward'
        if left_percentage > 0:
            left_percentage *= governor
            #index_l = 'forward'
            index_l = 'backward'
        else:
            #index_l = 'backward'
            index_l = 'forward'
            
        self.motor.MotorRun(0, index_l, abs(left_percentage))
        self.motor.MotorRun(1, index_r, abs(right_percentage))

def main(args=None):

    rclpy.init(args=args)
    
    # Initialize robot 
    robot = Robot('lidar_robot')

    print("Spinning...")
    rclpy.spin(robot)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# Adjust left and right max rpm if there is a way you can confirm if one wheel is faster than the other

# Adapt to our own needs as we gain more understanding of each section

# Play with queue number/topic frequency

# Will need to explain the conversions and critical parts of the code in the docs

# Try to simplify the motor class better to scrap the 'forward', 'backward' and 'index' if possible

# Find another name for 'governor'
