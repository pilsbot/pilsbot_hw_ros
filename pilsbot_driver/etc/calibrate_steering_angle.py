#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from pilsbot_driver_msgs.msg import SteeringAxleSensorsStamped
from std_msgs.msg import String

import sys
import select
import tty
import termios
class NonBlockingConsole(object):
    # thanks SO
    def __enter__(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


    def get_data(self):
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return False




class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1



class HeadMCUSub(Node):
    endstop_l_angle_rad = -1
    endstop_l_raw = -1
    endstop_r_angle_rad =  1
    endstop_r_raw = -1
    
    current_raw = -1
    
    def __init__(self):
        super().__init__('SUB')
        self.subscription = self.create_subscription(
            SteeringAxleSensorsStamped,
            'head_mcu',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
    
    def is_endstop_calibration_complete(self):
        return self.endstop_l_raw >= 0 and self.endstop_r_raw >= 0

    def listener_callback(self, msg):
        self.current_raw = msg.sensors.steering_angle_raw
        print("raw value:" + str(int(self.current_raw)), end="\r")
        if( not self.is_endstop_calibration_complete()):
            if(not msg.sensors.endstop_l):  #inverted
                if(self.endstop_l_raw != self.current_raw):
                    self.endstop_l_raw = self.current_raw 
                    self.get_logger().info("got left endstop: " +
                        str(self.endstop_l_angle_rad)+", " + str(self.current_raw))
            if(not msg.sensors.endstop_r):  #inverted
                if(self.endstop_r_raw != self.current_raw):
                    self.endstop_r_raw = self.current_raw
                    self.get_logger().info("got right endstop: " +
                        str(self.endstop_r_angle_rad)+", " + str(self.current_raw))

def main(args=None):
    rclpy.init(args=args)

    headMCUsub = HeadMCUSub()
    
    # Note: This should be determined by external measurement
    headMCUsub.endstop_l_angle_rad = -1.5
    headMCUsub.endstop_r_angle_rad = 1.5
    
    print("Assuming location of left endstop at " +str(headMCUsub.endstop_l_angle_rad) + " rad")
    print("                    right endstop at " +str(headMCUsub.endstop_r_angle_rad) + " rad")
    print("\nPlease move axle to both endstops manually.")

    while( not headMCUsub.is_endstop_calibration_complete() ):
        #rclpy.spin_once(minimal_publisher)
        rclpy.spin_once(headMCUsub)

    print("Got endstops.")
    print("Please move axle as precise as possible facing 'forward' (0 rad) and press [ENTER]\n")
    with NonBlockingConsole() as nbc:
        while( True ):
            #rclpy.spin_once(minimal_publisher)
            rclpy.spin_once(headMCUsub)
            if nbc.get_data() == '\n':  # x1b is ESC
                break

    center_raw = headMCUsub.current_raw
    
    calculated_center = (headMCUsub.endstop_l_raw + headMCUsub.endstop_r_raw) / 2
    calculation_deviation = abs(calculated_center - center_raw)
    deviation_percent = round((calculation_deviation/calculated_center)*100, 2)
    if( deviation_percent > 1):
        print("Warning: Center value '" + str(int(center_raw)) + "'" +
        " deviates from calculated center '" + str(int(calculated_center)) + "' by " +
        str(calculation_deviation) + " units (" + str(deviation_percent) + "%)")
    
    print("\nSuggested config:\n")
    print("{\"calibration_val\" : [")
    print("  " + str(int(headMCUsub.endstop_l_raw)) + ", " + str(headMCUsub.endstop_l_angle_rad) + ",")
    print("  " + str(int(center_raw)) + ", 0.0,")
    print("  " + str(int(headMCUsub.endstop_r_raw)) + ", " + str(headMCUsub.endstop_r_angle_rad)) # + ",")
    print("]}")

    headMCUsub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()