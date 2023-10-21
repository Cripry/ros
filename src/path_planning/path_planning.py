import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time

class PathPlanningNode(Node):

    def __init__(self):
        super().__init__('path_planning_node')
        self.subscription = self.create_subscription(Float32, 'lane_curvature_angle', self.callback, 10)
        self.arduino_serial = serial.Serial('/dev/ttyACM1', 115200)  # Check the correct port name for your Arduino

    def callback(self, msg):
        angle = msg.data

        # Basic logic to determine PWM and Servo Angle based on the lane angle
        pwm = 50  # Example value, you might have to adjust this based on the actual need
        servo_angle = 50  # Example value, you might adjust based on angle

        self.send_to_arduino(pwm, servo_angle)

    def send_to_arduino(self, pwm, servo_angle):
        if pwm != 300:
            command = "M" + str(pwm) + "\n"
            self.arduino_serial.write(command.encode())
            time.sleep(0.1)
        
        if servo_angle != 300:
            command = "S" + str(servo_angle) + "\n"
            self.arduino_serial.write(command.encode())
            time.sleep(0.1)

    def destroy_node(self):
        self.arduino_serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    path_planning_node = PathPlanningNode()
    rclpy.spin(path_planning_node)

    path_planning_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

