import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import RPi.GPIO as GPIO
from maott4996_sign.msg import MotorBevegelse

class MotorNode(Node):
    def __init__(self):
        super().__init__('maott4996_motornode')
        self.motor_setup()
        self.subscription = self.create_subscription(
            MotorBevegelse,
            'maott4996_motorstyring',
            self.motor_callback,
            10
        )
        self.get_logger().info('Motornode startet')
    
    def motor_setup(self):
        # Setup GPIO and PWM motor control
        motor1_in1 = 23
        motor1_in2 = 24
        motor2_in1 = 14
        motor2_in2 = 15

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(motor1_in1, GPIO.OUT)
        GPIO.setup(motor1_in2, GPIO.OUT)
        GPIO.setup(motor2_in1, GPIO.OUT)
        GPIO.setup(motor2_in2, GPIO.OUT)

        self.pwm_motor1_in1 = GPIO.PWM(motor1_in1, 100)
        self.pwm_motor1_in2 = GPIO.PWM(motor1_in2, 100)
        self.pwm_motor2_in1 = GPIO.PWM(motor2_in1, 100)
        self.pwm_motor2_in2 = GPIO.PWM(motor2_in2, 100) 

    def motor_callback(self, msg):
        venstre = msg.venstre_motor_bevegelse
        hoyre = msg.hoyre_motor_bevegelse
        self.get_logger().info(f'Motor venstre signal = {venstre}, motor høyre signal = {hoyre}')
        if venstre < -100 or venstre > 100 or hoyre < -100 or hoyre > 100:
            self.get_logger.error('Ulovlig verdi for motorsignal mottatt. Må være mellom -100 og 100')
            return
        if (venstre == 0):
            self.pwm_motor1_in1.stop()
            self.pwm_motor1_in2.stop()
        elif (venstre > 0):
            self.pwm_motor1_in1.start(venstre)
            self.pwm_motor1_in2.start(0)
        elif (venstre < 0):
            self.pwm_motor1_in1.start(0)
            self.pwm_motor1_in2.start(-venstre)
        if (hoyre == 0):
            self.pwm_motor2_in1.stop()
            self.pwm_motor2_in2.stop()
        elif (hoyre > 0):
            self.pwm_motor2_in1.start(hoyre)
            self.pwm_motor2_in2.start(0)
        elif (hoyre < 0):
            self.pwm_motor2_in1.start(0)
            self.pwm_motor2_in2.start(-hoyre)

def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('Motornode stoppet')
    finally:
        GPIO.cleanup()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
