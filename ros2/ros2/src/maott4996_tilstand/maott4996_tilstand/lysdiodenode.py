import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import RPi.GPIO as GPIO
from time import sleep
from maott4996_sign.msg import BryterTilstand

class LysdiodeNode(Node):
    def __init__(self):
        super().__init__('maott4996_lysdiodenode')
        self.lysdiode_blinker_setup()
        self.subscription = self.create_subscription(
            BryterTilstand,
            'maott4996_brytertilstand',
            self.bryter_callback,
            10
        )
        self.get_logger().info('Lysdiodenode startet')
    
    def lysdiode_blinker_setup(self):
        self.led_pin = 26
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.led_pin, GPIO.OUT)

    def lysdiode_brytertilstand_0(self):
        pwm_blink = GPIO.PWM(self.led_pin, 2)
        pwm_blink.start(50) # 50% duty cycle
        sleep(3)
        pwm_blink.stop()

    def lysdiode_brytertilstand_1(self):
        GPIO.output(self.led_pin, GPIO.HIGH)
        sleep(3)
        GPIO.output(self.led_pin, GPIO.LOW)
    
    def bryter_callback(self, msg):
        if msg.bryter_tilstand == 1:
            self.get_logger().info('Brytertilstand 1')
            self.lysdiode_brytertilstand_1()
        elif msg.bryter_tilstand == 0:
            self.lysdiode_brytertilstand_0()
            self.get_logger().info('Brytertilstand 0')
        else:
            self.get_logger().error(f'Ukjent brytertilstand {msg.bryter_tilstand} mottatt')

def main(args=None):
    rclpy.init(args=args)
    node = LysdiodeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('Lysdiodenode stoppet')
    finally:
        GPIO.cleanup()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
