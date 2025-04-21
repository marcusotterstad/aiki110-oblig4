import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import RPi.GPIO as GPIO
from maott4996_sign.srv import Maott4996Brytertilstand

class BryterTjeneste(Node):
    def __init__(self):
        super().__init__('maott4996_brytertjeneste')
        self.bryter_reader_setup()
        self.srv = self.create_service(
            Maott4996Brytertilstand,
            'maott4996_brytertilstand',
            self.bryter_callback
        )
        self.get_logger().info('Brytertjeneste startet')

    def bryter_reader_setup(self):
        self.switch_pin = 16
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.switch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def bryter_reader(self):
        tilstand = 1 if GPIO.input(self.switch_pin) == GPIO.LOW else 0
        # self.get_logger().info(f"Brytertilstand = {tilstand}")
        return tilstand

    def bryter_callback(self, request, response):
        if request.bryter_id != 1:
            response.bryter_tilstand = -1
        else:
            response.bryter_tilstand = self.bryter_reader()
        return response

def main(args=None):
    rclpy.init(args=args)
    node = BryterTjeneste()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('Brytertjeneste stoppet')
    finally:
        GPIO.cleanup()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
