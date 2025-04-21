import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from maott4996_sign.srv import Maott4996Brytertilstand
from maott4996_sign.msg import BryterTilstand  

class BryterMonitor(Node):
    def __init__(self):
        super().__init__('maott4996_brytermonitor')
        self.client = self.create_client(Maott4996Brytertilstand, 'maott4996_brytertilstand')
        self.get_logger().info('Brytermonitor startet')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Venter på brytertjenesten...')

        self.publisher = self.create_publisher(BryterTilstand, 'maott4996_brytertilstand', 10)

        self.prev_state = None
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1Hz
        self.get_logger().info('Monitor-timer aktivert, overvåker bryter...')


    def timer_callback(self):
        req = Maott4996Brytertilstand.Request()
        req.bryter_id = 1

        future = self.client.call_async(req)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            result = future.result()
            tilstand = result.bryter_tilstand

            if tilstand != self.prev_state:
                msg = BryterTilstand()
                msg.bryter_id = 1
                msg.bryter_tilstand = tilstand
                self.publisher.publish(msg)
                self.get_logger().info(f'Publiserte tilstand: {msg}')
                self.prev_state = tilstand
        except Exception as e:
            self.get_logger().error(f'Feil ved forespørsel: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = BryterMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('Brytermonitor stoppet')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

