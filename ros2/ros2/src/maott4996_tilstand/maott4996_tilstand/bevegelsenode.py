import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from time import sleep
from maott4996_sign.msg import BryterTilstand
from maott4996_sign.msg import MotorBevegelse

class BevegelseNode(Node):
    def __init__(self):
        # Initialiser noden inkludert sett opp subscribe og publish 
        super().__init__('maott4996_bevegelsenode')
        self.subscription = self.create_subscription(
            BryterTilstand,
            'maott4996_brytertilstand',
            self.bryter_callback,
            10
        )
        self.publisher = self.create_publisher(
            MotorBevegelse,
            'maott4996_motorstyring',
            10
        )
        self.get_logger().info('Bevegelsenode startet')

    def bryter_callback(self, msg):
        # Hvis brytertilstand er 1 (på) så utfør en piruett, i motsatt fall ingenting
        if msg.bryter_tilstand == 1:
            self.do_piruett()
        elif msg.bryter_tilstand != 0:
            self.get_logger().warn(f'Ugyldig brytertilstand mottatt: {msg.bryter_tilstand}')

    def do_piruett(self):
        # Vent i ett sekund og utfør så piruett
        sleep(1)

        # For å få en fin piruett, så setter vi samme hastighet på begge hjulene, men de går hver sin vei.
        # Vi bruker moderat motorhastighet slik at vi får en jevn og fin piruett.
        # Vi har ingen feedback på posisjon så finner en varighet som gir en ca 360 graders piruett
        self.motor_bevegelse(-25, 25, 1.4) 

    def motor_bevegelse(self, venstre_bevegelse, hoyre_bevegelse, varighet):
        # Lag en ny motormelding, sett ønsket bevegelse og publiser denne slik at motornoden kan bevege motor
        msg = MotorBevegelse()
        msg.venstre_motor_bevegelse = venstre_bevegelse
        msg.hoyre_motor_bevegelse = hoyre_bevegelse
        self.publisher.publish(msg)

        # Vent så lenge som gitt av varighet parameter før motor stopp melding publiseres
        sleep(varighet)
        msg.venstre_motor_bevegelse = 0
        msg.hoyre_motor_bevegelse = 0
        self.publisher.publish(msg)

def main(args=None):
    # Sett opp noden, kjør den inntil programmet stoppes av et keyboard interrupt og så gjør oppryddning
    rclpy.init(args=args)
    node = BevegelseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('Bevegelsesnode stoppet')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

