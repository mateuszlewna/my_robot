#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import math
import RPi.GPIO as GPIO
import time
from sensor_msgs.msg import JointState # Dodany import dla JointState

# Klasa do obsługi pojedynczego enkodera kwadraturowego
class Encoder:
    def __init__(self, pin_a, pin_b, name="encoder"):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.name = name
        self.count = 0

        # Ustaw tryb pinów i włącz pull-upy
        GPIO.setup(self.pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Początkowe stany pinów
        self.last_a_state = GPIO.input(self.pin_a)
        self.last_b_state = GPIO.input(self.pin_b) 

        # Dodaj detekcję zdarzeń (przerwania) dla obu pinów A i B
        # bouncetime=1 to bardzo krótki czas, dla lepszej stabilności można go zwiększyć, np. do 5-10
        GPIO.add_event_detect(self.pin_a, GPIO.BOTH, callback=self._encoder_callback, bouncetime=1)
        GPIO.add_event_detect(self.pin_b, GPIO.BOTH, callback=self._encoder_callback, bouncetime=1)

    def _encoder_callback(self, channel):
        current_a_state = GPIO.input(self.pin_a)
        current_b_state = GPIO.input(self.pin_b)

        # Logika dekodowania kwadraturowego
        if current_a_state != self.last_a_state:
            if current_a_state != current_b_state:
                self.count += 1
            else:
                self.count -= 1
        elif current_b_state != self.last_b_state:
            if current_a_state == current_b_state:
                self.count += 1
            else:
                self.count -= 1

        self.last_a_state = current_a_state
        self.last_b_state = current_b_state

    def get_and_reset_count(self):
        current_count = self.count
        self.count = 0
        return current_count

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('robot_odometry_publisher')

        # --- Parametry Robota (USTAWIONE NA TWOJE ZMIERZONE WARTOŚCI!) ---
        self.wheel_radius = 0.0325     # Promień koła w metrach (32.5 mm, z URDF)
        self.wheel_separation = 0.232*2  # Rozstaw kół w metrach (232 mm, z URDF, odległość między środkami kół)
                                       # Z Twojego URDF: (0.116 * 2) = 0.232 razy dwa zeby wyregulowac obrót
        self.ticks_per_revolution = 2507 # Uśredniona liczba impulsów na obrót KOŁA

        # Piny GPIO dla enkoderów (numery BCM) - Zgodnie z Twoimi podłączeniami
        # Lewa strona
        self.front_left_encoder_pin_a = 26 # Żółty - Lewy przedni
        self.front_left_encoder_pin_b = 15 # Zielony - Lewy przedni
        self.rear_left_encoder_pin_a = 21  # Żółty - Lewy tylny
        self.rear_left_encoder_pin_b = 11  # Zielony - Lewy tylny

        # Prawa strona
        self.front_right_encoder_pin_a = 1 # Żółty - Prawy przedni
        self.front_right_encoder_pin_b = 9 # Zielony - Prawy przedni
        self.rear_right_encoder_pin_a = 16 # Żółty - Prawy tylny
        self.rear_right_encoder_pin_b = 25 # Zielony - Prawy tylny

        # --- Inicjalizacja GPIO ---
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # --- Inicjalizacja obiektów enkoderów ---
        self.front_left_encoder = Encoder(self.front_left_encoder_pin_a, self.front_left_encoder_pin_b, "front_left_encoder")
        self.rear_left_encoder = Encoder(self.rear_left_encoder_pin_a, self.rear_left_encoder_pin_b, "rear_left_encoder")
        self.front_right_encoder = Encoder(self.front_right_encoder_pin_a, self.front_right_encoder_pin_b, "front_right_encoder")
        self.rear_right_encoder = Encoder(self.rear_right_encoder_pin_a, self.rear_right_encoder_pin_b, "rear_right_encoder")

        # --- Zmienne Stanu Odometrii ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Skumulowane pozycje kątowe jointów kół
        self.joint_positions = {
            'left_front_wheel_joint': 0.0,
            'left_back_wheel_joint': 0.0,
            'right_front_wheel_joint': 0.0,
            'right_back_wheel_joint': 0.0,
        }

        # --- Inicjalizacja komponentów ROS2 ---
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10) # WYDAWCA JOINT_STATES

        # --- Timer do aktualizacji odometrii i joint_states ---
        self.timer = self.create_timer(0.02, self.update_odometry) 
        self.get_logger().info("OdometryPublisher węzeł uruchomiony z pełnym dekodowaniem kwadraturowym.")

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt == 0:
            return

        delta_ticks_front_left = self.front_left_encoder.get_and_reset_count()
        delta_ticks_rear_left = self.rear_left_encoder.get_and_reset_count()
        delta_ticks_front_right = self.front_right_encoder.get_and_reset_count()
        delta_ticks_rear_right = self.rear_right_encoder.get_and_reset_count()

        # KOREKTA KIERUNKU (ODWRACAMY ZNAK DLA PRAWYCH KÓŁ, JEŚLI W ZALEŻNOŚCI OD PODŁĄCZENIA)
        # Zgodnie z Twoją uwagą: prawe koła zliczały ujemnie dla ruchu do przodu
        delta_ticks_front_right = -delta_ticks_front_right
        delta_ticks_rear_right = -delta_ticks_rear_right

        # Uśrednianie impulsów dla lewej i prawej strony
        avg_delta_ticks_left = (delta_ticks_front_left + delta_ticks_rear_left) / 2.0
        avg_delta_ticks_right = (delta_ticks_front_right + delta_ticks_rear_right) / 2.0

        # Konwertuj uśrednione tyki na przemieszczenie liniowe koła w metrach
        delta_left_wheel_dist = (avg_delta_ticks_left / self.ticks_per_revolution) * (2 * math.pi * self.wheel_radius)
        delta_right_wheel_dist = (avg_delta_ticks_right / self.ticks_per_revolution) * (2 * math.pi * self.wheel_radius)

        # Obliczanie przemieszczenia liniowego i kątowego robota
        delta_linear = (delta_left_wheel_dist + delta_right_wheel_dist) / 2.0
        delta_angular = (delta_right_wheel_dist - delta_left_wheel_dist) / self.wheel_separation

        # Aktualizacja pozycji i orientacji
        self.x += delta_linear * math.cos(self.theta + delta_angular / 2.0)
        self.y += delta_linear * math.sin(self.theta + delta_angular / 2.0)
        self.theta += delta_angular

        # Ogranicz kąt theta do zakresu (-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # --- Obliczanie i publikacja stanów jointów kół ---
        # Uproszczone: obliczamy kąt obrotu na podstawie przemieszczenia liniowego koła
        # dla continuous joints w URDF, liczymy sumaryczny kąt.
        angle_moved_left = delta_left_wheel_dist / self.wheel_radius
        angle_moved_right = delta_right_wheel_dist / self.wheel_radius

        # Aktualizuj skumulowane pozycje kątowe jointów
        # UWAGA: Kierunek obrotu dla prawych kół zależy od orientacji osi w URDF
        # Jeśli 'axis xyz="0 0 -1"' dla prawych kół w URDF, to obrót w jednym kierunku
        # może być + lub - w zależności od konwencji.
        # W URDF jest 'rpy="${pi/2} 0 0"' i 'axis xyz="0 0 -1"' - to może oznaczać,
        # że obrót dodatni w kodzie enkodera powinien być ujemnym kątem dla jointa
        # lub na odwrót. Jeśli koła będą obracać się w przeciwną stronę, zmień znak.
        self.joint_positions['left_front_wheel_joint'] += angle_moved_left
        self.joint_positions['left_back_wheel_joint'] += angle_moved_left
        # Prawdopodobnie musisz odwrócić znak dla prawych kół ze względu na axis xyz="0 0 -1" w URDF
        self.joint_positions['right_front_wheel_joint'] += -angle_moved_right 
        self.joint_positions['right_back_wheel_joint'] += -angle_moved_right


        # Tworzenie wiadomości JointState
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = current_time.to_msg()
        joint_state_msg.name = [
            'left_front_wheel_joint',
            'left_back_wheel_joint',
            'right_front_wheel_joint',
            'right_back_wheel_joint'
        ]
        joint_state_msg.position = [
            self.joint_positions['left_front_wheel_joint'],
            self.joint_positions['left_back_wheel_joint'],
            self.joint_positions['right_front_wheel_joint'],
            self.joint_positions['right_back_wheel_joint']
        ]
        # Opcjonalnie: dodaj prędkości kątowe, jeśli są potrzebne (dla wizualizacji nie są konieczne)
        joint_state_msg.velocity = [
            angle_moved_left / dt,
            angle_moved_left / dt,
            -angle_moved_right / dt, # Utrzymaj spójność z kierunkiem pozycji
            -angle_moved_right / dt
        ]

        self.joint_state_publisher.publish(joint_state_msg)

        # --- Publikacja Transformacji TF (`odom` -> `base_link`) ---
        odom_trans = TransformStamped()
        odom_trans.header.stamp = current_time.to_msg()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'

        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0

        q = self.euler_to_quaternion(0, 0, self.theta)
        odom_trans.transform.rotation.x = q[0]
        odom_trans.transform.rotation.y = q[1]
        odom_trans.transform.rotation.z = q[2]
        odom_trans.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(odom_trans)

        # --- Publikacja wiadomości Odometry (`/odom`) ---
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = odom_trans.transform.rotation

        current_linear_x = delta_linear / dt
        current_angular_z = delta_angular / dt

        odom_msg.twist.twist.linear.x = current_linear_x
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = current_angular_z

        odom_msg.pose.covariance = [0.0] * 36
        odom_msg.twist.covariance = [0.0] * 36

        self.odom_publisher.publish(odom_msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def destroy_node(self):
        GPIO.cleanup()
        self.get_logger().info("Wyczyszczono GPIO.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    try:
        rclpy.spin(odometry_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        odometry_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()