#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import math
import RPi.GPIO as GPIO
import time
from sensor_msgs.msg import JointState

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

        # Detekcja zdarzeń z mniejszym bouncetime
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

    def get_count(self):
        return self.count

    def reset(self):
        self.count = 0

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('robot_odometry_publisher')

        # --- Parametry Robota ---
        self.wheel_radius = 0.0325
        self.wheel_separation = 0.269
        self.ticks_per_revolution = 2373  # Zachowana wartość
        self.linear_slip_factor = 1.437  # do ruchu liniowego
        self.angular_slip_factor_left = 1.164  # Dla obrotu w lewo
        self.angular_slip_factor_right = 1.155  # Dla obrotu w prawo 

        # --- Piny GPIO dla enkoderów (numery BCM) ---
        self.front_left_encoder_pin_a = 26
        self.front_left_encoder_pin_b = 15
        self.rear_left_encoder_pin_a = 21
        self.rear_left_encoder_pin_b = 20
        self.front_right_encoder_pin_a = 1
        self.front_right_encoder_pin_b = 9
        self.rear_right_encoder_pin_a = 16
        self.rear_right_encoder_pin_b = 25

        # --- Inicjalizacja GPIO ---
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # --- Inicjalizacja obiektów enkoderów ---
        self.front_left_encoder = Encoder(self.front_left_encoder_pin_a, self.front_left_encoder_pin_b, "front_left")
        self.rear_left_encoder = Encoder(self.rear_left_encoder_pin_a, self.rear_left_encoder_pin_b, "rear_left")
        self.front_right_encoder = Encoder(self.front_right_encoder_pin_a, self.front_right_encoder_pin_b, "front_right")
        self.rear_right_encoder = Encoder(self.rear_right_encoder_pin_a, self.rear_right_encoder_pin_b, "rear_right")

        # --- Zmienne Stanu Odometrii ---
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.prev_delta_left = 0.0
        self.prev_delta_right = 0.0
        self.joint_positions = {
            'left_front_wheel_joint': 0.0,
            'left_back_wheel_joint': 0.0,
            'right_front_wheel_joint': 0.0,
            'right_back_wheel_joint': 0.0,
        }

        # --- Inicjalizacja komponentów ROS2 ---
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.02, self.update_odometry)  # 50 Hz
        self.get_logger().info("OdometryPublisher node started with full quadrature decoding and separate angular slip factors.")

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if dt == 0:
            return

        # --- SEKCJA ODCZYTU ENKODERÓW ---
        delta_ticks_front_left = self.front_left_encoder.get_count()
        delta_ticks_rear_left = self.rear_left_encoder.get_count()
        delta_ticks_front_right = -self.front_right_encoder.get_count()  # Negacja zachowana
        delta_ticks_rear_right = -self.rear_right_encoder.get_count()

        # Wygładzanie danych z enkoderów
        alpha_encoder = 0.95
        delta_ticks_front_left = alpha_encoder * delta_ticks_front_left + (1 - alpha_encoder) * getattr(self, 'prev_ticks_front_left', delta_ticks_front_left)
        delta_ticks_rear_left = alpha_encoder * delta_ticks_rear_left + (1 - alpha_encoder) * getattr(self, 'prev_ticks_rear_left', delta_ticks_rear_left)
        delta_ticks_front_right = alpha_encoder * delta_ticks_front_right + (1 - alpha_encoder) * getattr(self, 'prev_ticks_front_right', delta_ticks_front_right)
        delta_ticks_rear_right = alpha_encoder * delta_ticks_rear_right + (1 - alpha_encoder) * getattr(self, 'prev_ticks_rear_right', delta_ticks_rear_right)

        self.prev_ticks_front_left = delta_ticks_front_left
        self.prev_ticks_rear_left = delta_ticks_rear_left
        self.prev_ticks_front_right = delta_ticks_front_right
        self.prev_ticks_rear_right = delta_ticks_rear_right

        # Reset liczników
        self.front_left_encoder.reset()
        self.rear_left_encoder.reset()
        self.front_right_encoder.reset()
        self.rear_right_encoder.reset()

        # Ograniczenie wartości impulsów
        MAX_TICKS_PER_CYCLE = 2000  # Zwiększono dla dużych ruchów
        delta_ticks_front_left = min(max(delta_ticks_front_left, -MAX_TICKS_PER_CYCLE), MAX_TICKS_PER_CYCLE)
        delta_ticks_rear_left = min(max(delta_ticks_rear_left, -MAX_TICKS_PER_CYCLE), MAX_TICKS_PER_CYCLE)
        delta_ticks_front_right = min(max(delta_ticks_front_right, -MAX_TICKS_PER_CYCLE), MAX_TICKS_PER_CYCLE)
        delta_ticks_rear_right = min(max(delta_ticks_rear_right, -MAX_TICKS_PER_CYCLE), MAX_TICKS_PER_CYCLE)

        # Korekcja różnic między kołami
        if abs(delta_ticks_front_left - delta_ticks_rear_left) > 50:
            delta_ticks_front_left = delta_ticks_rear_left = (delta_ticks_front_left + delta_ticks_rear_left) / 2.0
        if abs(delta_ticks_front_right - delta_ticks_rear_right) > 50:
            delta_ticks_front_right = delta_ticks_rear_right = (delta_ticks_front_right + delta_ticks_rear_right) / 2.0

        # --- SEKCJA OBLICZEŃ ---
        dist_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_revolution
        
        delta_fl_dist = delta_ticks_front_left * dist_per_tick
        delta_rl_dist = delta_ticks_rear_left * dist_per_tick
        delta_fr_dist = delta_ticks_front_right * dist_per_tick
        delta_rr_dist = delta_ticks_rear_right * dist_per_tick

        # Uśrednienie dystansu z wygładzaniem
        alpha = 0.8
        delta_left_wheel_dist = alpha * ((delta_fl_dist + delta_rl_dist) / 2.0) + (1 - alpha) * self.prev_delta_left
        delta_right_wheel_dist = alpha * ((delta_fr_dist + delta_rr_dist) / 2.0) + (1 - alpha) * self.prev_delta_right
        self.prev_delta_left = delta_left_wheel_dist
        self.prev_delta_right = delta_right_wheel_dist

        delta_linear = (delta_left_wheel_dist + delta_right_wheel_dist) / 2.0 * self.linear_slip_factor
        
        # Dodanie progu MIN_TICK_DIFF
        MIN_TICK_DIFF = 5
        delta_ticks_left = (delta_ticks_front_left + delta_ticks_rear_left) / 2.0
        delta_ticks_right = (delta_ticks_front_right + delta_ticks_rear_right) / 2.0
        if abs(delta_ticks_left - delta_ticks_right) < MIN_TICK_DIFF:
            delta_angular = 0.0
        else:
            delta_angular = (delta_right_wheel_dist - delta_left_wheel_dist) / self.wheel_separation
            angular_slip_factor = self.angular_slip_factor_right if delta_angular >= 0 else self.angular_slip_factor_left
            delta_angular = -delta_angular * angular_slip_factor  # Negacja dla poprawienia kierunku łuków

        # Logowanie danych z enkoderów i obliczeń
        self.get_logger().info(f"Encoders: FL={delta_ticks_front_left:.2f}, RL={delta_ticks_rear_left:.2f}, "
                               f"FR={delta_ticks_front_right:.2f}, RR={delta_ticks_rear_right:.2f}, "
                               f"X={self.x:.3f}, Y={self.y:.3f}, Theta={self.theta:.3f}, "
                               f"Angular={delta_angular/dt if dt > 0 else 0.0:.3f}")

        self.x += delta_linear * math.cos(self.theta + delta_angular / 2.0)
        self.y += delta_linear * math.sin(self.theta + delta_angular / 2.0)
        self.theta += delta_angular
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Obliczenia i publikacja stanów złączy
        self.joint_positions['left_front_wheel_joint'] += delta_fl_dist / self.wheel_radius
        self.joint_positions['left_back_wheel_joint'] += delta_rl_dist / self.wheel_radius
        self.joint_positions['right_front_wheel_joint'] += delta_fr_dist / self.wheel_radius
        self.joint_positions['right_back_wheel_joint'] += delta_rr_dist / self.wheel_radius

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = current_time.to_msg()
        joint_state_msg.name = list(self.joint_positions.keys())
        joint_state_msg.position = list(self.joint_positions.values())
        self.joint_state_publisher.publish(joint_state_msg)

        # Publikacja Transformacji TF
        odom_trans = TransformStamped()
        odom_trans.header.stamp = current_time.to_msg()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'
        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.rotation.z = math.sin(self.theta / 2)
        odom_trans.transform.rotation.w = math.cos(self.theta / 2)
        self.tf_broadcaster.sendTransform(odom_trans)

        # Publikacja wiadomości Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation = odom_trans.transform.rotation
        odom_msg.twist.twist.linear.x = delta_linear / dt if dt > 0 else 0.0
        odom_msg.twist.twist.angular.z = delta_angular / dt if dt > 0 else 0.0
        self.odom_publisher.publish(odom_msg)

    def destroy_node(self):
        GPIO.cleanup()
        self.get_logger().info("GPIO cleaned up.")
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