import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, LaserScan
from synapse_msgs.msg import EdgeVectors, TrafficStatus
import math

QOS_PROFILE_DEFAULT = 10

PI = math.pi

LEFT_TURN = +1.0
RIGHT_TURN = -1.0

TURN_MIN = 0.0
TURN_MAX = 1.0
SPEED_MIN = 0.0
SPEED_MAX = 1.0
SPEED_25_PERCENT = SPEED_MAX / 4
SPEED_50_PERCENT = SPEED_25_PERCENT * 2
SPEED_75_PERCENT = SPEED_25_PERCENT * 3

THRESHOLD_OBSTACLE_VERTICAL = 1.4
THRESHOLD_OBSTACLE_VERTICAL_1 = 0.1
THRESHOLD_OBSTACLE_VERTICAL_2 = 0.8
THRESHOLD_OBSTACLE_HORIZONTAL = 0.38
RAMP_THRESHOLD = 0.8  # Threshold for ramp detection
Counter = 0
Reverse = False

class LineFollower(Node):
    """ Initializes line follower node with the required publishers and subscriptions. """
    def __init__(self):
        super().__init__('line_follower')

        # Subscription for edge vectors.
        self.subscription_vectors = self.create_subscription(
            EdgeVectors,
            '/edge_vectors',
            self.edge_vectors_callback,
            QOS_PROFILE_DEFAULT
        )

        # Publisher for joy (for moving the rover in manual mode).
        self.publisher_joy = self.create_publisher(
            Joy,
            '/cerebri/in/joy',
            QOS_PROFILE_DEFAULT
        )

        # Subscription for traffic status.
        self.subscription_traffic = self.create_subscription(
            TrafficStatus,
            '/traffic_status',
            self.traffic_status_callback,
            QOS_PROFILE_DEFAULT
        )

        # Subscription for LIDAR data.
        self.subscription_lidar = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            QOS_PROFILE_DEFAULT
        )

        self.traffic_status = TrafficStatus()

        self.obstacle_detected = False
        self.ramp_detected = False

        # Variables to store obstacle distances
        self.min_front_left = float('inf')
        self.min_front_right = float('inf')
        self.min_side_left = float('inf')
        self.min_side_right = float('inf')

        # Initialize counter
        self.counter = 0

        # Initialize Reverse attribute
        self.Reverse = False

    def rover_move_manual_mode(self, speed, turn):
        msg = Joy()
        msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]
        msg.axes = [0.0, speed, 0.0, turn]
        self.publisher_joy.publish(msg)

    def reverse_direction(self):
        """ Function to reverse the buggy's direction. """
        speed = -0.25  # Adjust this speed as needed for reversing
        turn = 0.0
        self.rover_move_manual_mode(speed, turn)
        print("Reversing direction")

    def edge_vectors_callback(self, message):
        speed = 0.75
        turn = TURN_MIN

        vectors = message
        half_width = vectors.image_width / 2

        if vectors.vector_count == 0:  # No vectors
            speed = 0.35
        elif vectors.vector_count == 1:  # Curve
            deviation = vectors.vector_1[1].x - vectors.vector_1[0].x
            turn = 1.6* (deviation / vectors.image_width)
            speed = 0.45
        elif vectors.vector_count == 2:  # Straight
            middle_x_left = (vectors.vector_1[0].x + vectors.vector_1[1].x) / 2
            middle_x_right = (vectors.vector_2[0].x + vectors.vector_2[1].x) / 2
            middle_x = (middle_x_left + middle_x_right) / 2
            deviation = half_width - middle_x
            turn = (deviation / half_width) / 2

        if self.traffic_status.stop_sign:
            speed = SPEED_MIN
            print("Stop sign detected")

        if self.ramp_detected:
            speed = 0.35
            print("Ramp/bridge detected")
            self.rover_move_manual_mode(speed, turn)
            return

        if self.obstacle_detected:
            pass  # No action defined yet

        self.rover_move_manual_mode(speed, turn)

    def traffic_status_callback(self, message):
        self.traffic_status = message

    def lidar_callback(self, message):
        shield_vertical = 5.7
        shield_horizontal = 1.0
        speed = 0.10
        turn = 0.0
        theta = math.atan(shield_vertical / shield_horizontal)
        # Get the middle half of the ranges array.
        length = len(message.ranges)
        ranges = message.ranges[int(length / 4): int(3 * length / 4)]

        # Separate the ranges into front and sides.
        front_ranges = ranges[int(len(ranges) * theta / PI): int(len(ranges) * (PI - theta) / PI)]
        side_ranges_right = ranges[:int(len(ranges) * theta / PI)]
        side_ranges_left = ranges[int(len(ranges) * (PI - theta) / PI):]
        
        # Find the minimum values in each range section.
        self.min_front_right = min(front_ranges[:len(front_ranges) // 2], default=float('inf'))
        self.min_front_left = min(front_ranges[len(front_ranges) // 2:], default=float('inf'))
        self.min_side_left = min(side_ranges_left, default=float('inf'))
        self.min_side_right = min(side_ranges_right, default=float('inf'))
        
        # Check for ramp detection
        if 25 <= self.counter <= 38 or 38 <= self.counter <= 57 or 65<= self.counter <= 120:  # Counter check for ramp end
            self.ramp_detected = False
        if all(value <= 1.4 for value in front_ranges):  # Check for ramp beginning
            self.ramp_detected = True
            print("Ramp detected")
            self.counter += 1 
        elif (self.min_front_left <= THRESHOLD_OBSTACLE_VERTICAL or 
              self.min_front_right <= THRESHOLD_OBSTACLE_VERTICAL or 
              self.min_side_left <= THRESHOLD_OBSTACLE_HORIZONTAL or 
              self.min_side_right <= THRESHOLD_OBSTACLE_HORIZONTAL):   
              # Handle cases with obstacles detected in three points
            if (self.min_front_left < THRESHOLD_OBSTACLE_VERTICAL and 
                self.min_front_right < THRESHOLD_OBSTACLE_VERTICAL and 
                self.min_side_left < THRESHOLD_OBSTACLE_HORIZONTAL):
                # Obstacles in front_left, front_right, and side_left
                if (THRESHOLD_OBSTACLE_VERTICAL - self.min_front_right) > (THRESHOLD_OBSTACLE_HORIZONTAL - self.min_front_left):
                    turn = RIGHT_TURN
                else:
                    turn = LEFT_TURN
                print("Obstacle detected in front_left, front_right, and side_left")

            elif (self.min_front_left < THRESHOLD_OBSTACLE_VERTICAL and 
                  self.min_front_right < THRESHOLD_OBSTACLE_VERTICAL and 
                  self.min_side_right < THRESHOLD_OBSTACLE_HORIZONTAL):
                # Obstacles in front_left, front_right, and side_right
                if (THRESHOLD_OBSTACLE_VERTICAL - self.min_front_left) > (THRESHOLD_OBSTACLE_HORIZONTAL - self.min_side_right):
                    turn = LEFT_TURN
                else:
                    turn = RIGHT_TURN
                print("Obstacle detected in front_left, front_right, and side_right")

            elif (self.min_front_left < THRESHOLD_OBSTACLE_VERTICAL and 
                  self.min_side_left < THRESHOLD_OBSTACLE_HORIZONTAL and 
                  self.min_side_right < THRESHOLD_OBSTACLE_HORIZONTAL):
                # Obstacles in front_left, side_left, and side_right
                if (THRESHOLD_OBSTACLE_VERTICAL - self.min_front_left) > (THRESHOLD_OBSTACLE_HORIZONTAL - self.min_side_left):
                    turn = RIGHT_TURN
                else:
                    turn = LEFT_TURN
                print("Obstacle detected in front_left, side_left, and side_right")

            elif (self.min_front_right < THRESHOLD_OBSTACLE_VERTICAL and 
                  self.min_side_left < THRESHOLD_OBSTACLE_HORIZONTAL and 
                  self.min_side_right < THRESHOLD_OBSTACLE_HORIZONTAL):
                # Obstacles in front_right, side_left, and side_right
                if (THRESHOLD_OBSTACLE_VERTICAL - self.min_front_right) > (THRESHOLD_OBSTACLE_HORIZONTAL - self.min_side_left):
                    turn = LEFT_TURN
                else:
                    turn = RIGHT_TURN
                print("Obstacle detected in front_right, side_left, and side_right")  
              
            elif self.min_front_left < THRESHOLD_OBSTACLE_VERTICAL_2 and self.min_front_right < THRESHOLD_OBSTACLE_VERTICAL_2:
              turn = RIGHT_TURN if (THRESHOLD_OBSTACLE_VERTICAL - self.min_front_right) > (THRESHOLD_OBSTACLE_HORIZONTAL - self.min_front_left) else LEFT_TURN
            elif self.min_front_left < THRESHOLD_OBSTACLE_VERTICAL_2:
                turn = RIGHT_TURN
                print("Obstacle detected in front_left__2")        
            elif self.min_front_right < THRESHOLD_OBSTACLE_VERTICAL_2:
                turn = LEFT_TURN
                print("Obstacle detected in front_left__2")           
              
            elif self.min_front_left < THRESHOLD_OBSTACLE_VERTICAL_1 and self.min_front_right < THRESHOLD_OBSTACLE_VERTICAL_1:
              turn = RIGHT_TURN if (THRESHOLD_OBSTACLE_VERTICAL - self.min_front_right) > (THRESHOLD_OBSTACLE_HORIZONTAL - self.min_front_left) else LEFT_TURN
            elif self.min_front_left < THRESHOLD_OBSTACLE_VERTICAL_1:
                turn = RIGHT_TURN
                print("Obstacle detected in front_left__1")        
            elif self.min_front_right < THRESHOLD_OBSTACLE_VERTICAL_1:
                turn = LEFT_TURN
                print("Obstacle detected in front_left__1")         
            elif self.min_front_left < THRESHOLD_OBSTACLE_VERTICAL and self.min_side_left < THRESHOLD_OBSTACLE_HORIZONTAL:
                turn = RIGHT_TURN
                print("Obstacle detected in both front_left and side_left")
            elif self.min_front_left < THRESHOLD_OBSTACLE_VERTICAL and self.min_side_right < THRESHOLD_OBSTACLE_HORIZONTAL:
                turn = LEFT_TURN if (THRESHOLD_OBSTACLE_VERTICAL - self.min_front_left) > (THRESHOLD_OBSTACLE_HORIZONTAL - self.min_side_right) else RIGHT_TURN
                print("Obstacle detected in both front_left and side_right")
            elif self.min_front_right < THRESHOLD_OBSTACLE_VERTICAL and self.min_side_left < THRESHOLD_OBSTACLE_HORIZONTAL:
                turn = RIGHT_TURN if (THRESHOLD_OBSTACLE_VERTICAL - self.min_front_right) > (THRESHOLD_OBSTACLE_HORIZONTAL - self.min_side_left) else LEFT_TURN
                print("Obstacle detected in both front_right and side_left")
            elif self.min_front_right < THRESHOLD_OBSTACLE_VERTICAL and self.min_side_right < THRESHOLD_OBSTACLE_HORIZONTAL:
                turn = LEFT_TURN
                print("Obstacle detected in both front_right and side_right")
            elif self.min_front_left < THRESHOLD_OBSTACLE_VERTICAL and self.min_front_right < THRESHOLD_OBSTACLE_VERTICAL:
                turn = RIGHT_TURN if (THRESHOLD_OBSTACLE_VERTICAL - self.min_front_right) > (THRESHOLD_OBSTACLE_HORIZONTAL - self.min_front_left) else LEFT_TURN
                print("Blocked path detected: obstacles in both front_left and front_right")
            elif self.min_front_left < THRESHOLD_OBSTACLE_VERTICAL:
                turn = RIGHT_TURN
                print("Obstacle detected in front_left")
            elif self.min_front_right < THRESHOLD_OBSTACLE_VERTICAL:
                turn = LEFT_TURN
                print("Obstacle detected in front_right")
            elif self.min_side_left < THRESHOLD_OBSTACLE_HORIZONTAL:
                turn = RIGHT_TURN
                print("Obstacle detected in side_left")
            elif self.min_side_right < THRESHOLD_OBSTACLE_HORIZONTAL:
                turn = LEFT_TURN
                print("Obstacle detected in side_right")
    
        self.rover_move_manual_mode(speed, turn)            

def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollower()
    rclpy.spin(line_follower)
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

