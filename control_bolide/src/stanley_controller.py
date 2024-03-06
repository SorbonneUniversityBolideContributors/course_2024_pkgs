#!/usr/bin/env python3

__author__ = "Nicolas HAMMJE"
__status__ = "In Developpement"

#%% IMPORTS
import rospy
import math
from std_msgs.msg import Float32
from control_bolide.msg import SpeedDirection
from geometry_msgs.msg import TransformStamped, Pose
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import interp1d
from scipy import signal




#%% CLASS
class StanleyController:
    '''
    This class implements a stanley controller and (soon) an obstacle avoidance system. 
    '''
    def __init__(self):
        rospy.init_node('stanley_controller_node')
        
        self.WAYPOINTS_PATH = str(rospy.get_param("~waypoints_path", "~/bolide_ws/course_2024_pkgs/control_bolide/racelines/esclangon_loop_2.csv"))
        # self.scan_topic = str(rospy.get_param("~scan_topic", "lidar_data"))
        self.ODOM_TOPIC = str(rospy.get_param("~odom_topic", "/pf/pose/odom"))
        self.CMD_TOPIC = str(rospy.get_param("~cmd_topic", "cmd_vel"))
        
        
        
        self.K_E = float(rospy.get_param("~K_E", "2.0"))
        self.K_H = float(rospy.get_param("~K_H", "1.5"))
        self.VELOCITY_PERCENTAGE = float(rospy.get_param("~velocity_percentage", "0.3"))
        self.STEERING_LIMIT = float(rospy.get_param("~steering_limit", "15.0"))


        # init publisher
        self.drive_pub = rospy.Publisher(self.CMD_TOPIC, SpeedDirection, queue_size=10)
        self.current_waypoint_pub = rospy.Publisher("current_waypoint", Marker, queue_size=10)
        self.waypoint_pub = rospy.Publisher("next_waypoint", Marker, queue_size=10)

        self.odom_sub = rospy.Subscriber(self.ODOM_TOPIC, Odometry, self.odomCB, queue_size=1)
        # self.timer = rospy.Timer(rospy.Duration(0.4), self.timer_callback)


        self.waypoint_utils = WaypointUtils(
            node=self,
            interpolation_distance=0.05,
            filepath=self.WAYPOINTS_PATH
        )

        self.utils = Utils()


        # init speed and direction
        self.current_speed = 0.0
        self.current_direction = 0.0
        self.target_velocity = 0.0

        self.current_pose = None
        self.current_pose_wheelbase_front = None
        self.goal_pos = None
        self.closest_wheelbase_rear_point = None
        

    def drive_to_target_stanley(self):

        K_V = 0

        closest_wheelbase_front_point_car, closest_wheelbase_front_point_world = self.waypoint_utils.get_waypoint_stanley(
            self.current_pose_wheelbase_front
        )

        path_heading = math.atan2(
            closest_wheelbase_front_point_world[1] - self.closest_wheelbase_rear_point[1],
            closest_wheelbase_front_point_world[0] - self.closest_wheelbase_rear_point[0],
        )

        current_heading = math.atan2(
            self.current_pose_wheelbase_front.position.y - self.current_pose.position.y,
            self.current_pose_wheelbase_front.position.x - self.current_pose.position.x,
        )


        if current_heading < 0:
            current_heading += 2 * math.pi
        if path_heading < 0:
            path_heading += 2 * math.pi

        # calculate the errors
        crosstrack_error = math.atan2(
            self.K_E * closest_wheelbase_front_point_car[1], K_V + self.target_velocity
        )  # y value in car frame
        heading_error = path_heading - current_heading
        if heading_error > math.pi:
            heading_error -= 2 * math.pi
        elif heading_error < -math.pi:
            heading_error += 2 * math.pi

        heading_error *= self.K_H

        # Calculate the steering angle using the Stanley controller formula
        angle = heading_error + crosstrack_error

        rospy.loginfo(f"heading_error: {heading_error:.2f} ... crosstrack_error: {crosstrack_error:.2f} angle: {np.degrees(angle):.2f}")
        rospy.loginfo(f"current_heading: {current_heading:.2f} ... path_heading: {path_heading:.2f}")

        angle = np.clip(angle, -np.radians(self.STEERING_LIMIT), np.radians(self.STEERING_LIMIT))

        velocity = self.target_velocity * self.VELOCITY_PERCENTAGE

        drive_msg = SpeedDirection()

        #Max Speed is 5 m/s

        velocity_normalized = velocity / 5.

        velocity_normalized = np.clip(velocity_normalized, -1.0, 1.0)

        angle_normalized = angle / np.radians(15.0)


        drive_msg.speed = velocity_normalized
        drive_msg.direction = -angle_normalized
        self.drive_pub.publish(drive_msg)



    def odomCB(self, pose_msg: Odometry):

        print("ODOM")

        self.current_pose = pose_msg.pose.pose

        current_pose_quaternion = np.array(
            [
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w,
            ]
        )

        self.current_pose_wheelbase_front = Pose()
        current_pose_xyz = R.from_quat(current_pose_quaternion).apply((0.195,0,0)) + (
            self.current_pose.position.x,
            self.current_pose.position.y,
            0,
        )

        self.current_pose_wheelbase_front.position.x = current_pose_xyz[0]
        self.current_pose_wheelbase_front.position.y = current_pose_xyz[1]
        self.current_pose_wheelbase_front.position.z = current_pose_xyz[2]
        self.current_pose_wheelbase_front.orientation = self.current_pose.orientation


        self.closest_wheelbase_rear_point, self.target_velocity = self.waypoint_utils.get_closest_waypoint_with_velocity(
            self.current_pose
        )

        self.utils.draw_marker(
            pose_msg.header.frame_id,
            pose_msg.header.stamp,
            self.closest_wheelbase_rear_point,
            self.current_waypoint_pub,
            color="blue",
        )



        self.goal_pos, goal_pos_world = self.waypoint_utils.get_waypoint(self.current_pose, self.target_velocity)

        self.utils.draw_marker(pose_msg.header.frame_id, pose_msg.header.stamp, goal_pos_world, self.waypoint_pub, color="red")

        self.drive_to_target_stanley()


class Utils:
    def __init__(self):
        pass

    def draw_marker(self, frame_id, stamp, position, publisher, color="red", id=0):
        if position is None:
            return
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.id = id
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.a = 1.0
        if color == "red":
            marker.color.r = 1.0
        elif color == "green":
            marker.color.g = 1.0
        elif color == "blue":
            marker.color.b = 1.0
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0.0
        publisher.publish(marker)



    def draw_marker_array(self, frame_id, stamp, positions, publisher):
        marker_array = MarkerArray()
        for i, position in enumerate(positions):
            if position is None:
                continue
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.id = i
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = 0.0
            marker.lifetime = Duration(seconds=0.1).to_msg()
            marker_array.markers.append(marker)
        publisher.publish(marker_array)



    def draw_lines(self, frame_id, stamp, path, publisher):
        points = []
        for i in range(len(path) - 1):
            a = path[i]
            b = path[i + 1]
            point = Point()
            point.x = a[0]
            point.y = a[1]
            points.append(copy.deepcopy(point))
            point.x = b[0]
            point.y = b[1]
            points.append(copy.deepcopy(point))

        line_list = Marker()
        line_list.header.frame_id = frame_id
        line_list.header.stamp = stamp
        line_list.id = 0
        line_list.type = line_list.LINE_LIST
        line_list.action = line_list.ADD
        line_list.scale.x = 0.1
        line_list.color.a = 1.0
        line_list.color.r = 0.0
        line_list.color.g = 1.0
        line_list.color.b = 0.0
        line_list.points = points
        publisher.publish(line_list)




    def traverse_grid(self, start, end):
        """
        Bresenham's line algorithm for fast voxel traversal

        CREDIT TO: Rogue Basin
        CODE TAKEN FROM: http://www.roguebasin.com/index.php/Bresenham%27s_Line_Algorithm
        """
        # Setup initial conditions
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1

        # Determine how steep the line is
        is_steep = abs(dy) > abs(dx)

        # Rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2

        # Swap start and end points if necessary and store swap state
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1

        # Recalculate differentials
        dx = x2 - x1
        dy = y2 - y1

        # Calculate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1

        # Iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
        return points

    
class WaypointUtils:
    def __init__(self, node, filepath, interpolation_distance):
        self.node = node

        self.waypoints_world, self.velocities = self.load_and_interpolate_waypoints(
            file_path=filepath, interpolation_distance=interpolation_distance
        )

        self.index = 0 
        self.velocity_index = 0

        self.min_lookahead = 1.0
        self.max_lookahead = 3.0

        self.min_lookahead_speed = 3.0
        self.max_lookahead_speed = 6.0





        print(f"Loaded {len(self.waypoints_world)} waypoints")


    def transform_waypoints(self, waypoints, car_position, pose):
        # translation
        waypoints = waypoints - car_position

        # rotation
        quaternion = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        waypoints = R.inv(R.from_quat(quaternion)).apply(waypoints)

        return waypoints

    def get_closest_waypoint_with_velocity(self, pose):
        # get current position of car
        if pose is None:
            return

        position = (pose.position.x, pose.position.y, 0)

        waypoints_car = self.transform_waypoints(self.waypoints_world, position, pose)
       
        # get distance from car to all waypoints
        distances = np.linalg.norm(waypoints_car, axis=1)

        # get indices of waypoints sorted by ascending distance
        self.velocity_index = np.argmin(distances)

        return self.waypoints_world[self.velocity_index], self.velocities[self.velocity_index]

    def get_waypoint(self, pose, target_velocity, fixed_lookahead=None):
        # get current position of car
        if pose is None:
            return
        position = (pose.position.x, pose.position.y, 0)

        # transform way-points from world to vehicle frame of reference
        
        waypoints_car = self.transform_waypoints(self.waypoints_world, position, pose)
       
        # get distance from car to all waypoints
        distances = np.linalg.norm(waypoints_car, axis=1)

        # get indices of waypoints that are within L, sorted by descending distance
        # Use dynamic lookahead for this part

        if fixed_lookahead:
            self.L = fixed_lookahead
        else:
            # Lookahead is proportional to velocity
            self.L = min(
                max(
                    self.min_lookahead,
                    self.min_lookahead
                    + (self.max_lookahead - self.min_lookahead)
                    * (target_velocity - self.min_lookahead_speed)
                    / (self.max_lookahead_speed - self.min_lookahead_speed),
                ),
                self.max_lookahead,
            )

        indices_L = np.argsort(np.where(distances < self.L, distances, -1))[::-1]

        # set goal point to be the farthest valid waypoint within distance L
        for i in indices_L:
            # check waypoint is in front of car
            x = waypoints_car[i][0]
            if x > 0:
                self.index = i
                return waypoints_car[self.index], self.waypoints_world[self.index]
        return None, None


    def get_waypoint_stanley(self, pose):
        # get current position of car
        if pose is None:
            return
        position = (pose.position.x, pose.position.y, 0)

        # transform way-points from world to vehicle frame of reference
        
        waypoints_car = self.transform_waypoints(self.waypoints_world, position, pose)
        
        # get distance from car to all waypoints
        distances = np.linalg.norm(waypoints_car, axis=1)

        # get indices of waypoints sorted by ascending distance
        index = np.argmin(distances)

        return waypoints_car[index], self.waypoints_world[index]


    def load_and_interpolate_waypoints(self, file_path, interpolation_distance=0.05):
        # Read waypoints from csv, first two columns are x and y, third column is velocity
        # Exclude last row, because that closes the loop
        points = np.genfromtxt(file_path, delimiter=",")[:, :2]
        velocities = np.genfromtxt(file_path, delimiter=",")[:, 2]

        # Add first point as last point to complete loop
        rospy.loginfo(str(velocities))

        # interpolate, not generally needed because interpolation can be done with the solver, where you feed in target distance between points
        if interpolation_distance != 0 and interpolation_distance is not None:
            # Calculate the cumulative distances between points
            distances = np.sqrt(np.sum(np.diff(points, axis=0) ** 2, axis=1))
            cumulative_distances = np.insert(np.cumsum(distances), 0, 0)

            # Calculate the number of segments based on the desired distance threshold
            total_distance = cumulative_distances[-1]
            segments = int(total_distance / interpolation_distance)

            # Linear length along the line
            distance = np.cumsum(np.sqrt(np.sum(np.diff(points, axis=0) ** 2, axis=1)))
            # Normalize distance between 0 and 1
            distance = np.insert(distance, 0, 0) / distance[-1]

            # Interpolate
            alpha = np.linspace(0, 1, segments)
            interpolator = interp1d(distance, points, kind="slinear", axis=0)
            interpolated_points = interpolator(alpha)

            # Interpolate velocities
            velocity_interpolator = interp1d(distance, velocities, kind="slinear")
            interpolated_velocities = velocity_interpolator(alpha)

            # Add z-coordinate to be 0
            interpolated_points = np.hstack((interpolated_points, np.zeros((interpolated_points.shape[0], 1))))
            assert len(interpolated_points) == len(interpolated_velocities)
            return interpolated_points, interpolated_velocities

        else:
            # Add z-coordinate to be 0
            points = np.hstack((points, np.zeros((points.shape[0], 1))))
            return points, velocities


#%% MAIN
if __name__ == '__main__':
    print("Stanley Avoidance Initialized")
    controller = StanleyController()
    
    try: 
        rospy.spin()

    except KeyboardInterrupt:
        rospy.loginfo("KeyboardInterrupt received. Shutting down...")
    finally:
        # Clean up and unregister keyboard events
        controller.destroy_node()