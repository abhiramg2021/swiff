#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import tf2_ros
from tf_transformations import quaternion_from_euler


class FrontierExploration(Node):
    NEIGHBORS = [
        (0, 1),
        (0, -1),
        (1, 0),
        (-1, 0),
        (1, 1),
        (1, -1),
        (-1, 1),
        (-1, -1),
    ]

    def __init__(self):
        super().__init__("frontier_exploration")

        self.map_subscriber = self.create_subscription(
            OccupancyGrid, "map", self.map_callback, 10
        )

        self.goal_pose_publisher = self.create_publisher(PoseStamped, "goal_pose", 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def index_to_coord(self, index: int, grid_width: int) -> tuple[int, int]:
        """
        Converts a 1D index to x, y coordinates

        Args:
            index (int): 1D index
            grid_width (int): width of the map
        Returns:
            tuple[int, int]: x, y coordinates
        """

        x = index % grid_width
        y = index // grid_width
        return x, y

    def coord_to_index(self, x: int, y: int, grid_width: int) -> int:
        """
        Converts x, y coordinates to a 1D index

        Args:
            x (int): x coordinate
            y (int): y coordinate
            grid_width (int): width of the map
        Returns:
            int: 1D index
        """
        return y * grid_width + x

    def is_frontier_cell(self, index: int, grid: list[int], grid_width: int) -> bool:
        """
        Indicates whether grid[index] is a frontier cell.
        Check if there is any neighbor cells that are explored (cell value must equal 0).

        Args:
            index (int): cell index
            grid (list[int]): map
            grid_width (int): width of the map

        Returns:
            bool: True if grid[index] is a frontier cell
        """
        if grid[index] != -1:
            return False

        num_free_neighbors = 0
        for dx, dy in FrontierExploration.NEIGHBORS:
            x, y = self.index_to_coord(index, grid_width)

            neighbor_index = self.coord_to_index(x + dx, y + dy, grid_width)

            if neighbor_index < 0 or neighbor_index >= len(grid):
                continue

            if grid[neighbor_index] == 0:
                num_free_neighbors += 1

            if num_free_neighbors >= 4:
                return True

        return False

    def compute_frontier_cells(
        self, grid: list[int], grid_width: int
    ) -> list[tuple[int, int]]:
        """
        Compute the frontier cells for the map.

        Args:
            grid (list[int]): map, A 1D list of integers (-1 = unexplored, 0 = explored, 100 = obstacle)
            grid_width (int): width of the map

        Returns:
            list[tuple[int, int]]: A list of coordinates that form the frontier cells. Instead of index I, the (x,y) coordinates are returned.
        """

        frontier_cells = []
        for index in range(len(grid)):
            if self.is_frontier_cell(index, grid, grid_width):
                x, y = self.index_to_coord(index, grid_width)
                frontier_cells.append((x, y))

        return frontier_cells

    def compute_frontier_components(
        self, frontier_cells: list[tuple[int, int]]
    ) -> list[list[tuple[int, int]]]:
        """
        Compute frontier components from frontier cells.

        Args:
            frontier_cells (list[tuple[int, int]]): frontier cells

        Returns:
            list[list[tuple[int, int]]]: frontier components
        """
        frontier_components = []
        visited = set()

        for cell in frontier_cells:
            if cell in visited:
                continue

            frontier_component = []

            stack = [cell]

            while stack:
                current_cell = stack.pop()
                frontier_component.append(current_cell)
                visited.add(current_cell)

                for dx, dy in FrontierExploration.NEIGHBORS:
                    x, y = current_cell
                    x += dx
                    y += dy

                    if (x, y) in visited:
                        continue

                    if (x, y) in frontier_cells:
                        stack.append((x, y))

            frontier_components.append(frontier_component)
        return frontier_components

    def find_centroid(self, points: list[tuple[int, int]]) -> tuple[int, int]:
        """
        Finds centroid of a set of 2D coords

        Args:
            points (list[tuple[int, int]]): 2D coords

        Returns:
            tuple[int, int]: centroid
        """
        sum_x = sum([p[0] for p in points])
        sum_y = sum([p[1] for p in points])
        return sum_x / len(points), sum_y / len(points)

    def compute_frontier_centroids(
        self, frontier_components: list[list[tuple[int, int]]]
    ) -> list[tuple[int, int]]:
        """
        Find the centroids of all frontier components

        Args:
            frontier_components (list[list[tuple[int, int]]]): frontier components
        Returns:
            list[tuple[int, int]]: centroids of frontier components
        """
        return [self.find_centroid(component) for component in frontier_components]

    def convert_coords_to_map_coords(
        self, coords: list[tuple[int, int]], map_info: dict
    ) -> list[tuple[float, float]]:
        """
        Convert 2d coordinate indices of the map into coordinates in the map frame.

        Args:
            coords (list[tuple[int, int]]): 2D coordinate indices
            map_info (dict): map info

        Returns:
            list[tuple[float, float]]: 2D coordinates in the map frame
        """
        map_resolution = map_info.resolution
        map_origin = map_info.origin

        return [
            (
                map_origin.position.x + coord[0] * map_resolution,
                map_origin.position.y + coord[1] * map_resolution,
            )
            for coord in coords
        ]

    def euclidean_dist(
        self, point1: tuple[float, float], point2: tuple[float, float]
    ) -> float:
        """
        Simple euclidean distance between two points

        Args:
            point1 (tuple[float, float]): x, y coordinates of point 1
            point2 (tuple[float, float]): x, y coordinates of point 2

        Returns:
            float: Euclidean distance between point1 and point2
        """
        return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5

    def get_robot_position(self) -> tuple[float, float]:
        """
        Get the robot position by looking up the transform from odom to base_footprint.
        Assuming the transform between map to odom is negligible.
        Returns:
            tuple[float, float]: x, y coordinates of the robot position
        """
        transform = self.tf_buffer.lookup_transform(
            "odom", "base_footprint", rclpy.time.Time()
        )
        translation = transform.transform.translation
        return translation.x, translation.y

    def calculate_heading(self, point1, point2) -> float:
        """
        Calculate the heading from point1 to point2

        Args:
            point1 (tuple[float, float]): x, y coordinates of point 1
            point2 (tuple[float, float]): x, y coordinates of point 2

        Returns:
            float: heading in radians
        """
        dx = point2[0] - point1[0]
        dy = point2[1] - point1[1]
        return math.atan2(dy, dx)

    def choose_centroid(
        self,
        centroids: list[tuple[float, float]],
        robot_pos: tuple[float, float],
        min_dist: float = 0.5,
    ) -> tuple[float, float]:
        """
        Choose the centroid that is closest to the robot position

        Args:
            centroids (list[tuple[float, float]]): list of centroids
            robot_pos (tuple[float, float]): x, y coordinates of the robot
            min_dist (float): minimum distance from the robot to the centroid (so we don't pick centroids that are too close to the robot)

        Returns:
            tuple[float, float]: x, y coordinates of the chosen centroid
        """
        closest_centroid = None
        closest_centroid_dist = float("inf")
        for centroid in centroids:
            dist = self.euclidean_dist(robot_pos, centroid)
            if dist < closest_centroid_dist and dist > min_dist:
                closest_centroid = centroid
                closest_centroid_dist = dist

        return closest_centroid

    def publish_new_goal(
        self, robot_coords: tuple[float, float], coords: tuple[float, float]
    ) -> None:
        """
        Publish a new goal to the goal_pose topic

        Args:
            coords (tuple[float, float]): x, y coordinates of the goal
        """

        goal_pose = PoseStamped()

        goal_pose.pose.position.x = coords[0]
        goal_pose.pose.position.y = coords[1]
        goal_pose.pose.position.z = 0.0

        h = self.calculate_heading(robot_coords, coords)
        q = quaternion_from_euler(0, 0, h)
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        goal_pose.header.frame_id = "map"

        self.goal_pose_publisher.publish(goal_pose)

        print("Goal pose: ", goal_pose.pose.position.x, goal_pose.pose.position.y)

    def map_callback(self, msg: OccupancyGrid) -> None:

        frontier_cells = self.compute_frontier_cells(msg.data, msg.info.width)

        if len(frontier_cells) == 0:
            print("No frontier cells found.")
            return

        frontier_components = self.compute_frontier_components(frontier_cells)
        centroids = self.compute_frontier_centroids(frontier_components)
        centroids = self.convert_coords_to_map_coords(centroids, msg.info)

        robot_pos = self.get_robot_position()

        # find closest centroid to robot position
        closest_centroid = self.choose_centroid(centroids, robot_pos, min_dist=1)

        if closest_centroid is None:
            print("No valid centroid found.")
            return

        self.publish_new_goal(robot_pos, closest_centroid)


def main(args=None):
    rclpy.init(args=args)
    frontier_exploration = FrontierExploration()
    rclpy.spin(frontier_exploration)
    frontier_exploration.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
