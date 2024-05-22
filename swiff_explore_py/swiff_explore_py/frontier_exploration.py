from collections import deque
from heapq import heappop, heappush

import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped

import tf2_ros

# Type aliases

coordinate_in_cells = tuple[int, int]
coordinate_in_cells_and_cost = tuple[*coordinate_in_cells, float]

coordinate_in_meters = tuple[float, float]
coordinate_in_meters_and_cost = tuple[*coordinate_in_meters, float]


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

    def __init__(self) -> None:

        super().__init__("frontier_exploration")

        self.map_subscriber = self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self.map_callback, 10
        )

        self.goal_status_subscriber = self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self.goal_status_callback,
            10,
        )

        self.goal_pose_publisher = self.create_publisher(PoseStamped, "goal_pose", 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.current_goal = None
        self.next_goals = []
        self.attempted_goals = []

    def euclidean_dist(self, p1: coordinate_in_cells, p2: coordinate_in_cells) -> float:

        return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

    def publish_goal(self, goal: coordinate_in_meters) -> None:
        """
        Publish a new goal to the goal_pose topic

        Args:
            coord (coordinate_in_meters): x, y coordinates of the goal in meters
        """

        print("new goal: ", goal)

        goal_pose = PoseStamped()

        goal_pose.pose.position.x = goal[0]
        goal_pose.pose.position.y = goal[1]
        goal_pose.pose.position.z = 0.0

        # TODO: modify nav2_params so not aliging to goal heading does not matter whatsoever.

        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        self.goal_pose_publisher.publish(goal_pose)

        self.current_goal = goal

    def update_goal_choice(self) -> None:
        """
        Check if one of the next suggested goals is a better goal than the current goal. If so, publish the new goal.
        """

        if len(self.next_goals) == 0:
            return

        if (
            self.current_goal != None
            and self.euclidean_dist(self.current_goal, self.next_goals[0]) < 0.1
        ):
            return
        self.publish_goal(self.next_goals[0][:2])
        self.next_goals = self.next_goals[1:]

    def goal_status_callback(self, msg: GoalStatusArray) -> None:
        """
        A callback function that handles events of when goals are completed or failed.
        """

        current_goal_status = msg.status_list[-1].status

        if current_goal_status == 4:
            print("Goal reached!")
            self.attempted_goals.append(self.current_goal)
            self.update_goal_choice()
        elif current_goal_status == 6:
            print("Goal failed!")
            self.attempted_goals.append(self.current_goal)
            self.update_goal_choice()

    def get_robot_position(self) -> coordinate_in_meters:
        """
        Get the robot position by looking up the transform from the odom frame to base_footprint frame.
        Assuming the transform between the map frame to odom frame is negligible.
        Returns:
            coordinate_in_meters: x, y coordinates of the robot position in meters
        """
        transform = self.tf_buffer.lookup_transform(
            "odom", "base_footprint", rclpy.time.Time()
        )
        translation = transform.transform.translation
        return translation.x, translation.y

    def get_frontier_cells(
        self, map_: np.ndarray, map_origin: coordinate_in_cells, map_resolution: float
    ) -> dict[coordinate_in_cells, float]:
        """
        Use Dijkstra's to find all the frontier cells in the map.

        Args:
            map_ (np.ndarray): 2D numpy array representing the map

        Returns:
            dict[coordinate_in_cells, float]: frontier cells -> cost value
        """

        alpha = 0.001  # TODO: make this a parameter

        map_height, map_width = map_.shape

        robot_position_in_meters = self.get_robot_position()

        robot_position_in_cells = (
            int((robot_position_in_meters[0] - map_origin[0]) / map_resolution),
            int((robot_position_in_meters[1] - map_origin[1]) / map_resolution),
        )

        frontier_cells = dict()

        priority_queue = [(0, robot_position_in_cells)]  # (cost, (x, y))
        visited = set()

        while priority_queue:
            cost, (cx, cy) = heappop(priority_queue)

            if (cx, cy) in visited:
                continue

            visited.add((cx, cy))

            if map_[cy, cx] == -1:
                frontier_cells[(cx, cy)] = cost
            elif map_[cy, cx] < 50:

                for dx, dy in FrontierExploration.NEIGHBORS:
                    nx, ny = cx + dx, cy + dy

                    if 0 <= nx < map_width and 0 <= ny < map_height:
                        if (nx, ny) not in visited:
                            # add 1 to account for distance traveled, and then add the cost of the cell.
                            heappush(
                                priority_queue,
                                (cost + 1 + map_[ny, nx] * alpha, (nx, ny)),
                            )
        return frontier_cells

    def get_frontier_components(
        self, frontier_cells: dict[coordinate_in_cells, float]
    ) -> list[list[coordinate_in_cells_and_cost]]:
        """
        Args:
            frontier_cells (dict[coordinate_in_cells, float]): frontier cells with cost values

        Returns:
            list[list[tuple[coordinate_in_cells_and_cost]]]: A list of frontier components. Each frontier component is a list of frontier cells.
        """

        frontier_components = []
        visited = set()

        for sx, sy in frontier_cells:
            if (sx, sy) in visited:
                continue

            frontier_component = []

            stack = [(sx, sy)]

            while stack:
                cx, cy = stack.pop()
                frontier_component.append((cx, cy, frontier_cells[(cx, cy)]))
                visited.add((cx, cy))

                for dx, dy in FrontierExploration.NEIGHBORS:
                    nx, ny = cx + dx, cy + dy

                    if (nx, ny) in visited:
                        continue

                    if (nx, ny) in frontier_cells:
                        stack.append((nx, ny))
            frontier_components.append(frontier_component)
        return frontier_components

    def get_frontier_centroids(
        self, frontier_components: list[list[coordinate_in_meters_and_cost]]
    ) -> list[coordinate_in_meters_and_cost]:
        """
        Given a list of frontier components, compute the centroid of each component. The cost of each centroid is the average cost of the frontier component.

        Args:
            frontier_components (list[list[coordinate_in_meters_and_cost]]): frontier components
        Returns:
            list[coordinate_in_meters_and_cost]: centroids of frontier components
        """

        centroids = []

        for component in frontier_components:
            centroid = (
                sum([x for x, _, _ in component]) / len(component),
                sum([y for _, y, _ in component]) / len(component),
                sum([dist for _, _, dist in component]) / len(component),
            )
            centroids.append(centroid)

        return centroids

    def filter_centroids(
        self, centroids: list[coordinate_in_meters_and_cost]
    ) -> list[coordinate_in_meters_and_cost]:
        """
        Filter out centroids that are too close to previously attempted goals.

        Args:
            centroids (list[coordinate_in_meters_and_cost]): centroids of frontier components

        Returns:
            list[coordinate_in_meters_and_cost]: filtered centroids
        """

        if len(self.attempted_goals) == 0:
            return centroids

        attempted_goals = np.array(self.attempted_goals)

        filtered_centroids = []

        # TODO: this can be optimized with numpy magic
        for centroid in centroids:
            skip_centroid = False
            for goal in attempted_goals:
                if self.euclidean_dist(centroid, goal) < 0.5:
                    skip_centroid = True
                    break
            if not skip_centroid:
                filtered_centroids.append(centroid)
        return filtered_centroids

    def visualize_coordinates(self, coordinates: list[coordinate_in_meters]) -> None:

        raise NotImplementedError

    def explore(
        self, map_: np.ndarray, map_origin: coordinate_in_meters, map_resolution: float
    ) -> list[coordinate_in_meters_and_cost]:
        """
        A frontier exploration algorithm.

        Args:
            map_ (np.ndarray): 2D numpy array representing the map
            map_origin (coordinate_in_meters): x, y coordinates of the map origin. The origin maps to (0, 0) in the numpy array.
            map_resolution (float): resolution of the map in meters per cell

        Returns:
            list[coordinate_in_meters_and_cost]: A list of suggested goals. Each goal is a tuple of (x, y, cost). x and y are in meters.
        """

        frontier_cells = self.get_frontier_cells(map_, map_origin, map_resolution)

        frontier_components = self.get_frontier_components(frontier_cells)

        centroids = self.get_frontier_centroids(frontier_components)

        centroids = [
            (
                x * map_resolution + map_origin[0],
                y * map_resolution + map_origin[1],
                cost * map_resolution,
            )
            for x, y, cost in centroids
        ]

        centroids = self.filter_centroids(centroids)

        centroids.sort(key=lambda x: x[2])

        return centroids

    def map_callback(self, msg: OccupancyGrid) -> None:

        map_ = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        map_resolution = msg.info.resolution

        self.next_goals = self.explore(map_, map_origin, map_resolution)

        self.update_goal_choice()


def main(args=None):
    rclpy.init(args=args)
    frontier_exploration = FrontierExploration()
    rclpy.spin(frontier_exploration)
    frontier_exploration.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
