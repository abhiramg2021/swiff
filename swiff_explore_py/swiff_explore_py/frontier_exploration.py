#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped


class FrontierExploration(Node):
    DIRECTIONS = [
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

    def index_to_coord(self, index, width) -> tuple[int, int]:
        """
        Converts a 1D index to x, y coordinates

        Args:
            index (int): 1D index
            width (int): width of the map
        Returns:
            tuple[int, int]: x, y coordinates
        """

        x = index % width
        y = index // width
        return x, y

    def coord_to_index(self, x, y, width) -> int:
        """
        Converts x, y coordinates to a 1D index

        Args:
            x (int): x coordinate
            y (int): y coordinate
            width (int): width of the map
        Returns:
            int: 1D index
        """
        return y * width + x

    def is_frontier_cell(self, i, map_, map_width) -> bool:
        """
        Indicates whether cell i is a frontier cell.
        Check if there is any neighbor cells that are explored (cell value must equal 0).

        Args:
            i (int): cell index
            map_ (list[int]): map
            map_width (int): width of the map

        Returns:
            bool: True if cell i is a frontier cell
        """
        # TOdo rename map to grid
        # not an unexplored cell
        if map_[i] != -1:
            return False
        k = 0
        for dx, dy in FrontierExploration.DIRECTIONS:
            x, y = self.index_to_coord(i, map_width)
            x += dx
            y += dy

            j = self.coord_to_index(x, y, map_width)

            if j < 0 or j >= len(map_):
                continue

            if map_[j] == 0:
                k += 1

            if k >= 2:
                return True

        return False

    def compute_frontier_cells(self, map_, map_width) -> list[tuple[int, int]]:
        """
        Compute the frontier cells for the map.

        Args:
            map_ (list[int]): map, A 1D list of integers (-1 = unexplored, 0 = explored, 100 = obstacle)
            map_width (int): width of the map

        Returns:
            list[tuple[int, int]]: A list of coordinates that form the frontier cells. Instead of index I, the (x,y) coordinates are returned.
        """

        frontier_cells = []
        for i in range(len(map_)):
            if self.is_frontier_cell(i, map_, map_width):
                x, y = self.index_to_coord(i, map_width)
                frontier_cells.append((x, y))

        return frontier_cells

    def compute_frontier_components(
        self, frontier_cells, map_width
    ) -> list[list[tuple[int, int]]]:
        """
        Compute frontier components from frontier cells.

        Args:
            frontier_cells (list[tuple[int, int]]): frontier cells
            map_width (int): width of the map

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

                for dx, dy in FrontierExploration.DIRECTIONS:
                    x, y = current_cell
                    x += dx
                    y += dy

                    if (x, y) in visited:
                        continue

                    if (x, y) in frontier_cells:
                        stack.append((x, y))

            frontier_components.append(frontier_component)
        return frontier_components

    def find_centroid(self, points):
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

    def euclidean_dist(self, point1, point2):
        return ((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2) ** 0.5

    def map_callback(self, msg):

        frontier_cells = self.compute_frontier_cells(msg.data, msg.info.width)

        frontier_components = self.compute_frontier_components(
            frontier_cells, msg.info.width
        )
        # compute centroids
        centroids = [self.find_centroid(component) for component in frontier_components]

        # transform centroids to map frame
        centroids = [
            (centroid[0] * msg.info.resolution, centroid[1] * msg.info.resolution)
            for centroid in centroids
        ]
        robot_pos = (0, 0)  # get robot position from tf and transform to map frame

        # find closest centroid to robot position
        closest_centroid = min(
            centroids, key=lambda c: self.euclidean_dist(c, robot_pos)
        )

        # transform centroid to map frame
        closest_centroid = (
            msg.info.origin.position.x + closest_centroid[0],
            msg.info.origin.position.y + closest_centroid[1],
        )

        # publish that centroid to /goal_node.
        goal_pose = PoseStamped()

        goal_pose.pose.position.x = closest_centroid[0]
        goal_pose.pose.position.y = closest_centroid[1]
        goal_pose.pose.position.z = 0.0

        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        goal_pose.header.frame_id = "map"

        print("Goal pose: ", goal_pose.pose.position.x, goal_pose.pose.position.y)

        self.goal_pose_publisher.publish(goal_pose)


def main(args=None):
    rclpy.init(args=args)
    frontier_exploration = FrontierExploration()
    rclpy.spin(frontier_exploration)
    frontier_exploration.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
