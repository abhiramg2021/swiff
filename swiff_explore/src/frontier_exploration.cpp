// #include "swiff_explore/frontier_exploration.hpp"

// namespace swiff_explore
// {
//   FrontierExploration::FrontierExploration(const rclcpp::NodeOptions &options)
//       : Node("frontier_exploration", options)
//   {
//     map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
//         "/map", 10, std::bind(&FrontierExploration::process_map, this, std::placeholders::_1));

//     RCLCPP_INFO(this->get_logger(), "FrontierExploration node has been started.");
//   }

//   FrontierExploration::~FrontierExploration()
//   {
//     RCLCPP_INFO(this->get_logger(), "FrontierExploration node has been terminated.");
//   }

//   /**
//    * @brief Convert the one-dimensional index to a two-dimensional coordinate
//    * @param index The one-dimensional index
//    * @param map_width The width of the map
//    * @return The two-dimensional coordinate
//    */
//   Coord get_coord(int index, int map_width)
//   {
//     int x = index % map_width;
//     int y = index / map_width;
//     return {x, y};
//   }

//   /**
//    * @brief Convert the two-dimensional coordinate to a one-dimensional index
//    * @param x The x-coordinate
//    * @param y The y-coordinate
//    * @param map_width The width of the map
//    * @return The one-dimensional index
//    */
//   int get_index(int x, int y, int map_width)
//   {
//     return y * map_width + x;
//   }

//   /**
//    * @brief Check if a cell is a frontier cell
//    * @param i The index of the cell
//    * @param map_width The width of the map
//    * @param data The map data
//    * @return True if the cell is a frontier cell, false otherwise
//    */
//   bool is_frontier_cell(int i, int map_width, const std::vector<int8_t> &data)
//   {
//     // Make sure this cell is unexplored
//     if (data[i] != -1)
//     {
//       return false;
//     }

//     // Get 2D coordinates of the cell
//     Coord coord = get_coord(i, map_width);
//     int x = coord.x;
//     int y = coord.y;

//     // Iterate over neighbors
//     for (int dx = -1; dx <= 1; dx++)
//     {
//       for (int dy = -1; dy <= 1; dy++)
//       {
//         // Skip checking self
//         if (dx == 0 && dy == 0)
//         {
//           continue;
//         }

//         // Compute neighbor coordinates
//         int nx = x + dx;
//         int ny = y + dy;

//         // Ensure neighbor is within map bounds
//         if (nx < 0 || nx >= map_width || ny < 0 || ny >= map_width)
//         {
//           continue;
//         }

//         // Compute neighbor index
//         int ni = get_index(nx, ny, map_width);

//         // Check if neighbor is free space
//         if (ni >= 0 && ni < static_cast<int>(data.size()) && data[ni] == 0)
//         {
//           return true;
//         }
//       }
//     }

//     return false;
//   }

//   /**
//    * @brief Process the incoming map message to update the frontier
//    * @param msg The occupancy grid message
//    */
//   void process_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
//   {
//     // Iterate through points in the map
//     for (size_t i = 0; i < msg->data.size(); i++)
//     {
//       if (is_frontier_cell(i, msg->info.width, msg->data))
//       {
//         // Add to frontier list
//         // TODO: Implement the logic to handle frontier cells
//       }
//     }

//     // TODO: Call separate_frontiers()

//     // TODO: Compute centroids of the frontiers

//     // TODO: Print the closest frontier

//     RCLCPP_INFO(this->get_logger(), "Updating frontier.");
//   }
// }

// #include <rclcpp_components/register_node_macro.hpp>
// RCLCPP_COMPONENTS_REGISTER_NODE(swiff_explore::FrontierExploration)
