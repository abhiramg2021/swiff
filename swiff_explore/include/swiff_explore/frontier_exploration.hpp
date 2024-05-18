#ifndef FRONTIER_EXPLORATION_HPP
#define FRONTIER_EXPLORATION_HPP

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

namespace swiff_explore
{

  struct Coord
  {
    int x;
    int y;
  };

  class FrontierExploration : public rclcpp::Node
  {
  public:
    explicit FrontierExploration(const rclcpp::NodeOptions &options);
    ~FrontierExploration();

  private:
    void process_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    Coord get_coord(int index, int map_width);
    int get_index(int x, int y, int map_width);
    bool is_frontier_cell(int i, int map_width, int[] & data);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  };
}

#endif
