/**
 * @file raycaster_layer.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-05-14
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "raycaster_layer.hh"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(raycaster_layer_namespace::EnemiesLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace raycaster_layer_namespace
{

  void EnemiesLayer::EnemiesPointCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
  {
    point_inited = true;
    point_y = msg->point.y;
    point_x = msg->point.x;
    ROS_INFO("get point: x %f y %f", point_x, point_y);
  }

  void EnemiesLayer::OccupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
  {
    ROS_INFO("get map");

    global_map_inited = true;
    grid_map::GridMapRosConverter::fromOccupancyGrid(*msg, "elevation",
                                                     global_map);
    global_map["raycast"].resizeLike(global_map["elevation"]);
    // shutdown the map subscrber if _static_map flag is on
    if (_static_map)
    {
      ROS_INFO("Shutting down the map subscriber. _static_map flag is on");
      map_sub.shutdown();
    }
  }

  EnemiesLayer::EnemiesLayer() : global_map({"elevation", "raycast"}) {}

  void EnemiesLayer::onInitialize()
  {
    ros::NodeHandle nh("~/" + name_);
    rolling_window_ = layered_costmap_->isRolling();

    EnemiesLayer::matchSize();
    current_ = true;

    global_frame_ = layered_costmap_->getGlobalFrameID();

    double transform_tolerance;
    nh.param("transform_tolerance", transform_tolerance, 0.2);

    if (!nh.getParam("map_topic", map_topic))
    {
      ROS_ERROR("~map_topic paramater not provieded");
    }

    if (!nh.getParam("static_map", _static_map))
    {
      ROS_INFO("~static_map paramater not provieded, set to true by default");
      _static_map = true;
    }

    map_sub = nh.subscribe<nav_msgs::OccupancyGrid>(
        map_topic, 1000, &EnemiesLayer::OccupancyGridCallback, this);

    point_sub = nh.subscribe<geometry_msgs::PointStamped>(
        "/clicked_point", 1000, &EnemiesLayer::EnemiesPointCallback, this);

    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);

    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
        &EnemiesLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
  }

  void EnemiesLayer::matchSize()
  {
    Costmap2D *master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
              master->getOriginX(), master->getOriginY());
  }

  void EnemiesLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
  {
    enabled_ = config.enabled;
  }

  void EnemiesLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x,
                                  double *min_y, double *max_x, double *max_y)
  {
    if (rolling_window_)
      updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
    if (!enabled_)
      return;

    // update the global current status
    current_ = true;
  }

  void EnemiesLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i,
                                 int max_j)
  {
    if (global_map_inited && point_inited)
    {
      {
        global_map.clear("raycast");
        auto k = global_map.getResolution();

        auto cols = global_map.getSize()(1) - 1;
        auto rows = global_map.getSize()(0) - 1;
        grid_map::Index center;
        global_map.getIndex(grid_map::Position(point_x, point_y), center);

        grid_map::Index corner[4] = {
            grid_map::Index(0, 0), grid_map::Index(0, cols),
            grid_map::Index(rows, cols), grid_map::Index(rows, 0)};
        ROS_INFO("Running line iterator demo. col : %d row: %d k: %f", cols,
                 rows, k);
        for (size_t i = 0; i < 4; i++)
        {
          ROS_INFO(". to  x: %d y: %d k: %f", corner[(i + 1) % 4].x(),
                   corner[(i + 1) % 4].y(), k);
          for (grid_map::LineIterator iterator(global_map, corner[i],
                                               corner[(i + 1) % 4]);
               !iterator.isPastEnd(); ++iterator)
          {

            for (grid_map::LineIterator iterator_inner(global_map, center,
                                                       (*iterator).transpose());
                 !iterator_inner.isPastEnd(); ++iterator_inner)
            {
              if (global_map.at("elevation", *iterator_inner) < 10)
              {
                global_map.at("raycast", *iterator_inner) = 1;
              }
              else
                break;
            }
          }
        }
      }

      this->costmap2dConverter_.initializeFromGridMap(this->global_map, master_grid);
      // Copy data.
      this->costmap2dConverter_.setCostmap2DFromGridMap(this->global_map, "raycast", master_grid);
    }
  }

} // end namespace