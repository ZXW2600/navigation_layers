/**
 * @file raycaster_layer.hh
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-05-14
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __RAYCASTER_LAYER_HH__
#define __RAYCASTER_LAYER_HH__

#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_costmap_2d/grid_map_costmap_2d.hpp>

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <string>

#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

namespace raycaster_layer_namespace
{

  class EnemiesLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
  {
  public:
    EnemiesLayer();

    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                              double *min_x, double *min_y, double *max_x,
                              double *max_y);
    virtual void updateCosts(costmap_2d::Costmap2D &master_grid, int min_i,
                             int min_j, int max_i, int max_j);
    bool isDiscretized() { return true; }

    virtual void matchSize();

  private:

    void EnemiesPointCallback(const geometry_msgs::PointStamped::ConstPtr &msg);
    void OccupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

    ros::Subscriber map_sub;   // global reference map
    ros::Subscriber point_sub; //

    bool rolling_window_;

    std::string global_frame_; ///< @brief The global frame for the costmap

    std::string map_topic;
    bool _static_map;

    grid_map::Costmap2DConverter<grid_map::GridMap, grid_map::Costmap2DDirectTranslationTable> costmap2dConverter_;

    grid_map::GridMap global_map;
    bool global_map_inited = false;

    float point_x = 0, point_y = 0;
    bool point_inited = false;
  };
} // namespace raycaster_layer_namespace

#endif // __RAYCASTER_LAYER_HH__
