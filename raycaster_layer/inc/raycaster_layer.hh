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

#include <costmap_2d/GenericPluginConfig.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

namespace raycaster_layer_namespace {

class EnemiesLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D {
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
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
};
} // namespace raycaster_layer_namespace

#endif // __RAYCASTER_LAYER_HH__
