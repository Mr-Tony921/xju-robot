//
// Created by tony on 2022/10/19.
//

#pragma once

#include <cstdlib>
#include <mutex>
#include <utility>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <XmlRpcException.h>
#include <XmlRpcValue.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/footprint.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

#include "costmap_plugins/keepOutZone.h"

namespace costmap_2d {
// point with integer coordinates
struct PointInt {
int x;
int y;
};

class KeepOutLayer : public CostmapLayer {
public:
  KeepOutLayer() {
    costmap_ = nullptr;  // this is the unsigned char* member of parent class Costmap2D.
    fill_zones_ = false;
    map_received_ = false;
    updating_ = false;
    recording_ = false;
    rolling_min_x_ = 0;
    rolling_max_x_ = 0;
    rolling_min_y_ = 0;
    rolling_max_y_ = 0;
  }

  virtual ~KeepOutLayer() = default;

  virtual void onInitialize();

  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y, double* max_x, double* max_y);

  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void reset();

  virtual void matchSize();

private:
  struct Zone {
  Zone() : id(0), grid(LETHAL_OBSTACLE) {}

  Zone(uint32_t id, uint8_t grid, std::vector<geometry_msgs::Point> edges)
    : id(id), grid(grid), edges(std::move(edges)) {
    this->edges.reserve(50);
  }

  uint32_t id;
  uint8_t grid;
  std::vector<geometry_msgs::Point> edges;
  };

  typedef std::list<Zone> ZoneList;
  typedef std::list<Zone>::iterator ZoneIter;
  typedef std::list<Zone>::const_iterator ZoneConstIter;
  typedef std::vector<geometry_msgs::Point> PointVector;

  void incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map);

  void incomingPoint(const geometry_msgs::PointStampedConstPtr& point);

  void setZoneValue(unsigned char* grid, const PointVector& zone, uint8_t value, bool fill_zone);

  void rasterizeZone(const std::vector<PointInt>& zone, std::vector<PointInt>& zone_cells, bool fill);

  void zoneOutlineCells(const std::vector<PointInt>& zone, std::vector<PointInt>& zone_cells);

  void raytrace(int x0, int y0, int x1, int y1, std::vector<PointInt>& cells);

  bool keepOutZoneSrv(costmap_plugins::keepOutZone::Request& req,
                      costmap_plugins::keepOutZone::Response& res);

  inline bool findAvailableId(uint32_t& id, ZoneConstIter& iter) const;

  inline bool addZone(const PointVector& edges, uint32_t& id, uint8_t cost);

  inline void setAllZonesCost();

  inline bool inZone(const std::vector<PointInt>& zone, PointInt& point);

  std::mutex data_mutex_;
  bool fill_zones_;
  bool map_received_;
  bool rolling_window_;
  int rolling_min_x_, rolling_min_y_, rolling_max_x_, rolling_max_y_;
  ZoneList keep_out_zones_;
  ZoneIter id_search_start_iter_;
  std::atomic_bool updating_, recording_;
  std::vector<geometry_msgs::PointStamped> record_zone_;

  ros::Publisher zone_pub_;
  ros::Subscriber map_sub_, point_sub_;
  ros::ServiceServer keep_out_zone_srv_;
};
}  // namespace costmap_2d
