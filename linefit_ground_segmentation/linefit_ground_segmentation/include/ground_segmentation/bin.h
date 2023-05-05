#ifndef GROUND_SEGMENTATION_BIN_H_
#define GROUND_SEGMENTATION_BIN_H_

#include <atomic>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "ground_segmentation/utils.hpp"
#include <limits>

using PointType = PointXYZILID;
class Bin {
public:
  struct MinZPoint {
    MinZPoint() : z(0), d(0) {}
    MinZPoint(const double& d, const double& z) : z(z), d(d) {}
    bool operator==(const MinZPoint& comp) {return z == comp.z && d == comp.d;}

    double z;
    double d;
  };

private:

  std::atomic<bool> has_point_;
  std::atomic<double> min_z;
  std::atomic<double> min_z_range;

public:

  Bin();

  /// \brief Fake copy constructor to allow vector<vector<Bin> > initialization.
  Bin(const Bin& segment);

  void addPoint(const PointType& point);

  void addPoint(const double& d, const double& z);

  MinZPoint getMinZPoint();

  inline bool hasPoint() {return has_point_;}

};


using PointType = PointXYZILID;
Bin::Bin() : min_z(std::numeric_limits<double>::max()), has_point_(false) {}

Bin::Bin(const Bin& bin) : min_z(std::numeric_limits<double>::max()),
                                           has_point_(false) {}

void Bin::addPoint(const PointType& point) {
  const double d = sqrt(point.x * point.x + point.y * point.y);
  addPoint(d, point.z);
}

void Bin::addPoint(const double& d, const double& z) {
  has_point_ = true;
  if (z < min_z) {
    min_z = z;
    min_z_range = d;
  }
}

Bin::MinZPoint Bin::getMinZPoint() {
  MinZPoint point;

  if (has_point_) {
    point.z = min_z;
    point.d = min_z_range;
  }

  return point;
}






#endif /* GROUND_SEGMENTATION_BIN_H_ */
