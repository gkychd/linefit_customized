#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "ground_segmentation/utils.hpp"


using PointType = PointXYZILID;
typedef pcl::PointCloud<PointType> PointCloud;

typedef std::pair<PointType, PointType> PointLine;
