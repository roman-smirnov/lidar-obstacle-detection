#ifndef COMMON_H_
#define COMMON_H_

#include <string>
#include <iosfwd>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <vector>
#include <utility>
#include <type_traits>
#include <unordered_set>

// TODO: REPLACE BOOST SHARED_PTR WITH STD_SHARED_PTR
using String = std::string;

// TODO: USE POINTXYZ INSTEAD OF XYZI
using Point = pcl::PointXYZI;

using PointCloud = pcl::PointCloud<Point>;

using Viewer = pcl::visualization::PCLVisualizer;

template<class T>
using SharedPtr = boost::shared_ptr<T>;

template<class T, class... Args>
SharedPtr<T> makeShared(Args&& ...args) {
  return boost::make_shared<T>(std::forward<Args>(args)...);
}

template<class T>
String toString(T&& val) {
  return std::to_string(std::forward<T>(val));
}
template<class T>
using Pair = std::pair<T, T>;

template<class T>
using Vector = std::vector<T>;

template<class T>
using Set = std::unordered_set<T>;

#endif //COMMON_H_