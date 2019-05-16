#ifndef RANSAC3D_H_
#define RANSAC3D_H_

#include <unordered_set>
#include <random>
#include <cmath>

template<typename PointT>
std::unordered_set<int> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

#endif /* RANSAC3D_H_*/