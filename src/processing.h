// PCL lib Functions for processing point clouds 

#ifndef PROCESSING_H_
#define PROCESSING_H_

//#include <Eigen/Core>
//#include <Eigen/Dense>

#include "box.h"
//#include "kdtree.h"
//#include "ransac.h"
#include "common.h"

//PointCloud FilterCloud(PointCloud cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);
//
//Pair<PointCloud> SegmentPlane(PointCloud cloud, int max_iter, float distance_threshold);
//
//Vector<Vector<int>> euclideanCluster(Vector<Vector<float>>& points, KdTree *tree, float distanceTol);
//
//void proximity(Vector<Vector<int>>& points,
//               int& id,
//               Vector<int>& cluster,
//               Vector<bool>& isProcessed,
//               KdTree *tree,
//               float distanceTol);
//
//Vector<PointCloud> Clustering(PointCloud cloud, float clusterTolerance, int minSize, int maxSize);
//
//Box BoundingBox(const PointCloud& cluster);

SharedPtr<PointCloud> loadPcd(const String& file);

Vector<boost::filesystem::path> streamPcd(const String& dataPath);

#endif // PROCESSING_H_