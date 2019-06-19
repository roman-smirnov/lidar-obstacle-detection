// PCL lib Functions for processing point clouds 

#include "processing.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <ctime>
#include <cstdlib>
#include <chrono>

//SharedPtr<PointCloud> FilterCloud(const PointCloud& cloud,
//                       float filterRes,
//                       Eigen::Vector4f minPoint,
//                       Eigen::Vector4f maxPoint) {
//
//  // Time segmentation process
//  auto startTime = std::chrono::steady_clock::now();
//
//  // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
//  // Create the filtering object
//  auto cloud_filtered = makeShared<PointCloud>();
//
//  //std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
//  //   << " data points (" << pcl::getFieldsList (*cloud) << ").";
//  pcl::VoxelGrid<Point> sor;
//  sor.setInputCloud(cloud);
//  sor.setLeafSize(filterRes, filterRes, filterRes);
//  sor.filter(*cloud_filtered);
//
//  auto endTime = std::chrono::steady_clock::now();
//  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//  std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
//
//  // ROI
//  auto cloud_cropped = makeShared<PointCloud>();
//  pcl::CropBox<Point> cb;
//  cb.setInputCloud(cloud_filtered);
//  cb.setMin(minPoint);
//  cb.setMax(maxPoint);
//  cb.setNegative(false);
//  cb.filter(*cloud_cropped);
//
//  // Remove ego car
//  auto cloud_removeEgo = makeShared<PointCloud>();
//  pcl::CropBox<Point> cb_ego;
//  cb_ego.setInputCloud(cloud_cropped);
//  cb_ego.setMin(Eigen::Vector4f(-3, -2, -5, 1));
//  cb_ego.setMax(Eigen::Vector4f(3, 2, 2, 1));
//  cb_ego.setNegative(true);
//  cb_ego.filter(*cloud_removeEgo);
//
//  return cloud_removeEgo;
//
//}
//
//Pair<PointCloud> SegmentPlane(const PointCloud& cloud, int max_iter, float distance_threshold) {
//
//  // Time segmentation process
//  auto startTime = std::chrono::steady_clock::now();
//
//  Set<int> inliers = findPlaneViaRansac(cloud, max_iter, distance_threshold);
//
//  auto obstCloud = makeShared<PointCloud>();
//  auto planeCloud = makeShared<PointCloud>();
//
//  if (inliers.size() == 0) {
//    std::cout << "could not estimate a planar model for the given dataset" << std::endl;
//  }
//
//  for (int index = 0; index < cloud.points.size(); index++) {
//    Point point = cloud->points[index];
//    if (inliers.count(index))
//      planeCloud->points.push_back(point);
//    else
//      obstCloud->points.push_back(point);
//  }
//
//  auto endTime = std::chrono::steady_clock::now();
//  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//  std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
//
//  return Pair<PointCloud>(obstCloud, planeCloud);;
//}
//
//Vector<Vector<int>> euclideanCluster(Vector<Vector<float>>& points, KdTree *tree, float distanceTol) {
//  Vector<Vector<int>> clusters;
//  Vector<bool> isProcessed(points.size(), false);
//
//  for (int id = 0; id < points.size(); id++) {
//    Vector<int> cluster;
//    if (!isProcessed[id]) {
//      proximity(points, id, cluster, isProcessed, tree, distanceTol);
//      clusters.push_back(cluster);
//    }
//  }
//  return clusters;
//}
//
//void proximity(Vector<Vector<int>>& points,
//               int& id,
//               Vector<int>& cluster,
//               Vector<bool>& isProcessed,
//               KdTree *tree,
//               float distanceTol) {
//
//  isProcessed[id] = true;
//  cluster.push_back(id);
//  Vector<int> ids = tree->search(points[id], distanceTol);
//  for (auto id : ids) {
//    if (!isProcessed[id])
//      proximity(points, id, cluster, isProcessed, tree, distanceTol);
//  }
//
//}
//
//Vector<PointCloud> Clustering(PointCloud cloud, float clusterTolerance, int minSize, int maxSize){
//  // Time clustering process
//  auto startTime = std::chrono::steady_clock::now();
//  Vector<Vector<float>> points;
//  KdTree *tree = new KdTree(3);
//  for (int i = 0; i < cloud->points.size(); i++) {
//    Vector<float> point{cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
//    points.push_back(point);
//    tree->insert(point, i);
//  }
//
//  Vector<Vector<int>> clustersIds = euclideanCluster(points, tree, clusterTolerance);
//
//  Vector<PointCloud> clusters;
//
//  for (std::vector<int> ids : clustersIds) {
//    PointCloud cloudCluster(new PointCloud);
//    if (ids.size() >= minSize && ids.size() <= maxSize) {
//      for (int id: ids)
//        cloudCluster->points.push_back(cloud->points[id]);
//      cloudCluster->width = cloudCluster->points.size();
//      cloudCluster->height = 1;
//      cloudCluster->is_dense = true;
//      clusters.push_back(cloudCluster);
//    }
//  }
//
//  auto endTime = std::chrono::steady_clock::now();
//  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//
//  std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters"
//            << std::endl;
//
//  return clusters;
//}
//
//Box BoundingBox(const PointCloud& cluster){
//  Point max_point, min_point;
//  pcl::getMinMax3D(cluster, max_point, min_point);
//  return Box(max_point, min_point);
//}


SharedPtr<PointCloud> loadPcd(const String& file) {
  auto cloud = makeShared<PointCloud>();
  if (pcl::io::loadPCDFile<Point>(file, *cloud) == -1) {
    std::cerr << "failed to read file" << std::endl;
    exit(EXIT_FAILURE);
  }
  std::cout << "Loaded " << cloud->points.size() << " points from " + file << std::endl;
  return cloud;
}

Vector<boost::filesystem::path> streamPcd(const String& dataPath) {
  Vector<boost::filesystem::path>
      paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});
  sort(paths.begin(), paths.end());  //sort files in accending order for chronological playback
  return paths;
}