// TODO: READ THE PCL STYLE GUIDE
//  http://www.pointclouds.org/documentation/advanced/pcl_style_guide.php

// TODO: REVIEW PCL CLASS REFERENCE
//  http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_p_c_l_visualizer.html

// TODO: REVIEW PCL VIZ EXAMPLES
//  http://docs.pointclouds.org/trunk/group__visualization.html
// TODO: GET RID OF THIS SHIT!!!!

#include "render.h"
#include "processing.h"
//using CloudProcessor = ProcessPointClouds<Point>;

// TODO:: Viewer wrapper class so as to not pass it around

namespace {
constexpr int DISTANCE_AWAY_IN_METERS = 16;
//constexpr auto PATH_TO_PCD_STREAM_DATA = "../src/sensors/data/pcd/data_1";
constexpr auto PATH_TO_PCD_STREAM_DATA = "../src/sensors/data/pcd/data_2";
constexpr auto VIEWER_WINDOW_NAME = "Lidar Obstacle Detection Viewer";
constexpr auto START_MESSAGE = "starting enviroment";
}

////TODO: EXTRACT MAGIC NUMBER TO CONSTANTS
////TODO: REMOVE EIGENLIB VECTORS
//void cityBlock(Viewer& viewer, const PointCloud& cloud) {
//  PointCloud filterCloud =
//      FilterCloud(cloud, 0.2f, Eigen::Vector4f(-20, -5, -2, 1), Eigen::Vector4f(30, 7, 0.5, 1));
//
//  const auto&[obstacle_cluster, ground_cluster] = SegmentPlane(filterCloud, 100, 0.2);
//  renderGround(viewer, ground_cluster);
//
//  Vector<PointCloud> clusters = Clustering(obstacle_cluster, 0.4f, 30, 500);
//
//  for (int i = 0; i < clusters.size(); ++i) {
//    Box box = BoundingBox(clusters[i]);
//    renderBox(viewer, box, i);
//    renderObstacle(viewer, clusters[i], i);
//  }
//}

void setCameraAxisAlignedView(Viewer& viewer) {
  viewer.setCameraPosition(-DISTANCE_AWAY_IN_METERS, -DISTANCE_AWAY_IN_METERS, DISTANCE_AWAY_IN_METERS, 1, 1, 0);
  viewer.addCoordinateSystem(1.0);
}

void setCameraTopDownView(Viewer& viewer) {
  viewer.setCameraPosition(0, 0, DISTANCE_AWAY_IN_METERS, 1, 0, 1);
  viewer.addCoordinateSystem(1.0);
}

void setCameraSideView(Viewer& viewer) {
  viewer.setCameraPosition(0, -DISTANCE_AWAY_IN_METERS, 0, 0, 0, 1);
  viewer.addCoordinateSystem(1.0);
}

void setCameraFirstPersonView(Viewer& viewer) {
  viewer.setCameraPosition(-10, 0, 0, 0, 0, 1);
}

//TODO: PUT THIS INSIDE VIEWER ABSTRACTION
void initCamera(Viewer& viewer) {
  // TODO: setBackgroundColor(Colors::kBlack)
  viewer.setBackgroundColor(0, 0, 0);
  viewer.initCameraParameters();
}

int main(int argc, char **argv) {
  std::cout << START_MESSAGE << std::endl;

  auto viewer = makeShared<Viewer>(VIEWER_WINDOW_NAME);
  initCamera(*viewer);
  setCameraAxisAlignedView(*viewer);

  // TODO: MOVE DATA DIR OUTSIDE SRC DIR
  Vector<boost::filesystem::path> stream = streamPcd(PATH_TO_PCD_STREAM_DATA);
  auto streamIterator = stream.begin();

  while (!viewer->wasStopped()) {
    // TODO: impl Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load pcd and run obstacle detection process
    auto input_point_cloud = loadPcd((*streamIterator).string());
//    cityBlock(*viewer, *input_point_cloud);
    renderGround(*viewer, *input_point_cloud);

    streamIterator++;
    if (streamIterator == stream.end())
      streamIterator = stream.begin();

    viewer->spinOnce();
  }

}