//#include "ransac.h"
//#include <random>
//#include <cmath>
//#include <cstdlib>
//
////  TODO: ADD OUR RANDOM METHOD FROM PARTICLE FILTER
//  // TODO: REPLACE WITH MY OWN IMPROVEMENTS!!!
//
//Set<int> findPlaneViaRansac(const PointCloud& cloud, int max_iter, float distance_threshold) {
//  Set<int> inliersResult;
//  std::srand(time(NULL));
//  int inlier_num = 0;
//
//  for (int i = 0; i < max_iter; ++i) {
//    int tempInlierNum = 0;
//    std::unordered_set<int> tempInliersResult;
//    // Randomly sample subset and fit line
//    int ind1 = 0;
//    int ind2 = 0;
//    int ind3 = 0;
//
//
//    // TODO: EXTRACT RAND_POINT FUNC
//    while (ind1 == ind2 || ind2 == ind3 || ind3 == ind1) {
//      ind1 = (int) (((double) std::rand() / RAND_MAX) * (cloud->size() - 1));
//      ind2 = (int) (((double) std::rand() / RAND_MAX) * (cloud->size() - 1));
//      ind3 = (int) (((double) std::rand() / RAND_MAX) * (cloud->size() - 1));
//    }
//
//    Point p1 = cloud->at(ind1);
//    Point p2 = cloud->at(ind2);
//    Point p3 = cloud->at(ind3);
//
//    // TODO: EXTRACT CALCPLANE FUNCTION
//    double A = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
//    double B = (p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z);
//    double C = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
//    double D = -(A * p1.x + B * p1.y + C * p1.z);
//
//    // Measure distance between every point and fitted line
//    // If distance is smaller than threshold count it as inlier
//    for (int index = 0; index < cloud->size(); ++index) {
//      Point p = cloud->at(index);
//      // TODO: USE MATH WRAPPER FROM PARTICLE FILTER
//      if (std::fabs(A * p.x + B * p.y + C * p.z + D) / std::sqrt(A * A + B * B + C * C) < distance_threshold) {
//        tempInlierNum += 1;
//        tempInliersResult.insert(index);
//      }
//    }
//
//    if (tempInlierNum > inlier_num) {
//      inlier_num = tempInlierNum;
//      inliersResult = tempInliersResult;
//    }
//  }
//  // Return indicies of inliers from fitted line with most inliers
//  return inliersResult;
//}
//
//
