#include "ransac3d.h"

template<typename PointT>
std::unordered_set<int> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
  std::unordered_set<int> inliersResult;
  srand(time(NULL));
  
  
  int inlier_num = 0;
  // For max iterations 
  for(int i=0; i < maxIterations; ++i)
  { 
    int tempInlierNum = 0;
    std::unordered_set<int> tempInliersResult;
    // Randomly sample subset and fit line
    int ind1 = 0;
    int ind2 = 0;
    int ind3 = 0;

    while(ind1==ind2 || ind2==ind3 || ind3==ind1)
    {
      ind1 = (int)(((double) rand() / RAND_MAX)*(cloud->size()-1));
      ind2 = (int)(((double) rand() / RAND_MAX)*(cloud->size()-1));
      ind3 = (int)(((double) rand() / RAND_MAX)*(cloud->size()-1));
    }

    PointT p1 = cloud->at(ind1);
    PointT p2 = cloud->at(ind2);
    PointT p3 = cloud->at(ind3);

    double A = (p2.y - p1.y)*(p3.z - p1.z) - (p2.z - p1.z)*(p3.y - p1.y);
    double B = (p2.z - p1.z)*(p3.x - p1.x) - (p2.x - p1.x)*(p3.z - p1.z);
    double C = (p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x);
    double D = -(A*p1.x + B*p1.y + C*p1.z);

    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier
    for(int index=0; index < cloud->size(); ++index)
    {
      PointT p = cloud->at(index);
      if(std::fabs(A*p.x + B*p.y + C*p.z + D) / std::sqrt(A*A+B*B+C*C) < distanceTol)
      {
        tempInlierNum += 1;
        tempInliersResult.insert(index);
      }
    }

    if(tempInlierNum > inlier_num)
    {
      inlier_num = tempInlierNum;
      inliersResult = tempInliersResult;
    }

  }

  // Return indicies of inliers from fitted line with most inliers
  return inliersResult;
}


