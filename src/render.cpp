/* \author Aaron Brown */
// Functions and structs used to render the enviroment
// such as cars and the highway

#include "render.h"
#include "color.h"

// TODO: Imple Viewer Wrapper!!!!!

namespace {

constexpr auto OBSTACLE_CLOUD_NAME_PREFIX = "obstacle_cloud";
constexpr auto GROUND_CLOUD_NAME = "ground_cloud";

constexpr auto BOX_WIREFRAME_NAME_PREFIX = "box_wireframe";
constexpr auto BOX_SURFACE_NAME_PREFIX = "box_surface";

constexpr int GROUND_POINT_SIZE = 4;
constexpr int OBSTACLE_POINT_SIZE = 4;
constexpr float WIREFRAME_OPACITY = 1.0;
constexpr float SURFACE_OPACITY = 0.4;

constexpr Color WIREFRAME_COLOR = Color::getRed();
constexpr Color SURFACE_COLOR = Color::getRed();

constexpr auto COLOR_PROPERTY = pcl::visualization::PCL_VISUALIZER_COLOR;
constexpr auto POINT_SIZE_PROPERTY = pcl::visualization::PCL_VISUALIZER_POINT_SIZE;
constexpr auto OPACITY_PROPERTY = pcl::visualization::PCL_VISUALIZER_OPACITY;
constexpr auto REPRESENTATION_PROPERTY = pcl::visualization::PCL_VISUALIZER_REPRESENTATION;
constexpr auto WIREFRAME_PROPERTY = pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME;
constexpr auto SURFACE_PROPERTY = pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE;

}

void setPointSize(Viewer& viewer, int point_size, const String& name) {
  viewer.setPointCloudRenderingProperties(POINT_SIZE_PROPERTY, point_size, name);
}

void setPointColor(Viewer& viewer, const Color& color, const String& name) {
  viewer.setPointCloudRenderingProperties(COLOR_PROPERTY, color.r, color.g, color.b, name);
}

void setShapeColor(Viewer& viewer, const Color& color, const String& name) {
  viewer.setShapeRenderingProperties(COLOR_PROPERTY, color.r, color.g, color.b, name);
}

void setShapeOpacity(Viewer& viewer, const float opacity, const String& name) {
  viewer.setShapeRenderingProperties(OPACITY_PROPERTY, opacity, name);
}

void setWireframe(Viewer& viewer, const String& name) {
  viewer.setShapeRenderingProperties(REPRESENTATION_PROPERTY, WIREFRAME_PROPERTY, name);
}

void setSurface(Viewer& viewer, const String& name) {
  viewer.setShapeRenderingProperties(REPRESENTATION_PROPERTY, SURFACE_PROPERTY, name);
}

void addCube(Viewer& viewer, const Box& box, const String& name) {
  viewer.addCube(box.xmin, box.xmax, box.ymin, box.ymax, box.zmin, box.zmax, 1, 1, 1, name);
}

void renderGround(Viewer& viewer, const PointCloud& cloud) {
  viewer.addPointCloud<Point>(cloud.makeShared(), GROUND_CLOUD_NAME);
  setPointColor(viewer, Color::getGreen(), GROUND_CLOUD_NAME);
  setPointSize(viewer, GROUND_POINT_SIZE, GROUND_CLOUD_NAME);
}

void renderObstacle(Viewer& viewer, const PointCloud& cloud, int id) {
  const String name = OBSTACLE_CLOUD_NAME_PREFIX + toString(id);
  viewer.addPointCloud<Point>(cloud.makeShared(), name);
  setPointColor(viewer, Color::getCyan(), name);
  setPointSize(viewer, OBSTACLE_POINT_SIZE, name);
}

// TODO: IMPL PCA BBOX and replace
// TODO: IMPL NAME/ID TYPE
void renderBoxWireframe(Viewer& viewer, Box box, int id) {
  const String name = BOX_WIREFRAME_NAME_PREFIX + toString(id);
  addCube(viewer, box, name);
  setWireframe(viewer, name);
  setShapeColor(viewer, WIREFRAME_COLOR, name);
  setShapeOpacity(viewer, WIREFRAME_OPACITY, name);
}

void renderBoxSurface(Viewer& viewer, Box box, int id) {
  const String name = BOX_SURFACE_NAME_PREFIX + toString(id);
  addCube(viewer, box, name);
  setSurface(viewer, name);
  setShapeColor(viewer, SURFACE_COLOR, name);
  setShapeOpacity(viewer, SURFACE_OPACITY, name);
}

void renderBox(Viewer& viewer, Box box, int id) {
  renderBoxWireframe(viewer, box, id);
  renderBoxSurface(viewer, box, id);
}
