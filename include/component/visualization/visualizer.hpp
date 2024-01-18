#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP
#include "../tcsobject.hpp"
#include "opencv2/opencv.hpp"
class TCS;
namespace visual {
class Visualizer {
 public:
  bool init(double resolution, std::shared_ptr<TCS>);
  void run();
  void update();

 private:
  double resolution;
  std::thread visual_th;
  cv::Mat map_view;
  cv::Mat vehicle_view;
  Eigen::Vector4i mat_limit;
  std::weak_ptr<TCS> tcs;
  std::thread th;
};
}  // namespace visual

#endif