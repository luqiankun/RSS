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
  void stop() { dispose = true; }
  ~Visualizer() {
    stop();
    if (th.joinable()) {
      th.join();
    }
    LOG(TRACE) << "visual"
               << " close";
  }

 private:
  double resolution;
  std::shared_ptr<cv::Mat> map_view;
  std::shared_ptr<cv::Mat> vehicle_view;
  Eigen::Vector4i mat_limit;
  std::weak_ptr<TCS> tcs;
  std::thread th;
  bool dispose{false};
};
}  // namespace visual

#endif