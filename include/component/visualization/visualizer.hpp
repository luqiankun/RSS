#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP
#include "../tcsobject.hpp"
#include "opencv2/opencv.hpp"
class TCS;
namespace kernel {
namespace driver {
class Vehicle;
}
}  // namespace kernel
namespace visual {
class Visualizer {
 private:
  void arrow_line(cv::Mat, cv::Point2i start, cv::Point2i end, cv::Scalar color,
                  int thickness, bool single = true);
  void arrow_dashed_line(cv::Mat, cv::Point2i start, cv::Point2i end,
                         cv::Scalar color, int thickness, bool forward);
  void dashed_line(cv::Mat, cv::Point2i start, cv::Point2i end,
                   cv::Scalar color, int thickness, int num);
  void paint_vehicle(cv::Mat, std::shared_ptr<kernel::driver::Vehicle>);
  void paint_step(cv::Mat, std::shared_ptr<kernel::driver::Vehicle>);

 public:
  bool init(double resolution, std::shared_ptr<TCS>);
  void run();
  void update();
  void stop() { dispose = true; }
  void paused() { is_paused = true; }
  void restart() { is_paused = false; }
  ~Visualizer() {
    stop();
    if (th.joinable()) {
      th.join();
    }
    CLOG(INFO, "visual") << "visual"
                         << " close";
  }

 public:
  double resolution;
  std::shared_ptr<cv::Mat> map_view;
  std::shared_ptr<cv::Mat> vehicle_view;
  Eigen::Vector4i mat_limit;
  std::weak_ptr<TCS> tcs;
  std::thread th;
  bool dispose{false};
  bool is_run{false};
  bool is_paused{false};
};
}  // namespace visual

#endif