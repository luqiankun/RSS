#include "../../../include/component/visualization/visualizer.hpp"

#include "../../../include/kernel/driver/vehicle.hpp"
#include "../../../include/main/tcs.hpp"
std::tuple<int, int, int> hexToRgb(const std::string &hexColor) {
  int r = 0, g = 0, b = 0;

  // 去除#字符
  if (hexColor.length() == 7 && hexColor[0] == '#') {
    std::string rgbPart = hexColor.substr(1, 6);

    // 解析每种颜色的16进制值
    sscanf(rgbPart.c_str(), "%02x%02x%02x", &r, &g, &b);
  }

  return std::make_tuple(r, g, b);
}
namespace visual {
bool Visualizer::init(double resolution, std::shared_ptr<TCS> tcs) {
  this->resolution = resolution;
  this->tcs = tcs;
  int min_x = std::numeric_limits<int>::max();
  int max_x = std::numeric_limits<int>::min();
  int min_y = std::numeric_limits<int>::max();
  int max_y = std::numeric_limits<int>::min();
  for (auto &x : tcs->resource->points) {
    if (x->layout.position.x() <= min_x) {
      min_x = x->layout.position.x();
    }
    if (x->layout.position.x() >= max_x) {
      max_x = x->layout.position.x();
    }
    if (x->layout.position.y() <= min_y) {
      min_y = x->layout.position.y();
    }
    if (x->layout.position.y() >= max_y) {
      max_y = x->layout.position.y();
    }
  }
  for (auto &x : tcs->resource->locations) {
    if (x->layout.position.x() <= min_x) {
      min_x = x->layout.position.x();
    }
    if (x->layout.position.x() >= max_x) {
      max_x = x->layout.position.x();
    }
    if (x->layout.position.y() <= min_y) {
      min_y = x->layout.position.y();
    }
    if (x->layout.position.y() >= max_y) {
      max_y = x->layout.position.y();
    }
  }
  mat_limit =
      Eigen::Vector4i(min_x - (max_x - min_x) / 8, max_x + (max_x - min_x) / 8,
                      min_y - (max_y - min_y) / 8, max_y + (max_y - min_y) / 8);
  LOG(INFO) << "mat_limit" << mat_limit;
  auto width =
      static_cast<uint32_t>((mat_limit.y() - mat_limit.x()) / resolution);
  auto height =
      static_cast<uint32_t>((mat_limit.w() - mat_limit.z()) / resolution);
  cv::Mat mat = cv::Mat(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
  LOG(INFO) << "image size " << mat.size;
  for (auto &x : tcs->resource->points) {
    auto p_x = static_cast<uint32_t>((x->layout.position.x() - mat_limit.x()) /
                                     resolution);
    auto p_y =
        mat.rows - static_cast<uint32_t>(
                       (x->layout.position.y() - mat_limit.z()) / resolution);
    // LOG(INFO) << p_x << " " << p_y;
    cv::putText(mat, x->name, cv::Point2i(p_x + 5, p_y + 20),
                cv::HersheyFonts::FONT_HERSHEY_DUPLEX, 0.3,
                cv::Scalar(90, 90, 90), 1, cv::LINE_AA);
    cv::circle(mat, cv::Point2i(p_x, p_y), 3, cv::Scalar(0, 0, 0), 1,
               cv::LINE_AA);
  }
  for (auto &x : tcs->resource->paths) {
    auto be_x = static_cast<uint32_t>(
        (x->source_point.lock()->layout.position.x() - mat_limit.x()) /
        resolution);
    auto be_y =
        mat.rows -
        static_cast<uint32_t>(
            (x->source_point.lock()->layout.position.y() - mat_limit.z()) /
            resolution);
    auto end_layout_x = static_cast<uint32_t>(
        (x->destination_point.lock()->layout.position.x() - mat_limit.x()) /
        resolution);
    auto end_layout_y =
        mat.rows -
        static_cast<uint32_t>(
            (x->destination_point.lock()->layout.position.y() - mat_limit.z()) /
            resolution);
    double arrowed_len = 5;
    Eigen::Vector2f line;
    line.x() = static_cast<double>(end_layout_x) - static_cast<double>(be_x);
    line.y() = static_cast<double>(end_layout_y) - static_cast<double>(be_y);
    Eigen::Vector2f norm_line = line.normalized() * (line.norm() - 6);
    double tip = arrowed_len / line.norm();
    cv::Scalar color = cv::Scalar(90, 90, 90);
    int thickness = 1;
    if (x->locked) {
      color = cv::Scalar(255, 255, 255);
      thickness = 3;
    }
    cv::arrowedLine(mat,
                    cv::Point2i(be_x + line.normalized().x() * 6,
                                be_y + line.normalized().y() * 6),
                    cv::Point2i(be_x + norm_line.x(), be_y + norm_line.y()),
                    color, thickness, cv::LINE_AA, 0, tip);
    if (x->max_reverse_vel > 0) {
      cv::arrowedLine(mat,
                      cv::Point2i(be_x + norm_line.x(), be_y + norm_line.y()),
                      cv::Point2i(be_x + line.normalized().x() * 6,
                                  be_y + line.normalized().y() * 6),
                      color, thickness, cv::LINE_AA, 0, tip);
    }
  }

  for (auto &x : tcs->resource->locations) {
    auto p_x = static_cast<uint32_t>((x->layout.position.x() - mat_limit.x()) /
                                     resolution);

    auto p_y =
        mat.rows - static_cast<uint32_t>(
                       (x->layout.position.y() - mat_limit.z()) / resolution);
    cv::Scalar color = cv::Scalar(10, 20, 20);
    int thickness = 2;
    if (x->locked) {
      color = cv::Scalar(150, 150, 150);
      thickness = -1;
    }
    cv::putText(mat, x->name, cv::Point2i(p_x - 30, p_y - 22),
                cv::HersheyFonts::FONT_HERSHEY_DUPLEX, 0.3,
                cv::Scalar(90, 90, 90), 1, cv::LINE_AA);
    cv::rectangle(mat, cv::Rect(p_x - 14, p_y - 10, 28, 20), color, thickness,
                  cv::LINE_AA, 0);
    cv::circle(mat, cv::Point2i(p_x, p_y), 3, cv::Scalar(70, 70, 70), 1,
               cv::LINE_AA);

    if (x->link.lock()) {
      auto link_x = static_cast<uint32_t>(
          (x->link.lock()->layout.position.x() - mat_limit.x()) / resolution);
      auto link_y =
          mat.rows - static_cast<uint32_t>(
                         (x->link.lock()->layout.position.y() - mat_limit.z()) /
                         resolution);
      int step_x = (static_cast<int>(link_x) - static_cast<int>(p_x)) / 8;
      int step_y = (static_cast<int>(link_y) - static_cast<int>(p_y)) / 8;
      cv::line(mat, cv::Point2i(p_x, p_y),
               cv::Point2i(p_x + step_x, p_y + step_y), cv::Scalar(10, 40, 230),
               1, cv::LINE_AA, 0);
      cv::line(mat, cv::Point2i(p_x + 2 * step_x, p_y + 2 * step_y),
               cv::Point2i(p_x + 3 * step_x, p_y + 3 * step_y),
               cv::Scalar(10, 40, 230), 1, cv::LINE_AA, 0);
      cv::line(mat, cv::Point2i(p_x + 4 * step_x, p_y + 4 * step_y),
               cv::Point2i(p_x + 5 * step_x, p_y + 5 * step_y),
               cv::Scalar(10, 40, 230), 1, cv::LINE_AA, 0);
      cv::line(mat, cv::Point2i(p_x + 6 * step_x, p_y + 6 * step_y),
               cv::Point2i(p_x + 7 * step_x, p_y + 7 * step_y),
               cv::Scalar(10, 40, 230), 1, cv::LINE_AA, 0);
    }
  }
  // axis label
  int label_x = 30;
  int label_y = mat.rows - 30;
  int len_x = mat.cols - 30 - 50;
  int len_y = mat.rows - 30 - 50;
  cv::circle(mat, cv::Point2i(label_x, label_y), 5, cv::Scalar(0, 0, 255), 1,
             cv::LINE_AA, 0);
  cv::arrowedLine(mat, cv::Point2i(label_x, label_y),
                  cv::Point2i(label_x, label_y - len_y), cv::Scalar(10, 40, 80),
                  1, cv::LINE_AA, 0, 0.01);
  cv::arrowedLine(mat, cv::Point2i(label_x, label_y),
                  cv::Point2i(label_x + len_x, label_y), cv::Scalar(10, 40, 80),
                  1, cv::LINE_AA, 0, 0.01);
  int step = 50;
  for (int i = 0; i < mat.cols / step; i++) {
    int label_real_x =
        static_cast<int>((label_x + step * i) * resolution) + mat_limit.x();
    cv::line(mat, cv::Point2i(label_x + step * i, label_y),
             cv::Point2i(label_x + step * i, label_y - 5),
             cv::Scalar(10, 40, 80), 1, cv::LINE_AA, 0);
    cv::putText(mat, std::to_string(label_real_x),
                cv::Point2i(label_x + step * i - 9, label_y + 13), 1, 0.2,
                cv::Scalar(10, 10, 80), 1, cv::LINE_AA);
  }
  for (int i = 0; i < mat.rows / step; i++) {
    int label_real_y =
        static_cast<int>((mat.rows - (label_y - step * i)) * resolution) +
        mat_limit.z();
    cv::putText(mat, std::to_string(label_real_y),
                cv::Point2i(label_x + 5, label_y - step * i - 5), 1, 0.2,
                cv::Scalar(10, 10, 80), true, cv::LINE_AA);
    cv::line(mat, cv::Point2i(label_x, label_y - step * i),
             cv::Point2i(label_x + 5, label_y - step * i),
             cv::Scalar(10, 40, 80), 1, cv::LINE_AA, 0);
  }
  map_view = std::shared_ptr<cv::Mat>(new cv::Mat(mat));
  return true;
}

void Visualizer::run() {
  if (!is_run) {
    is_run = true;
    th = std::thread{[&] {
      LOG(INFO) << "visualization run";
      while (!dispose) {
        update();
        auto img = std::shared_ptr<cv::Mat>(new cv::Mat(map_view->clone()));
        auto mask = std::shared_ptr<cv::Mat>(new cv::Mat);
        cv::cvtColor(*vehicle_view, *mask, cv::COLOR_BGR2GRAY);
        cv::threshold(*mask, *mask, 10, 255, cv::THRESH_BINARY);
        // cv::bitwise_not(mask, mask);

        cv::bitwise_not(*map_view, *img, *mask);
        cv::add(*img, *vehicle_view, *img);
        namedWindow("TCS_VIEW", cv::WINDOW_NORMAL);
        cv::imshow("TCS_VIEW", *img);
        if (cv::waitKey(200) == 27) {  // 如果用户按下 ESC 键，退出循环
          break;
        }
      }
      cv::destroyAllWindows();
      cv::waitKey(1);
    }};
  }
}
void Visualizer::update() {
  if (!tcs.lock()) {
    return;
  }
  vehicle_view = std::shared_ptr<cv::Mat>(new cv::Mat(
      cv::Size(map_view->cols, map_view->rows), CV_8UC3, cv::Scalar(0)));
  if (!tcs.lock()) {
    dispose = true;
    return;
  }
  if (!tcs.lock()->dispatcher) {
    return;
  }
  for (auto &v : tcs.lock()->dispatcher->vehicles) {
    // 自身坐标系
    Eigen::Vector3i left_bottom = Eigen::Vector3i::Ones();
    Eigen::Vector3i right_bottom = Eigen::Vector3i::Ones();
    Eigen::Vector3i left_mid = Eigen::Vector3i::Ones();
    Eigen::Vector3i right_mid = Eigen::Vector3i::Ones();
    Eigen::Vector3i left_top = Eigen::Vector3i::Ones();
    Eigen::Vector3i right_top = Eigen::Vector3i::Ones();
    left_bottom.x() = 0 - v->length;
    left_bottom.y() = 0 - v->width;
    right_bottom.x() = v->length;
    right_bottom.y() = -v->width;
    left_mid.x() = 0 - 2 * v->length / 3;
    left_mid.y() = 0 - v->width;
    right_mid.x() = 2 * v->length / 3;
    right_mid.y() = 0 - v->width;
    left_top.x() = 0 - 2 * v->length / 3;
    left_top.y() = v->width;
    right_top.x() = 2 * v->length / 3;
    right_top.y() = v->width;

    // transport
    Eigen::Matrix<int, 3, 3> T;
    T << cos(v->angle), -sin(v->angle), v->position.x(), sin(v->angle),
        cos(v->angle), v->position.y(), 0, 0, 1;
    // map
    Eigen::Vector3i left_bottom_ = T * left_bottom;
    Eigen::Vector3i right_bottom_ = T * right_bottom;
    Eigen::Vector3i left_mid_ = T * left_mid;
    Eigen::Vector3i right_mid_ = T * right_mid;
    Eigen::Vector3i left_top_ = T * left_top;
    Eigen::Vector3i right_top_ = T * right_top;
    cv::Point2i left_bottom_cv(
        static_cast<uint32_t>((left_bottom_.x() - mat_limit.x()) / resolution),
        vehicle_view->rows -
            static_cast<uint32_t>((left_bottom_.y() - mat_limit.z()) /
                                  resolution));
    cv::Point2i right_bottom_cv(
        static_cast<uint32_t>((right_bottom_.x() - mat_limit.x()) / resolution),
        vehicle_view->rows -
            static_cast<uint32_t>((right_bottom_.y() - mat_limit.z()) /
                                  resolution));
    cv::Point2i left_mid_cv(
        static_cast<uint32_t>((left_mid_.x() - mat_limit.x()) / resolution),
        vehicle_view->rows - static_cast<uint32_t>(
                                 (left_mid_.y() - mat_limit.z()) / resolution));
    cv::Point2i right_mid_cv(
        static_cast<uint32_t>((right_mid_.x() - mat_limit.x()) / resolution),
        vehicle_view->rows -
            static_cast<uint32_t>((right_mid_.y() - mat_limit.z()) /
                                  resolution));
    cv::Point2i left_top_cv(
        static_cast<uint32_t>((left_top_.x() - mat_limit.x()) / resolution),
        vehicle_view->rows - static_cast<uint32_t>(
                                 (left_top_.y() - mat_limit.z()) / resolution));
    cv::Point2i right_top_cv(
        static_cast<uint32_t>((right_top_.x() - mat_limit.x()) / resolution),
        vehicle_view->rows -
            static_cast<uint32_t>((right_top_.y() - mat_limit.z()) /
                                  resolution));
    auto color = hexToRgb(v->color);
    cv::line(
        *vehicle_view, left_bottom_cv, right_bottom_cv,
        cv::Scalar(std::get<2>(color), std::get<1>(color), std::get<0>(color)),
        3, cv::LINE_AA);
    cv::line(
        *vehicle_view, left_mid_cv, left_top_cv,
        cv::Scalar(std::get<2>(color), std::get<1>(color), std::get<0>(color)),
        3, cv::LINE_AA);
    cv::line(
        *vehicle_view, right_mid_cv, right_top_cv,
        cv::Scalar(std::get<2>(color), std::get<1>(color), std::get<0>(color)),
        3, cv::LINE_AA);
    cv::putText(*vehicle_view, v->name, cv::Point2i(0, 9) + left_bottom_cv, 1,
                0.7, cv::Scalar(10, 10, 80), 1, cv::LINE_AA, true);

    //
    // 路线
    if (v->current_order) {
      if (v->current_order->state ==
          data::order::TransportOrder::State::BEING_PROCESSED) {
        for (auto &dr : v->current_order->driverorders) {
          if (dr->state == data::order::DriverOrder::State::TRAVELLING ||
              dr->state == data::order::DriverOrder::State::OPERATING) {
            // current_step
            if (dr->route->current_step) {
              auto beg = dr->route->current_step->path->source_point.lock();
              auto end =
                  dr->route->current_step->path->destination_point.lock();
              auto beg_x = static_cast<uint32_t>(
                  (beg->position.x() - mat_limit.x()) / resolution);
              auto beg_y =
                  vehicle_view->rows -
                  static_cast<uint32_t>((beg->position.y() - mat_limit.z()) /
                                        resolution);
              auto end_x = static_cast<uint32_t>(
                  (end->position.x() - mat_limit.x()) / resolution);
              auto end_y =
                  vehicle_view->rows -
                  static_cast<uint32_t>((end->position.y() - mat_limit.z()) /
                                        resolution);
              cv::circle(*vehicle_view, cv::Point2i(beg_x, beg_y), 5,
                         cv::Scalar(std::get<2>(color), std::get<1>(color),
                                    std::get<0>(color)),
                         -1, cv::LINE_AA);
              cv::circle(*vehicle_view, cv::Point2i(end_x, end_y), 5,
                         cv::Scalar(std::get<2>(color), std::get<1>(color),
                                    std::get<0>(color)),
                         -1, cv::LINE_AA);
              double arrowed_len = 12;
              if (dr->route->current_step->vehicle_orientation ==
                  data::order::Step::Orientation::FORWARD) {
                cv::arrowedLine(
                    *vehicle_view, cv::Point2i(beg_x, beg_y),
                    cv::Point2i(end_x, end_y),
                    cv::Scalar(std::get<2>(color), std::get<1>(color),
                               std::get<0>(color)),
                    5, 8, 0,
                    arrowed_len /
                        Eigen::Vector2i(beg_x - end_x, beg_y - end_y).norm());
              } else if (dr->route->current_step->vehicle_orientation ==
                         data::order::Step::Orientation::BACKWARD) {
                cv::arrowedLine(
                    *vehicle_view, cv::Point2i(end_x, end_y),
                    cv::Point2i(beg_x, beg_y),
                    cv::Scalar(std::get<2>(color), std::get<1>(color),
                               std::get<0>(color)),
                    5, 8, 0,
                    arrowed_len /
                        Eigen::Vector2i(beg_x - end_x, beg_y - end_y).norm());
              }
            }

            // step
            for (auto &path : dr->route->steps) {
              auto beg = path->path->source_point.lock();
              auto end = path->path->destination_point.lock();
              auto beg_x = static_cast<uint32_t>(
                  (beg->position.x() - mat_limit.x()) / resolution);
              auto beg_y =
                  vehicle_view->rows -
                  static_cast<uint32_t>((beg->position.y() - mat_limit.z()) /
                                        resolution);
              auto end_x = static_cast<uint32_t>(
                  (end->position.x() - mat_limit.x()) / resolution);
              auto end_y =
                  vehicle_view->rows -
                  static_cast<uint32_t>((end->position.y() - mat_limit.z()) /
                                        resolution);
              cv::circle(*vehicle_view, cv::Point2i(beg_x, beg_y), 3,
                         cv::Scalar(0, 0, 139), -1, cv::LINE_AA);
              cv::circle(*vehicle_view, cv::Point2i(end_x, end_y), 3,
                         cv::Scalar(0, 0, 139), -1, cv::LINE_AA);
              double arrowed_len = 10;
              if (path->vehicle_orientation ==
                  data::order::Step::Orientation::FORWARD) {
                cv::arrowedLine(
                    *vehicle_view, cv::Point2i(beg_x, beg_y),
                    cv::Point2i(end_x, end_y),
                    cv::Scalar(std::get<2>(color), std::get<1>(color),
                               std::get<0>(color)),
                    5, 8, 0,
                    arrowed_len /
                        Eigen::Vector2i(beg_x - end_x, beg_y - end_y).norm());
              } else if (path->vehicle_orientation ==
                         data::order::Step::Orientation::BACKWARD) {
                cv::arrowedLine(
                    *vehicle_view, cv::Point2i(end_x, end_y),
                    cv::Point2i(beg_x, beg_y),
                    cv::Scalar(std::get<2>(color), std::get<1>(color),
                               std::get<0>(color)),
                    5, 8, 0,
                    arrowed_len /
                        Eigen::Vector2i(beg_x - end_x, beg_y - end_y).norm());
              }
            }
            // location link_point
            if (dr->destination->operation ==
                data::order::DriverOrder::Destination::OpType::NOP) {
              // TODO
            } else if (dr->destination->operation ==
                       data::order::DriverOrder::Destination::OpType::LOAD) {
              // TODO
              auto loction = std::dynamic_pointer_cast<data::model::Location>(
                  dr->destination->destination.lock());
              auto p_x = static_cast<uint32_t>(
                  (loction->position.x() - mat_limit.x()) / resolution);

              auto p_y =
                  vehicle_view->rows -
                  static_cast<uint32_t>(
                      (loction->position.y() - mat_limit.z()) / resolution);
              cv::rectangle(*vehicle_view, cv::Rect(p_x - 12, p_y - 10, 24, 20),
                            cv::Scalar(std::get<2>(color), std::get<1>(color),
                                       std::get<0>(color)),
                            4, cv::LINE_AA, 0);
            } else if (dr->destination->operation ==
                       data::order::DriverOrder::Destination::OpType::UNLOAD) {
              // TODO
              auto loction = std::dynamic_pointer_cast<data::model::Location>(
                  dr->destination->destination.lock());
              auto p_x = static_cast<uint32_t>(
                  (loction->position.x() - mat_limit.x()) / resolution);

              auto p_y =
                  vehicle_view->rows -
                  static_cast<uint32_t>(
                      (loction->position.y() - mat_limit.z()) / resolution);
              cv::rectangle(*vehicle_view, cv::Rect(p_x - 12, p_y - 10, 24, 20),
                            cv::Scalar(std::get<2>(color), std::get<1>(color),
                                       std::get<0>(color)),
                            4, cv::LINE_AA, 0);
            }
          }
        }
      }
    }
  }
}
}  // namespace visual