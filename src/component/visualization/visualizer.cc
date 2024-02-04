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
bool Visualizer::init(double res, std::shared_ptr<TCS> tcs) {
  this->resolution = res;
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
  CLOG(INFO, "visual") << "mat_limit" << mat_limit;

  if (this->resolution <= 0) {
    this->resolution =
        (mat_limit.y() - mat_limit.x()) * 1.0 / 1440;  // 自动设置分辨率
  }
  CLOG(INFO, "visual") << "resolution : " << resolution << " mm/pix";
  auto width =
      static_cast<uint32_t>((mat_limit.y() - mat_limit.x()) / resolution);
  auto height =
      static_cast<uint32_t>((mat_limit.w() - mat_limit.z()) / resolution);
  cv::Mat mat = cv::Mat(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
  CLOG(INFO, "visual") << "image size " << mat.size;
  for (auto &x : tcs->resource->points) {
    auto p_x = static_cast<uint32_t>((x->layout.position.x() - mat_limit.x()) /
                                     resolution);
    auto p_y =
        mat.rows - static_cast<uint32_t>(
                       (x->layout.position.y() - mat_limit.z()) / resolution);
    // CLOG(INFO,"visual") << p_x << " " << p_y;
    cv::putText(mat, x->name, cv::Point2i(p_x + 5, p_y + 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(90, 90, 90), 1,
                cv::LINE_8);
    cv::circle(mat, cv::Point2i(p_x, p_y), 4, cv::Scalar(0, 0, 0), 1,
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
    Eigen::Vector2f line;
    line.x() = static_cast<double>(end_layout_x) - static_cast<double>(be_x);
    line.y() = static_cast<double>(end_layout_y) - static_cast<double>(be_y);
    Eigen::Vector2f norm_line = line.normalized() * (line.norm() - 6);
    cv::Scalar color = cv::Scalar(9, 9, 9);
    int thickness = 1;
    if (x->locked) {
      color = cv::Scalar(255, 255, 255);
      thickness = 3;
    }
    if (x->max_reverse_vel > 0) {
      arrow_line(mat, cv::Point2i(be_x + line.x(), be_y + line.y()),
                 cv::Point2i(be_x, be_y), color, thickness, false);
    } else {
      arrow_line(mat, cv::Point2i(be_x + line.x(), be_y + line.y()),
                 cv::Point2i(be_x, be_y), color, thickness, true);
    }
  }

  for (auto &x : tcs->resource->locations) {
    auto p_x = static_cast<uint32_t>((x->layout.position.x() - mat_limit.x()) /
                                     resolution);

    auto p_y =
        mat.rows - static_cast<uint32_t>(
                       (x->layout.position.y() - mat_limit.z()) / resolution);
    cv::Scalar color = cv::Scalar(90, 90, 90);
    int thickness = 1;
    if (x->locked) {
      color = cv::Scalar(150, 150, 150);
      thickness = -1;
    }
    cv::putText(mat, x->name, cv::Point2i(p_x - 30, p_y - 22),
                cv::FONT_HERSHEY_SIMPLEX, 0.2, cv::Scalar(90, 90, 90), 1,
                cv::LINE_8);
    cv::rectangle(mat, cv::Rect(p_x - 14, p_y - 10, 24, 20), color, thickness,
                  cv::LINE_8, 0);
    cv::circle(mat, cv::Point2i(p_x, p_y), 3, cv::Scalar(70, 70, 70), 1,
               cv::LINE_AA);

    if (x->link.lock()) {
      auto link_x = static_cast<uint32_t>(
          (x->link.lock()->layout.position.x() - mat_limit.x()) / resolution);
      auto link_y =
          mat.rows - static_cast<uint32_t>(
                         (x->link.lock()->layout.position.y() - mat_limit.z()) /
                         resolution);
      dashed_line(mat, cv::Point2i(link_x, link_y), cv::Point2i(p_x, p_y),
                  cv::Scalar(30, 10, 201), 1, 5);
    }
  }
  map_view = std::shared_ptr<cv::Mat>(new cv::Mat(mat));
  return true;
}

void Visualizer::run() {
  if (!is_run) {
    is_run = true;
    th = std::thread{[&] {
      CLOG(INFO, "visual") << "visualization run";
      namedWindow("TCS_VIEW", cv::WINDOW_NORMAL);
      while (!dispose) {
        if (!is_paused) {
          update();
          auto img = std::shared_ptr<cv::Mat>(new cv::Mat(map_view->clone()));
          auto mask = std::shared_ptr<cv::Mat>(new cv::Mat);
          cv::cvtColor(*vehicle_view, *mask, cv::COLOR_BGR2GRAY);
          cv::threshold(*mask, *mask, 1, 255, cv::THRESH_BINARY);
          // cv::bitwise_not(mask, mask);

          cv::bitwise_not(*map_view, *img, *mask);
          cv::add(*img, *vehicle_view, *img);
          cv::imshow("TCS_VIEW", *img);
        }
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
  // TODO 资源枷锁 或者设置flag 旧模型析构前停止运行，新模型加载后重新运行
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
    paint_step(*vehicle_view, v);
    paint_vehicle(*vehicle_view, v);
  }
}

void Visualizer::arrow_line(cv::Mat mat, cv::Point2i start, cv::Point2i end,
                            cv::Scalar color, int thickness, bool single) {
  {
    Eigen::Vector2f line(end.x - start.x, end.y - start.y);
    int len_v = 3;
    int len = 8;
    line = line.normalized() * (line.norm() - 14);
    Eigen::Vector2f d = line - line.normalized() * len;
    Eigen::Vector2f horz_a(-d.y() / 2, d.x() / 2);
    Eigen::Vector2f horz_b(d.y() / 2, -d.x() / 2);
    Eigen::Vector2f a = d + horz_a.normalized() * len_v;
    Eigen::Vector2f b = d + horz_b.normalized() * len_v;
    cv::Point2i start_ = start + cv::Point2i((line.normalized() * 7).x(),
                                             (line.normalized() * 7).y());
    cv::Point2i end_ = start_ + cv::Point2i(line.x(), line.y());
    cv::Point2i a_ = start_ + cv::Point2i(a.x(), a.y());
    cv::Point2i b_ = start_ + cv::Point2i(b.x(), b.y());
    cv::line(mat, start_, end_, color, thickness, cv::LINE_4);
    std::vector<cv::Point2i> poly;
    poly.push_back(end_);
    poly.push_back(a_);
    poly.push_back(b_);
    cv::polylines(mat, poly, true, color, thickness, cv::LINE_8);
    cv::fillPoly(mat, poly, color, cv::LINE_8);
  }
  if (!single) {
    Eigen::Vector2f line(start.x - end.x, start.y - end.y);
    int len_v = 3;
    int len = 8;
    line = line.normalized() * (line.norm() - 14);
    Eigen::Vector2f d = line - line.normalized() * len;
    Eigen::Vector2f horz_a(-d.y() / 2, d.x() / 2);
    Eigen::Vector2f horz_b(d.y() / 2, -d.x() / 2);
    Eigen::Vector2f a = d + horz_a.normalized() * len_v;
    Eigen::Vector2f b = d + horz_b.normalized() * len_v;
    cv::Point2i start_ = end + cv::Point2i((line.normalized() * 7).x(),
                                           (line.normalized() * 7).y());
    cv::Point2i end_ = start_ + cv::Point2i(line.x(), line.y());
    cv::Point2i a_ = start_ + cv::Point2i(a.x(), a.y());
    cv::Point2i b_ = start_ + cv::Point2i(b.x(), b.y());
    std::vector<cv::Point2i> poly;
    poly.push_back(end_);
    poly.push_back(a_);
    poly.push_back(b_);
    cv::polylines(mat, poly, true, color, thickness, cv::LINE_8);
    cv::fillPoly(mat, poly, color, cv::LINE_8);
  }
}

void Visualizer::arrow_dashed_line(cv::Mat mat, cv::Point2i start,
                                   cv::Point2i end, cv::Scalar color,
                                   int thickness, bool forward) {
  if (forward) {
    Eigen::Vector2f line(end.x - start.x, end.y - start.y);
    int len_v = 3 + (thickness - 1) * 3;
    int len = 8 + (thickness - 1) * 3;
    line = line.normalized() * (line.norm() - 14);
    Eigen::Vector2f d = line - line.normalized() * len;
    Eigen::Vector2f horz_a(-d.y() / 2, d.x() / 2);
    Eigen::Vector2f horz_b(d.y() / 2, -d.x() / 2);
    Eigen::Vector2f a = d + horz_a.normalized() * len_v;
    Eigen::Vector2f b = d + horz_b.normalized() * len_v;
    cv::Point2i start_ = start + cv::Point2i((line.normalized() * 7).x(),
                                             (line.normalized() * 7).y());
    cv::Point2i end_ = start_ + cv::Point2i(line.x(), line.y());
    cv::Point2i a_ = start_ + cv::Point2i(a.x(), a.y());
    cv::Point2i b_ = start_ + cv::Point2i(b.x(), b.y());
    dashed_line(mat, start_, end_, color, thickness, line.norm() / 20);
    std::vector<cv::Point2i> poly;
    poly.push_back(end_);
    poly.push_back(a_);
    poly.push_back(b_);
    cv::polylines(mat, poly, true, color, 1, cv::LINE_8);
    cv::fillPoly(mat, poly, color, cv::LINE_8);
  } else {
    Eigen::Vector2f line(start.x - end.x, start.y - end.y);
    int len_v = 3 + (thickness - 1) * 3;
    int len = 8 + (thickness - 1) * 3;
    line = line.normalized() * (line.norm() - 14);
    Eigen::Vector2f d = line - line.normalized() * len;
    Eigen::Vector2f horz_a(-d.y() / 2, d.x() / 2);
    Eigen::Vector2f horz_b(d.y() / 2, -d.x() / 2);
    Eigen::Vector2f a = d + horz_a.normalized() * len_v;
    Eigen::Vector2f b = d + horz_b.normalized() * len_v;
    cv::Point2i start_ = end + cv::Point2i((line.normalized() * 7).x(),
                                           (line.normalized() * 7).y());
    cv::Point2i end_ = start_ + cv::Point2i(line.x(), line.y());
    cv::Point2i a_ = start_ + cv::Point2i(a.x(), a.y());
    cv::Point2i b_ = start_ + cv::Point2i(b.x(), b.y());
    dashed_line(mat, end_, start_, color, thickness, line.norm() / 20);
    std::vector<cv::Point2i> poly;
    poly.push_back(end_);
    poly.push_back(a_);
    poly.push_back(b_);
    cv::polylines(mat, poly, true, color, thickness, cv::LINE_8);
    cv::fillPoly(mat, poly, color, cv::LINE_8);
  }
}

void Visualizer::dashed_line(cv::Mat mat, cv::Point2i start, cv::Point2i end,
                             cv::Scalar color, int thickness, int num) {
  cv::line(mat, start, end, cv::Scalar(255, 255, 255), thickness, cv::LINE_AA,
           0);
  double step_x = (end.x - start.x) * 1.0 / num / 2;
  double step_y = (end.y - start.y) * 1.0 / num / 2;
  for (int i = 2; i < num * 2 + 0; i = i + 2) {
    cv::line(
        mat, cv::Point2i(start.x + i * step_x, start.y + i * step_y),
        cv::Point2i(start.x + (i + 1) * step_x, start.y + (i + 1) * step_y),
        color, thickness, cv::LINE_8, 0);
    cv::line(
        mat,
        cv::Point2i(start.x + (i + 1) * step_x, start.y + (i + 1) * step_y),
        cv::Point2i(start.x + (i)*step_x, start.y + (i)*step_y), color,
        thickness, cv::LINE_8, 0);
  }
}

void Visualizer::paint_vehicle(cv::Mat mat,
                               std::shared_ptr<kernel::driver::Vehicle> v) {
  auto rgb = hexToRgb(v->color);
  auto color = cv::Scalar(std::get<2>(rgb), std::get<1>(rgb), std::get<0>(rgb));
  int len = v->length;
  if (len < mat.cols * resolution / 40) {
    len = mat.cols * resolution / 40;
  }
  Eigen::Vector3i A(-0.21 * len, 0.26 * len, 1);
  Eigen::Vector3i B(0.09 * len, 0.26 * len, 1);
  Eigen::Vector3i C(0.47 * len, 0.015 * len, 1);
  Eigen::Vector3i D(0.54 * len, -0.235 * len, 1);
  Eigen::Vector3i E(-0.457 * len, -0.235 * len, 1);
  Eigen::Vector3i F(-0.458 * len, 0.015 * len, 1);
  Eigen::Vector3i G(0.292 * len, 0.015 * len, 1);
  Eigen::Vector3i H(-0.3 * len, 0.028 * len, 1);
  Eigen::Vector3i I(-0.21 * len, -0.235 * len, 1);
  Eigen::Vector3i J(0.29 * len, -0.23 * len, 1);
  // transport
  Eigen::Matrix<int, 3, 3> T;
  T << cos(v->angle), -sin(v->angle), v->position.x(), sin(v->angle),
      cos(v->angle), v->position.y(), 0, 0, 1;
  A = T * A;
  B = T * B;
  C = T * C;
  D = T * D;
  E = T * E;
  F = T * F;
  G = T * G;
  H = T * H;
  I = T * I;
  J = T * J;
  //
  double radius = 0.1 * len / resolution;
  //
  cv::Point2i A_(
      static_cast<uint32_t>((A.x() - mat_limit.x()) / resolution),
      mat.rows - static_cast<uint32_t>((A.y() - mat_limit.z()) / resolution));
  cv::Point2i B_(
      static_cast<uint32_t>((B.x() - mat_limit.x()) / resolution),
      mat.rows - static_cast<uint32_t>((B.y() - mat_limit.z()) / resolution));
  cv::Point2i C_(
      static_cast<uint32_t>((C.x() - mat_limit.x()) / resolution),
      mat.rows - static_cast<uint32_t>((C.y() - mat_limit.z()) / resolution));
  cv::Point2i D_(
      static_cast<uint32_t>((D.x() - mat_limit.x()) / resolution),
      mat.rows - static_cast<uint32_t>((D.y() - mat_limit.z()) / resolution));
  cv::Point2i E_(
      static_cast<uint32_t>((E.x() - mat_limit.x()) / resolution),
      mat.rows - static_cast<uint32_t>((E.y() - mat_limit.z()) / resolution));
  cv::Point2i F_(
      static_cast<uint32_t>((F.x() - mat_limit.x()) / resolution),
      mat.rows - static_cast<uint32_t>((F.y() - mat_limit.z()) / resolution));
  cv::Point2i G_(
      static_cast<uint32_t>((G.x() - mat_limit.x()) / resolution),
      mat.rows - static_cast<uint32_t>((G.y() - mat_limit.z()) / resolution));
  cv::Point2i H_(
      static_cast<uint32_t>((H.x() - mat_limit.x()) / resolution),
      mat.rows - static_cast<uint32_t>((H.y() - mat_limit.z()) / resolution));
  cv::Point2i I_(
      static_cast<uint32_t>((I.x() - mat_limit.x()) / resolution),
      mat.rows - static_cast<uint32_t>((I.y() - mat_limit.z()) / resolution));
  cv::Point2i J_(
      static_cast<uint32_t>((J.x() - mat_limit.x()) / resolution),
      mat.rows - static_cast<uint32_t>((J.y() - mat_limit.z()) / resolution));
  //
  std::vector<cv::Point2i> poly{A_, B_, G_, C_, D_, E_, F_, H_};

  cv::polylines(mat, poly, true, color, 1, cv::LINE_8);
  cv::fillPoly(mat, poly, color, cv::LINE_8);

  //
  cv::circle(mat, I_, radius, color, -1, cv::LINE_AA);
  cv::circle(mat, J_, radius, color, -1, cv::LINE_AA);
  cv::putText(mat, v->name, cv::Point2i(-5, 12) + I_, cv::FONT_HERSHEY_SIMPLEX,
              0.7, color, 1, cv::LINE_8);
}

void Visualizer::paint_step(cv::Mat mat,
                            std::shared_ptr<kernel::driver::Vehicle> v) {
  auto color = hexToRgb(v->color);
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
            auto end = dr->route->current_step->path->destination_point.lock();
            auto beg_x = static_cast<uint32_t>(
                (beg->layout.position.x() - mat_limit.x()) / resolution);
            auto beg_y =
                mat.rows -
                static_cast<uint32_t>(
                    (beg->layout.position.y() - mat_limit.z()) / resolution);
            auto end_x = static_cast<uint32_t>(
                (end->layout.position.x() - mat_limit.x()) / resolution);
            auto end_y =
                mat.rows -
                static_cast<uint32_t>(
                    (end->layout.position.y() - mat_limit.z()) / resolution);
            cv::circle(mat, cv::Point2i(beg_x, beg_y), 5,
                       cv::Scalar(std::get<2>(color), std::get<1>(color),
                                  std::get<0>(color)),
                       -1, cv::LINE_AA);
            cv::circle(mat, cv::Point2i(end_x, end_y), 5,
                       cv::Scalar(std::get<2>(color), std::get<1>(color),
                                  std::get<0>(color)),
                       -1, cv::LINE_AA);
            if (dr->route->current_step->vehicle_orientation ==
                data::order::Step::Orientation::FORWARD) {
              arrow_dashed_line(
                  mat, cv::Point2i(beg_x, beg_y), cv::Point2i(end_x, end_y),
                  cv::Scalar(std::get<2>(color), std::get<1>(color),
                             std::get<0>(color)),
                  3, true);
            } else if (dr->route->current_step->vehicle_orientation ==
                       data::order::Step::Orientation::BACKWARD) {
              arrow_dashed_line(
                  mat, cv::Point2i(end_x, end_y), cv::Point2i(beg_x, beg_y),
                  cv::Scalar(std::get<2>(color), std::get<1>(color),
                             std::get<0>(color)),
                  3, false);
            }
          }

          // step
          for (auto &path : dr->route->steps) {
            auto beg = path->path->source_point.lock();
            auto end = path->path->destination_point.lock();
            auto beg_x = static_cast<uint32_t>(
                (beg->layout.position.x() - mat_limit.x()) / resolution);
            auto beg_y =
                mat.rows -
                static_cast<uint32_t>(
                    (beg->layout.position.y() - mat_limit.z()) / resolution);
            auto end_x = static_cast<uint32_t>(
                (end->layout.position.x() - mat_limit.x()) / resolution);
            auto end_y =
                mat.rows -
                static_cast<uint32_t>(
                    (end->layout.position.y() - mat_limit.z()) / resolution);
            cv::circle(mat, cv::Point2i(beg_x, beg_y), 2, cv::Scalar(0, 0, 139),
                       -1, cv::LINE_AA);
            cv::circle(mat, cv::Point2i(end_x, end_y), 2, cv::Scalar(0, 0, 139),
                       -1, cv::LINE_AA);
            if (path->vehicle_orientation ==
                data::order::Step::Orientation::FORWARD) {
              arrow_dashed_line(
                  mat, cv::Point2i(beg_x, beg_y), cv::Point2i(end_x, end_y),
                  cv::Scalar(std::get<2>(color), std::get<1>(color),
                             std::get<0>(color)),
                  3, true);
            } else if (path->vehicle_orientation ==
                       data::order::Step::Orientation::BACKWARD) {
              arrow_dashed_line(
                  mat, cv::Point2i(end_x, end_y), cv::Point2i(beg_x, beg_y),
                  cv::Scalar(std::get<2>(color), std::get<1>(color),
                             std::get<0>(color)),
                  3, false);
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
                (loction->layout.position.x() - mat_limit.x()) / resolution);

            auto p_y =
                mat.rows - static_cast<uint32_t>(
                               (loction->layout.position.y() - mat_limit.z()) /
                               resolution);
            cv::rectangle(mat, cv::Rect(p_x - 13, p_y - 11, 26, 22),
                          cv::Scalar(std::get<2>(color), std::get<1>(color),
                                     std::get<0>(color)),
                          3, cv::LINE_8, 0);
          } else if (dr->destination->operation ==
                     data::order::DriverOrder::Destination::OpType::UNLOAD) {
            // TODO
            auto loction = std::dynamic_pointer_cast<data::model::Location>(
                dr->destination->destination.lock());
            auto p_x = static_cast<uint32_t>(
                (loction->layout.position.x() - mat_limit.x()) / resolution);

            auto p_y =
                mat.rows - static_cast<uint32_t>(
                               (loction->layout.position.y() - mat_limit.z()) /
                               resolution);
            cv::rectangle(mat, cv::Rect(p_x - 12, p_y - 10, 24, 20),
                          cv::Scalar(std::get<2>(color), std::get<1>(color),
                                     std::get<0>(color)),
                          2, cv::LINE_8, 0);
          }
        }
      }
    }
  }
}

}  // namespace visual