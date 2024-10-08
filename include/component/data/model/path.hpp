#ifndef PATH_HPP
#define PATH_HPP
#include "../../rssresource.hpp"
#include "../../util/nurbs.hpp"
#include "./action.hpp"
#include "point.hpp"
namespace data::model {
class Path : public RSSResource {
 public:
  enum class ConnectType {
    /**
     * A direct connection.
     */
    DIRECT,
    /**
     * An elbow connection.
     */
    ELBOW,
    /**
     * A slanted connection.
     */
    SLANTED,
    /**
     * A polygon path with any number of vertecies.
     */
    POLYPATH,
    /**
     * A bezier curve with 2 control points.
     */
    BEZIER,
    /**
     * A bezier curve with 3 control points.
     */
    BEZIER_3
  };
  struct PathLayout {
    int layer_id{0};
    ConnectType connect_type{ConnectType::DIRECT};
    std::vector<Vector2i> control_points;
  };
  static std::string get_connect_type(ConnectType t) {
    if (t == ConnectType::BEZIER) {
      return "BEZIER";
    } else if (t == ConnectType::BEZIER_3) {
      return "BEZIER_3";
    } else if (t == ConnectType::DIRECT) {
      return "DIRECT";
    } else if (t == ConnectType::ELBOW) {
      return "ELBOW";
    } else if (t == ConnectType::POLYPATH) {
      return "POLYPATH";
    } else {
      return "SLANTED";
    }
  }
  std::vector<nurbs::Point> get_nurbs_points(double step) const {
    if (layout.connect_type == ConnectType::BEZIER) {
      std::vector<nurbs::Point> ctrl_ps;
      ctrl_ps.push_back({float(source_point.lock()->position.x()) / 1000,
                         float(source_point.lock()->position.y()) / 1000, 0});
      for (auto& x : layout.control_points) {
        ctrl_ps.push_back(
            {float(x.x() * 50) / 1000, float(-x.y() * 50) / 1000, 0});
      }
      ctrl_ps.push_back({float(destination_point.lock()->position.x()) / 1000,
                         float(destination_point.lock()->position.y()) / 1000,
                         0});
      std::vector<double> weights{1, 1, 1, 1};
      int degree = 3;
      return nurbs::calculate_nurbs(ctrl_ps, weights, degree, step);
    } else if (layout.connect_type == ConnectType::BEZIER_3) {
      std::vector<nurbs::Point> ctrl_ps;
      ctrl_ps.push_back({float(source_point.lock()->position.x()) / 1000,
                         float(source_point.lock()->position.y()) / 1000, 0});
      for (auto& x : layout.control_points) {
        ctrl_ps.push_back(
            {float(x.x() * 50) / 1000, float(-x.y() * 50) / 1000, 0});
      }
      ctrl_ps.push_back({float(destination_point.lock()->position.x()) / 1000,
                         float(destination_point.lock()->position.y()) / 1000,
                         0});
      std::vector<double> weights{1, 1, 1, 1, 1, 1, 1};
      int degree = 6;
      return nurbs::calculate_nurbs(ctrl_ps, weights, degree, step);
    } else {
      std::vector<nurbs::Point> res;
      Eigen::Vector2f vec{destination_point.lock()->position.x() -
                              source_point.lock()->position.x(),
                          destination_point.lock()->position.y() -
                              source_point.lock()->position.y()};
      Eigen::Vector2f n = vec * step;
      for (int i = 0; i < 1.0 / step; ++i) {
        Eigen::Vector2f temp =
            Eigen::Vector2f(source_point.lock()->position.x(),
                            source_point.lock()->position.y()) +
            n * i;
        res.push_back({temp.x(), temp.y(), 0});
      }
      return res;
    }
  }
  static ConnectType new_connect_type(const std::string& name) {
    if (name == "BEZIER") {
      return ConnectType::BEZIER;
    } else if (name == "BEZIER_3") {
      return ConnectType::BEZIER_3;
    } else if (name == "DIRECT") {
      return ConnectType::DIRECT;
    } else if (name == "ELBOW") {
      return ConnectType::ELBOW;
    } else {
      return ConnectType::POLYPATH;
    }
  }

 public:
  using RSSResource::RSSResource;
  std::weak_ptr<Point> source_point;
  std::weak_ptr<Point> destination_point;
  int64_t length{0};
  int max_vel{0};
  int max_reverse_vel{0};
  bool locked{false};
  PathLayout layout;
  Actions acts;  // vda action
  PeripheralActions per_acts;
  std::optional<double> orientation_reverse;
  std::optional<double> orientation_forward;
};
}  // namespace data::model
#endif
