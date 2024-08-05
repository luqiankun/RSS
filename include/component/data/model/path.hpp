#ifndef PATH_HPP
#define PATH_HPP
#include "../../rssresource.hpp"
#include "./action.hpp"
#include "point.hpp"
namespace data {
namespace model {
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
};
}  // namespace model
}  // namespace data
#endif
