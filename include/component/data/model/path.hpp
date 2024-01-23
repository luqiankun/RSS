#ifndef PATH_HPP
#define PATH_HPP
#include "../../tcsresource.hpp"
#include "point.hpp"
namespace data {
namespace model {
class Path : public TCSResource {
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
    std::vector<Eigen::Vector2i> control_points;
  };

 public:
  using TCSResource::TCSResource;
  std::weak_ptr<Point> source_point;
  std::weak_ptr<Point> destination_point;
  int64_t length{0};
  int max_vel{0};
  int max_reverse_vel{0};
  bool locked{false};
  PathLayout layout;
};
}  // namespace model
}  // namespace data
#endif
