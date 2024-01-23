#ifndef POINT_HPP
#define POINT_HPP
#include "../../tcsresource.hpp"
namespace data {
namespace model {
class Path;
class Location;
class Point : public TCSResource {
 public:
  using TCSResource::TCSResource;
  struct Layout {
    Eigen::Vector2i position;
    Eigen::Vector2i label_offset;
    int layer_id{0};
  };
  enum class Type {
    REPORT_POSITION,
    HALT_POSITION,
    PARK_POSITION,
    NORMAL_POSITION,
    UNKNOWN
  };
  Eigen::Vector3i position{0, 0, 0};  // x y
  Layout layout;
  Type type{Type::UNKNOWN};
  std::vector<std::shared_ptr<Path>> incoming_paths;
  std::vector<std::shared_ptr<Path>> outgoing_paths;
  std::vector<std::shared_ptr<Location>> attached_links;
};
}  // namespace model
}  // namespace data
#endif