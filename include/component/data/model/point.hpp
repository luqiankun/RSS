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
  enum class Type {
    REPORT_POSITION,
    HALT_POSITION,
    PARK_POSITION,
    NORMAL_POSITION,
    UNKNOWN
  };
  Eigen::Vector2i pose{0, 0};  // x y
  Eigen::Vector2i layout{0, 0};
  Type type{Type::UNKNOWN};
  std::vector<std::shared_ptr<Path>> incoming_paths;
  std::vector<std::shared_ptr<Path>> outgoing_paths;
  std::vector<std::shared_ptr<Location>> attached_links;
};
}  // namespace model
}  // namespace data
#endif