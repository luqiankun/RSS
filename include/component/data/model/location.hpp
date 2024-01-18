#ifndef LOCATION_HPP
#define LOCATION_HPP
#include "../../tcsresource.hpp"
#include "point.hpp"
namespace data {
namespace model {
class Location : public TCSResource {
 public:
  using TCSResource::TCSResource;
  Eigen::Vector3i position{0, 0, 0};
  Eigen::Vector2i layout{0, 0};
  bool locked{false};
  std::weak_ptr<Point> link;
};
}  // namespace model
}  // namespace data
#endif
