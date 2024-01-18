#ifndef PATH_HPP
#define PATH_HPP
#include "../../tcsresource.hpp"
#include "point.hpp"
namespace data {
namespace model {
class Path : public TCSResource {
 public:
 public:
  using TCSResource::TCSResource;
  std::weak_ptr<Point> source_point;
  std::weak_ptr<Point> destination_point;
  int64_t length{0};
  int max_vel{0};
  int max_reverse_vel{0};
  bool locked{false};
};
}  // namespace model
}  // namespace data
#endif
