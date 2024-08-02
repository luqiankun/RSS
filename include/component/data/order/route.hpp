#ifndef STEP_HPP
#define STEP_HPP
#include "../model/path.hpp"
namespace data {
namespace order {
class Step : public TCSObject {
 public:
  enum class Orientation { FORWARD, BACKWARD, UNDEFINED };
  using TCSObject::TCSObject;
  int wait_time{0};
  std::shared_ptr<model::Path> path;
  int route_index{0};
  bool execution_allowed{false};
  Orientation vehicle_orientation{Orientation::UNDEFINED};
};
class Route : public TCSObject {
 public:
  using TCSObject::TCSObject;
  std::deque<std::shared_ptr<Step>> steps;
  std::shared_ptr<Step> current_step;
  int64_t costs{0};
};
}  // namespace order
}  // namespace data
#endif