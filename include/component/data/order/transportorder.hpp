#ifndef TRANSPORTORDER_HPP
#define TRANSPORTORDER_HPP

#include "../model/location.hpp"
#include "./driverorder.hpp"

namespace kernel {
namespace driver {
class Vehicle;
}
}  // namespace kernel
namespace data {
namespace order {
class TransportOrder : public TCSObject {
 public:
  using TCSObject::TCSObject;
  enum class State {
    RAW,
    ACTIVE,
    DISPATCHABLE,
    BEING_PROCESSED,
    WITHDRAWN,
    FINISHED,
    FAILED,
    UNROUTABLE
  };
  ~TransportOrder() {
    if (state == State::BEING_PROCESSED) {
      LOG(INFO) << "order: " << name << " not finished, now will be drop";
    } else {
      LOG(INFO) << name << " drop";
    }
  }

 public:
  std::chrono::system_clock::time_point create_time;
  std::chrono::system_clock::time_point end_time;
  std::chrono::system_clock::time_point dead_time;
  std::string type;
  State state{State::RAW};
  size_t name_hash;
  std::deque<std::weak_ptr<TransportOrder>> dependencies;
  std::deque<std::shared_ptr<DriverOrder>> driverorders;
  int current_driver_index{0};
  bool dispensable{false};
  std::weak_ptr<kernel::driver::Vehicle> intended_vehicle;
  std::weak_ptr<kernel::driver::Vehicle> processing_vehicle;
};
}  // namespace order
}  // namespace data
#endif