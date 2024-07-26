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
class OrderSequence;
class TransportOrder : public TCSObject {
 public:
  using TCSObject::TCSObject;
  enum class State {
    RAW,
    ACTIVE,
    DISPATCHABLE,
    BEING_PROCESSED,
    WITHDRAWL,
    FINISHED,
    FAILED,
    UNROUTABLE
  };
  std::string get_state() {
    if (state == State::RAW) {
      return "RAW";
    } else if (state == State::ACTIVE) {
      return "ACTIVE";

    } else if (state == State::DISPATCHABLE) {
      return "DISPATCHABLE";

    } else if (state == State::BEING_PROCESSED) {
      return "BEING_PROCESSED";

    } else if (state == State::WITHDRAWL) {
      return "WITHDRAWN";

    } else if (state == State::FINISHED) {
      return "FINISHED";

    } else if (state == State::FAILED) {
      return "FAILED";

    } else {
      return "UNROUTABLE";
    }
  }
  ~TransportOrder() {
    if (state == State::BEING_PROCESSED) {
      CLOG(INFO, order_log)
          << "order: " << name << " not finished, now will be drop";
    } else {
      CLOG(INFO, order_log) << name << " drop\n";
    }
  }

 public:
  std::chrono::system_clock::time_point create_time;
  std::chrono::system_clock::time_point end_time;
  std::chrono::system_clock::time_point dead_time;
  std::string type;
  std::string peripheral_reservation_token;
  State state{State::RAW};
  std::deque<std::weak_ptr<TransportOrder>> dependencies;
  std::deque<std::shared_ptr<DriverOrder>> driverorders;
  std::weak_ptr<OrderSequence> ordersequence;
  int current_driver_index{0};
  bool dispensable{false};
  std::weak_ptr<kernel::driver::Vehicle> intended_vehicle;
  std::weak_ptr<kernel::driver::Vehicle> processing_vehicle;
  bool anytime_drop{false};  // 不影响车辆状态，可随时丢弃的
};
}  // namespace order
}  // namespace data
#endif