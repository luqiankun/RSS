#ifndef TRANSPORTORDER_HPP
#define TRANSPORTORDER_HPP

#include <shared_mutex>

#include "../../../3rdparty/log/easylogging++.h"
#include "./driverorder.hpp"
namespace kernel::driver {
class Vehicle;
}
namespace kernel::dispatch {
class ConflictPool;
}
namespace data::order {
enum class Level {
  Level_0 = 0,
  Level_1 = 1,
  Level_2 = 2,
  Level_3 = 3,
  Level_4 = 4,
  Level_5 = 5,
  Level_6 = 6,
};
class OrderSequence;
class TransportOrder : public RSSObject {
 public:
  using RSSObject::RSSObject;
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
  [[nodiscard]] std::string get_state() const {
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
  ~TransportOrder() override {
    if (state == State::BEING_PROCESSED) {
      CLOG(WARNING, order_log)
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
  std::weak_ptr<kernel::dispatch::ConflictPool> conflict_pool;
  bool anytime_drop{false};           // 不影响车辆状态，可随时丢弃的
  int priority{int(Level::Level_0)};  // 优先级
  int switch_veh{false};
  mutable std::shared_mutex mutex;
};
}  // namespace data::order
#endif