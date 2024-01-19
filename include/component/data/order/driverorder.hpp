#ifndef DRIVERORDER_HPP
#define DRIVERORDER_HPP

#include "route.hpp"
namespace data {
namespace order {
class TransportOrder;
class DriverOrder : public TCSObject {
 public:
  std::string get_cmd_name() {
    if (route->steps.empty()) {
      return "->operation]";
    } else {
      std::string msg;
      if (route->current_step->vehicle_orientation ==
          Step::Orientation::FORWARD) {
        msg = "->forward]";
      } else {
        msg = "->backward]";
      }
      return "[" + route->current_step->name + msg;
    }
  }

 public:
  using TCSObject::TCSObject;
  enum class State { PRISTINE, TRAVELLING, OPERATING, FINISHED, FAILED };

  struct Destination {
    enum class OpType {
      NOP,    // 去某point
      LOAD,   // 去某location load
      UNLOAD  // 去某location unload
    };
    std::map<std::string, std::string> properties;
    std::weak_ptr<TCSResource> destination;
    OpType operation{OpType::NOP};
  };
  std::shared_ptr<Destination> destination;
  std::shared_ptr<Route> route;
  State state{State::PRISTINE};
  std::weak_ptr<TransportOrder> transport_order;
};
}  // namespace order
}  // namespace data
#endif