#ifndef DRIVERORDER_HPP
#define DRIVERORDER_HPP

#include "route.hpp"
namespace data {
namespace order {
class TransportOrder;
class DriverOrder : public TCSObject {
 public:
  using TCSObject::TCSObject;
  enum class State { PRISTINE, TRAVELLING, OPERATING, FINISHED, FAILED };

  struct Destination {
    using OpType = data::model::Actions::OpType;
    std::map<std::string, std::string> properties;
    std::weak_ptr<TCSResource> destination;
    OpType operation{OpType::NOP};

    std::string get_type() {
      return data::model::Actions::get_type(operation);
    };
  };
  std::string get_cmd_name() {
    if (route->steps.empty()) {
      return "->operation]";
    } else {
      std::string msg{"{"};
      for (auto& x : route->current_steps) {
        if (x->vehicle_orientation == Step::Orientation::FORWARD) {
          msg.append("[");
          msg.append(x->name);
          msg.append("->forward]");
        } else {
          msg.append("[");
          msg.append(x->name);
          msg.append("->backward]");
        }
      }
      msg.append("}");
      return msg;
    }
  }
  std::string get_state() {
    if (state == State::PRISTINE) {
      return "PRISTINE";
    } else if (state == State::TRAVELLING) {
      return "TRAVELLING";

    } else if (state == State::OPERATING) {
      return "OPERATING";

    } else if (state == State::FINISHED) {
      return "FINISHED";

    } else {
      return "FAILED";
    }
  };

 public:
  std::shared_ptr<Destination> destination;
  std::shared_ptr<Route> route;
  State state{State::PRISTINE};
  std::weak_ptr<TransportOrder> transport_order;
};
}  // namespace order
}  // namespace data
#endif