#ifndef DRIVERORDER_HPP
#define DRIVERORDER_HPP

#include "route.hpp"
namespace data::order {
class TransportOrder;
class DriverOrder : public RSSObject {
public:
  using RSSObject::RSSObject;
  enum class State { PRISTINE, TRAVELLING, OPERATING, FINISHED, FAILED };

  struct Destination {
    using OpType = data::model::Actions::OpType;
    std::map<std::string, std::string> properties;
    std::weak_ptr<RSSResource> destination;
    OpType operation{OpType::NOP};

    [[nodiscard]] std::string get_type() const {
      return data::model::Actions::get_type(operation);
    };
  };
  [[nodiscard]] std::string get_cmd_name() const {
    if (route->steps.empty()) {
      return "->operation]";
    } else {
      std::string msg{"{"};
      if (route->current_step) {
        if (route->current_step->vehicle_orientation ==
            Step::Orientation::FORWARD) {
          msg.append("[");
          msg.append(route->current_step->name);
          msg.append("->forward]");
        } else {
          msg.append("[");
          msg.append(route->current_step->name);
          msg.append("->backward]");
        }
      }
      msg.append("}");
      return msg;
    }
  }
  [[nodiscard]] std::string get_state() const {
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
} // namespace data::order
#endif