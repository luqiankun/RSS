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
    enum class OpType {
      NOP,     // 啥也不干
      LOAD,    // 去某location load
      UNLOAD,  // 去某location unload
      MOVE,    // 去某point
      CHARGE,  // 充电
      CLOSE,
      OPEN,
      LIFT
    };
    std::map<std::string, std::string> properties;
    std::weak_ptr<TCSResource> destination;
    OpType operation{OpType::NOP};
    static std::optional<OpType> get_optype(const std::string& op) {
      if (op == "NOP") {
        return OpType::NOP;
      } else if (op == "LOAD") {
        return OpType::LOAD;
      } else if (op == "UNLOAD") {
        return OpType::UNLOAD;
      } else if (op == "CLOSE") {
        return OpType::CLOSE;
      } else if (op == "OPEN") {
        return OpType::OPEN;
      } else if (op == "LIFT") {
        return OpType::LIFT;
      } else if (op == "CHARGE") {
        return OpType::CHARGE;
      } else if (op == "MOVE") {
        return OpType::MOVE;
      } else {
        return std::nullopt;
      }
    }
    std::string get_type() {
      if (operation == OpType::MOVE) {
        return "MOVE";
      } else if (operation == OpType::LOAD) {
        return "LOAD";
      } else if (operation == OpType::UNLOAD) {
        return "UNLOAD";
      } else if (operation == OpType::CHARGE) {
        return "CHARGE";
      } else if (operation == OpType::OPEN) {
        return "OPEN";
      } else if (operation == OpType::CLOSE) {
        return "CLOSE";
      } else if (operation == OpType::LIFT) {
        return "LIFT";
      } else {
        return "NOP";
      }
    }
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
  }

 public:
  std::shared_ptr<Destination> destination;
  std::shared_ptr<Route> route;
  State state{State::PRISTINE};
  std::weak_ptr<TransportOrder> transport_order;
};
}  // namespace order
}  // namespace data
#endif