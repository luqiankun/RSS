#include "../../../include/kernel/allocate/order.hpp"

namespace kernel {
namespace allocate {
std::shared_ptr<data::order::DriverOrder> OrderPool::route_to_driverorder(
    std::shared_ptr<data::order::Route> route,
    std::shared_ptr<data::order::DriverOrder::Destination> dest) {
  auto ord =
      std::make_shared<data::order::DriverOrder>("driverorder_" + route->name);
  ord->destination = dest;
  ord->route = route;
  ord->state = data::order::DriverOrder::State::PRISTINE;
  return ord;
}
std::shared_ptr<data::order::DriverOrder::Destination>
OrderPool::res_to_destination(
    const std::shared_ptr<TCSResource>& res,
    data::order::DriverOrder::Destination::OpType op) {
  auto destination = std::make_shared<data::order::DriverOrder::Destination>();
  destination->operation = op;
  destination->destination = res;
  return destination;
}
void OrderPool::cancel_order(size_t order_uuid) {
  for (auto& x : orderpool) {
    if (x->name_hash == order_uuid) {
      x->state = data::order::TransportOrder::State::WITHDRAWN;
    }
  }
  for (auto& x : ended_orderpool) {
    if (x->name_hash == order_uuid) {
      x->state = data::order::TransportOrder::State::WITHDRAWN;
    }
  }
}

std::shared_ptr<data::order::TransportOrder> OrderPool::pop() {
  if (orderpool.empty()) {
    return nullptr;
  } else {
    auto current = orderpool.front();
    ended_orderpool.push_back(current);
    orderpool.pop_front();
    return current;
  }
}

void OrderPool::cancel_all_order() {
  for (auto& x : orderpool) {
    if (x->state == data::order::TransportOrder::State::RAW ||
        x->state == data::order::TransportOrder::State::ACTIVE ||
        x->state == data::order::TransportOrder::State::DISPATCHABLE ||
        x->state == data::order::TransportOrder::State::BEING_PROCESSED)
      x->state = data::order::TransportOrder::State::WITHDRAWN;
  }
  for (auto& x : ended_orderpool) {
    if (x->state == data::order::TransportOrder::State::RAW ||
        x->state == data::order::TransportOrder::State::ACTIVE ||
        x->state == data::order::TransportOrder::State::DISPATCHABLE ||
        x->state == data::order::TransportOrder::State::BEING_PROCESSED)
      x->state = data::order::TransportOrder::State::WITHDRAWN;
  }
}
}  // namespace allocate
}  // namespace kernel