#include "../../../include/kernel/allocate/order.hpp"

namespace kernel {
namespace allocate {
DriverOrderPtr OrderPool::route_to_driverorder(RoutePtr route, DestPtr dest) {
  auto ord =
      std::make_shared<data::order::DriverOrder>("driverorder_" + route->name);
  ord->destination = dest;
  ord->route = route;
  ord->state = data::order::DriverOrder::State::PRISTINE;
  return ord;
}
DestPtr OrderPool::res_to_destination(
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
      x->state = data::order::TransportOrder::State::WITHDRAWL;
    }
  }
  for (auto& x : ended_orderpool) {
    if (x->name_hash == order_uuid) {
      x->state = data::order::TransportOrder::State::WITHDRAWL;
    }
  }
}

TransOrderPtr OrderPool::pop() {
  update_quence();
  if (orderpool.empty()) {
    return nullptr;
  } else {
    auto current = orderpool.front();
    ended_orderpool.push_back(current);
    orderpool.pop_front();
    return current;
  }
}

void OrderPool::update_quence() {
  for (auto& seq : orderquence) {
    for (int i = 0; i < seq->orders.size(); i++) {
      if (seq->orders[i]->state == data::order::TransportOrder::State::FAILED) {
        seq->failure_fatal = true;
        break;
      } else if (seq->orders[i]->state ==
                 data::order::TransportOrder::State::FINISHED) {
        seq->finished_index = i;
      } else if (seq->orders[i]->state ==
                 data::order::TransportOrder::State::BEING_PROCESSED) {
        seq->processing_vehicle = seq->orders[i]->processing_vehicle;
      }
    }
    if (!seq->failure_fatal) {
      if (seq->complete) {
        if (seq->finished_index == seq->orders.size()) {
          seq->finished = true;
        }
      }
    }
  }
}

void OrderPool::cancel_all_order() {
  for (auto& x : orderpool) {
    if (x->state == data::order::TransportOrder::State::RAW ||
        x->state == data::order::TransportOrder::State::ACTIVE ||
        x->state == data::order::TransportOrder::State::DISPATCHABLE ||
        x->state == data::order::TransportOrder::State::BEING_PROCESSED)
      x->state = data::order::TransportOrder::State::WITHDRAWL;
  }
  for (auto& x : ended_orderpool) {
    if (x->state == data::order::TransportOrder::State::RAW ||
        x->state == data::order::TransportOrder::State::ACTIVE ||
        x->state == data::order::TransportOrder::State::DISPATCHABLE ||
        x->state == data::order::TransportOrder::State::BEING_PROCESSED)
      x->state = data::order::TransportOrder::State::WITHDRAWL;
  }
}
}  // namespace allocate
}  // namespace kernel