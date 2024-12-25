#include "../../../include/kernel/allocate/order.hpp"

#include "../../../include/kernel/driver/vehicle.hpp"
namespace kernel::allocate {
using std::vector;

DriverOrderPtr OrderPool::route_to_driverorder(const RoutePtr &route,
                                               const DestPtr &dest) {
  auto ord =
      std::make_shared<data::order::DriverOrder>("driverorder_" + route->name);
  ord->destination = dest;
  ord->route = route;
  ord->state = data::order::DriverOrder::State::PRISTINE;
  return ord;
}
DestPtr OrderPool::res_to_destination(
    const std::shared_ptr<RSSResource> &res,
    const data::order::DriverOrder::Destination::OpType op) {
  auto destination = std::make_shared<data::order::DriverOrder::Destination>();
  destination->operation = op;
  destination->destination = res;
  return destination;
}
void OrderPool::cancel_order(size_t order_uuid) {
  for (auto &[name, ords] : orderpool) {
    for (const auto &o : ords) {
      if (o->name_hash == order_uuid) {
        o->state = data::order::TransportOrder::State::WITHDRAWL;
      }
    }
  }
  for (const auto &x : ended_orderpool) {
    if (x->name_hash == order_uuid) {
      x->state = data::order::TransportOrder::State::WITHDRAWL;
    }
  }
}

void OrderPool::pop(const TransOrderPtr &order) {
  for (auto &[name, ords] : orderpool) {
    if (ords.back() == order) {
      ords.pop_back();
      ended_orderpool.push_back(order);
      break;
    }
  }
  for (auto iter = orderpool.begin(); iter != orderpool.end();) {
    if (iter->second.empty()) {
      iter = orderpool.erase(iter);
    } else {
      ++iter;
    }
  }
}
void OrderPool::push(const TransOrderPtr &order) {
  std::string name{"unspecified"};
  if (order->intended_vehicle.lock()) {
    name = order->intended_vehicle.lock()->name;
  }
  bool flag = false;
  for (auto &o : orderpool) {
    if (o.first == order->intended_vehicle.lock()->name) {
      o.second.push_back(order);
      std::sort(o.second.begin(), o.second.end(), OrderCmp());
      flag = true;
      break;
    }
  }
  if (!flag) {
    std::deque<TransOrderPtr> queue;
    queue.push_back(order);
    orderpool.emplace_back(
        std::pair{order->intended_vehicle.lock()->name, queue});
  }
}
void OrderPool::redistribute(const TransOrderPtr &order) {
  for (auto &x : ended_orderpool) {
    if (x == order) {
      order->state = data::order::TransportOrder::State::RAW;
      push(order);
      break;
    }
  }
}

std::pair<std::string, TransOrderPtr> OrderPool::get_next_ord() {
  update_quence();
  if (orderpool.empty()) {
    return std::pair{"", nullptr};
  } else {
    if (cur_index >= orderpool.size()) {
      cur_index = 0;
    }
    auto current = orderpool.at(cur_index);
    cur_index++;
    return {current.first, current.second.back()};
  }
}

void OrderPool::update_quence() const {
  for (auto &seq : orderquence) {
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
  for (auto &x : orderpool) {
    for (auto &o : x.second) {
      o->state = data::order::TransportOrder::State::WITHDRAWL;
      ended_orderpool.push_back(o);
    }
  }
  orderpool.clear();
  for (auto &x : ended_orderpool) {
    x->state = data::order::TransportOrder::State::WITHDRAWL;
  }
}
}  // namespace kernel::allocate