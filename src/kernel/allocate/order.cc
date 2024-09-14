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
    for (auto iter = ords.begin(); iter != ords.end();) {
      if (*iter == order) {
        iter = ords.erase(iter);
        ended_orderpool.push_back(order);
      } else {
        ++iter;
      }
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
  if (order->intended_vehicle.lock()) {
    bool flag = false;
    for (auto &o : orderpool) {
      if (o.first == order->intended_vehicle.lock()->name) {
        o.second.push_back(order);
        flag = true;
        break;
      }
    }
    if (!flag) {
      orderpool.emplace_back(
         order->intended_vehicle.lock()->name, std::vector{order});
    }
  } else {
    orderpool.emplace_back(std::pair{"", std::vector{order}});
  }
}

std::pair<std::string, std::vector<TransOrderPtr>> OrderPool::get_next_vec() {
  update_quence();
  if (orderpool.empty()) {
    return std::pair{"", std::vector<TransOrderPtr>{}};
  } else {
    if (cur_index >= orderpool.size()) {
      cur_index = 0;
    }
    auto current = orderpool.at(cur_index);
    cur_index++;

    return current;
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
} // namespace kernel::allocate