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
  std::unique_lock<std::mutex> lock(mut);
  for (auto &[name, ords] : orderpool) {
    for (const auto &o : ords) {
      if (o->name_hash == order_uuid) {
        o->state = data::order::TransportOrder::State::WITHDRAWL;
      }
    }
  }
  for (const auto &x : temp_orderpool) {
    if (x->name_hash == order_uuid) {
      x->state = data::order::TransportOrder::State::WITHDRAWL;
    }
  }
  for (const auto &x : ended_orderpool) {
    if (x->name_hash == order_uuid) {
      x->state = data::order::TransportOrder::State::WITHDRAWL;
    }
  }
}

void OrderPool::pop(const TransOrderPtr &order) {
  std::unique_lock<std::mutex> lock(mut);
  for (auto &[name, ords] : orderpool) {
    if (ords.back() == order) {
      ords.pop_back();
      ended_orderpool.push_back(order);
      break;
    }
  }
  for (auto it = temp_orderpool.begin(); it != temp_orderpool.end();) {
    if (*it == order) {
      it = temp_orderpool.erase(it);
      ended_orderpool.push_back(order);
      break;
    } else {
      ++it;
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

void OrderPool::patch(const TransOrderPtr &order) {
  for (auto it = temp_orderpool.begin(); it != temp_orderpool.end();) {
    if (*it == order) {
      it = temp_orderpool.erase(it);
      push(order);
      return;
    } else {
      ++it;
    }
  }
  push(order);
}

void OrderPool::push(const TransOrderPtr &order) {
  std::string name{"unspecified"};
  if (order->intended_vehicle.lock()) {
    name = order->intended_vehicle.lock()->name;
  }
  if (name == "unspecified") {
    temp_orderpool.push_back(order);
  } else {
    for (auto &x : orderpool) {
      if (x.first == name) {
        if (order->anytime_drop) {
          if (x.second.back()->anytime_drop) {
            x.second.pop_back();
          }
          std::stable_sort(x.second.begin(), x.second.end(), OrderCmp());
          x.second.push_back(order);
        } else {
          x.second.push_back(order);
          std::stable_sort(x.second.begin(), x.second.end(), OrderCmp());
        }

        // for (auto it = x.second.begin(); it != x.second.end() - 1;) {
        //   if ((*it)->anytime_drop) {
        //     it = x.second.erase(it);
        //   } else {
        //     ++it;
        //   }
        // }
        return;
      }
    }
    std::deque<TransOrderPtr> queue;
    queue.push_back(order);
    orderpool.emplace_back(std::pair{name, queue});
  }
}
void OrderPool::redistribute(const TransOrderPtr &order) {
  std::unique_lock<std::mutex> lock(mut);
  for (auto it = ended_orderpool.begin(); it != ended_orderpool.end();) {
    if (*it == order) {
      ended_orderpool.erase(it);
      order->state = data::order::TransportOrder::State::RAW;
      order->priority += 1;
      if (order->current_driver_index == 0) {
        // 还没执行,可以切换车辆
        order->intended_vehicle.reset();
        order->processing_vehicle.reset();
      }
      push(order);
      break;
    } else {
      it++;
    }
  }
}

std::pair<std::string, TransOrderPtr> OrderPool::get_next_ord() {
  std::unique_lock<std::mutex> lock(mut);
  update_quence();

  if (orderpool.empty()) {
    if (temp_orderpool.empty()) {
      return std::pair{"", nullptr};
    } else {
      auto x = temp_orderpool.front();
      return {"", x};
    }
  } else {
    if (cur_index >= orderpool.size()) {
      cur_index = 0;
    }
    auto current = orderpool.at(cur_index);
    cur_index++;
    if (current.second.empty()) {
      if (temp_orderpool.empty()) {
        return std::pair{"", nullptr};
      } else {
        auto x = temp_orderpool.front();
        return {"", x};
      }
    } else {
      return {current.first, current.second.back()};
    }
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
  std::unique_lock<std::mutex> lock(mut);
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
  for (auto &x : temp_orderpool) {
    x->state = data::order::TransportOrder::State::WITHDRAWL;
    ended_orderpool.push_back(x);
  }
}
std::vector<TransOrderPtr> OrderPool::get_all_order() {
  std::unique_lock<std::mutex> lock(mut);
  std::vector<kernel::allocate::TransOrderPtr> pro_orders = {};
  for (auto &o : orderpool) {
    for (auto &ord : o.second) {
      pro_orders.push_back(ord);
    }
  }
  std::vector<kernel::allocate::TransOrderPtr> end_orders{
      ended_orderpool.begin(), ended_orderpool.end()};
  std::sort(pro_orders.begin(), pro_orders.end(),
            [](const kernel::allocate::TransOrderPtr &a,
               const kernel::allocate::TransOrderPtr &b) {
              return a->create_time > b->create_time;
            });
  std::sort(end_orders.begin(), end_orders.end(),
            [=](const kernel::allocate::TransOrderPtr &a,
                const kernel::allocate::TransOrderPtr &b) {
              return a->create_time > b->create_time;
            });
  pro_orders.insert(pro_orders.end(), end_orders.begin(), end_orders.end());
  pro_orders.insert(pro_orders.end(), temp_orderpool.begin(),
                    temp_orderpool.end());
  return pro_orders;
}
}  // namespace kernel::allocate