#include "../../../include/kernel/dispatch/dispatch.hpp"

#include <utility>

#include "../../../include/kernel/allocate/order.hpp"
#include "../../../include/kernel/driver/vehicle.hpp"
namespace kernel {
namespace dispatch {
std::shared_ptr<data::order::Route> Dispatcher::paths_to_route(
    std::vector<std::shared_ptr<data::model::Point>> ps) {
  std::string route_name = ps.front()->name + "_" + ps.back()->name;
  auto res = std::make_shared<data::order::Route>(route_name);
  int index{0};
  for (auto it = ps.begin(); it != ps.end() - 1; it++) {
    for (auto& x : resource->paths) {
      if (x->source_point.lock() == *it &&
          x->destination_point.lock() == *(it + 1)) {
        auto step = std::make_shared<data::order::Step>(x->name);
        step->route_index = index++;
        step->vehicle_orientation = data::order::Step::Orientation::FORWARD;
        step->path = x;
        res->steps.push_back(step);
        res->costs += x->length;
      } else if (x->source_point.lock() == *(it + 1) &&
                 x->destination_point.lock() == *(it)) {
        auto step = std::make_shared<data::order::Step>(x->name);
        step->route_index = index++;
        step->path = x;
        step->vehicle_orientation = data::order::Step::Orientation::BACKWARD;
        res->steps.push_back(step);
        res->costs += x->length;
      }
    }
  }
  if (!res->steps.empty()) {
    res->current_step = res->steps.front();
  }
  return res;
}

std::shared_ptr<data::order::DriverOrder> Dispatcher::route_to_driverorder(
    std::shared_ptr<data::order::Route> route,
    std::shared_ptr<data::order::DriverOrder::Destination> dest) {
  auto ord =
      std::make_shared<data::order::DriverOrder>("driverorder_" + route->name);
  ord->destination = dest;
  ord->route = route;
  ord->state = data::order::DriverOrder::State::PRISTINE;
  return ord;
}

std::pair<Dispatcher::ResType, std::shared_ptr<TCSResource>> Dispatcher::find(
    const std::string& name) {
  std::shared_ptr<TCSResource> res;
  for (auto& p : resource->points) {
    if (p->name == name) {
      res = p;
      return std::pair<ResType, std::shared_ptr<TCSResource>>(ResType::Point,
                                                              res);
    }
  }
  for (auto& l : resource->locations) {
    if (l->name == name) {
      res = l;
      return std::pair<ResType, std::shared_ptr<TCSResource>>(ResType::Location,
                                                              res);
    }
  }
  return std::pair<ResType, std::shared_ptr<TCSResource>>(ResType::Err, res);
}

std::shared_ptr<data::order::TransportOrder> Dispatcher::new_orderseq(
    std::vector<Oper> oper, size_t uuid, int strategy) {
  auto time = std::chrono::system_clock::now();
  std::shared_ptr<data::order::TransportOrder> ord =
      std::make_shared<data::order::TransportOrder>("transorder_" +
                                                    get_time_fmt(time));
  ord->uuid = uuid;
  ord->create_time = time;
  ord->state = data::order::TransportOrder::State::RAW;
  LOG(INFO) << "new ord " << ord->name << " uuid " << ord->uuid;
  if (oper.empty()) {
    LOG(WARNING) << ord->name << " op is null";
    ord->state = data::order::TransportOrder::State::FAILED;
    return ord;
  }
  auto start_check = find(std::get<0>(oper.front()));
  if (start_check.first == ResType::Err) {
    ord->state = data::order::TransportOrder::State::FAILED;
    return ord;
  }
  std::shared_ptr<data::model::Point> start;
  if (start_check.first == ResType::Point) {
    start = std::dynamic_pointer_cast<data::model::Point>(start_check.second);
  } else if (start_check.first == ResType::Location) {
    start = std::dynamic_pointer_cast<data::model::Location>(start_check.second)
                ->link.lock();
  }
  std::shared_ptr<driver::Vehicle> vehicle;
  if (strategy >= 0) {
    for (auto& v : vehicles) {
      if (v->uuid == strategy) {
        vehicle = v;
      }
    }
  } else {
    LOG(INFO) << "will find vehicle";
    vehicle = select_vehicle(start);
  }
  if (!vehicle) {
    ord->state = data::order::TransportOrder::State::FAILED;
    LOG(WARNING) << ord->name << " vehicle null ";
    return ord;
  }
  ord->intended_vehicle = vehicle;
  LOG(INFO) << "intended_vehicle " << vehicle->name;
  //
  auto start_planner = vehicle->current_point;
  std::shared_ptr<data::model::Point> end_planner;
  for (auto& op : oper) {
    auto dest = std::get<0>(op);
    LOG(INFO) << dest;
    auto op_type = std::get<1>(op);
    // dest
    start_check = find(dest);
    auto destination = res_to_destination(start_check.second, op_type);
    bool able{false};
    // point
    auto obj_point = find(dest);
    if (obj_point.first == ResType::Point) {
      end_planner =
          std::dynamic_pointer_cast<data::model::Point>(obj_point.second);
    } else if (obj_point.first == ResType::Location) {
      end_planner =
          std::dynamic_pointer_cast<data::model::Location>(obj_point.second)
              ->link.lock();
    }
    if (!end_planner) {
      ord->state = data::order::TransportOrder::State::UNROUTABLE;
      LOG(WARNING) << ord->name << " can not find obj";
      return ord;
    }
    // 路径
    auto path = planner->find_paths(start_planner, end_planner);
    if (path.empty()) {
      ord->state = data::order::TransportOrder::State::UNROUTABLE;
      LOG(WARNING) << ord->name << " can not routable";
      return ord;
    } else {
      auto driverorder =
          route_to_driverorder(paths_to_route(path.front()), destination);
      driverorder->transport_order = ord;
      ord->driverorders.push_back(driverorder);
      able = true;
    }

    if (!able) {
      ord->state = data::order::TransportOrder::State::UNROUTABLE;
      return ord;
    }
    // next
    start_planner = end_planner;
  }
  LOG(INFO) << ord->name << " init ok";
  return ord;
}

std::shared_ptr<data::order::DriverOrder::Destination>
Dispatcher::res_to_destination(
    const std::shared_ptr<TCSResource>& res,
    data::order::DriverOrder::Destination::OpType op) {
  auto destination = std::make_shared<data::order::DriverOrder::Destination>();
  destination->operation = op;
  destination->destination = res;
  return destination;
}

std::shared_ptr<driver::Vehicle> Dispatcher::select_vehicle(
    std::shared_ptr<data::model::Point> start) {
  // TODO 距离判断
  if (vehicles.empty()) {
    return nullptr;
  }
  std::vector<std::shared_ptr<driver::Vehicle>> idle_temp;
  std::vector<std::shared_ptr<driver::Vehicle>> busy_temp;
  std::vector<std::shared_ptr<driver::Vehicle>> charge_temp;

  for (auto& v : vehicles) {
    // if (v->state == driver::Vehicle::State::IDLE &&
    //     v->proc_state == driver::Vehicle::ProcState::AWAITING_ORDER) {
    if (v->state == driver::Vehicle::State::IDLE) {
      if (!v->paused) {
        idle_temp.push_back(v);
      }
    } else if (v->state == driver::Vehicle::State::EXECUTING) {
      busy_temp.push_back(v);
    } else if (v->state == driver::Vehicle::State::CHARGING) {
      charge_temp.push_back(v);
    }
  }
  if (idle_temp.empty()) {
    if (busy_temp.empty()) {
      if (charge_temp.empty()) {
        return nullptr;
      } else {
        std::sort(
            charge_temp.begin(), charge_temp.end(),
            [=](const std::shared_ptr<driver::Vehicle>& a,
                const std::shared_ptr<driver::Vehicle>& b) {
              auto dis_a = Eigen::Vector2i(a->position.x(), a->position.y()) -
                           start->pose;
              auto dis_b = Eigen::Vector2i(b->position.x(), b->position.y()) -
                           start->pose;
              return dis_a.norm() < dis_b.norm();
            });
        return charge_temp.front();
      }
    } else {
      std::sort(
          busy_temp.begin(), busy_temp.end(),
          [=](const std::shared_ptr<driver::Vehicle>& a,
              const std::shared_ptr<driver::Vehicle>& b) {
            auto dis_a =
                Eigen::Vector2i(a->position.x(), a->position.y()) - start->pose;
            auto dis_b =
                Eigen::Vector2i(b->position.x(), b->position.y()) - start->pose;
            return dis_a.norm() < dis_b.norm();
          });
      return busy_temp.front();
    }
  } else {
    std::sort(idle_temp.begin(), idle_temp.end(),
              [=](const std::shared_ptr<driver::Vehicle>& a,
                  const std::shared_ptr<driver::Vehicle>& b) {
                auto dis_a = Eigen::Vector2i(a->position.x(), a->position.y()) -
                             start->pose;
                auto dis_b = Eigen::Vector2i(b->position.x(), b->position.y()) -
                             start->pose;
                return dis_a.norm() < dis_b.norm();
              });
    return idle_temp.front();
  }
}

std::vector<std::vector<std::shared_ptr<driver::Vehicle>>>
Dispatcher::deadlock_loop() {
  std::vector<std::vector<std::shared_ptr<driver::Vehicle>>> res;
  // 死锁检测
  for (auto& v : vehicles) {
    std::stack<std::shared_ptr<driver::Vehicle>> vs;
    std::vector<std::shared_ptr<driver::Vehicle>> exist;
    vs.push(v);
    while (!vs.empty()) {
      auto top = vs.top();
      auto it = std::find(exist.begin(), exist.end(), top);
      if (it != exist.end()) {
        res.emplace_back(
            std::vector<std::shared_ptr<driver::Vehicle>>(it, exist.end()));
      } else {
        exist.push_back(top);
      }
      vs.pop();
      for (auto& r : v->future_claim_resources) {
        auto owner = find_owner(r);
        if (owner) {
          vs.push(owner);
        }
      }
    }
  }
  return res;
}

std::shared_ptr<driver::Vehicle> Dispatcher::find_owner(
    const std::shared_ptr<TCSResource>& res) {
  for (auto& v : vehicles) {
    for (auto& r : v->claim_resources) {
      if (r == res) {
        return v;
      }
    }
  }
  return nullptr;
}

void Dispatcher::cancel_vehicle_order(size_t order_uuid) {
  for (auto& v : vehicles) {
    for (auto& ord : v->orders) {
      if (ord->uuid == order_uuid) {
        ord->state = data::order::TransportOrder::State::WITHDRAWN;
      }
    }
  }
}
void Dispatcher::cancel_order(size_t order_uuid) {
  for (auto& x : orderpool->orderpool) {
    if (x->uuid == order_uuid) {
      x->state = data::order::TransportOrder::State::WITHDRAWN;
    }
  }
}

void Dispatcher::cancel_vehicle_all_order(size_t vehicle_uuid) {
  for (auto& v : vehicles) {
    if (v->uuid == vehicle_uuid) {
      v->orders.clear();
    }
  }
}

void Dispatcher::dispatch_once() {
  if (orderpool->orderpool.empty()) {
    return;
  } else {
    auto current = orderpool->orderpool.front();
    orderpool->ended_orderpool.push_back(current);
    orderpool->orderpool.pop_front();
    LOG(INFO) << current->name << " status: raw";
    if (current->state == data::order::TransportOrder::State::RAW) {
      // TODO
      if (current->driverorders.empty()) {
        current->state = data::order::TransportOrder::State::FINISHED;
        LOG(INFO) << current->name << " status: finished";
      } else {
        current->state = data::order::TransportOrder::State::ACTIVE;
        LOG(INFO) << current->name << " status: active";
      }
    }
    if (current->state == data::order::TransportOrder::State::ACTIVE) {
      // TODO
      auto v = current->intended_vehicle.lock();
      if (v) {
        //
        current->state = data::order::TransportOrder::State::DISPATCHABLE;
        LOG(INFO) << current->name << " status: dispatchable";

      } else {
        //
        current->state = data::order::TransportOrder::State::FAILED;
        LOG(INFO) << current->name << " status: failed";
      }
    }
    if (current->state == data::order::TransportOrder::State::DISPATCHABLE) {
      current->state = data::order::TransportOrder::State::BEING_PROCESSED;
      LOG(INFO) << current->name << " status: being_processed";
      current->intended_vehicle.lock()->receive_task(current);
      current->processing_vehicle = current->intended_vehicle;
      // TODO
    }
    if (current->state == data::order::TransportOrder::State::BEING_PROCESSED) {
      // wait  do nothing
    }
    if (current->state == data::order::TransportOrder::State::WITHDRAWN) {
      // TODO
    } else if (current->state == data::order::TransportOrder::State::FINISHED) {
      // TODO
    } else if (current->state == data::order::TransportOrder::State::FAILED) {
      // TODO
    } else if (current->state ==
               data::order::TransportOrder::State::UNROUTABLE) {
      // TODO
    } else {
    }
  }
}
void Dispatcher::brake_deadlock(
    std::vector<std::vector<std::shared_ptr<driver::Vehicle>>>) {
  // TODO
}

void Dispatcher::run() {
  dispatch_th = std::thread([&] {
    LOG(INFO) << this->name << " run....";
    while (true) {
      dispatch_once();
      brake_deadlock(deadlock_loop());
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
  });
}

}  // namespace dispatch
}  // namespace kernel