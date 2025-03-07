#include "../../../include/kernel/dispatch/dispatch.hpp"

#include <utility>

#include "../../../include/3rdparty/uuid/uuid.hpp"
#include "../../../include/component/data/model/alleyway.hpp"
#include "../../../include/component/util/tools.hpp"
#include "../../../include/kernel/driver/vehicle.hpp"
#include "../../../include/kernel/planner/planner.hpp"
#include "../../../include/main/rss.hpp"
const float kLen = 2;
const int search_radius = 20000;  // mm
namespace kernel::dispatch {
VehPtr Dispatcher::select_vehicle(const allocate::PointPtr &start) {
  if (vehicles.empty()) {
    return nullptr;
  }
  std::vector<VehPtr> idle_temp;
  std::vector<VehPtr> busy_temp;
  std::vector<VehPtr> charge_temp;

  for (auto &v : vehicles) {
    // if (v->state == driver::Vehicle::State::IDLE &&
    //     v->proc_state == driver::Vehicle::ProcState::AWAITING_ORDER) {
    if (v->state == driver::Vehicle::State::IDLE &&
        v->avoid_state == driver::Vehicle::Avoid::Normal &&
        v->orderpool.lock()->idel_orderpool(v->name)) {
      if (!v->paused) {
        idle_temp.emplace_back(v);
      }
    } else if (v->state == driver::Vehicle::State::EXECUTING) {
      busy_temp.emplace_back(v);
    } else if (v->state == driver::Vehicle::State::CHARGING) {
      charge_temp.emplace_back(v);
    }
  }
  if (idle_temp.empty()) {
    return nullptr;
    if (busy_temp.empty()) {
      if (charge_temp.empty()) {
        return nullptr;
      } else {
        auto planner = charge_temp.front()->planner.lock();
        std::sort(
            charge_temp.begin(), charge_temp.end(),
            [=](const VehPtr &a, const VehPtr &b) {
              auto a_len =
                  planner->find_paths_with_vertex(start, a->current_point)[0]
                      .second;
              auto b_len =
                  planner->find_paths_with_vertex(start, b->current_point)[0]
                      .second;
              return a_len <= b_len;
            });
        return charge_temp.front();
      }
    } else {
      auto planner = busy_temp.front()->planner.lock();
      std::sort(
          busy_temp.begin(), busy_temp.end(),
          [=](const VehPtr &a, const VehPtr &b) {
            auto a_len =
                planner->find_paths_with_vertex(start, a->current_point)[0]
                    .second;
            auto b_len =
                planner->find_paths_with_vertex(start, b->current_point)[0]
                    .second;
            return a_len <= b_len;
          });
      return busy_temp.front();
    }
  } else {
    auto planner = idle_temp.front()->planner.lock();

    std::sort(idle_temp.begin(), idle_temp.end(),
              [=](const VehPtr &a, const VehPtr &b) {
                auto a_len =
                    planner->find_paths_with_vertex(start, a->current_point)[0]
                        .second;
                auto b_len =
                    planner->find_paths_with_vertex(start, b->current_point)[0]
                        .second;
                // LOG(INFO) << a->name << "{" << a_len << "} " << b->name <<
                // "{"
                //           << b_len;
                return a_len <= b_len;
              });
    return idle_temp.front();
  }
}

std::vector<VehPtr> Dispatcher::deadlock_loop() {
  std::vector<VehPtr> res;
  // 死锁检测
  for (auto &v : vehicles) {
    std::stack<VehPtr> vs;
    std::vector<VehPtr> exist;
    vs.push(v);
    while (!vs.empty()) {
      auto &x = vs.top();
      vs.pop();
      if (auto it = std::find(exist.begin(), exist.end(), x);
          it != exist.end()) {
        return {it, exist.end()};
      }
      exist.push_back(x);

      auto depens = find_depends(x);
      if (!depens.empty()) {
        for (auto &x_dep : depens) {
          vs.push(x_dep);
        }
      } else {
        exist.pop_back();
      }
    }
  }
  return res;
}

std::set<VehPtr> Dispatcher::find_depends(const VehPtr &vs) {
  std::set<VehPtr> res;
  std::shared_lock<std::shared_mutex> lock(vs->res_mut);
  for (auto &f : vs->future_allocate_resources) {
    if (!f) {
      continue;
    }
    if (auto f_veh = f->owner.lock()) {
      auto veh = std::dynamic_pointer_cast<driver::Vehicle>(f_veh);
      res.insert(veh);
    }
  }
  return res;
}

std::vector<VehPtr> Dispatcher::block_loop() {
  std::vector<VehPtr> res;
  // 阻挡检测
  for (auto &v : vehicles) {
    res.push_back(v);
    std::shared_lock<std::shared_mutex> lock(v->res_mut);
    for (auto &x : v->future_allocate_resources) {
      if (auto f_veh = x->owner.lock()) {
        if (const auto veh = std::dynamic_pointer_cast<driver::Vehicle>(f_veh);
            veh->state != driver::Vehicle::State::EXECUTING) {
          res.push_back(veh);
          return res;
        }
      }
    }
    res.pop_back();
  }
  return res;
}

void Dispatcher::stop() {
  dispose = true;
  notify();
}

Dispatcher::~Dispatcher() {
  stop();
  if (dispatch_th.joinable()) {
    dispatch_th.join();
  }
  CLOG(INFO, dispatch_log) << name << " close\n";
}

void Dispatcher::idle_detect() {
  auto now = get_now_utc_time();
  for (auto &v : vehicles) {
    if (v->state == driver::Vehicle::State::UNKNOWN) {
      continue;
    }
    if (v->process_charging) {
      continue;
    }
    //  充电
    if (v->energy_level <= v->energy_level_critical) {
      if (v->current_order) {
        v->current_order->state = data::order::TransportOrder::State::WITHDRAWL;
      }
      v->cancel_all_order();
      v->state = driver::Vehicle::State::ERROR;
    } else if (v->energy_level <= v->energy_level_recharge) {
      if (v->current_order) {
        v->current_order->state = data::order::TransportOrder::State::WITHDRAWL;
      }
      v->cancel_all_order();
      go_charge("TOder_" + uuids::to_string(get_uuid()) + "_" + v->name +
                    "_goto_charge",
                v);
    } else if (v->energy_level <= v->energy_level_good) {
      if (v->state == driver::Vehicle::State::IDLE) {
        go_charge("TOder_" + uuids::to_string(get_uuid()) + "_" + v->name +
                      "_goto_charge",
                  v);
      }
    } else {
      //
      if (v->state == driver::Vehicle::State::IDLE) {
        auto dt = now - v->idle_time;
        auto dt_s = std::chrono::duration_cast<std::chrono::seconds>(dt);
        if (dt_s.count() > 15) {
          if (v->park_point && v->park_point != v->last_point) {
            go_home("TOder_" + uuids::to_string(get_uuid()) + "_" + v->name +
                        "_goto_park",
                    v);
          } else if (v->last_point->type !=
                     data::model::Point::Type::PARK_POSITION) {
            if (auto x = get_park_point(v->last_point)) {
              go_home("TOder_" + uuids::to_string(get_uuid()) + "_" + v->name +
                          "_goto_park",
                      v);
            }
          }
        }
      }
    }
  }
}

void Dispatcher::dispatch_once() {
  // for (auto &v : vehicles) {
  //   if (v && v->current_order) {
  //     if (v->current_order->state ==
  //         data::order::TransportOrder::State::WITHDRAWL) {
  //       v->command_done();
  //     }
  //   }
  // }
  bool exist = false;
  for (auto &x : vehicles) {
    if (x->state == driver::Vehicle::State::IDLE &&
        x->avoid_state == driver::Vehicle::Avoid::Normal &&
        x->orderpool.lock()->idel_orderpool(x->name)) {
      exist = true;
      break;
    }
  }
  std::shared_ptr<data::order::TransportOrder> current_ord;
  if (exist && !random_list_empty()) {
    current_ord = get_next_random_ord().second;
  } else {
    current_ord = get_next_ord().second;
  }
  if (!current_ord) {
    return;
  }
  std::unique_lock<std::shared_mutex> lock(current_ord->mutex);
  if (current_ord->state == data::order::TransportOrder::State::RAW) {
    // 重写
    if (current_ord->driverorders.empty()) {
      // 空的
      current_ord->state = data::order::TransportOrder::State::UNROUTABLE;
    }
    auto v = current_ord->intended_vehicle.lock();
    if (!v) {
      current_ord->switch_veh = true;
      if (auto_select) {
        int index_driver = current_ord->current_driver_index;
        if (index_driver > current_ord->driverorders.size()) {
          // 不存在
          throw std::runtime_error("index_driver > driverorders.size()");
        }
        auto dest = current_ord->driverorders[current_ord->current_driver_index]
                        ->destination->destination.lock();
        if (!find_res) {
          current_ord->state = data::order::TransportOrder::State::UNROUTABLE;
          return;
        }
        auto dest_check = find_res(dest->name);
        allocate::PointPtr start;
        if (dest_check.first == allocate::ResourceManager::ResType::Point) {
          start =
              std::dynamic_pointer_cast<data::model::Point>(dest_check.second);
        } else if (dest_check.first ==
                   allocate::ResourceManager::ResType::Location) {
          start = std::dynamic_pointer_cast<data::model::Location>(
                      dest_check.second)
                      ->link.lock();
        }
        auto v_select = select_vehicle(start);
        if (!v_select) {
          // CLOG(WARNING, dispatch_log)
          //     << current_ord->name << " not exist idle vehicle \n";
        } else {
          current_ord->intended_vehicle = v_select;
          pathc_order(current_ord);
        }
      }
    }
    if (!current_ord->intended_vehicle.lock()) {
      // CLOG(WARNING, dispatch_log)
      //     << current_ord->name << " not exist intended_vehicle \n";
      return;
    }
    current_ord->state = data::order::TransportOrder::State::ACTIVE;
    CLOG(INFO, dispatch_log) << current_ord->name << " status: [active]\n";
  } else if (current_ord->state == data::order::TransportOrder::State::ACTIVE) {
    for (auto &x : current_ord->dependencies) {
      if (const auto dep = x.lock()) {
        if (dep->state != data::order::TransportOrder::State::FINISHED) {
          return;
        }
      }
    }
    auto su = current_ord->intended_vehicle.lock()->state;
    if (su == driver::Vehicle::State::ERROR) {
      CLOG(ERROR, driver_log)
          << current_ord->intended_vehicle.lock()->name << " ERROR.\n";
      current_ord->state = data::order::TransportOrder::State::FAILED;
      // pop_order(current_ord);
    } else if (su != driver::Vehicle::State::IDLE) {
      CLOG(INFO, "dispatch_log") << current_ord->name << " status: [busy]\n";
      return;
    } else {
      // 规划路径
      if (current_ord->intended_vehicle.lock()->plan_route(current_ord)) {
        auto conflict_state = conflict_pool->get_state(current_ord);
        if (conflict_state == Conflict::State::Err) {
          // TODO 直接失败还是继续处理？
          CLOG(ERROR, dispatch_log)
              << current_ord->name << " status: [conflict]\n";
          current_ord->state = data::order::TransportOrder::State::FAILED;
        }

        else if (conflict_state == Conflict::State::END) {
          current_ord->state = data::order::TransportOrder::State::DISPATCHABLE;
          CLOG(INFO, dispatch_log)
              << current_ord->name << " status: [dispatchable]\n";
        } else {
          conflict_pool->solve(current_ord);
        }

      } else {
        current_ord->state = data::order::TransportOrder::State::UNROUTABLE;
        CLOG(INFO, dispatch_log)
            << current_ord->name << " status: [unroutable]\n";
      }
    }
  } else if (current_ord->state ==
             data::order::TransportOrder::State::WITHDRAWL) {
    // current_ord->state = data::order::TransportOrder::State::FAILED;
    CLOG(WARNING, dispatch_log)
        << current_ord->name << " status: [withdrawl]\n";
  } else if (current_ord->state ==
             data::order::TransportOrder::State::DISPATCHABLE) {
    auto su = current_ord->intended_vehicle.lock()->state;
    if (su == driver::Vehicle::State::IDLE ||
        su == driver::Vehicle::State::CHARGING) {
      if (current_ord->intended_vehicle.lock()->avoid_state ==
              driver::Vehicle::Avoid::Avoiding &&
          !current_ord->anytime_drop) {
        // 不是避让订单
        return;
      }
      current_ord->processing_vehicle = current_ord->intended_vehicle;
      lock.unlock();
      current_ord->intended_vehicle.lock()->receive_task(current_ord);
      // pop_order(current_ord);
    } else if (su == driver::Vehicle::State::EXECUTING) {
      // if (current_ord->intended_vehicle.lock()->current_order->state ==
      //     data::order::TransportOrder::State::WITHDRAWL) {
      //   current_ord->intended_vehicle.lock()->receive_task(current_ord);
      //   current_ord->processing_vehicle = current_ord->intended_vehicle;
      //   pop_order(current_ord);
      // } else {
      // }
    } else if (su == driver::Vehicle::State::UNAVAILABLE) {
      current_ord->state = data::order::TransportOrder::State::UNROUTABLE;
      CLOG(ERROR, driver_log)
          << current_ord->intended_vehicle.lock()->name << " UNROUTABLE.\n";
      // pop_order(current_ord);
    } else if (su == driver::Vehicle::State::ERROR) {
      CLOG(ERROR, driver_log)
          << current_ord->intended_vehicle.lock()->name << " ERROR.\n";
      current_ord->state = data::order::TransportOrder::State::FAILED;
      // pop_order(current_ord);
    } else {
      CLOG(ERROR, driver_log)
          << current_ord->intended_vehicle.lock()->name << " UNKNOWN.\n";
      current_ord->state = data::order::TransportOrder::State::FAILED;
      // pop_order(current_ord);
    }
    // }
  } else if (current_ord->state ==
             data::order::TransportOrder::State::BEING_PROCESSED) {
    // pop_order(current_ord);
    CLOG(INFO, dispatch_log)
        << current_ord->name << " status: [beging_processed]\n";

    // wait  do nothing
  } else if (current_ord->state ==
             data::order::TransportOrder::State::FINISHED) {
    pop_order(current_ord);
    CLOG(WARNING, dispatch_log) << current_ord->name << " status: [finished]\n";
  } else if (current_ord->state == data::order::TransportOrder::State::FAILED) {
    pop_order(current_ord);
    CLOG(WARNING, dispatch_log) << current_ord->name << " status: [failed]\n";
  } else if (current_ord->state ==
             data::order::TransportOrder::State::UNROUTABLE) {
    pop_order(current_ord);
    CLOG(WARNING, dispatch_log)
        << current_ord->name << " status: [unroutable]\n";
  }
}
void Dispatcher::brake_deadlock(const std::vector<VehPtr> &d_loop) {
  // TODO
  if (!d_loop.empty()) {
    if (!d_loop.front()->current_order) {
      if (d_loop.back()->current_order) {
        auto ord = d_loop.back()->redistribute_cur_order();
      } else {
        auto ord = d_loop.front()->redistribute_cur_order();
      }
    } else if (!d_loop.back()->current_order) {
      if (d_loop.front()->current_order) {
        auto ord = d_loop.front()->redistribute_cur_order();
      } else {
        auto ord = d_loop.back()->redistribute_cur_order();
      }
    } else if (d_loop.front()->current_order->priority >
               d_loop.back()->current_order->priority) {
      auto ord = d_loop.back()->redistribute_cur_order();

    } else if (d_loop.front()->current_order->create_time >
               d_loop.back()->current_order->create_time) {
      auto ord = d_loop.front()->redistribute_cur_order();
    } else {
      auto ord = d_loop.back()->redistribute_cur_order();
    }
  }
}
void Dispatcher::brake_blocklock(const std::vector<VehPtr> &d_loop) {
  // TODO
  if (!d_loop.empty()) {
    // if (d_loop.back()->avoid_state == driver::Vehicle::Avoid::Avoiding &&
    //     d_loop.back()->state == driver::Vehicle::State::IDLE) {
    //   // d_loop.back()->cancel_all_order();
    //   d_loop.back()->avoid_state = driver::Vehicle::Avoid::Normal;
    //   // d_loop.back()->redistribute_cur_order();
    //   d_loop.front()->redistribute_cur_order();
    // }
    // if (!d_loop.front()->current_order) {
    //   if (d_loop.back()->current_order) {
    //     d_loop.back()->redistribute_cur_order();
    //   }
    // } else if (!d_loop.back()->current_order) {
    //   if (d_loop.front()->current_order) {
    //     d_loop.front()->redistribute_cur_order();
    //   }
    // } else if (d_loop.front()->current_order->priority >
    //            d_loop.back()->current_order->priority) {
    //   d_loop.back()->redistribute_cur_order();

    // } else if (d_loop.front()->current_order->create_time >
    //            d_loop.back()->current_order->create_time) {
    //   d_loop.front()->redistribute_cur_order();
    // } else {
    //   d_loop.back()->redistribute_cur_order();
    // }
  }
}
void Dispatcher::run() {
  dispatch_th = std::thread([&] {
    CLOG(INFO, dispatch_log) << this->name << " run....\n";
    // int con = 0;
    while (!dispose) {
      // std::unique_lock<std::mutex> lock(mut);
      // cv.wait_for(lock, std::chrono::milliseconds(10),
      //             [&] { return !dispose; });
      if (dispose) {
        break;
      }
      // idle_detect();
      auto deadloop = deadlock_loop();
      if (!deadloop.empty()) {
        std::stringstream ss;
        ss << "[";
        for (auto &v : deadloop) {
          if (v == deadloop.back()) {
            ss << v->name;
            break;
          }
          ss << v->name << " ,";
        }
        ss << "]\n";
        CLOG_EVERY_N(50, ERROR, dispatch_log) << "deadlock --> " << ss.str();
        brake_deadlock(deadloop);
      }
      // auto blockloop = block_loop();
      // if (!blockloop.empty()) {
      //   assert(blockloop.size() == 2);
      //   CLOG_EVERY_N(200, ERROR, dispatch_log)
      //       << "blocklock --> " << blockloop.front()->name << " blocked by "
      //       << blockloop.back()->name << "\n";
      //   brake_blocklock(blockloop);
      // }
      dispatch_once();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });
}
int Conflict::update() {
  assert(state == State::Raw);
  std::stack<VehPtr>().swap(vehicles);
  graph.clear();
  rm_depends.clear();
  dispthed.clear();
  auto veh = order->intended_vehicle.lock();
  if (!veh) {
    state = State::Err;
    return -1;
  }
  int index_driver = order->current_driver_index;
  if (index_driver > order->driverorders.size()) {
    // 不存在
    throw std::runtime_error("index_driver > driverorders.size()");
  }
  auto obj_ = order->driverorders[order->current_driver_index]
                  ->destination->destination.lock();
  auto obj_rss = resource_manager.lock()->find(obj_->name);
  if (obj_rss.first == allocate::ResourceManager::ResType::Point) {
    this->obj = std::dynamic_pointer_cast<data::model::Point>(obj_rss.second);
  } else if (obj_rss.first == allocate::ResourceManager::ResType::Location) {
    this->obj = std::dynamic_pointer_cast<data::model::Location>(obj_rss.second)
                    ->link.lock();
  } else {
    CLOG(ERROR, dispatch_log) << "obj not found\n";
    state = State::Err;
  }
  path = planner.lock()->find_paths(veh->current_point, obj).front();
  int avoiding = 0;
  for (int i = 1; i < path.size(); i++) {
    auto owner = path[i]->owner.lock();
    if (owner) {
      auto veh2 = std::dynamic_pointer_cast<driver::Vehicle>(owner);
      if (veh2 == veh) continue;
      if (veh2->state == driver::Vehicle::State::IDLE) {
        vehicles.push(veh2);
        rm_depends.insert(veh2);
        // if (veh2->avoid_state == driver::Vehicle::Avoid::Avoiding) {
        //   avoiding++;
        // } else {
        //   vehicles.push(veh2);
        //   rm_depends.insert(veh2);
        // }
      } else if (veh2->state == driver::Vehicle::State::ERROR) {
        LOG(ERROR) << "path of vehicle error\n";
        state = State::Err;
        return -1;
      }
    }
  }
  return avoiding;
}
struct Cmp {
  // 大顶堆
  bool operator()(std::pair<allocate::PointPtr, float> a,
                  std::pair<allocate::PointPtr, float> b) {
    return a.second < b.second;
  }
};
void Conflict::solve_once() {
  auto lock = dispatcher.lock();
  if (!lock) {
    CLOG(ERROR, dispatch_log) << "dispatcher not found\n";
    state = State::Err;
    return;
  }
  if (state == State::Raw) {
    int avoiding = update();
    if (avoiding < 0) {
      state = State::Err;
      return;
    } else if (avoiding > 0) {
      return;
    }
    if (vehicles.empty()) {
      state = State::Solved;
    } else {
      bool main_veh_move{false};
      while (!vehicles.empty() && !main_veh_move) {
        auto top = vehicles.top();
        std::vector<allocate::PointPtr> rm_ps;
        if (order->intended_vehicle.lock()->avoid_state ==
            driver::Vehicle::Avoid::Avoiding) {
          rm_ps = order->intended_vehicle.lock()->avoid_points;
        }
        auto allocate_point = select_point(top, rm_ps);
        if (allocate_point == nullptr) {
          CLOG(ERROR, dispatch_log) << "no point for " << top->name << "\n";
          state = State::Err;
          return;
        } else {
          // 有可选分配点
          graph.push_back(std::make_pair(top, allocate_point));
          vehicles.pop();
          // TODO 选中点是否有车辆占用
          //  检查冲突
          auto cur_path =
              planner.lock()->find_paths(top->current_point, allocate_point);
          if (cur_path.empty()) {
            CLOG(ERROR, dispatch_log) << "no path for " << top->name << "\n";
            state = State::Err;
            return;
          }
          for (int i = 1; i < cur_path.front().size(); i++) {
            auto owner = cur_path.front()[i]->owner.lock();
            if (owner) {
              auto veh = std::dynamic_pointer_cast<driver::Vehicle>(owner);
              if (veh == order->intended_vehicle.lock()) {
                // 如果是主车辆
                main_veh_move = true;
                break;
              }
              if (veh->state == driver::Vehicle::State::IDLE && top != veh) {
                if (veh->avoid_state == driver::Vehicle::Avoid::Avoiding) {
                  if (rm_depends.find(veh) != rm_depends.end()) {
                    // 如果是队列里的车辆，不管
                  } else {
                    state = State::Raw;
                    return;
                  }
                } else {
                  if (rm_depends.find(veh) != rm_depends.end()) {
                    // 如果是队列里的车辆，不管
                  } else {
                    // 新冲突车辆
                    vehicles.push(veh);
                    rm_depends.insert(veh);
                  }
                }
              }
            }
          }
        }
      }
      if (main_veh_move) {
        state = State::SelfMove;
      } else {
        // 分完后判断是否冲突
        bool conflict = false;
        int try_count = 0;
        while (true) {
          //
          for (int i = 0; i < graph.size(); i++) {
            for (int j = i + 1; j < graph.size(); j++) {
              if (one_of_other(graph[i], graph[j])) {
                // 有冲突
                // 交换
                auto temp = graph[i].second;
                graph[i].second = graph[j].second;
                graph[j].second = temp;
                goto sovle_again;
              }
            }
          }
          //
          break;
        sovle_again:
          try_count++;
          if (try_count > 100) {
            conflict = true;
            break;
          }
        }
        if (!conflict) {
          // 无冲突
          // 推挤
          // std::set<std::shared_ptr<data::model::Alleyway>> visited;
          // for (auto &g : graph) {
          //   auto alley =
          //   resource_manager.lock()->get_alleyway(g.second).first; if
          //   (visited.find(alley) == visited.end()) {
          //     visited.insert(alley);
          //     int max_index = alley->size();
          //     int cur_index = max_index - 1;
          //     int point_index = 0;
          //     int veh_index = 0;
          //     for (auto it = alley->vertices.rbegin();
          //          it != alley->vertices.rend(); it++) {
          //       auto owner = (*it)->owner.lock();
          //       if (owner) {
          //         for (auto &v_ : graph) {
          //           if (v_.first == owner) {
          //             continue;
          //           }
          //         }
          //         auto veh =
          //         std::dynamic_pointer_cast<driver::Vehicle>(owner); if
          //         (veh->state == driver::Vehicle::State::IDLE &&
          //             veh->avoid_state == driver::Vehicle::Avoid::Normal) {
          //           bool flg = false;
          //           if (veh_index != point_index) {
          //             int v_index = 0;
          //             for (auto &exist : graph) {
          //               if (exist.first == veh) {
          //                 flg = true;
          //                 break;
          //               }
          //               v_index++;
          //             }
          //             if (!flg) {
          //               graph.push_back(std::make_pair(
          //                   veh, *(alley->vertices.rbegin() + veh_index)));
          //             } else {
          //               graph[v_index].second =
          //                   *(alley->vertices.rbegin() + veh_index);
          //             }
          //           }
          //           veh_index++;
          //         }
          //       }
          //       point_index++;
          //     }
          //   }
          // }
          state = State::Solved;
          // 车辆设置为避让状态
          std::vector<allocate::PointPtr> rm_ps;
          if (order->intended_vehicle.lock()->avoid_state ==
              driver::Vehicle::Avoid::Avoiding) {
            rm_ps = order->intended_vehicle.lock()->avoid_points;
          }
          rm_ps.insert(rm_ps.end(), path.begin(), path.end());
          for (auto &x : graph) {
            x.first->avoid_state = driver::Vehicle::Avoid::Avoiding;
            x.first->avoid_points.clear();
            x.first->avoid_points = path;
          }
        } else {
          CLOG(ERROR, dispatch_log) << "conflict\n";
          state = State::Err;
        }
      }
    }
  } else if (state == State::Solved) {
    state = State::Dispatching;
    dispthed.clear();
  } else if (state == State::Dispatching) {
    if (graph.empty()) {
      state = State::Dispatched;
    }
    while (!graph.empty()) {
      auto node = graph.back();
      auto veh = node.first;
      auto point = node.second;
      auto path = planner.lock()->find_paths(veh->current_point, point).front();
      // 判断是否有交换冲突
      for (auto &v : dispatcher.lock()->vehicles) {
        if (v == veh) {
          continue;
        }
        if (v->state == driver::Vehicle::State::EXECUTING) {
          if (is_swap_conflict(path, v)) {
            return;
          }
        }
      }
      // 无冲突
      auto new_ord = std::make_shared<data::order::TransportOrder>(
          "AOrder-" + uuids::to_string(get_uuid()));
      new_ord->create_time = get_now_utc_time();
      new_ord->dead_time = new_ord->create_time + std::chrono::minutes(100);
      int pro = 3;
      if (veh->current_order) {
        pro = veh->current_order->priority + 1;
      }
      new_ord->priority = pro;
      new_ord->intended_vehicle = veh;
      new_ord->anytime_drop = true;
      new_ord->type = "MOVE";
      new_ord->state = data::order::TransportOrder::State::RAW;
      auto driver_order =
          std::make_shared<data::order::DriverOrder>("DOder_" + point->name);
      auto dest = orderpool.lock()->res_to_destination(
          point, data::model::Actions::OpType::MOVE);
      driver_order->destination = dest;
      driver_order->transport_order = new_ord;
      new_ord->driverorders.push_back(driver_order);
      std::unique_lock<std::shared_mutex> lock(orderpool.lock()->mut);
      orderpool.lock()->push(new_ord);
      dispatcher.lock()->notify();
      dispthed.insert({veh, point});
      graph.pop_back();
    }
  } else if (state == State::Dispatched) {
    auto veh = order->intended_vehicle.lock();
    auto path = planner.lock()->find_paths(veh->current_point, obj).front();

    for (auto &x : path) {
      auto owner = x->owner.lock();
      if (owner && owner != veh) {
        auto cf_veh = std::dynamic_pointer_cast<driver::Vehicle>(owner);
        if (cf_veh->avoid_state == driver::Vehicle::Avoid::Avoiding &&
            cf_veh->state == driver::Vehicle::State::IDLE) {
          return;
        }
        // if (dispthed.find(cf_veh) == dispthed.end()) {
        //   if (cf_veh->state == driver::Vehicle::State::IDLE) {
        //     if (cf_veh->avoid_state == driver::Vehicle::Avoid::Avoiding) {
        //       return;
        //     } else {
        //       state = State::Raw;
        //       return;
        //     }
        //   }
        // }
      }
    }
    // 判断是否有交换冲突
    for (auto &v : dispatcher.lock()->vehicles) {
      if (v == veh) {
        continue;
      }
      if (v->state == driver::Vehicle::State::EXECUTING) {
        if (is_swap_conflict(path, v)) {
          // state = State::Raw;
          return;
        }
      }
    }
    // 无冲突
    state = State::END;
  } else if (state == State::SelfMove) {
    // 主车辆移动
    graph.clear();
    std::vector<allocate::PointPtr> rm_ps;
    if (order->intended_vehicle.lock()->avoid_state ==
        driver::Vehicle::Avoid::Avoiding) {
      rm_ps = order->intended_vehicle.lock()->avoid_points;
    }
    auto allocate_point =
        select_point(order->intended_vehicle.lock(), rm_ps, true);
    if (allocate_point == nullptr) {
      CLOG(ERROR, dispatch_log)
          << "no point for " << order->intended_vehicle.lock()->name << "\n";
      state = State::Err;
    } else {
      // 有可选分配点
      // order->intended_vehicle.lock()->avoid_state =
      //     driver::Vehicle::Avoid::Avoiding;
      auto path =
          planner.lock()
              ->find_paths(order->intended_vehicle.lock()->current_point,
                           allocate_point)
              .front();
      for (auto &x : path) {
        auto owner = x->owner.lock();
        if (owner && owner != order->intended_vehicle.lock()) {
          auto cf_veh = std::dynamic_pointer_cast<driver::Vehicle>(owner);
          if (dispthed.find(cf_veh) != dispthed.end() &&
              cf_veh->avoid_state == driver::Vehicle::Avoid::Avoiding &&
              cf_veh->state == driver::Vehicle::State::IDLE) {
            // 判断是否有交换冲突
            allocate::PointPtr cf_obj = dispthed[cf_veh];
            auto path_left = planner.lock()
                                 ->find_paths(cf_veh->current_point, cf_obj)
                                 .front();
            auto step_left = resource_manager.lock()->paths_to_route(path_left);
            auto step_right = resource_manager.lock()->paths_to_route(path);
            float cur_cost = 0;  // mm单位
            bool flg = false;
            for (auto &cur_s : step_left->steps) {
              float v_cost = 0;
              for (auto &v_s : step_right->steps) {
                if (cur_s->path == v_s->path &&
                    cur_s->vehicle_orientation != v_s->vehicle_orientation) {
                  if (fabs(cur_cost - v_cost) < kLen * (v_s->path->length)) {
                    flg = true;
                    break;
                  }
                }
                v_cost += v_s->path->length;
              }
              cur_cost += cur_s->path->length;
            }
            if (flg) {
              state = State::Raw;
              return;
            }
          }
        }
      }
      // 判断是否有交换冲突
      for (auto &v : dispatcher.lock()->vehicles) {
        if (v == order->intended_vehicle.lock()) {
          continue;
        }
        if (v->state == driver::Vehicle::State::EXECUTING) {
          if (is_swap_conflict(path, v)) {
            state = State::Raw;
            return;
          }
        }
      }
      // 发单
      order->intended_vehicle.lock()->avoid_state =
          driver::Vehicle::Avoid::Normal;
      auto new_ord = std::make_shared<data::order::TransportOrder>(
          "AOrder-" + uuids::to_string(get_uuid()));
      new_ord->create_time = get_now_utc_time();
      new_ord->dead_time = new_ord->create_time + std::chrono::minutes(100);
      if (order->intended_vehicle.lock()->current_order) {
        new_ord->priority =
            order->intended_vehicle.lock()->current_order->priority + 1;
      } else {
        new_ord->priority = 3;
      }
      new_ord->intended_vehicle = order->intended_vehicle.lock();
      new_ord->type = "MOVE";
      new_ord->anytime_drop = true;
      new_ord->state = data::order::TransportOrder::State::RAW;
      auto driver_order = std::make_shared<data::order::DriverOrder>(
          "DOrder-" + allocate_point->name);
      auto dest = orderpool.lock()->res_to_destination(
          allocate_point, data::model::Actions::OpType::MOVE);
      driver_order->destination = dest;
      driver_order->transport_order = new_ord;
      new_ord->driverorders.push_back(driver_order);
      std::unique_lock<std::shared_mutex> lock(orderpool.lock()->mut);
      orderpool.lock()->push(new_ord);
      dispatcher.lock()->notify();
      state = State::Raw;
    }
  }
}
float Conflict::distance(allocate::PointPtr a, allocate::PointPtr b) {
  auto path = planner.lock()->find_paths_with_vertex(a, b);
  if (path.empty()) {
    return std::numeric_limits<float>::max();
  }
  float len = path.front().second / 1e3;
  return 2 / (1 + exp(-len * 0.04)) - 1;
  // return len;
}
float Conflict::occupancy(std::shared_ptr<data::model::Alleyway> way,
                          VehPtr vehicle) {
  float occupancy = 0;
  std::unordered_set<std::shared_ptr<RSSObject>> objs;
  std::unordered_set<VehPtr> vehs;
  auto this_alley =
      resource_manager.lock()->get_alleyway(vehicle->current_point).first;
  for (auto &x : way->vertices) {
    // bool flg = false;
    // for (auto &s : path) {
    //   if (s == x) {
    //     flg = true;
    //     break;
    //   }
    // }
    // if (flg) continue;
    auto owner = x->owner.lock();
    if (owner) {
      auto veh = std::dynamic_pointer_cast<driver::Vehicle>(owner);
      // if (veh->state == driver::Vehicle::State::IDLE &&
      //     veh->avoid_state == driver::Vehicle::Avoid::Normal) {
      //   objs.insert(owner);
      //   occupancy++;
      // }
      objs.insert(owner);
      occupancy++;

    } else {
      // 检查是否被分配了
      for (auto &p : graph) {
        if (p.second == x) {
          vehs.insert(p.first);
          occupancy++;
          break;
        }
      }
    }
  }
  for (auto &v : vehs) {
    if (objs.find(v) != objs.end()) {
      // 分配到了自己的巷道
      occupancy--;
    }
  }
  auto intent =
      resource_manager.lock()
          ->get_alleyway(order->intended_vehicle.lock()->current_point)
          .first;
  if (this_alley && this_alley == way && intent != this_alley) occupancy--;

  return occupancy / way->size();
}
bool Conflict::one_of_other(std::pair<VehPtr, allocate::PointPtr> a,
                            std::pair<VehPtr, allocate::PointPtr> b) {
  auto path_a =
      planner.lock()->find_paths(a.first->current_point, a.second).front();
  auto path_b =
      planner.lock()->find_paths(b.first->current_point, b.second).front();
  std::vector<std::shared_ptr<data::model::Point>> left;
  std::vector<std::shared_ptr<data::model::Point>> right;
  if (path_a.size() > path_b.size()) {
    left = path_a;
    right = path_b;
  } else {
    left = path_b;
    right = path_a;
  }
  int flag = 0;
  for (auto &x : left) {
    if (x == right.front()) {
      flag++;
    } else if (x == right.back()) {
      flag++;
    }
  }
  if (flag >= 2) {
    return true;
  } else {
    return false;
  }
}
/**
 * @brief 路径p和车辆v是否有冲突
 *
 * @param p
 * @param v
 */
bool Conflict::is_swap_conflict(std::vector<allocate::PointPtr> p, VehPtr v) {
  return false;
  std::shared_lock<std::shared_mutex> lock(v->current_order->mutex);
  if (v->state == driver::Vehicle::State::IDLE) {
    return false;
  }
  if (v->current_order == nullptr) {
    return false;
  }
  auto cur_step = resource_manager.lock()->paths_to_route(p);
  int index_driver = v->current_order->current_driver_index;
  if (index_driver > v->current_order->driverorders.size()) {
    // 不存在
    return false;
  }
  std::deque<std::shared_ptr<data::order::Step>> v_step(
      v->current_order->driverorders[v->current_order->current_driver_index]
          ->route->steps);
  if (v->current_order->driverorders[v->current_order->current_driver_index]
          ->route->current_step)
    v_step.push_front(
        v->current_order->driverorders[v->current_order->current_driver_index]
            ->route->current_step);
  if (v_step.empty()) {
    return false;
  }
  auto last_step = cur_step->steps.back();
  for (auto &x : v_step) {
    if (last_step->path == x->path &&
        last_step->vehicle_orientation == x->vehicle_orientation) {
      return true;
    }
  }
  float cur_cost = 0;  // mm单位
  for (auto &cur_s : cur_step->steps) {
    float v_cost = 0;
    for (auto &v_s : v_step) {
      if (cur_s->path == v_s->path &&
          cur_s->vehicle_orientation != v_s->vehicle_orientation) {
        LOG(ERROR) << cur_s->path->length << "->" << cur_cost << " "
                   << v_s->path->length << "->" << v_cost << "   "
                   << fabs(cur_cost - v_cost) << "  ___   "
                   << kLen * (v_s->path->length);
        if (fabs(cur_cost - v_cost) < kLen * (v_s->path->length)) {
          return true;
        }
      }
      v_cost += v_s->path->length;
    }
    cur_cost += cur_s->path->length;
  }
  return false;
}
void Conflict::swap_obj(std::pair<VehPtr, allocate::PointPtr> a,
                        std::pair<VehPtr, allocate::PointPtr> b) {
  auto a_obj = a.second;
  auto b_obj = b.second;
  for (auto &x : graph) {
    if (x.first == a.first) {
      x.second = b_obj;
    } else if (x.first == b.first) {
      x.second = a_obj;
    }
  }
}
allocate::PointPtr Conflict::select_point(VehPtr v,
                                          std::vector<allocate::PointPtr> rm_ps,
                                          bool rm_self_alley) {
  auto all_idel_ps = resource_manager.lock()->get_all_idel_points(
      v, rm_self_alley);  // 所有空闲点
  // 创建候选队列
  std::priority_queue<std::pair<allocate::PointPtr, float>,
                      std::vector<std::pair<allocate::PointPtr, float>>, Cmp>
      alley_ps;
  std::priority_queue<std::pair<allocate::PointPtr, float>,
                      std::vector<std::pair<allocate::PointPtr, float>>, Cmp>
      normal_ps;
  std::vector<allocate::PointPtr> rms;
  if (path.size() > 1) {
    auto [a1, d1] = resource_manager.lock()->get_alleyway(path[0]);
    auto [a2, d2] = resource_manager.lock()->get_alleyway(path[1]);
    if (a1) {
      if (a2) {
        if (a1 == a2 && d1 > d2) {
          for (int i = d1; i < a1->size(); i++) {
            rms.push_back(a1->vertices[i]);
          }
        }
      } else if (a1->get_common_point() == path[1]) {
        for (int i = d1; i < a1->size(); i++) {
          rms.push_back(a1->vertices[i]);
        }
      }
    }
  }
  for (auto &x : all_idel_ps) {
    bool flag = false;
    for (auto &p : path) {
      if (p == x) {
        flag = true;
        break;
      }
    }
    if (flag) {
      // 在路径上的不能选
      continue;
    }
    flag = false;
    for (auto &p : graph) {
      if (p.second == x) {
        flag = true;
        break;
      }
    }
    if (flag) {
      // 已经被选走的不能选
      continue;
    }
    flag = false;
    for (auto &p : rm_ps) {
      if (p == x) {
        flag = true;
        break;
      }
    }
    if (flag) {
      // 需要避让的不能选
      continue;
    }
    if (x == order->intended_vehicle.lock()->current_point) {
      continue;
    }
    flag = false;
    for (auto &p : rms) {
      if (p == x) {
        flag = true;
        break;
      }
    }
    if (flag) {
      // 如果路径是出巷道，里面的不能选
      continue;
    }
    auto [flg, dep] = resource_manager.lock()->get_alleyway(x);
    float distance = Conflict::distance(v->current_point, x);  // 距离
    if (flg) {
      float occupied = Conflict::occupancy(flg, v);
      if (occupied > kMaxOccupancy) {
        // 车道点占用率过高
        continue;
      }
      float score = 1 / (occupied + distance + 1e-4);
      alley_ps.push(std::make_pair(x, score));
    } else {
      float occupied = 1;
      float score = 1 / (occupied + distance + 1e-4);
      alley_ps.push(std::make_pair(x, score));
    }
  }
  allocate::PointPtr allocate_point{nullptr};
  if (!alley_ps.empty()) {
    allocate_point = alley_ps.top().first;
  } else if (!normal_ps.empty()) {
    allocate_point = normal_ps.top().first;
  }
  return allocate_point;
}
Conflict::State ConflictPool::get_state(allocate::TransOrderPtr order) {
  if (conflicts.find(order) == conflicts.end()) {
    //
    auto conflict = std::make_shared<Conflict>();
    conflict->order = order;
    conflict->resource_manager = rss.lock()->resource;
    conflict->planner = rss.lock()->planner;
    conflict->dispatcher = rss.lock()->dispatcher;
    conflict->orderpool = rss.lock()->orderpool;
    conflicts[order] = conflict;
    order->conflict_pool = shared_from_this();
  }
  if (conflicts[order]->state == Conflict::State::END) {
    conflicts.erase(order);
    return Conflict::State::END;
  }
  return conflicts[order]->state;
}
void ConflictPool::solve(allocate::TransOrderPtr order) {
  conflicts[order]->solve_once();
}
void ConflictPool::reset_state(allocate::TransOrderPtr order) {
  if (conflicts.find(order) != conflicts.end()) {
    conflicts.erase(order);
  }
}
}  // namespace kernel::dispatch