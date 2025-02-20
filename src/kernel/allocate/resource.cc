#include "../../../include/kernel/allocate/resource.hpp"

#include "../../../include/component/data/model/alleyway.hpp"
#include "../../../include/kernel/driver/vehicle.hpp"

namespace kernel::allocate {
std::shared_ptr<data::order::Route> ResourceManager::paths_to_route(
    std::vector<PointPtr> ps) {
  std::string route_name = ps.front()->name + "_" + ps.back()->name;
  auto res = std::make_shared<data::order::Route>(route_name);
  int index{0};
  for (auto it = ps.begin(); it != ps.end() - 1; ++it) {
    for (const auto &x : paths) {
      if (x->source_point.lock() == *it &&
          x->destination_point.lock() == *(it + 1)) {
        auto step = std::make_shared<data::order::Step>(x->name);
        step->route_index = index++;
        step->vehicle_orientation = data::order::Step::Orientation::FORWARD;
        step->path = x;
        step->execution_allowed = true;
        res->steps.push_back(step);
        res->costs += x->length;
      } else if (x->source_point.lock() == *(it + 1) &&
                 x->destination_point.lock() == *(it)) {
        auto step = std::make_shared<data::order::Step>(x->name);
        step->route_index = index++;
        step->path = x;
        step->execution_allowed = true;
        step->vehicle_orientation = data::order::Step::Orientation::BACKWARD;
        res->steps.push_back(step);
        res->costs += x->length;
      }
    }
  }
  if (!res->steps.empty()) {
    res->current_step = res->steps.front();
    res->steps.back()->type = data::order::Step::Type::BACK;
    res->steps.front()->type = data::order::Step::Type::FRONT;
  }
  return res;
}

std::pair<ResourceManager::ResType, TCSResourcePtr> ResourceManager::find(
    const std::string &name) {
  TCSResourcePtr res;
  const auto it =
      std::find_if(points.begin(), points.end(),
                   [&](const PointPtr &p) { return p->name == name; });
  if (it != points.end()) {
    res = *it;
    return std::pair<ResType, TCSResourcePtr>({ResType::Point, res});
  }
  const auto it_p =
      std::find_if(paths.begin(), paths.end(),
                   [&](const PathPtr &p) { return p->name == name; });
  if (it_p != paths.end()) {
    res = *it_p;
    return std::pair<ResType, TCSResourcePtr>({ResType::Path, res});
  }
  const auto it_l = std::find_if(
      locations.begin(), locations.end(),
      [&](const LocationPtr &p) { return (p->name == name) && (!p->locked); });
  if (it_l != locations.end()) {
    res = *it_l;
    return std::pair<ResType, TCSResourcePtr>({ResType::Location, res});
  }
  return std::pair<ResType, TCSResourcePtr>({ResType::Err, res});
}

bool ResourceManager::claim(const std::vector<TCSResourcePtr> &res,
                            const ClientPtr &client) {
  std::unique_lock<std::mutex> lock(mut);
  for (auto &r : res) {
    for (auto &p : points) {
      if (static_cast<RSSResource *>(r.get()) == p.get()) {
        p->future_owner.push_back(client);
      }
    }
    for (auto &p : paths) {
      if (static_cast<RSSResource *>(r.get()) == p.get()) {
        p->future_owner.push_back(client);
      }
    }
    for (auto &p : locations) {
      if (static_cast<RSSResource *>(r.get()) == p.get()) {
        p->future_owner.push_back(client);
      }
    }
  }
  client->claim_resources.emplace_back(res.begin(), res.end());
  return true;
}

bool ResourceManager::allocate(std::vector<TCSResourcePtr> res,
                               const ClientPtr &client) {
  std::unique_lock<std::mutex> lock(mut);
  for (auto it = client->allocated_resources.begin();
       it != client->allocated_resources.end();) {
    if (it->empty()) {
      it = client->allocated_resources.erase(it);
    } else {
      ++it;
    }
  }
  for (auto it = client->claim_resources.begin();
       it != client->claim_resources.end();) {
    if (it->empty()) {
      it = client->claim_resources.erase(it);
    } else {
      ++it;
    }
  }
  for (const auto &r : rules) {
    // LOG(INFO) << r->name;
    if (!r->pass(res, client)) {
      CLOG_EVERY_N(200, WARNING, allocate_log)
          << r->name << " not pass of " << client->name << "\n";
      return false;
    }
  }
  std::stringstream ss;
  for (auto &r : res) {
    for (auto &p : points) {
      if (static_cast<RSSResource *>(r.get()) == p.get()) {
        ss << p->name << " ";
        p->owner = client;
        p->envelope_keys.insert(client->envelope_key);
        break;
      }
    }
    for (auto &p : paths) {
      if (static_cast<RSSResource *>(r.get()) == p.get()) {
        ss << p->name << " ";
        p->owner = client;
        p->envelope_keys.insert(client->envelope_key);
        break;
      }
    }
    for (auto &p : locations) {
      if (static_cast<RSSResource *>(r.get()) == p.get()) {
        ss << p->name << " ";
        p->owner = client;
        break;
      }
    }
  }
  client->allocated_resources.emplace_back(res.begin(), res.end());
  for (auto &r : res) {
    for (auto x = client->future_allocate_resources.begin();
         x != client->future_allocate_resources.end();) {
      if (*x == r) {
        x = client->future_allocate_resources.erase(x);
        CLOG(INFO, allocate_log)
            << r->name << " from future_allocate move to allocate of "
            << client->name << "\n";
      } else {
        ++x;
      }
    }
  }

  CLOG_IF(!ss.str().empty(), INFO, allocate_log)
      << client->name << " allocate { " << ss.str() << "}\n";
  return true;
}

bool ResourceManager::free(const std::vector<TCSResourcePtr> &res,
                           const ClientPtr &client) {
  std::unique_lock<std::mutex> lock(mut);
  for (auto &r : res) {
    for (auto &ar : client->allocated_resources) {
      for (auto it = ar.begin(); it != ar.end();) {
        if (r.get() == it->get()) {
          it = ar.erase(it);
          r->owner.reset();
          if (r->envelope_keys.find(client->envelope_key) !=
              r->envelope_keys.end()) {
            r->envelope_keys.erase(client->envelope_key);
          }
        } else {
          ++it;
        }
      }
    }
  }
  for (auto it = client->allocated_resources.begin();
       it != client->allocated_resources.end();) {
    if (it->empty()) {
      it = client->allocated_resources.erase(it);
    } else {
      ++it;
    }
  }
  return true;
}

bool ResourceManager::unclaim(const std::vector<TCSResourcePtr> &res,
                              const ClientPtr &client) {
  std::unique_lock<std::mutex> lock(mut);
  for (auto &r : res) {
    for (auto &cr : client->claim_resources) {
      for (auto it = cr.begin(); it != cr.end();) {
        if (r.get() == it->get()) {
          for (auto x = (*it)->future_owner.begin();
               x != (*it)->future_owner.end();) {
            if (x->lock() == client) {
              x = (*it)->future_owner.erase(x);
            } else {
              ++x;
            }
          }
          it = cr.erase(it);
        } else {
          ++it;
        }
      }
    }
  }
  for (auto x = client->claim_resources.begin();
       x != client->claim_resources.end();) {
    if (x->empty()) {
      x = client->claim_resources.erase(x);
    } else {
      ++x;
    }
  }
  return true;
}
PointPtr ResourceManager::get_recent_park_point(const PointPtr &cur) const {
  // CLOG(INFO, "allocate") << "now" << cur->name << " " << cur->position;
  std::vector<double> dis;
  auto temp_ps = points;
  std::sort(
      temp_ps.begin(), temp_ps.end(),
      [&](const PointPtr &a, const PointPtr &b) {
        const double d1 = (cur->position - a->position).cast<double>().norm();
        const double d2 = (cur->position - b->position).cast<double>().norm();
        return d1 < d2;
      });
  for (auto &x : temp_ps) {
    if (x->type != data::model::Point::Type::PARK_POSITION) {
      continue;
    }
    if (!is_connected(cur, x)) {
      continue;
    }
    if (x->owner.lock()) {
      continue;
    }
    return x;
  }
  return nullptr;
}
LocationPtr ResourceManager::get_recent_charge_loc(const PointPtr &cur) const {
  std::vector<double> dis;
  auto temp_loc = locations;
  std::sort(
      temp_loc.begin(), temp_loc.end(),
      [&](const LocationPtr &a, const LocationPtr &b) {
        const double d1 = (cur->position - a->position).cast<double>().norm();
        const double d2 = ((cur->position - b->position).cast<double>()).norm();
        return d1 < d2;
      });
  for (auto &x : temp_loc) {
    if (x->type.lock()->allowed_ops.find("charge") ==
        x->type.lock()->allowed_ops.end()) {
      continue;
    }
    if (!x->link.lock()) {
      continue;
    }
    if (!is_connected(cur, x->link.lock())) {
      continue;
    }
    if (x->owner.lock()) {
      continue;
    }
    return x;
  }
  return nullptr;
}
std::pair<std::shared_ptr<data::model::Alleyway>, int>
ResourceManager::get_alleyway(PointPtr p) {
  for (auto &x : alleyways) {
    auto [flg, dep] = x->is_in(p);
    if (flg) {
      return {x, dep};
    }
  }
  return {nullptr, -1};
}
std::vector<std::shared_ptr<data::model::Point>>
ResourceManager::get_all_idel_points(std::shared_ptr<driver::Vehicle> v,
                                     bool rm_self_alley) {
  std::set<std::shared_ptr<data::model::Point>> res;
  auto [self_alley, index] = get_alleyway(v->current_point);
  for (auto &x : points) {
    auto [flg, dep] = get_alleyway(x);
    if (rm_self_alley && self_alley == flg) continue;
    if (flg) {
      // 巷道点
      // int occupancy = 0;
      // for (auto &x : flg->vertices) {
      //   auto owner = x->owner.lock();
      //   if (owner) {
      //     auto veh = std::dynamic_pointer_cast<driver::Vehicle>(owner);
      //     if (veh->state == driver::Vehicle::State::IDLE && veh != v) {
      //       occupancy++;
      //     }
      //   }
      // }
      // for (int i = 0; i < flg->size() - occupancy; i++)
      //   res.insert(flg->vertices[i]);
      for (int i = 0; i < flg->size(); i++) res.insert(flg->vertices[i]);
    } else {
      // if (x->owner.lock()) {
      //   auto veh =
      //   std::dynamic_pointer_cast<driver::Vehicle>(x->owner.lock()); if
      //   (veh->state == driver::Vehicle::State::EXECUTING ||
      //       veh->avoid_state == driver::Vehicle::Avoid::Avoiding) {
      //     res.insert(x);
      //   }

      // } else {
      //   res.insert(x);
      // }
      res.insert(x);
    }
  }
  return {res.begin(), res.end()};
}
}  // namespace kernel::allocate