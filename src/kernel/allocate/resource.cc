#include "../../../include/kernel/allocate/resource.hpp"

#include "../../../include/kernel/schedule/schedule.hpp"

namespace kernel {
namespace allocate {
std::shared_ptr<data::order::Route> ResourceManager::paths_to_route(
    std::vector<PointPtr> ps) {
  std::string route_name = ps.front()->name + "_" + ps.back()->name;
  auto res = std::make_shared<data::order::Route>(route_name);
  int index{0};
  for (auto it = ps.begin(); it != ps.end() - 1; it++) {
    for (auto& x : paths) {
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
  }
  return res;
}

std::pair<ResourceManager::ResType, TCSResourcePtr> ResourceManager::find(
    const std::string& name) {
  TCSResourcePtr res;
  auto it = std::find_if(points.begin(), points.end(),
                         [&](const PointPtr& p) { return p->name == name; });
  if (it != points.end()) {
    res = *it;
    return std::pair<ResType, TCSResourcePtr>(ResType::Point, res);
  }
  auto it_p = std::find_if(paths.begin(), paths.end(),
                           [&](const PathPtr& p) { return p->name == name; });
  if (it_p != paths.end()) {
    res = *it_p;
    return std::pair<ResType, TCSResourcePtr>(ResType::Path, res);
  }
  auto it_l = std::find_if(
      locations.begin(), locations.end(),
      [&](const LocationPtr& p) { return (p->name == name) && (!p->locked); });
  if (it_l != locations.end()) {
    res = *it_l;
    return std::pair<ResType, TCSResourcePtr>(ResType::Location, res);
  }
  return std::pair<ResType, TCSResourcePtr>(ResType::Err, res);
}

bool ResourceManager::claim(const std::vector<TCSResourcePtr>& res,
                            const ClientPtr& client) {
  std::unique_lock<std::mutex> lock(mut);
  for (auto& r : res) {
    for (auto& p : points) {
      if (static_cast<RSSResource*>(r.get()) == p.get()) {
        p->future_owner.push_back(client);
      }
    }
    for (auto& p : paths) {
      if (static_cast<RSSResource*>(r.get()) == p.get()) {
        p->future_owner.push_back(client);
      }
    }
    for (auto& p : locations) {
      if (static_cast<RSSResource*>(r.get()) == p.get()) {
        p->future_owner.push_back(client);
      }
    }
  }
  client->claim_resources.push_back(
      std::unordered_set<TCSResourcePtr>{res.begin(), res.end()});
  return true;
}

bool ResourceManager::allocate(std::vector<TCSResourcePtr> res,
                               ClientPtr client) {
  std::unique_lock<std::mutex> lock(mut);
  for (auto& r : rules) {
    // LOG(INFO) << r->name;
    if (!r->pass(res, client)) {
      CLOG_N_TIMES(5, WARNING, allocate_log)
          << r->name << " not pass of " << client->name;
      return false;
    }
  }
  std::stringstream ss;
  for (auto& r : res) {
    for (auto& p : points) {
      if (static_cast<RSSResource*>(r.get()) == p.get()) {
        ss << p->name << " ";
        p->owner = client;
        p->envelope_keys.insert(client->envelope_key);
        break;
      }
    }
    for (auto& p : paths) {
      if (static_cast<RSSResource*>(r.get()) == p.get()) {
        ss << p->name << " ";
        p->owner = client;
        p->envelope_keys.insert(client->envelope_key);
        break;
      }
    }
    for (auto& p : locations) {
      if (static_cast<RSSResource*>(r.get()) == p.get()) {
        ss << p->name << " ";
        p->owner = client;
        break;
      }
    }
  }
  client->allocated_resources.push_back(
      std::unordered_set<TCSResourcePtr>{res.begin(), res.end()});
  for (auto& r : res) {
    for (auto x = client->future_allocate_resources.begin();
         x != client->future_allocate_resources.end();) {
      if (*x == r) {
        x = client->future_allocate_resources.erase(x);
        CLOG(INFO, allocate_log)
            << r->name << " from future_allocate move to allocate of "
            << client->name << "\n";
      } else {
        x++;
      }
    }
  }
  CLOG_IF(!ss.str().empty(), INFO, allocate_log)
      << client->name << " allocate " << ss.str() << "\n";
  return true;
}

bool ResourceManager::free(const std::vector<TCSResourcePtr>& res,
                           const ClientPtr& client) {
  std::unique_lock<std::mutex> lock(mut);
  for (auto& r : res) {
    for (auto& ar : client->allocated_resources) {
      for (auto it = ar.begin(); it != ar.end();) {
        if (r.get() == (*it).get()) {
          it = ar.erase(it);
          r->owner.reset();
          if (r->envelope_keys.find(client->envelope_key) !=
              r->envelope_keys.end()) {
            r->envelope_keys.erase(client->envelope_key);
          }
        } else {
          it++;
        }
      }
    }
    for (auto it = client->allocated_resources.begin();
         it != client->allocated_resources.end();) {
      if (it->empty()) {
        it = client->allocated_resources.erase(it);
      } else {
        it++;
      }
    }
  }
  return true;
}

bool ResourceManager::unclaim(const std::vector<TCSResourcePtr>& res,
                              const ClientPtr& client) {
  std::unique_lock<std::mutex> lock(mut);
  for (auto& r : res) {
    for (auto& cr : client->claim_resources) {
      for (auto it = cr.begin(); it != cr.end();) {
        if (r.get() == (*it).get()) {
          for (auto x = (*it)->future_owner.begin();
               x != (*it)->future_owner.end();) {
            if ((*x).lock() == client) {
              x = (*it)->future_owner.erase(x);
            } else {
              x++;
            }
          }
          it = cr.erase(it);
        } else {
          it++;
        }
      }
    }
  }
  for (auto x = client->claim_resources.begin();
       x != client->claim_resources.end();) {
    if (x->empty()) {
      x = client->claim_resources.erase(x);
    } else {
      x++;
    }
  }
  return true;
}
PointPtr ResourceManager::get_recent_park_point(PointPtr cur) {
  // CLOG(INFO, "allocate") << "now" << cur->name << " " << cur->position;
  std::vector<double> dis;
  std::sort(points.begin(), points.end(),
            [&](const PointPtr& a, const PointPtr& b) {
              double d1 = std::numeric_limits<double>::max();
              double d2 = std::numeric_limits<double>::max();
              d1 = (cur->position - a->position).cast<double>().norm();
              d2 = (cur->position - b->position).cast<double>().norm();
              return d1 < d2;
            });
  for (int i = 0; i < points.size(); i++) {
    if (points.at(i)->type != data::model::Point::Type::PARK_POSITION) {
      continue;
    }
    if (!is_connected(cur, points.at(i))) {
      continue;
    }
    if (points.at(i)->owner.lock()) {
      continue;
    }
    return points.at(i);
  }
  return nullptr;
}
LocationPtr ResourceManager::get_recent_charge_loc(PointPtr cur) {
  std::vector<double> dis;
  std::sort(locations.begin(), locations.end(),
            [&](const LocationPtr& a, const LocationPtr& b) {
              double d1 = std::numeric_limits<double>::max();
              double d2 = std::numeric_limits<double>::max();
              d1 = (cur->position - a->position).cast<double>().norm();
              d2 = ((cur->position - b->position).cast<double>()).norm();
              return d1 < d2;
            });
  for (int i = 0; i < locations.size(); i++) {
    if (locations.at(i)->type.lock()->allowed_ops.find("Charge") ==
        locations.at(i)->type.lock()->allowed_ops.end()) {
      continue;
    }
    if (!locations.at(i)->link.lock()) {
      continue;
    }
    if (!is_connected(cur, locations.at(i)->link.lock())) {
      continue;
    }
    if (locations.at(i)->owner.lock()) {
      continue;
    }
    return locations.at(i);
  }
  return nullptr;
}

}  // namespace allocate
}  // namespace kernel
