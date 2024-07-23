#include "../../../include/kernel/allocate/resource.hpp"

#include "../../../include/kernel/schedule/schedule.hpp"

namespace kernel {
namespace allocate {
std::shared_ptr<data::order::Route> ResourceManager::paths_to_route(
    std::vector<std::shared_ptr<data::model::Point>> ps) {
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
    res->current_steps.push_back(res->steps.front());
  }
  return res;
}

std::pair<ResourceManager::ResType, std::shared_ptr<TCSResource>>
ResourceManager::find(const std::string& name) {
  std::shared_ptr<TCSResource> res;
  for (auto& p : points) {
    if (p->name == name) {
      res = p;
      return std::pair<ResType, std::shared_ptr<TCSResource>>(ResType::Point,
                                                              res);
    }
  }
  for (auto& p : paths) {
    if (p->name == name) {
      res = p;
      return std::pair<ResType, std::shared_ptr<TCSResource>>(ResType::Path,
                                                              res);
    }
  }
  for (auto& l : locations) {
    if (l->name == name) {
      if (l->locked) {
        break;
      }
      res = l;
      return std::pair<ResType, std::shared_ptr<TCSResource>>(ResType::Location,
                                                              res);
    }
  }
  return std::pair<ResType, std::shared_ptr<TCSResource>>(ResType::Err, res);
}

bool ResourceManager::claim(
    const std::vector<std::shared_ptr<TCSResource>>& res,
    const std::shared_ptr<schedule::Client>& client) {
  std::unique_lock<std::mutex> lock(mut);
  for (auto& r : res) {
    for (auto& p : points) {
      if (static_cast<TCSResource*>(r.get()) == p.get()) {
        client->claim_resources.insert(p);
        p->future_owner.push_back(client);
      }
    }
    for (auto& p : paths) {
      if (static_cast<TCSResource*>(r.get()) == p.get()) {
        client->claim_resources.insert(p);
        p->future_owner.push_back(client);
      }
    }
    for (auto& p : locations) {
      if (static_cast<TCSResource*>(r.get()) == p.get()) {
        client->claim_resources.insert(p);
        p->future_owner.push_back(client);
      }
    }
  }
  return true;
}

bool ResourceManager::allocate(std::vector<std::shared_ptr<TCSResource>> res,
                               std::shared_ptr<schedule::Client> client) {
  std::unique_lock<std::mutex> lock(mut);
  for (auto& r : rules) {
    // LOG(INFO) << r->name;
    if (!r->pass(res, client)) {
      DCLOG(WARNING, allocate_log)
          << r->name << " not pass of " << client->name;
      return false;
    }
  }
  std::stringstream ss;
  ss << client->name << " allocate ";
  for (auto& r : res) {
    for (auto& p : points) {
      if (static_cast<TCSResource*>(r.get()) == p.get()) {
        ss << p->name << " ";
        p->owner = client;
        p->envelope_keys.insert(client->envelope_key);
        client->allocated_resources.insert(p);
        break;
      }
    }
    for (auto& p : paths) {
      if (static_cast<TCSResource*>(r.get()) == p.get()) {
        ss << p->name << " ";
        p->owner = client;
        p->envelope_keys.insert(client->envelope_key);
        client->allocated_resources.insert(p);
        break;
      }
    }
    for (auto& p : locations) {
      if (static_cast<TCSResource*>(r.get()) == p.get()) {
        ss << p->name << " ";
        p->owner = client;
        client->allocated_resources.insert(p);
        break;
      }
    }
  }

  for (auto& r : res) {
    for (auto x = client->future_claim_resources.begin();
         x != client->future_claim_resources.end();) {
      if (*x == r) {
        x = client->future_claim_resources.erase(x);
        CLOG(INFO, allocate_log)
            << r->name << " from future_claim move to allocate of "
            << client->name << "\n";
      } else {
        x++;
      }
    }
  }
  CLOG(INFO, allocate_log) << ss.str() << "\n";
  return true;
}

bool ResourceManager::free(const std::vector<std::shared_ptr<TCSResource>>& res,
                           const std::shared_ptr<schedule::Client>& client) {
  std::unique_lock<std::mutex> lock(mut);
  for (auto& r : res) {
    for (auto it = client->allocated_resources.begin();
         it != client->allocated_resources.end();) {
      if (r.get() == (*it).get()) {
        it = client->allocated_resources.erase(it);
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
  return true;
}

bool ResourceManager::unclaim(
    const std::vector<std::shared_ptr<TCSResource>>& res,
    const std::shared_ptr<schedule::Client>& client) {
  std::unique_lock<std::mutex> lock(mut);
  for (auto& r : res) {
    for (auto it = client->claim_resources.begin();
         it != client->claim_resources.end();) {
      if (r.get() == (*it).get()) {
        for (auto x = (*it)->future_owner.begin();
             x != (*it)->future_owner.end();) {
          if ((*x).lock() == client) {
            x = (*it)->future_owner.erase(x);
          } else {
            x++;
          }
        }
        it = client->claim_resources.erase(it);
      } else {
        it++;
      }
    }
  }
  return true;
}
std::shared_ptr<data::model::Point> ResourceManager::get_recent_park_point(
    std::shared_ptr<data::model::Point> cur) {
  // CLOG(INFO, "allocate") << "now" << cur->name << " " << cur->position;
  std::vector<double> dis;
  std::sort(points.begin(), points.end(),
            [&](std::shared_ptr<data::model::Point>& a,
                std::shared_ptr<data::model::Point>& b) {
              double d1 = std::numeric_limits<double>::max();
              double d2 = std::numeric_limits<double>::max();
              d1 = (cur->position - a->position).norm();
              d2 = (cur->position - b->position).norm();
              return d1 < d2;
            });
  for (int i = 0; i < points.size(); i++) {
    if (points.at(i)->type != data::model::Point::Type::PARK_POSITION) {
      continue;
    }
    if (is_connected(cur, points.at(i))) {
      return points.at(i);
    }
  }
  return nullptr;
}
std::shared_ptr<data::model::Location> ResourceManager::get_recent_charge_loc(
    std::shared_ptr<data::model::Point> cur) {
  std::vector<double> dis;
  std::sort(locations.begin(), locations.end(),
            [&](std::shared_ptr<data::model::Location>& a,
                std::shared_ptr<data::model::Location>& b) {
              double d1 = std::numeric_limits<double>::max();
              double d2 = std::numeric_limits<double>::max();
              d1 = (cur->position - a->position).norm();
              d2 = (cur->position - b->position).norm();
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
    if (is_connected(cur, locations.at(i)->link.lock())) {
      return locations.at(i);
    }
  }
  return nullptr;
}

}  // namespace allocate
}  // namespace kernel
