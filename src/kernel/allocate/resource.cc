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
    res->current_step = res->steps.front();
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

bool ResourceManager::allocate(
    const std::vector<std::shared_ptr<TCSResource>>& res,
    const std::shared_ptr<schedule::Client>& client) {
  std::unique_lock<std::mutex> lock(mut);
  for (auto& r : res) {
    for (auto& p : points) {
      if (static_cast<TCSResource*>(r.get()) == p.get()) {
        client->allocate_resources.insert(p);
        p->future_owner.push_back(client);
      }
    }
    for (auto& p : paths) {
      if (static_cast<TCSResource*>(r.get()) == p.get()) {
        client->allocate_resources.insert(p);
        p->future_owner.push_back(client);
      }
    }
    for (auto& p : locations) {
      if (static_cast<TCSResource*>(r.get()) == p.get()) {
        client->allocate_resources.insert(p);
        p->future_owner.push_back(client);
      }
    }
  }
  return true;
}

bool ResourceManager::claim(std::vector<std::shared_ptr<TCSResource>> res,
                            std::shared_ptr<schedule::Client> client) {
  std::stringstream ss;
  ss << client->name << " claim ";
  std::unique_lock<std::mutex> lock(mut);
  for (auto& r : res) {
    for (auto& p : points) {
      if (static_cast<TCSResource*>(r.get()) == p.get()) {
        ss << p->name << " ";
        auto it = p->owner.lock();
        if (!it || it == client) {
          p->owner = client;
          for (auto x = client->future_claim_resources.begin();
               x != client->future_claim_resources.end();) {
            if (*x == p) {
              x = client->future_claim_resources.erase(x);
              LOG(INFO) << p->name << " from future to claim of "
                        << client->name;
            } else {
              x++;
            }
          }
          client->claim_resources.insert(p);
        } else {
          return false;
        }
      }
    }
    for (auto& p : paths) {
      if (static_cast<TCSResource*>(r.get()) == p.get()) {
        ss << p->name << " ";
        auto it = p->owner.lock();
        if (!it || it == client) {
          p->owner = client;
          for (auto x = client->future_claim_resources.begin();
               x != client->future_claim_resources.end();) {
            if (*x == p) {
              x = client->future_claim_resources.erase(x);
              LOG(INFO) << p->name << " from future to claim of "
                        << client->name;
            } else {
              x++;
            }
          }
          client->claim_resources.insert(p);
        } else {
          return false;
        }
      }
    }
    for (auto& p : locations) {
      if (static_cast<TCSResource*>(r.get()) == p.get()) {
        ss << p->name << " ";
        auto it = p->owner.lock();
        if (!it || it == client) {
          p->owner = client;
          for (auto x = client->future_claim_resources.begin();
               x != client->future_claim_resources.end();) {
            if (*x == p) {
              x = client->future_claim_resources.erase(x);
              LOG(INFO) << p->name << " from future to claim of "
                        << client->name;
            } else {
              x++;
            }
          }
          client->claim_resources.insert(p);
        } else {
          return false;
        }
      }
    }
  }
  LOG(INFO) << ss.str();
  return true;
}

bool ResourceManager::unclaim(
    const std::vector<std::shared_ptr<TCSResource>>& res,
    const std::shared_ptr<schedule::Client>& client) {
  std::unique_lock<std::mutex> lock(mut);
  for (auto& r : res) {
    r->owner.reset();
    for (auto it = client->claim_resources.begin();
         it != client->claim_resources.end();) {
      if (r.get() == (*it).get()) {
        it = client->claim_resources.erase(it);
      } else {
        it++;
      }
    }
  }
  return true;
}

bool ResourceManager::free(const std::vector<std::shared_ptr<TCSResource>>& res,
                           const std::shared_ptr<schedule::Client>& client) {
  std::unique_lock<std::mutex> lock(mut);
  for (auto& r : res) {
    for (auto it = client->allocate_resources.begin();
         it != client->allocate_resources.end();) {
      if (r.get() == (*it).get()) {
        it = client->allocate_resources.erase(it);
        for (auto x = (*it)->future_owner.begin();
             x != (*it)->future_owner.end();) {
          if ((*x).lock() == client) {
            x = (*it)->future_owner.erase(x);
          } else {
            x++;
          }
        }
      } else {
        it++;
      }
    }
  }
  return true;
}

}  // namespace allocate
}  // namespace kernel
