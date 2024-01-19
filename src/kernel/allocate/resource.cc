#include "../../../include/kernel/allocate/resource.hpp"

#include "../../../include/kernel/schedule/schedule.hpp"

namespace kernel {
namespace allocate {
bool ResourceManager::allocate(
    const std::vector<std::shared_ptr<TCSResource> >& res,
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

bool ResourceManager::claim(
    const std::vector<std::shared_ptr<TCSResource> >& res,
    const std::shared_ptr<schedule::Client>& client) {
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
              LOG(INFO) << p->name << " from future to claim";
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
              LOG(INFO) << p->name << " from future to claim";
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
              LOG(INFO) << p->name << " from future to claim";
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
    const std::vector<std::shared_ptr<TCSResource> >& res,
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

bool ResourceManager::free(
    const std::vector<std::shared_ptr<TCSResource> >& res,
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
