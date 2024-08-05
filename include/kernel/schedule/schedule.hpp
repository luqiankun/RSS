#ifndef SCHEDULE_HPP
#define SCHEDULE_HPP
#include "../../component/rssresource.hpp"
#include "../allocate/resource.hpp"
#include "../driver/command.hpp"
namespace kernel {
namespace schedule {
class Client : public RSSObject {
 public:
  virtual ~Client() = default;
  using RSSObject::RSSObject;
  std::list<std::unordered_set<std::shared_ptr<RSSResource>>> claim_resources;
  std::list<std::unordered_set<std::shared_ptr<RSSResource>>>
      allocated_resources;
  std::unordered_set<std::shared_ptr<RSSResource>> future_allocate_resources;
  std::string envelope_key;
};
class Scheduler : public RSSObject,
                  public std::enable_shared_from_this<Scheduler> {
 public:
  using RSSObject::RSSObject;
  std::shared_ptr<driver::Command> new_command(
      std::shared_ptr<driver::Vehicle>);
  void add_command(std::shared_ptr<driver::Command>);
  void run();
  ~Scheduler() {
    commands.clear();
    dispose = true;
    con_var.notify_all();
    if (schedule_th.joinable()) {
      schedule_th.join();
    }
    CLOG(INFO, "schedule") << name << " close\n";
  }

 public:
  std::list<std::shared_ptr<driver::Command>> commands;
  std::weak_ptr<allocate::ResourceManager> resource;
  std::thread schedule_th;
  std::condition_variable con_var;
  std::mutex mut;
  bool dispose{false};
};
}  // namespace schedule
}  // namespace kernel
#endif