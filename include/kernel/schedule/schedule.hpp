#ifndef SCHEDULE_HPP
#define SCHEDULE_HPP
#include <condition_variable>

#include "../../component/rssresource.hpp"
#include "../allocate/resource.hpp"
#include "../driver/command.hpp"
namespace kernel::schedule {
class Client : public RSSObject {
 public:
  ~Client() override = default;
  using RSSObject::RSSObject;
  std::list<std::unordered_set<std::shared_ptr<RSSResource>>>
      claim_resources;  // 声明的资源
  std::list<std::unordered_set<std::shared_ptr<RSSResource>>>
      allocated_resources;  // 已分配的资源
  std::unordered_set<std::shared_ptr<RSSResource>>
      future_allocate_resources;  // 将来可能分配的资源
  std::string envelope_key;
  std::shared_mutex res_mut;
};
class Scheduler : public RSSObject,
                  public std::enable_shared_from_this<Scheduler> {
 public:
  using RSSObject::RSSObject;
  std::shared_ptr<driver::Command> new_command(
      const std::shared_ptr<driver::Vehicle> &);
  void add_command(const std::shared_ptr<driver::Command> &);
  void run();
  ~Scheduler() override {
    commands.clear();
    dispose = true;
    con_var.notify_all();
    if (schedule_th.joinable()) {
      schedule_th.join();
    }
    CLOG(INFO, "schedule") << name << " close\n";
  }

 public:
  std::vector<
      std::pair<std::string, std::list<std::shared_ptr<driver::Command>>>>
      commands;  // 命令池
  std::weak_ptr<allocate::ResourceManager> resource;
  std::thread schedule_th;
  std::condition_variable con_var;
  std::mutex mut;
  bool dispose{false};
  int cur_index{0};
};
}  // namespace kernel::schedule
#endif