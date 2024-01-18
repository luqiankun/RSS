
#include "../../../include/kernel/schedule/schedule.hpp"

#include "../../../include/kernel/driver/command.hpp"
#include "../../../include/kernel/driver/vehicle.hpp"

namespace kernel {
namespace schedule {
void Scheduler::run() {
  schedule_th = std::thread([&] {
    LOG(INFO) << this->name << " run....";
    while (!dispose) {
      std::unique_lock<std::mutex> lock(mut);
      con_var.wait(lock, [&] { return !commands.empty() || dispose; });
      if (dispose) {
        break;
      }
      for (auto it = commands.begin(); it != commands.end();) {
        if ((*it)->state == driver::Command::State::DISPOSABLE) {
          it = commands.erase(it);
        } else {
          (*it)->run_once();
          it++;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });
}

void Scheduler::add_command(std::shared_ptr<driver::Command> cmd) {
  if (cmd) {
    LOG(INFO) << "add new cmd " << cmd->name;
    commands.push_back(cmd);
    con_var.notify_one();
  }
}
std::shared_ptr<driver::Command> Scheduler::new_command(
    std::shared_ptr<driver::Vehicle> v) {
  // TODO
  if (!v->current_order) {
    LOG(WARNING) << "no ord there";
    return nullptr;
  }
  if (v->current_order->state !=
      data::order::TransportOrder::State::BEING_PROCESSED) {
    return nullptr;
  }
  std::string cmd_name{
      "command_" + v->name + "_" + v->current_order->name + "_" +
      v->current_order->driverorders[v->current_order->current_driver_index]
          ->get_cmd_name()};
  auto cmd = std::make_shared<driver::Command>(cmd_name);
  cmd->vehicle = v;
  cmd->order = v->current_order;
  cmd->scheduler = shared_from_this();
  return cmd;
}
}  // namespace schedule
}  // namespace kernel