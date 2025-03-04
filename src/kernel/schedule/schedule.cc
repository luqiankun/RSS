
#include "../../../include/kernel/schedule/schedule.hpp"

#include "../../../include/component/util/tools.hpp"
#include "../../../include/kernel/driver/command.hpp"
#include "../../../include/kernel/driver/vehicle.hpp"

namespace kernel::schedule {
void Scheduler::run() {
  CLOG(INFO, "schedule") << this->name << " run....\n";
  schedule_th = std::thread([&] {
    while (!dispose) {
      std::unique_lock<std::mutex> lock(mut);
      con_var.wait(lock, [&] {
        bool empty = true;
        for (auto &x : commands) {
          if (!x.second.empty()) {
            empty = false;
            break;
          }
        }
        return !empty || dispose;
      });
      if (dispose) {
        break;
      }
      if (cur_index >= commands.size()) {
        cur_index = 0;
      }
      if (!commands[cur_index].second.empty()) {
        auto cmd = commands[cur_index].second.front();

        if (cmd->state == driver::Command::State::DISPOSABLE) {
          commands[cur_index].second.pop_front();
        } else {
          cmd->run_once();
        }
      }
      cur_index++;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });
}

void Scheduler::add_command(const std::shared_ptr<driver::Command> &cmd) {
  if (cmd) {
    CLOG(DEBUG, "schedule") << "add new cmd " << cmd->name << "\n";
    for (auto &x : commands) {
      if (x.first == cmd->vehicle.lock()->name) {
        x.second.push_back(cmd);
        con_var.notify_one();
        return;
      }
    }
    commands.push_back({cmd->vehicle.lock()->name, {cmd}});
    con_var.notify_one();
  }
}
std::shared_ptr<driver::Command> Scheduler::new_command(
    const std::shared_ptr<driver::Vehicle> &v) {
  if (!v->current_order) {
    CLOG(WARNING, "schedule") << "no ord there";
    return nullptr;
  }
  if (v->current_order->state !=
      data::order::TransportOrder::State::BEING_PROCESSED) {
    CLOG(ERROR, "schedule") << "this order can not be processing";
    return nullptr;
  }
  std::string cmd_name{
      "command_" + v->name + "_" + uuids::to_string(get_uuid()) + "_" +
      v->current_order->driverorders[v->current_order->current_driver_index]
          ->get_cmd_name()};
  auto cmd = std::make_shared<driver::Command>(cmd_name);
  // LOG(DEBUG) << cmd_name << "\n";
  cmd->vehicle = v;
  cmd->order = v->current_order;
  cmd->scheduler = shared_from_this();
  cmd->move = [=](auto step) { v->execute_move(step); };
  cmd->action = [=](auto step) { v->execute_action(step); };
  return cmd;
}
}  // namespace kernel::schedule