
#include "../../include/main/tcs.hpp"

#include "../../include/component/log/easylogging++.h"
INITIALIZE_EASYLOGGINGPP
int main(int argc, const char** argv) {
  MY_EASYLOG_CONIFG("./tcs_logs")
  LOG(INFO) << "---------------------------------------------------------------"
               "--------------";
  auto tcs = std::make_shared<TCS>();
  if (tcs->init_all("/home/luqk/c++/TCS/config/map.xml")) {
    tcs->run();
  }
  std::cin.get();
  auto op = kernel::dispatch::Oper(
      "Location-0025", data::order::DriverOrder::Destination::OpType::LOAD);
  auto op2 = kernel::dispatch::Oper(
      "Location-0019", data::order::DriverOrder::Destination::OpType::UNLOAD);
  std::vector<kernel::dispatch::Oper> ops;
  std::vector<kernel::dispatch::Oper> ops2;
  ops.push_back(op);
  ops2.push_back(op2);
  std::hash<std::string> hash_fn;
  tcs->add_task(ops, hash_fn("test"));
  std::this_thread::sleep_for(std::chrono::microseconds(10));
  tcs->add_task(ops2, hash_fn("test2"));
  std::cin.get();
  tcs->cancel_order(hash_fn("test"));
  std::cin.get();
}