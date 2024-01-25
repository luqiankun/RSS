
#include "../../include/main/tcs.hpp"

#include "../../include/component/tools/log/easylogging++.h"
#include "../../include/main/httpsrv.hpp"
INITIALIZE_EASYLOGGINGPP
int main(int argc, const char** argv) {
  MY_EASYLOG_CONIFG("./tcs_logs")
  LOG(INFO) << "---------------------------------------------------------------"
               "--------------";
  auto tcs = std::make_shared<TCS>();
  if (tcs->init_all("/home/luqk/c++/TCS/config/map.xml")) {
    tcs->run();
    std::cin.get();
    auto op = Oper("Location-0025",
                   data::order::DriverOrder::Destination::OpType::LOAD);
    auto op2 = Oper("Location-0019",
                    data::order::DriverOrder::Destination::OpType::UNLOAD);
    std::vector<Oper> ops;
    std::vector<Oper> ops2;
    ops.push_back(op);
    ops2.push_back(op2);
    std::hash<std::string> hash_fn;
    // tcs->add_task(ops, "test1");
    // std::this_thread::sleep_for(std::chrono::microseconds(10));
    // tcs->add_task(ops2, "test2");
    // std::cin.get();
    // tcs->paused_vehicle("Vehicle-0001");
    // std::cin.get();
    // tcs->recovery_vehicle("Vehicle-0001");

    // tcs->stop();
    // tcs->init_all("/home/luqk/c++/TCS/config/map.xml");
    // tcs->run();
    // // tcs->cancel_order(hash_fn("test"));
    // tcs->add_task(ops, "test1");
    // std::this_thread::sleep_for(std::chrono::microseconds(10));
    // tcs->add_task(ops2, "test2");
    std::cin.get();
  } else {
    LOG(ERROR) << "init failed";
  }
}