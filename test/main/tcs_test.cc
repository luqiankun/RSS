
#include "../../include/main/tcs.hpp"

#include "../../include/component/log/easylogging++.h"

INITIALIZE_EASYLOGGINGPP
int main(int argc, const char** argv) {
  el::Configurations conf;
  conf.setGlobally(el::ConfigurationType::Format,
                   "[%levshort] %datetime %fbase:%line] %msg");
  conf.setGlobally(el::ConfigurationType::Filename,
                   "/home/luqk/c++/TCS/build/logs/tcs.log");
  conf.set(el::Level::Debug, el::ConfigurationType::Format,
           "%datetime{%d/%M} %func [%fbase:%line] %msg");
  el::Loggers::reconfigureLogger("default", conf);
  el::Loggers::reconfigureAllLoggers(conf);
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
  auto ord = tcs->dispatcher->new_orderseq(ops, hash_fn("test"));
  auto ord2 = tcs->dispatcher->new_orderseq(ops2, hash_fn("test2"));
  tcs->dispatcher->orderpool->orderpool.push_back(ord);
  tcs->dispatcher->orderpool->orderpool.push_back(ord2);
  std::cin.get();
}