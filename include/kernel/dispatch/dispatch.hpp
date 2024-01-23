#ifndef DISPATCH_HPP
#define DISPATCH_HPP
#include <boost/signals2.hpp>

#include "../allocate/order.hpp"
#include "../allocate/resource.hpp"
#include "../planner/planner.hpp"
using Oper = std::tuple<
    std::string,
    data::order::DriverOrder::Destination::OpType>;  // 目标点和动作类型
namespace kernel {
namespace dispatch {
class Dispatcher : public TCSObject {
 public:
  using TCSObject::TCSObject;

  std::shared_ptr<driver::Vehicle> select_vehicle(
      std::shared_ptr<data::model::Point>);
  std::set<std::shared_ptr<driver::Vehicle>> find_owners(
      const std::shared_ptr<driver::Vehicle>&);
  std::vector<std::shared_ptr<driver::Vehicle>>
  deadlock_loop();  // 死锁车辆环路
  void brake_deadlock(
      std::vector<std::shared_ptr<driver::Vehicle>>);  // TODO 解锁
  void dispatch_once();
  void idle_detect();
  void run();
  void stop();
  ~Dispatcher();

 public:
  std::thread dispatch_th;
  bool dispose{false};
  std::vector<std::shared_ptr<driver::Vehicle>> vehicles;
  /// signals
  boost::signals2::signal<std::pair<allocate::ResourceManager::ResType,
                                    std::shared_ptr<TCSResource>>(
      const std::string& name)>
      find_res;
  boost::signals2::signal<void(std::vector<Oper>, const std::string&,
                               const std::string&)>
      add_task;
  boost::signals2::signal<std::shared_ptr<data::order::TransportOrder>()>
      get_next_ord;
};
}  // namespace dispatch
}  // namespace kernel
#endif