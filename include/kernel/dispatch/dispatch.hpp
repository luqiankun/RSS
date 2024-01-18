#ifndef DISPATCH_HPP
#define DISPATCH_HPP
#include "../allocate/order.hpp"
#include "../allocate/resource.hpp"
#include "../planner/planner.hpp"
namespace kernel {
namespace dispatch {
using Oper = std::tuple<
    std::string,
    data::order::DriverOrder::Destination::OpType>;  // 目标点和动作类型
class Dispatcher : public TCSObject {
 public:
  using TCSObject::TCSObject;
  enum class ResType { Point = 0, Location = 1, Err = 2 };
  std::shared_ptr<data::order::Route> paths_to_route(
      std::vector<std::shared_ptr<data::model::Point>>);
  std::pair<ResType, std::shared_ptr<TCSResource>> find(
      const std::string& name);
  std::shared_ptr<data::order::DriverOrder::Destination> res_to_destination(
      const std::shared_ptr<TCSResource>&,
      data::order::DriverOrder::Destination::OpType);
  std::shared_ptr<data::order::DriverOrder> route_to_driverorder(
      std::shared_ptr<data::order::Route>,
      std::shared_ptr<data::order::DriverOrder::Destination>);
  std::shared_ptr<data::order::TransportOrder> new_orderseq(
      std::vector<Oper>, std::size_t uuid,
      int strategy = -1);  // strategy<0 自动  >=0 指定执行端
  std::shared_ptr<driver::Vehicle> select_vehicle(
      std::shared_ptr<data::model::Point>);
  std::shared_ptr<driver::Vehicle> find_owner(
      const std::shared_ptr<TCSResource>&);
  std::vector<std::vector<std::shared_ptr<driver::Vehicle>>>
  deadlock_loop();  // 死锁车辆环路
  void brake_deadlock(
      std::vector<std::vector<std::shared_ptr<driver::Vehicle>>>);  // 解锁
  void dispatch_once();
  void add_vehicle(const std::string& type, const std::string& name);
  void cancel_vehicle_order(size_t order_uuid);  // 已分配未下发的订单
  void cancel_order(size_t order_uuid);          // 未分配的订单
  void cancel_vehicle_all_order(size_t vehicle_uuid);  // 已分配未下发的所有订单
  void run();

 public:
  std::shared_ptr<allocate::ResourceManager> resource;
  std::vector<std::shared_ptr<driver::Vehicle>> vehicles;
  std::shared_ptr<planner::Planner> planner;
  std::shared_ptr<allocate::OrderPool> orderpool;
  std::thread dispatch_th;
};
}  // namespace dispatch
}  // namespace kernel
#endif