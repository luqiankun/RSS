#ifndef RSS_HPP
#define RSS_HPP
#include <shared_mutex>

#include "../3rdparty/xml/pugixml.hpp"
#include "../component/util/tools.hpp"
#include "../kernel/dispatch/dispatch.hpp"
#include "../kernel/schedule/schedule.hpp"
using json = jsoncons::json;
using RET = std::pair<int, std::string>;
inline std::string MQTT_IP{"127.0.0.1"};
inline int MQTT_PORT{1883};
/**
 * @brief 管理调度系统的启动关闭，重置等
 *
 */
class RSS : public std::enable_shared_from_this<RSS> {
 public:
  // transport order
  RET get_transport_orders(const std::string &vehicle = "") const;
  RET get_transport_order(const std::string &ord_name) const;
  RET post_move_order(const std::string &vehicle, const std::string &point);
  RET post_transport_order(const std::string &ord_name,
                           const std::string &body) const;
  RET post_transport_order_withdrawl(const std::string &ord_name,
                                     bool immediate = false,
                                     bool disableVehicle = false) const;
  // ord sequence
  RET get_ordersequences(const std::string &vehicle = "") const;
  RET get_ordersequence(const std::string &sequence_name) const;
  RET post_ordersequence(const std::string &sequence_name,
                         const std::string &body) const;
  // vehicle
  RET get_vehicles(const std::string &state = "") const;
  RET get_vehicle(const std::string &vehicle) const;
  RET put_vehicle_paused(const std::string &, bool) const;
  RET post_vehicle_withdrawl(const std::string &vehicle, bool immediate = false,
                             bool disableVehicle = false) const;
  // model
  RET get_model() const;
  RET put_model(const std::string &body);
  RET put_path_locked(const std::string &path_name, bool new_value) const;
  RET put_location_locked(const std::string &loc_name, bool new_value) const;
  RET put_model_xml(const std::string &body);
  RET get_view() const;
  RET post_reroute() const;
  RET put_vehicle_simrate(int) const;
  RET post_vehicle_reroute(const std::string &, bool) const;
  RET put_vehicle_enable(const std::string &, bool) const;
  RET put_vehicle_integration_level(const std::string &,
                                    const std::string &) const;
  RET post_vehicle_path_to_point(const std::string &,
                                 const std::string &) const;

 public:
  bool init_dispatcher();
  bool init_scheduler();
  bool init_orderpool();
  bool init_planner();
  bool init_resource();
  bool is_connect(const std::shared_ptr<data::model::Point> &,
                  const std::shared_ptr<data::model::Point> &) const;
  void home_order(const std::string &name,
                  const std::shared_ptr<kernel::driver::Vehicle> &v) const;
  void charge_order(const std::string &name,
                    const std::shared_ptr<kernel::driver::Vehicle> &v) const;
  void cancel_order(const std::string &order_name) const;  // 取消某个订单
  void cancel_all_order() const;                           // 取消所有订单
  void cancel_vehicle_all_order(
      const std::string &vehicle_name) const;  // 取消某辆车所有订单
  void run();
  // 车辆操作
  void add_vehicle(const std::string &type, const std::string &name);
  void paused_vehicle(const std::string &name) const;
  void recovery_vehicle(const std::string &name) const;
  std::string get_vehicles_step() const;
  void reroute() const;
  void stop();
  void init();
  bool is_exist_active_order() const;
  ~RSS();

 public:
  bool is_run{false};
  std::shared_ptr<kernel::allocate::ResourceManager> resource;
  std::shared_ptr<kernel::schedule::Scheduler> scheduler;
  std::shared_ptr<kernel::dispatch::Dispatcher> dispatcher;
  std::shared_ptr<kernel::allocate::OrderPool> orderpool;
  std::shared_ptr<kernel::planner::Planner> planner;
  mutable std::shared_mutex mutex;
};
inline std::string get_log_path(const std::string &path) {
  auto data = get_time_fmt(get_now_utc_time());
#ifdef _WIN32
  return (std::string(path) + "\\" + data + ".log");
#else
  return (std::string(path) + "/" + data + ".log");
#endif
}
inline std::string get_log_name(const std::string &path) {
#ifdef _WIN32
  return (std::string(path) + "\\rss_" + ".log");
#else
  return (std::string(path) + "/rss_" + ".log");
#endif
}
enum StatusCode {
  // Information responses
  Continue_100 = 100,
  SwitchingProtocol_101 = 101,
  Processing_102 = 102,
  EarlyHints_103 = 103,

  // Successful responses
  OK_200 = 200,
  Created_201 = 201,
  Accepted_202 = 202,
  NonAuthoritativeInformation_203 = 203,
  NoContent_204 = 204,
  ResetContent_205 = 205,
  PartialContent_206 = 206,
  MultiStatus_207 = 207,
  AlreadyReported_208 = 208,
  IMUsed_226 = 226,

  // Redirection messages
  MultipleChoices_300 = 300,
  MovedPermanently_301 = 301,
  Found_302 = 302,
  SeeOther_303 = 303,
  NotModified_304 = 304,
  UseProxy_305 = 305,
  unused_306 = 306,
  TemporaryRedirect_307 = 307,
  PermanentRedirect_308 = 308,

  // Client error responses
  BadRequest_400 = 400,
  Unauthorized_401 = 401,
  PaymentRequired_402 = 402,
  Forbidden_403 = 403,
  NotFound_404 = 404,
  MethodNotAllowed_405 = 405,
  NotAcceptable_406 = 406,
  ProxyAuthenticationRequired_407 = 407,
  RequestTimeout_408 = 408,
  Conflict_409 = 409,
  Gone_410 = 410,
  LengthRequired_411 = 411,
  PreconditionFailed_412 = 412,
  PayloadTooLarge_413 = 413,
  UriTooLong_414 = 414,
  UnsupportedMediaType_415 = 415,
  RangeNotSatisfiable_416 = 416,
  ExpectationFailed_417 = 417,
  ImATeapot_418 = 418,
  MisdirectedRequest_421 = 421,
  UnprocessableContent_422 = 422,
  Locked_423 = 423,
  FailedDependency_424 = 424,
  TooEarly_425 = 425,
  UpgradeRequired_426 = 426,
  PreconditionRequired_428 = 428,
  TooManyRequests_429 = 429,
  RequestHeaderFieldsTooLarge_431 = 431,
  UnavailableForLegalReasons_451 = 451,

  // Server error responses
  InternalServerError_500 = 500,
  NotImplemented_501 = 501,
  BadGateway_502 = 502,
  ServiceUnavailable_503 = 503,
  GatewayTimeout_504 = 504,
  HttpVersionNotSupported_505 = 505,
  VariantAlsoNegotiates_506 = 506,
  InsufficientStorage_507 = 507,
  LoopDetected_508 = 508,
  NotExtended_510 = 510,
  NetworkAuthenticationRequired_511 = 511,
};
#endif