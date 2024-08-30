#ifndef HTTPSERVER_HPP
#define HTTPSERVER_HPP
#define CPPHTTPLIB_THREAD_POOL_COUNT 20
#include "../3rdparty/http/httplib.h"
#include "../3rdparty/log/easylogging++.h"
const auto http_log{"http"};
using TCSRep = std::pair<int, std::string>;
class HTTPServer {
 public:
  HTTPServer(const std::string& ip = "0.0.0.0", int port = 8080);
  void listen();
  ~HTTPServer() {
    if (srv.is_running()) {
      srv.stop();
      CLOG(INFO, http_log) << "http server close";
    }
  }

 public:
  std::function<TCSRep(std::string)> get_transport_orders;
  std::function<TCSRep(std::string)> get_transport_order;
  std::function<TCSRep(std::string, std::string)> post_transport_order;
  std::function<TCSRep(std::string, std::string)> post_move_order;
  std::function<TCSRep(std::string, bool, bool)> post_transport_order_withdrawl;
  std::function<TCSRep(std::string)> get_ordersequences;
  std::function<TCSRep(std::string)> get_ordersequence;
  std::function<TCSRep(std::string, std::string)> post_ordersequence;
  std::function<TCSRep(std::string)> get_vehicles;
  std::function<TCSRep(std::string)> get_vehicle;
  std::function<TCSRep(std::string, bool, bool)> post_vehicle_withdrawl;
  std::function<TCSRep(std::string, bool)> put_vehicle_paused;
  std::function<TCSRep()> get_model;
  std::function<TCSRep(std::string)> put_model;
  std::function<TCSRep(std::string)> put_model_xml;
  std::function<TCSRep(std::string, bool)> put_path_locked;
  std::function<TCSRep(std::string, bool)> put_location_locked;
  std::function<TCSRep()> get_view;
  std::function<TCSRep()> post_reroute;
  std::function<TCSRep(std::string, bool)> post_vheicle_reroute;
  std::function<TCSRep(std::string, bool)> put_vehicle_enabled;
  std::function<TCSRep(std::string, std::string)> put_vehicle_integration_level;
  std::function<TCSRep(std::string, std::string)> post_vehicle_path_to_point;

 public:
  std::string ip{"0.0.0.0"};
  int port{8080};
  httplib::Server srv;
};
#endif