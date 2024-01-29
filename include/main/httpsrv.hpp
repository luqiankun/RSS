#ifndef HTTPSERVER_HPP
#define HTTPSERVER_HPP
#include <Eigen/Eigen>

#include "../component/tools/http/httplib.h"
#include "../component/tools/json/json.hpp"
#include "../component/util/timer.hpp"
using TCSRep = std::pair<int, std::string>;
class HTTPServer {
 public:
  HTTPServer(const std::string& ip = "0.0.0.0", int port = 8080);
  void listen();
  ~HTTPServer() {
    if (srv.is_running()) {
      srv.stop();
      LOG(INFO) << "http server close";
    }
  }

 public:
  std::function<TCSRep(std::string)> get_transport_orders;
  std::function<TCSRep(std::string)> get_transport_order;
  std::function<TCSRep(std::string, std::string)> post_transport_order;
  std::function<TCSRep(std::string, bool, bool)> post_transport_order_withdrawl;
  std::function<TCSRep(std::string)> get_ordersequences;
  std::function<TCSRep(std::string)> get_ordersequence;
  std::function<TCSRep(std::string, std::string)> post_ordersequence;
  std::function<TCSRep(std::string)> get_vehicles;
  std::function<TCSRep(std::string)> get_vehicle;
  std::function<TCSRep(std::string, bool, bool)> post_vehicle_withdrawl;
  std::function<TCSRep()> get_model;
  std::function<TCSRep(std::string)> put_model;
  std::function<TCSRep(std::string)> put_model_xml;
  std::function<TCSRep(std::string, bool)> put_path_locked;
  std::function<TCSRep(std::string, bool)> put_location_locked;

 public:
  std::string ip{"0.0.0.0"};
  int port{8080};
  httplib::Server srv;
};
#endif