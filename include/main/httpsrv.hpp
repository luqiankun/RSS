#ifndef HTTPSERVER_HPP
#define HTTPSERVER_HPP
#include <boost/signals2.hpp>
#include <eigen3/Eigen/Eigen>

#include "../component/tools/http/httplib.h"
#include "../component/tools/json/json.hpp"
#include "../component/util/timer.hpp"
using TCSRep = std::pair<int, std::string>;
class HTTPServer {
 public:
  HTTPServer(const std::string& ip = "0.0.0.0", int port = 8080);
  void listen();

 public:
  boost::signals2::signal<TCSRep(std::string)> get_transport_orders;
  boost::signals2::signal<TCSRep(std::string)> get_transport_order;
  boost::signals2::signal<TCSRep(std::string, std::string)>
      post_transport_order;
  boost::signals2::signal<TCSRep(std::string, bool, bool)>
      post_transport_order_withdrawl;
  boost::signals2::signal<TCSRep(std::string)> get_ordersequences;
  boost::signals2::signal<TCSRep(std::string)> get_ordersequence;
  boost::signals2::signal<TCSRep(std::string, std::string)> post_ordersequence;
  boost::signals2::signal<TCSRep(std::string)> get_vehicles;
  boost::signals2::signal<TCSRep(std::string)> get_vehicle;
  boost::signals2::signal<TCSRep(std::string, bool, bool)>
      post_vehicle_withdrawl;
  boost::signals2::signal<TCSRep()> get_model;
  boost::signals2::signal<TCSRep(std::string)> put_model;
  boost::signals2::signal<TCSRep(std::string, bool)> put_path_locked;
  boost::signals2::signal<TCSRep(std::string, bool)> put_location_locked;

 private:
  std::string ip{"0.0.0.0"};
  int port{8080};
  httplib::Server srv;
};
#endif