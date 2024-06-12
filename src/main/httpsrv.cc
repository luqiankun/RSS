#include "../../include/main/httpsrv.hpp"
std::string dump_headers(const httplib::Headers &headers) {
  std::string s;
  char buf[BUFSIZ];

  for (auto it = headers.begin(); it != headers.end(); ++it) {
    const auto &x = *it;
    snprintf(buf, sizeof(buf), "%s: %s\n", x.first.c_str(), x.second.c_str());
    s += buf;
  }

  return s;
}
std::string log(const httplib::Request &req, const httplib::Response &res) {
  std::string s;
  char buf[BUFSIZ];

  s += "================================\n";

  snprintf(buf, sizeof(buf), "%s %s %s", req.method.c_str(),
           req.version.c_str(), req.path.c_str());
  s += buf;

  std::string query;
  for (auto it = req.params.begin(); it != req.params.end(); ++it) {
    const auto &x = *it;
    snprintf(buf, sizeof(buf), "%c%s=%s",
             (it == req.params.begin()) ? '?' : '&', x.first.c_str(),
             x.second.c_str());
    query += buf;
  }
  snprintf(buf, sizeof(buf), "%s\n", query.c_str());
  s += buf;

  s += dump_headers(req.headers);
  s += "\n";
  // if (!req.body.empty()) {
  //   s += req.body;
  // }
  s += "\n";
  s += "--------------------------------\n";

  snprintf(buf, sizeof(buf), "%d %s\n", res.status, res.version.c_str());
  s += buf;
  s += dump_headers(res.headers);
  s += "\n";
  // if (!res.body.empty()) {
  //   s += res.body;
  // }
  s += "\n";

  return s;
}
HTTPServer::HTTPServer(const std::string &ip, int port) {
  this->ip = ip;
  this->port = port;
  srv.set_default_headers({{"Access-Control-Allow-Origin", "*"},
                           {"Access-Control-Allow-Credentials", "false"},
                           {"Allow", "GET, POST, HEAD, OPTIONS,PUT"}});
  srv.Get("/transportOrders",
          [=](const httplib::Request &req, httplib::Response &res) {
            //
            std::string veh{""};
            if (req.has_param("intendedVehicle")) {
              veh = req.get_param_value("intendedVehicle");
            }
            auto ret = get_transport_orders(veh);
            res.status = ret.first;
            if (!ret.second.empty()) {
              res.set_content(ret.second, "application/json");
            }
          });
  srv.Get(R"(/transportOrder/([^/]+))",
          [=](const httplib::Request &req, httplib::Response &res) {
            auto m = req.matches.end() - 1;
            std::string ord_name = *m;
            auto ret = get_transport_order(ord_name);
            res.status = ret.first;
            if (!ret.second.empty()) {
              res.set_content(ret.second, "application/json");
            }
          });
  srv.Post(R"(/transportOrders/([^/]+))",
           [=](const httplib::Request &req, httplib::Response &res) {
             auto m = req.matches.end() - 1;
             std::string ord_name = *m;
             auto ret = post_transport_order(ord_name, req.body);
             res.status = ret.first;
             if (!ret.second.empty()) {
               res.set_content(ret.second, "application/json");
             }
           });
  srv.Post(R"(/transportOrders/([^/]+)/withdrawal)",
           [=](const httplib::Request &req, httplib::Response &res) {
             auto m = req.matches.end() - 1;
             std::string ord_name = *m;
             bool immediate{false};
             bool disableVehicle{false};
             if (req.has_param("immediate")) {
               if (req.get_param_value("immediate") == "true" ||
                   req.get_param_value("immediate") == "1") {
                 immediate = true;
               }
             }
             if (req.has_param("disableVehicle")) {
               if (req.get_param_value("disableVehicle") == "true" ||
                   req.get_param_value("disableVehicle") == "1") {
                 immediate = true;
               }
             }
             auto ret = post_transport_order_withdrawl(ord_name, immediate,
                                                       disableVehicle);
             res.status = ret.first;
             if (!ret.second.empty()) {
               res.set_content(ret.second, "application/json");
             }
           });

  srv.Get(R"(/orderSequences)",
          [=](const httplib::Request &req, httplib::Response &res) {
            std::string veh{""};
            if (req.has_param("intendedVehicle")) {
              veh = req.get_param_value("intendedVehicle");
            }
            auto ret = get_ordersequences(veh);
            res.status = ret.first;
            if (!ret.second.empty()) {
              res.set_content(ret.second, "application/json");
            }
          });
  srv.Get(R"(/orderSequences/([^/]+))",
          [=](const httplib::Request &req, httplib::Response &res) {
            std::string veh{""};
            auto m = req.matches.end() - 1;
            veh = *m;
            auto ret = get_ordersequence(veh);
            res.status = ret.first;
            if (!ret.second.empty()) {
              res.set_content(ret.second, "application/json");
            }
          });
  srv.Post(R"(/orderSequences/([^/]+))",
           [=](const httplib::Request &req, httplib::Response &res) {
             std::string veh{""};
             auto m = req.matches.end() - 1;
             veh = *m;
             auto ret = post_ordersequence(veh, req.body);
             res.status = ret.first;
             if (!ret.second.empty()) {
               res.set_content(ret.second, "application/json");
             }
           });
  srv.Get(R"(/vehicles)",
          [=](const httplib::Request &req, httplib::Response &res) {
            std::string state{""};
            if (req.has_param("procState")) {
              state = req.get_param_value("procState");
            }
            auto ret = get_vehicles(state);
            res.status = ret.first;
            if (!ret.second.empty()) {
              res.set_content(ret.second, "application/json");
            }
          });
  srv.Get(R"(/vehicles/([^/]+))",
          [=](const httplib::Request &req, httplib::Response &res) {
            std::string state{""};
            state = *(req.matches.end() - 1);
            auto ret = get_vehicle(state);
            res.status = ret.first;
            if (!ret.second.empty()) {
              res.set_content(ret.second, "application/json");
            }
          });
  srv.Put(R"(/vehicles/([^/]+)/paused)",
          [=](const httplib::Request &req, httplib::Response &res) {
            std::string name;
            name = *(req.matches.end() - 1);
            bool state;
            if (req.has_param("newValue")) {
              state = req.get_param_value("newValue") == "true" ? true : false;
            }
            auto ret = put_vehicle_paused(name, state);
            res.status = ret.first;
            if (!ret.second.empty()) {
              res.set_content(ret.second, "application/json");
            }
          });
  srv.Options(R"(/vehicles/([^/]+)/paused)",
              [=](const httplib::Request &req, httplib::Response &res) {
                res.set_header("Access-Control-Allow-Methods", "PUT");
                res.status = 200;
              });
  srv.Post(R"(/vehicles/([^/]+)/withdrawal)",
           [=](const httplib::Request &req, httplib::Response &res) {
             std::string name{""};
             name = *(req.matches.end() - 1);
             bool immediate{false};
             bool disableVehicle{false};
             if (req.has_param("immediate")) {
               if (req.get_param_value("immediate") == "true" ||
                   req.get_param_value("immediate") == "1") {
                 immediate = true;
               }
             }
             if (req.has_param("disableVehicle")) {
               if (req.get_param_value("disableVehicle") == "true" ||
                   req.get_param_value("disableVehicle") == "1") {
                 immediate = true;
               }
             }
             if (req.has_param("disableVehicle")) {
               if (req.get_param_value("disableVehicle") == "true" ||
                   req.get_param_value("disableVehicle") == "1") {
                 immediate = true;
               }
             }
             auto ret = post_vehicle_withdrawl(name, immediate, disableVehicle);
             res.status = ret.first;
             if (!ret.second.empty()) {
               res.set_content(ret.second, "application/json");
             }
           });

  srv.Get(R"(/plantModel)",
          [=](const httplib::Request &, httplib::Response &res) {
            auto ret = get_model();
            res.status = ret.first;
            if (!ret.second.empty()) {
              res.set_content(ret.second, "application/json");
            }
          });
  srv.Put(R"(/plantModel)",
          [=](const httplib::Request &req, httplib::Response &res) {
            bool use_xml{false};
            if (req.has_param("type")) {
              use_xml = req.get_param_value("type") == "xml" ? true : false;
              LOG(INFO) << "---------------------";
            }
            if (use_xml) {
              auto ret = put_model_xml(req.body);
              res.status = ret.first;
              if (!ret.second.empty()) {
                res.set_content(ret.second, "application/json");
              }
            } else {
              auto ret = put_model(req.body);
              res.status = ret.first;
              if (!ret.second.empty()) {
                res.set_content(ret.second, "application/json");
              }
            }
          });
  srv.Options(R"(/plantModel)",
              [=](const httplib::Request &req, httplib::Response &res) {
                res.set_header("Access-Control-Allow-Methods", "PUT");
                res.set_header("Access-Control-Allow-Headers", "Content-Type");
                res.status = 200;
              });
  srv.Put(R"(/paths/([^/]+)/locked)",
          [=](const httplib::Request &req, httplib::Response &res) {
            bool value{false};
            if (req.get_param_value("newValue") == "true" ||
                req.get_param_value("newValue") == "1") {
              value = true;
            }
            auto m = req.matches.end() - 1;
            std::string name = *m;
            auto ret = put_path_locked(name, value);
            res.status = ret.first;
            if (!ret.second.empty()) {
              res.set_content(ret.second, "application/json");
            }
          });
  srv.Options(R"(/paths/([^/]+)/locked)",
              [=](const httplib::Request &req, httplib::Response &res) {
                res.set_header("Access-Control-Allow-Methods", "PUT");
                res.status = 200;
              });
  srv.Put(R"(/locations/([^/]+)/locked)",
          [=](const httplib::Request &req, httplib::Response &res) {
            bool value{false};
            if (req.get_param_value("newValue") == "true" ||
                req.get_param_value("newValue") == "1") {
              value = true;
            }
            auto m = req.matches.end() - 1;
            std::string name = *m;
            auto ret = put_location_locked(name, value);
            res.status = ret.first;
            if (!ret.second.empty()) {
              res.set_content(ret.second, "application/json");
            }
          });
  srv.Options(R"(/locations/([^/]+)/locked)",
              [=](const httplib::Request &req, httplib::Response &res) {
                res.set_header("Access-Control-Allow-Methods", "PUT");
                res.status = 200;
              });
  srv.Get("/getView", [=](const httplib::Request &req, httplib::Response &res) {
    auto ret = get_view();
    res.status = ret.first;
    if (!ret.second.empty()) {
      res.set_content(ret.second, "application/json");
    }
  });
  // srv.set_logger([](const httplib::Request &req, const httplib::Response
  // &res) {
  //   // printf("%s", log(req, res).c_str());
  //   CLOG(INFO, "http") << req.method << ": " << req.path
  //                      << "  Res Code: " << res.status;
  //   // CLOG(INFO, "http") << "http req and rep\n" << log(req, res);
  // });
}

void HTTPServer::listen() {
  CLOG(INFO, http_log) << "listen " << ip << ":" << port;
  srv.listen(ip, port);
}