#include "../../include/main/httpsrv.hpp"
std::string dump_headers(const httplib::Headers &headers) {
  std::string s;
  for (const auto &h : headers) {
    char buf[BUFSIZ];
    const auto &[header, body] = h;
    snprintf(buf, sizeof(buf), "%s: %s\n", header.c_str(), body.c_str());
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
  if (!req.body.empty()) {
    s += req.body;
  }
  s += "\n";
  s += "--------------------------------\n";

  snprintf(buf, sizeof(buf), "%d %s\n", res.status, res.version.c_str());
  s += buf;
  s += dump_headers(res.headers);
  s += "\n";
  if (!res.body.empty()) {
    s += res.body;
  }
  s += "\n";

  return s;
}
HTTPServer::HTTPServer(const std::string &ip, int port) {
  this->ip = ip;
  this->port = port;
  const std::string url_prefix = "/v1";
  srv.set_socket_options([](socket_t sock) {
    int opt = 1;
    int ret = setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const void *)&opt,
                         sizeof(opt));
    CLOG(INFO, http_log) << "setsockopt SO_REUSEADDR ret: " << ret;
  });
  srv.set_default_headers({{"Access-Control-Allow-Origin", "*"},
                           {"Access-Control-Allow-Credentials", "false"},
                           {"Allow", "GET, POST, HEAD, OPTIONS,PUT"}});
  srv.Get(url_prefix + "/transportOrders",
          [=](const httplib::Request &req, httplib::Response &res) {
            //
            std::string veh;
            if (req.has_param("intendedVehicle")) {
              veh = req.get_param_value("intendedVehicle");
            }
            const auto [id, msg] = get_transport_orders(veh);
            res.status = id;
            if (!msg.empty()) {
              res.set_content(msg, "application/json");
            }
          });
  srv.Get(url_prefix + R"(/transportOrders/([^/]+))",
          [=](const httplib::Request &req, httplib::Response &res) {
            auto m = req.matches.end() - 1;
            std::string ord_name = *m;
            auto ret = get_transport_order(ord_name);
            res.status = ret.first;
            if (!ret.second.empty()) {
              res.set_content(ret.second, "application/json");
            }
          });
  srv.Post(url_prefix + R"(/transportOrders/([^/]+))",
           [=](const httplib::Request &req, httplib::Response &res) {
             auto m = req.matches.end() - 1;
             std::string ord_name = *m;
             auto ret = post_transport_order(ord_name, req.body);
             res.status = ret.first;
             if (!ret.second.empty()) {
               res.set_content(ret.second, "application/json");
             }
           });
  srv.Post(url_prefix + R"(/moveOrder)", [=](const httplib::Request &req,
                                             httplib::Response &res) {
    if (!req.has_param("vehicle")) {
      res.status = httplib::BadRequest_400;
      res.set_content(R"(["not has param 'vehicle'"])", "application/json");
      return;
    }
    if (!req.has_param("point")) {
      res.status = httplib::BadRequest_400;
      res.set_content(R"(["not has param 'point'"])", "application/json");
      return;
    }
    auto ret = post_move_order(req.get_param_value("vehicle"),
                               req.get_param_value("point"));
    res.status = ret.first;
    if (!ret.second.empty()) {
      res.set_content(ret.second, "application/json");
    }
  });
  srv.Post(url_prefix + R"(/transportOrders/([^/]+)/withdrawal)",
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

  srv.Get(url_prefix + R"(/orderSequences)",
          [=](const httplib::Request &req, httplib::Response &res) {
            std::string veh;
            if (req.has_param("intendedVehicle")) {
              veh = req.get_param_value("intendedVehicle");
            }
            const auto [id, s] = get_ordersequences(veh);
            res.status = id;
            if (!s.empty()) {
              res.set_content(s, "application/json");
            }
          });
  srv.Get(url_prefix + R"(/orderSequences/([^/]+))",
          [=](const httplib::Request &req, httplib::Response &res) {
            const auto m = req.matches.end() - 1;
            const std::string veh = *m;
            const auto [code, msg] = get_ordersequence(veh);
            res.status = code;
            if (!msg.empty()) {
              res.set_content(msg, "application/json");
            }
          });
  srv.Post(url_prefix + R"(/orderSequences/([^/]+))",
           [=](const httplib::Request &req, httplib::Response &res) {
             const auto m = req.matches.end() - 1;
             const std::string veh = *m;
             const auto [id, msg] = post_ordersequence(veh, req.body);
             res.status = id;
             if (!msg.empty()) {
               res.set_content(msg, "application/json");
             }
           });
  srv.Get(url_prefix + R"(/vehicles)",
          [=](const httplib::Request &req, httplib::Response &res) {
            std::string state;
            if (req.has_param("procState")) {
              state = req.get_param_value("procState");
            }
            const auto [id, msg] = get_vehicles(state);
            res.status = id;
            if (!msg.empty()) {
              res.set_content(msg, "application/json");
            }
          });
  srv.Get(url_prefix + R"(/vehicles/([^/]+))",
          [=](const httplib::Request &req, httplib::Response &res) {
            const std::string state = *(req.matches.end() - 1);
            const auto [id, msg] = get_vehicle(state);
            res.status = id;
            if (!msg.empty()) {
              res.set_content(msg, "application/json");
            }
          });
  srv.Put(url_prefix + R"(/vehicles/([^/]+)/paused)",
          [=](const httplib::Request &req, httplib::Response &res) {
            const std::string name = *(req.matches.end() - 1);
            bool state{false};
            if (req.has_param("newValue")) {
              state = req.get_param_value("newValue") == "true" ? true : false;
            }
            const auto [id, msg] = put_vehicle_paused(name, state);
            res.status = id;
            if (!msg.empty()) {
              res.set_content(msg, "application/json");
            }
          });
  srv.Options(url_prefix + R"(/vehicles/([^/]+)/paused)",
              [=](const httplib::Request &req, httplib::Response &res) {
                res.set_header("Access-Control-Allow-Methods", "PUT");
                res.status = 200;
              });
  srv.Post(url_prefix + R"(/vehicles/([^/]+)/withdrawal)",
           [=](const httplib::Request &req, httplib::Response &res) {
             const std::string name = *(req.matches.end() - 1);
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

  srv.Get(url_prefix + R"(/plantModel)",
          [=](const httplib::Request &, httplib::Response &res) {
            auto ret = get_model();
            res.status = ret.first;
            if (!ret.second.empty()) {
              res.set_content(ret.second, "application/json");
            }
          });
  srv.Put(url_prefix + R"(/plantModel)",
          [=](const httplib::Request &req, httplib::Response &res) {
            bool use_xml{false};
            if (req.has_param("type")) {
              use_xml = req.get_param_value("type") == "xml" ? true : false;
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
  srv.Options(url_prefix + R"(/plantModel)",
              [=](const httplib::Request &req, httplib::Response &res) {
                res.set_header("Access-Control-Allow-Methods", "PUT");
                res.set_header("Access-Control-Allow-Headers", "Content-Type");
                res.status = 200;
              });
  srv.Put(url_prefix + R"(/paths/([^/]+)/locked)",
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
  srv.Options(url_prefix + R"(/paths/([^/]+)/locked)",
              [=](const httplib::Request &req, httplib::Response &res) {
                res.set_header("Access-Control-Allow-Methods", "PUT");
                res.status = 200;
              });
  srv.Put(url_prefix + R"(/locations/([^/]+)/locked)",
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
  srv.Options(url_prefix + R"(/locations/([^/]+)/locked)",
              [=](const httplib::Request &req, httplib::Response &res) {
                res.set_header("Access-Control-Allow-Methods", "PUT");
                res.status = 200;
              });
  srv.Get(url_prefix + "/getView",
          [=](const httplib::Request &req, httplib::Response &res) {
            auto ret = get_view();
            res.status = ret.first;
            if (!ret.second.empty()) {
              res.set_content(ret.second, "application/json");
            }
          });
  srv.Post(url_prefix + "/plantModel/topologyUpdateRequest",
           [=](const httplib::Request &req, httplib::Response &res) {
             auto ret = post_reroute();
             res.status = ret.first;
             if (!ret.second.empty()) {
               res.set_content(ret.second, "application/json");
             }
           });
  srv.Post(url_prefix + R"(/vehicles/([^/]+)/rerouteRequest)",
           [=](const httplib::Request &req, httplib::Response &res) {
             bool forced =
                 req.get_param_value("forced") == "true" ? true : false;
             std::string name = *(req.matches.end() - 1);
             auto ret = post_vheicle_reroute(name, forced);
             res.status = ret.first;
             if (!ret.second.empty()) {
               res.set_content(ret.second, "application/json");
             }
           });
  srv.Put(url_prefix + R"(/vehicles/([^/]+)/commAdapter/enabled)",
          [=](const httplib::Request &req, httplib::Response &res) {
            bool f = req.get_param_value("newValue") == "true" ? true : false;
            std::string name = *(req.matches.end() - 1);
            auto ret = put_vehicle_enabled(name, f);
            res.status = ret.first;
            if (!ret.second.empty()) {
              res.set_content(ret.second, "application/json");
            }
          });
  srv.Options(url_prefix + R"(/vehicles/([^/]+)/commAdapter/enabled)",
              [=](const httplib::Request &req, httplib::Response &res) {
                res.set_header("Access-Control-Allow-Methods", "PUT");
                res.status = 200;
              });
  srv.Put(url_prefix + R"(/vehicles/([^/]+)/integrationLevel)",
          [=](const httplib::Request &req, httplib::Response &res) {
            std::string f = req.get_param_value("newValue");
            std::string name = *(req.matches.end() - 1);
            auto ret = put_vehicle_integration_level(name, f);
            res.status = ret.first;
            if (!ret.second.empty()) {
              res.set_content(ret.second, "application/json");
            }
          });
  srv.Post(url_prefix + R"(/vehicles/([^/]+)/routeComputationQuery)",
           [=](const httplib::Request &req, httplib::Response &res) {
             std::string f = req.body;
             std::string name = *(req.matches.end() - 1);
             auto ret = post_vehicle_path_to_point(name, f);
             res.status = ret.first;
             if (!ret.second.empty()) {
               res.set_content(ret.second, "application/json");
             }
           });
  srv.set_logger([](const httplib::Request &req, const httplib::Response &res) {
    // printf("%s", log(req, res).c_str());
    // CLOG_IF(res.status != httplib::OK_200, DEBUG, "http")
    //     << req.remote_addr << ":" << req.remote_port << "| " << req.method
    //     << " " << req.path << " | Res Code: " << res.status << "\n";
    // CLOG_IF(res.status == httplib::OK_200, DEBUG, "http")
    //     << req.remote_addr << ":" << req.remote_port << "| " << req.method
    //     << " " << req.path << " | Res Code: " << res.status << "\n";
    CLOG(DEBUG, "http") << "http req and rep\n" << log(req, res);
  });
}

void HTTPServer::listen() {
  if (!srv.listen(ip, port)) {
    throw std::runtime_error("listen error");
  }
  CLOG(INFO, http_log) << "listen " << ip << ":" << port << "\n";
}