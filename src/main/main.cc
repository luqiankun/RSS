#include <Eigen/Eigen>
#include <csignal>

#include "../../include/3rdparty/fs/filesystem.hpp"
#include "../../include/3rdparty/log/easylogging++.h"
#include "../../include/3rdparty/yaml/yaml.hpp"
#include "../../include/main/httpsrv.hpp"
#include "../../include/main/rss.hpp"
INITIALIZE_EASYLOGGINGPP
#define ELPP_UNICODE
// signal
void (*signal(int sig, void (*func)(int)))(int);
std::condition_variable con;
std::mutex mut;
void signalHandler(int) { con.notify_one(); }

// log value
std::string log_enable{"true"};
std::string log_to_file{"false"};
std::string log_to_stdout{"true"};
std::string log_path{"logs"};
std::string log_fmt{"[%level] %datetime %fbase:%line] %msg"};
std::string log_size{"10485760"};
uint32_t log_max_num{5};
// auto init
bool init_enable{false};
std::string log_level{"info"};
std::string init_xml_path{""};
// mqtt value
std::string mqtt_ip{"192.168.0.39"};
int mqtt_port{1883};

// http value

std::string http_ip{"0.0.0.0"};
int http_port{55200};

std::regex reg{R"(^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$)"};

void read_params(const std::string &path) {
  try {
    Yaml::Node root;
    Yaml::Parse(root, path.c_str());
    if (!root["httpserver"]["ip"].IsNone()) {
      http_ip = root["httpserver"]["ip"].As<std::string>();
    }
    if (!root["httpserver"]["port"].IsNone()) {
      http_port = root["httpserver"]["port"].As<int>();
    }
    if (!root["log"]["enable"].IsNone()) {
      log_enable = root["log"]["enable"].As<bool>() ? "true" : "false";
    }
    if (!root["log"]["level"].IsNone()) {
      log_level = root["log"]["level"].As<std::string>();
    }
    if (!root["log"]["max_num"].IsNone()) {
      log_max_num = root["log"]["max_num"].As<uint32_t>();
    }
    if (!root["log"]["to_file"].IsNone()) {
      log_to_file = root["log"]["to_file"].As<bool>() ? "true" : "false";
    }
    if (!root["log"]["to_stdout"].IsNone()) {
      log_to_stdout = root["log"]["to_stdout"].As<bool>() ? "true" : "false";
    }
    if (!root["log"]["path"].IsNone()) {
      log_path = root["log"]["path"].As<std::string>();
    }
    if (!root["log"]["fmt"].IsNone()) {
      log_fmt = root["log"]["fmt"].As<std::string>();
    }
    if (!root["log"]["size"].IsNone()) {
      log_size = root["log"]["size"].As<std::string>();
    }
    if (!root["rss"]["auto_init"]["enable"].IsNone()) {
      init_enable = root["rss"]["auto_init"]["enable"].As<bool>();
    }
    if (!root["rss"]["auto_init"]["xml_path"].IsNone()) {
      init_xml_path = root["rss"]["auto_init"]["xml_path"].As<std::string>();
    }
    if (!root["rss"]["mqtt_addr"]["host"].IsNone()) {
      mqtt_ip = root["rss"]["mqtt_addr"]["host"].As<std::string>();
    }
    if (!root["rss"]["mqtt_addr"]["port"].IsNone()) {
      mqtt_port = root["rss"]["mqtt_addr"]["port"].As<int>();
    }
    CLOG(INFO, rss_log) << "read yaml param success\n";
  } catch (Yaml::Exception &ec) {
    CLOG(ERROR, rss_log) << "load param from <" << path << "> failed :" << " "
                         << ec.Message() << ", will use default params.";
  }
}

int main(int argc, char **argv) {
  el::Loggers::getLogger("rss");
  el::Loggers::getLogger("timer");
  el::Loggers::getLogger("order");
  el::Loggers::getLogger("http");
  el::Loggers::getLogger("driver");
  el::Loggers::getLogger("dispatch");
  el::Loggers::getLogger("planner");
  el::Loggers::getLogger("schedule");
  el::Loggers::getLogger("allocate");
  el::Loggers::getLogger("mqtt");
  std::string path{"config/config.yaml"};
  if (argc >= 2) {
    path = std::string(argv[1]);
  }
  read_params(path);
  el ::Configurations conf;
  el::Loggers::addFlag(el::LoggingFlag::StrictLogFileSizeCheck);
  conf.setGlobally(el ::ConfigurationType ::Format, log_fmt);
  conf.setGlobally(el ::ConfigurationType ::Filename,
                   "" + get_log_name(log_path));
  conf.setGlobally(el ::ConfigurationType ::ToFile, log_to_file);
  conf.setGlobally(el ::ConfigurationType ::Enabled, log_enable);
  conf.setGlobally(el ::ConfigurationType ::ToStandardOutput, log_to_stdout);
  conf.setGlobally(el ::ConfigurationType ::MillisecondsWidth, "4");
  conf.setGlobally(el ::ConfigurationType ::MaxLogFileSize, log_size);
  // conf.set(el ::Level ::Debug, el ::ConfigurationType ::Format,
  //          "%datetime{%d/%M} %func [%fbase:%line] %msg");
  if (log_level == "debug") {
    conf.set(el::Level::Trace, el ::ConfigurationType ::Enabled, "false");
  } else if (log_level == "info") {
    conf.set(el::Level::Trace, el ::ConfigurationType ::Enabled, "false");
    conf.set(el::Level::Debug, el ::ConfigurationType ::Enabled, "false");
  } else if (log_level == "warn") {
    conf.set(el::Level::Trace, el ::ConfigurationType ::Enabled, "false");
    conf.set(el::Level::Debug, el ::ConfigurationType ::Enabled, "false");
    conf.set(el::Level::Info, el ::ConfigurationType ::Enabled, "false");
  } else if (log_level == "error") {
    conf.set(el::Level::Trace, el ::ConfigurationType ::Enabled, "false");
    conf.set(el::Level::Debug, el ::ConfigurationType ::Enabled, "false");
    conf.set(el::Level::Info, el ::ConfigurationType ::Enabled, "false");
    conf.set(el::Level::Warning, el ::ConfigurationType ::Enabled, "false");
  } else if (log_level == "fatal") {
    conf.set(el::Level::Trace, el ::ConfigurationType ::Enabled, "false");
    conf.set(el::Level::Debug, el ::ConfigurationType ::Enabled, "false");
    conf.set(el::Level::Info, el ::ConfigurationType ::Enabled, "false");
    conf.set(el::Level::Warning, el ::ConfigurationType ::Enabled, "false");
    conf.set(el::Level::Error, el ::ConfigurationType ::Enabled, "false");
  } else {
    conf.set(el::Level::Global, el ::ConfigurationType ::Enabled, "false");
  }
  el ::Loggers ::reconfigureAllLoggers(conf);
  std::deque<std::string> logs_name;
  {
    try {
      auto dir_it = ghc::filesystem::directory_iterator(log_path);
      std::vector<ghc::filesystem::directory_entry> ns;
      for (auto &x : dir_it) {
        if (!x.is_directory() && x.path().filename() != "rss.log") {
          ns.push_back(x);
        }
      }
      std::sort(ns.begin(), ns.end(),
                [](const ghc::filesystem::directory_entry &a,
                   const ghc::filesystem::directory_entry &b) {
                  return a.last_write_time() < b.last_write_time();
                });
      for (auto &x : ns) {
        // LOG(INFO) << x.path().filename();
        logs_name.push_back(x.path().filename());
      }
      while (logs_name.size() > log_max_num) {
        auto obj = logs_name.front();
        logs_name.pop_front();
        ghc::filesystem::remove(ghc::filesystem::path(log_path) / obj);
      }
    } catch (std::exception &ec) {
      LOG(WARNING) << ec.what();
    }
  }
  el::Helpers::installPreRollOutCallback(
      [&](const char *filename, std::size_t size) {
        auto dest = get_log_path(log_path);
        logs_name.push_back(dest);
        ghc::filesystem::copy(get_log_name(log_path), dest);
        while (logs_name.size() > log_max_num) {
          auto obj = logs_name.front();
          logs_name.pop_front();
          ghc::filesystem::remove(ghc::filesystem::path(log_path) / obj);
        }
      });
  auto rss = std::make_shared<RSS>();
  auto srv = std::make_shared<HTTPServer>();
  {
    MQTT_IP = mqtt_ip;
    MQTT_PORT = mqtt_port;
  }
  {
    std::smatch m;
    if (!std::regex_match(http_ip, reg)) {
      CLOG(ERROR, rss_log) << "ip is format is wrong";
      return -1;
    }
    srv->ip = http_ip;
    srv->port = http_port;
  }

  // bind
  {
    srv->get_transport_order = std::bind(&RSS::get_transport_order,
                                         std::ref(rss), std::placeholders::_1);
    srv->get_transport_orders = std::bind(&RSS::get_transport_orders,
                                          std::ref(rss), std::placeholders::_1);
    srv->post_transport_order =
        std::bind(&RSS::post_transport_order, std::ref(rss),
                  std::placeholders::_1, std::placeholders::_2);
    srv->post_move_order =
        std::bind(&RSS::post_move_order, std::ref(rss), std::placeholders::_1,
                  std::placeholders::_2);
    srv->post_transport_order_withdrawl = std::bind(
        &RSS::post_transport_order_withdrawl, std::ref(rss),
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    srv->get_ordersequences = std::bind(&RSS::get_ordersequences, std::ref(rss),
                                        std::placeholders::_1);
    srv->get_ordersequence = std::bind(&RSS::get_ordersequence, std::ref(rss),
                                       std::placeholders::_1);
    srv->post_ordersequence =
        std::bind(&RSS::post_ordersequence, std::ref(rss),
                  std::placeholders::_1, std::placeholders::_2);
    srv->get_vehicles =
        std::bind(&RSS::get_vehicles, std::ref(rss), std::placeholders::_1);
    srv->get_vehicle =
        std::bind(&RSS::get_vehicle, std::ref(rss), std::placeholders::_1);
    srv->post_vehicle_withdrawl = std::bind(
        &RSS::post_vehicle_withdrawl, std::ref(rss), std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3);
    srv->get_model = std::bind(&RSS::get_model, std::ref(rss));
    srv->put_model =
        std::bind(&RSS::put_model, std::ref(rss), std::placeholders::_1);
    srv->put_model_xml =
        std::bind(&RSS::put_model_xml, std::ref(rss), std::placeholders::_1);
    srv->put_path_locked =
        std::bind(&RSS::put_path_locked, std::ref(rss), std::placeholders::_1,
                  std::placeholders::_2);
    srv->put_location_locked =
        std::bind(&RSS::put_location_locked, std::ref(rss),
                  std::placeholders::_1, std::placeholders::_2);
    srv->get_view = std::bind(&RSS::get_view, std::ref(rss));
    srv->put_vehicle_paused =
        std::bind(&RSS::put_vehicle_paused, std::ref(rss),
                  std::placeholders::_1, std::placeholders::_2);
    srv->post_reroute = std::bind(&RSS::post_reroute, std::ref(rss));
    srv->post_vheicle_reroute =
        std::bind(&RSS::post_vehicle_reroute, std::ref(rss),
                  std::placeholders::_1, std::placeholders::_2);
    srv->put_vehicle_enabled =
        std::bind(&::RSS::put_vehicle_enable, std::ref(rss),
                  std::placeholders::_1, std::placeholders::_2);
    srv->put_vehicle_integration_level =
        std::bind(&::RSS::put_vehicle_integration_level, std::ref(rss),
                  std::placeholders::_1, std::placeholders::_2);
    srv->post_vehicle_path_to_point =
        std::bind(&::RSS::post_vehicle_path_to_point, std::ref(rss),
                  std::placeholders::_1, std::placeholders::_2);
  }
  std::thread th{[&] { srv->listen(); }};
  signal(SIGINT, signalHandler);
  std::thread th_wait{[&] {
    while (true) {
      std::unique_lock<std::mutex> lock(mut);
      con.wait(lock);
      break;
    }
    if (srv->srv.is_running()) {
      srv->srv.stop();
    }
    if (th.joinable()) {
      th.join();
    }
  }};
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  if (init_enable) {
    pugi::xml_document doc;
    CLOG(INFO, rss_log) << "auto init using " << init_xml_path;
    auto ret = doc.load_file(init_xml_path.c_str());
    if (ret.status == pugi::xml_parse_status::status_ok) {
      std::stringstream body;
      doc.save(body);
      rss->put_model_xml(body.str());
    } else {
      CLOG(ERROR, rss_log) << "parse xml failed: " << ret.description();
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  CLOG(INFO, rss_log) << "service start, press 'Ctrl + C' to exit.\n";
  assert(srv->srv.is_running());
  if (th_wait.joinable()) {
    th_wait.join();
  }
  CLOG(INFO, rss_log) << "service shutdown\n";
}