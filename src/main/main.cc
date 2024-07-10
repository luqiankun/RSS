#include <Eigen/Eigen>
#include <csignal>

#include "../../include/component/tools/fs/filesystem.hpp"
#include "../../include/component/tools/log/easylogging++.h"
#include "../../include/component/tools/yaml/yaml.hpp"
#include "../../include/main/httpsrv.hpp"
#include "../../include/main/tcs.hpp"
INITIALIZE_EASYLOGGINGPP
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
std::string init_xml_path{""};
// mqtt value
std::string mqtt_ip{"127.0.0.1"};
int mqtt_port{1883};

// http value

std::string http_ip{"0.0.0.0"};
int http_port{55200};

std::regex reg{R"(^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$)"};

void read_params(std::string path) {
  Yaml::Node root;
  try {
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
    if (!root["tcs"]["auto_init"]["enable"].IsNone()) {
      init_enable = root["tcs"]["auto_init"]["enable"].As<bool>();
    }
    if (!root["tcs"]["auto_init"]["xml_path"].IsNone()) {
      init_xml_path = root["tcs"]["auto_init"]["xml_path"].As<std::string>();
    }
    if (!root["tcs"]["mqtt_addr"]["host"].IsNone()) {
      init_xml_path = root["tcs"]["mqtt_addr"]["host"].As<std::string>();
    }
    if (!root["tcs"]["mqtt_addr"]["port"].IsNone()) {
      init_xml_path = root["tcs"]["mqtt_addr"]["port"].As<int>();
    }
    CLOG(INFO, tcs_log) << "read param success";
  } catch (Yaml::Exception& ec) {
    CLOG(ERROR, tcs_log) << "load param from <" << path << "> failed :"
                         << " " << ec.Message() << ", will use deafult params.";
  }
}

int main(int argc, char** argv) {
  el::Loggers::getLogger("tcs");
  el::Loggers::getLogger("timer");
  el::Loggers::getLogger("order");
  el::Loggers::getLogger("visual");
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
  conf.set(el ::Level ::Debug, el ::ConfigurationType ::Format,
           "%datetime{%d/%M} %func [%fbase:%line] %msg");
  el ::Loggers ::reconfigureAllLoggers(conf);
  std::deque<std::string> logs_name;
  {
    try {
      auto dir_it = ghc::filesystem::directory_iterator(log_path);
      std::vector<ghc::filesystem::directory_entry> ns;
      for (auto& x : dir_it) {
        if (!x.is_directory() && x.path().filename() != "tcs_main.log") {
          ns.push_back(x);
        }
      }
      std::sort(ns.begin(), ns.end(),
                [](ghc::filesystem::directory_entry& a,
                   ghc::filesystem::directory_entry& b) {
                  return a.last_write_time() < b.last_write_time();
                });
      for (auto& x : ns) {
        // LOG(INFO) << x.path().filename();
        logs_name.push_back(x.path().filename());
      }
      while (logs_name.size() > log_max_num) {
        auto obj = logs_name.front();
        logs_name.pop_front();
        ghc::filesystem::remove(ghc::filesystem::path(log_path) / obj);
      }
    } catch (std::exception& ec) {
      LOG(WARNING) << ec.what();
    }
  }
  el::Helpers::installPreRollOutCallback(
      [&](const char* filename, std::size_t size) {
        auto dest = get_log_path(log_path);
        logs_name.push_back(dest);
        ghc::filesystem::copy(get_log_name(log_path), dest);
        while (logs_name.size() > log_max_num) {
          auto obj = logs_name.front();
          logs_name.pop_front();
          ghc::filesystem::remove(ghc::filesystem::path(log_path) / obj);
        }
      });
  auto tcs = std::make_shared<TCS>(mqtt_ip, mqtt_port);
  auto srv = std::make_shared<HTTPServer>();
  {
    std::smatch m;
    if (!std::regex_match(http_ip, reg)) {
      CLOG(ERROR, tcs_log) << "ip's format is wrong";
      return -1;
    }
    srv->ip = http_ip;
    srv->port = http_port;
  }

  // bind
  {
    srv->get_transport_order =
        std::bind(&TCS::get_transport_order, tcs, std::placeholders::_1);
    srv->get_transport_orders =
        std::bind(&TCS::get_transport_orders, tcs, std::placeholders::_1);
    srv->post_transport_order =
        std::bind(&TCS::post_transport_order, tcs, std::placeholders::_1,
                  std::placeholders::_2);
    srv->post_transport_order_withdrawl = std::bind(
        &TCS::post_transport_order_withdrawl, tcs, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3);
    srv->get_ordersequences =
        std::bind(&TCS::get_ordersequences, tcs, std::placeholders::_1);
    srv->get_ordersequence =
        std::bind(&TCS::get_ordersequence, tcs, std::placeholders::_1);
    srv->post_ordersequence =
        std::bind(&TCS::post_ordersequence, tcs, std::placeholders::_1,
                  std::placeholders::_2);
    srv->get_vehicles =
        std::bind(&TCS::get_vehicles, tcs, std::placeholders::_1);
    srv->get_vehicle = std::bind(&TCS::get_vehicle, tcs, std::placeholders::_1);
    srv->post_vehicle_withdrawl =
        std::bind(&TCS::post_vehicle_withdrawl, tcs, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3);
    srv->get_model = std::bind(&TCS::get_model, tcs);
    srv->put_model = std::bind(&TCS::put_model, tcs, std::placeholders::_1);
    srv->put_model_xml =
        std::bind(&TCS::put_model_xml, tcs, std::placeholders::_1);
    srv->put_path_locked =
        std::bind(&TCS::put_path_locked, tcs, std::placeholders::_1,
                  std::placeholders::_2);
    srv->put_location_locked =
        std::bind(&TCS::put_location_locked, tcs, std::placeholders::_1,
                  std::placeholders::_2);
    srv->get_view = std::bind(&TCS::get_view, tcs);
    srv->put_vehicle_paused =
        std::bind(&TCS::put_vehicle_paused, tcs, std::placeholders::_1,
                  std::placeholders::_2);
    srv->post_reroute = std::bind(&TCS::post_reroute, tcs);
    srv->post_vheicle_reroute =
        std::bind(&TCS::post_vehicle_reroute, tcs, std::placeholders::_1,
                  std::placeholders::_2);
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
    CLOG(INFO, tcs_log) << "auto init using " << init_xml_path;
    auto ret = doc.load_file(init_xml_path.c_str());
    if (ret.status == pugi::xml_parse_status::status_ok) {
      std::stringstream body;
      doc.save(body);
      tcs->put_model_xml(body.str());
    } else {
      CLOG(ERROR, tcs_log) << "parse xml failed: " << ret.description();
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  CLOG(INFO, tcs_log) << "service start, press 'Ctrl + C' to exit.";
  if (th_wait.joinable()) {
    th_wait.join();
  }
  CLOG(INFO, tcs_log) << "service shutdown";
}