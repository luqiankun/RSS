#include <csignal>

#include "../../include/component/tools/log/easylogging++.h"
#include "../../include/main/httpsrv.hpp"
#include "../../include/main/tcs.hpp"
void (*signal(int sig, void (*func)(int)))(int);
std::condition_variable con;
std::mutex mut;
void signalHandler(int) { con.notify_one(); }

INITIALIZE_EASYLOGGINGPP
int main(int argc, const char** argv) {
  MY_EASYLOG_CONIFG("./tcs_logs")
  auto now = get_time_fmt(std::chrono::system_clock::now());
  LOG(INFO)
      << "start.....\n"
      << now
      << "\n-----------------------------------------------------------\n";
  auto tcs = std::make_shared<TCS>();
  auto srv = std::make_shared<HTTPServer>();
  if (true) {
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
    if (th_wait.joinable()) {
      th_wait.join();
    }
  } else {
    LOG(ERROR) << "init failed";
  }
}