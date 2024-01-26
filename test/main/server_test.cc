
#include <boost/asio.hpp>

#include "../../include/component/tools/log/easylogging++.h"
#include "../../include/main/httpsrv.hpp"
#include "../../include/main/tcs.hpp"
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
    srv->get_transport_order.connect(
        std::bind(&TCS::get_transport_order, tcs, std::placeholders::_1));
    srv->get_transport_orders.connect(
        std::bind(&TCS::get_transport_orders, tcs, std::placeholders::_1));
    srv->post_transport_order.connect(std::bind(&TCS::post_transport_order, tcs,
                                                std::placeholders::_1,
                                                std::placeholders::_2));
    srv->post_transport_order_withdrawl.connect(std::bind(
        &TCS::post_transport_order_withdrawl, tcs, std::placeholders::_1,
        std::placeholders::_2, std::placeholders::_3));
    srv->get_ordersequences.connect(
        std::bind(&TCS::get_ordersequences, tcs, std::placeholders::_1));
    srv->get_ordersequence.connect(
        std::bind(&TCS::get_ordersequence, tcs, std::placeholders::_1));
    srv->post_ordersequence.connect(std::bind(&TCS::post_ordersequence, tcs,
                                              std::placeholders::_1,
                                              std::placeholders::_2));
    srv->get_vehicles.connect(
        std::bind(&TCS::get_vehicles, tcs, std::placeholders::_1));
    srv->get_vehicle.connect(
        std::bind(&TCS::get_vehicle, tcs, std::placeholders::_1));
    srv->post_vehicle_withdrawl.connect(
        std::bind(&TCS::post_vehicle_withdrawl, tcs, std::placeholders::_1,
                  std::placeholders::_2, std::placeholders::_3));
    srv->get_model.connect(std::bind(&TCS::get_model, tcs));
    srv->put_model.connect(
        std::bind(&TCS::put_model, tcs, std::placeholders::_1));
    srv->put_model_xml.connect(
        std::bind(&TCS::put_model_xml, tcs, std::placeholders::_1));
    srv->put_path_locked.connect(std::bind(&TCS::put_path_locked, tcs,
                                           std::placeholders::_1,
                                           std::placeholders::_2));
    srv->put_location_locked.connect(std::bind(&TCS::put_location_locked, tcs,
                                               std::placeholders::_1,
                                               std::placeholders::_2));
    std::thread th{[&] { srv->listen(); }};
    boost::asio::io_service io;
    boost::asio::signal_set sig(io);
    sig.add(SIGINT);
    sig.add(SIGTERM);
    sig.async_wait([&](const boost::system::error_code& err, int signal) {
      if (srv->srv.is_running()) {
        srv->srv.stop();
      }
    });
    io.run();
    th.join();
    LOG(INFO) << "all stop";
  } else {
    LOG(ERROR) << "init failed";
  }
}