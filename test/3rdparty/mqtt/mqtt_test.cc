#include <regex>

#include "../../../include/component/vda5050/master.hpp"

int main() {
  std::string addr = "tcp://192.168.0.39:1883";
  mqtt::connect_options ops;
  ops.set_connect_timeout(2);
  ops.set_automatic_reconnect(true);
  ops.set_keep_alive_interval(10);
  ops.set_user_name("agv1");
  mqtt::will_options will_ops;
  will_ops.set_topic("/agv1");
  will_ops.set_qos(0);
  will_ops.set_payload(std::string("DDDD"));
  ops.set_will(will_ops);
  vda5050::MqttClient cli{addr, "dada"};
  cli.set_connected_handler([&](std::string cause) {
    std::cout << "on_connect "
              << "\n";
    cli.subscribe("/#", 0);
  });
  cli.set_connection_lost_handler(
      [](const std::string p) { std::cout << "dis " << p << '\n'; });
  cli.on();
  cli.connect(ops);
  cli.set_func("/simvga.*", [](mqtt::const_message_ptr t) {
    std::cout << t->to_string() << "\n";
  });
  std::cin.get();

  cli.disconnect()->wait();

  std::cin.get();
}