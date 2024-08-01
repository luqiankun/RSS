
#include "../../../include/3rdparty/http/httplib.h"
using namespace httplib;

std::string dump_headers(const Headers &headers) {
  std::string s;
  char buf[BUFSIZ];

  for (auto it = headers.begin(); it != headers.end(); ++it) {
    const auto &x = *it;
    snprintf(buf, sizeof(buf), "%s: %s\n", x.first.c_str(), x.second.c_str());
    s += buf;
  }

  return s;
}

std::string log(const Request &req, const Response &res) {
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
  s += "\r\n";
  s += req.body + "\n";

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

int main(void) {
#ifdef CPPHTTPLIB_OPENSSL_SUPPORT
  SSLServer svr(SERVER_CERT_FILE, SERVER_PRIVATE_KEY_FILE);
#else
  Server svr;
#endif

  if (!svr.is_valid()) {
    printf("server has an error...\n");
    return -1;
  }

  svr.Get("/", [=](const Request & /*req*/, Response &res) {
    res.set_redirect("/hi");
  });
  svr.Get("/transportOrders", [=](const Request &req, Response &res) {
    //
    if (req.has_param("transportOrders")) {
      auto ord = req.get_param_value("transportOrders");
      res.status = 400;
      res.set_content(ord, "application/json");
    } else {
      res.status = 404;
      res.set_content("", "application/json");
    }
  });
  svr.Get(R"(/hi/([^/]+))", [](const Request &req, Response &res) {
    auto m = req.matches[0];
    for (auto &m : req.matches) {
      std::cout << m.str() << "\n";
    }
    res.set_content("ok\n", "text/plain");
  });
  svr.Get(R"(/hi/([^/]+)/withdrawl)", [](const Request &req, Response &res) {
    auto m = req.matches[0];
    for (auto &m : req.matches) {
      std::cout << m.str() << "\n";
    }
    res.set_content("okd\n", "text/plain");
  });
  svr.Get("/slow", [](const Request & /*req*/, Response &res) {
    std::this_thread::sleep_for(std::chrono::seconds(2));
    res.set_content("Slow...\n", "text/plain");
  });

  svr.Get("/dump", [](const Request &req, Response &res) {
    res.set_content(dump_headers(req.headers), "text/plain");
  });

  svr.Get("/stop",
          [&](const Request & /*req*/, Response & /*res*/) { svr.stop(); });

  //   svr.set_error_handler([](const Request & /*req*/, Response &res) {
  //     const char *fmt =
  //         "<p>Error Status: <spanstyle='color:red;'>%d</spanstyle=></p>";
  //     char buf[BUFSIZ];
  //     snprintf(buf, sizeof(buf), fmt, res.status);
  //     res.set_content(buf, "text/html");
  //   });

  svr.set_logger([](const Request &req, const Response &res) {
    printf("%s", log(req, res).c_str());
  });

  svr.listen("localhost", 55200);

  return 0;
}