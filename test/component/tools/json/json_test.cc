#include "../../../../include/3rdparty/jsoncons/json.hpp"

#include <locale>
#include <regex>

#include "../../../../include/3rdparty/log/easylogging++.h"
#include "../../../../include/3rdparty/utest/utest.h"
#include "../../../../include/component/util/timer.hpp"

INITIALIZE_EASYLOGGINGPP

UTEST(json_parse, case1) {
  auto msg1 = "321312dasda{a_2d}%^";
  auto msg2 = R"({"name":12})";
  EXPECT_EXCEPTION(jsoncons::json v = jsoncons::json::parse(msg1);
                   , jsoncons::ser_error);
  jsoncons::json v2 = jsoncons::json::parse(msg2);
  auto res = jsoncons::json();
  auto msg = "Could not parse JSON input.";
  res.push_back(msg);
  std::string t;
  res.dump(t);
  std::cout << t << "\n";
}
UTEST(time_parse, case) {
  std::stringstream timeStr{"2021-09-30T15:46:37.394Z"};  // 要解析的时间字符串
  std::regex reg{R"((\d{4}-\d{2}-\d{2})T(\d{2}:\d{2}:\d{2}).(\d{1,10})Z)"};
  std::cmatch match;
  std::regex_search("2021-09-30T15:46:37.Z", match, reg);
  {
    for (auto it : match) {
      std::cout << it.str() << std::endl;
    }
  }
  try {
    std::istringstream ss("2021-09-30T15:46:37.394Z");

    std::tm t = {};
    ss >> std::get_time(&t, "%Y-%m-%dT%H:%M:%S");
    std::chrono::milliseconds ms(std::stoi(*(match.end() - 1)));
    // Convert to time_point
    long timeSinceEpoch = std::mktime(&t);
    auto duration = std::chrono::seconds(timeSinceEpoch);
    auto time_point = std::chrono::system_clock::time_point(duration);
    std::cout << "time_point: " << get_time_fmt(time_point + ms) << std::endl;

  } catch (const std::exception& e) {
    std::cerr << "Error parsing the string as a time point: " << e.what()
              << std::endl;
  }
}

UTEST_MAIN();
