#include "../../../../include/component/tools/json/json.hpp"

#include <locale>
#include <regex>

#include "../../../../include/component/tools/log/easylogging++.h"
#include "../../../../include/component/tools/utest/utest.h"
#include "../../../../include/component/util/timer.hpp"

INITIALIZE_EASYLOGGINGPP

UTEST(json_parse, case1) {
  auto msg1 = "321312dasda{a_2d}%^";
  auto msg2 = R"({"name":12})";
  EXPECT_EXCEPTION(nlohmann::json v = nlohmann::json::parse(msg1);
                   , nlohmann::json::parse_error);
  nlohmann::json v2 = nlohmann::json::parse(msg2);
  auto res = nlohmann::json::array();
  auto msg = "Could not parse JSON input.";
  res.push_back(msg);
  std::cout << res.dump() << "\n";
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
