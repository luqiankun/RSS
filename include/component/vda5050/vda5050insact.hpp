#ifndef VDA5050INSTANTACTION_HPP
#define VDA5050INSTANTACTION_HPP
#include <optional>
#include <variant>

#include "../../3rdparty/jsoncons/json.hpp"
#include "../data/order/driverorder.hpp"

namespace vda5050 {
namespace instantaction {
using ActionType = data::order::DriverOrder::Destination::OpType;

using ActionBlockingType = data::model::Actions::ActionBlockingType;

// action

using ActionParam = data::model::Actions::ActionParam;

class Action : public data::model::Actions::Action {
 public:
  using data::model::Actions::Action ::Action;
  void init(jsoncons::json& obj) override {
    if (obj["actionType"].as_string() == "startPause") {
      action_type = ActionType::STARTPAUSE;
    } else if (obj["actionType"].as_string() == "stopPause") {
      action_type = ActionType::STOPPAUSE;
    } else {
      action_type = ActionType::NOP;
    }
    action_id = obj["actionId"].as_string();
    if (obj.contains("actionDescription")) {
      action_description = obj["actionDescription"].as_string();
    }
    if (obj["blockingType"].as_string() == "NONE") {
      blocking_type = ActionBlockingType::NONE;
    } else if (obj["blockingType"].as_string() == "SOFT") {
      blocking_type = ActionBlockingType::SOFT;
    } else {
      blocking_type = ActionBlockingType::HARD;
    }
    if (obj.contains("actionParameters")) {
      action_parameters = std::vector<ActionParam>();
      for (auto& x : obj["actionParameters"].array_range()) {
        ActionParam param;
        param.key = x["key"].as_string();
        if (x["value"].is_bool()) {
          param.value = x["value"].as_bool();
        } else if (x["value"].is_string()) {
          param.value = x["value"].as_string();
        } else if (x["value"].is_number()) {
          param.value = x["value"].as_double();
        } else if (x["value"].is_array()) {
          param.value = std::vector<std::string>();
          for (auto& str : x["value"].array_range()) {
            std::get<std::vector<std::string>>(param.value)
                .push_back(str.as_string());
          }
        }
        action_parameters->push_back(param);
      }
    }
  }
  jsoncons::json to_json() override {
    jsoncons::json res;
    res["actionId"] = action_id;
    if (action_type == ActionType::STARTPAUSE) {
      res["actionType"] = "startPause";
    } else if (action_type == ActionType::STOPPAUSE) {
      res["actionType"] = "stopPause";
    } else {
      res["actionType"] = "NOP";
    }
    if (blocking_type == ActionBlockingType::HARD) {
      res["blockingType"] = "HARD";
    } else if (blocking_type == ActionBlockingType::NONE) {
      res["blockingType"] = "NONE";
    } else {
      res["blockingType"] = "SOFT";
    }
    if (action_description.has_value()) {
      res["actionDescription"] = action_description.value();
    }
    if (action_parameters.has_value()) {
      res["actionParameters"] = jsoncons::json::array();
      for (auto& x : action_parameters.value()) {
        jsoncons::json p;
        p["key"] = x.key;
        if (x.value.index() == 0) {
          p["value"] = std::get<0>(x.value);
        } else if (x.value.index() == 1) {
          p["value"] = std::get<1>(x.value);
        } else if (x.value.index() == 2) {
          p["value"] = std::get<2>(x.value);
        } else {
          p["value"] = jsoncons::json::array();
          for (auto& str : std::get<3>(x.value)) {
            p["value"].push_back(str);
          }
        }
        res["actionParameters"].push_back(p);
      }
    }
    return res;
  }
};

class InstantAction {
 public:
  int header_id;
  std::string timestamp{};
  std::string version{};
  std::string manufacturer{};
  std::string serial_number{};
  std::vector<Action> actions;
  InstantAction() {}
  explicit InstantAction(jsoncons::json& obj)
      : header_id(obj["headerId"].as_integer<int>()),
        timestamp(obj["timestamp"].as_string()),
        manufacturer(obj["manufacturer"].as_string()),
        serial_number(obj["serialNumber"].as_string()),
        version(obj["version"].as_string()) {
    for (auto& x : obj["actions"].array_range()) {
      auto a = Action();
      a.init(x);
      actions.push_back(a);
    }
  }
  jsoncons::json to_json() {
    jsoncons::json res;
    res["headerId"] = header_id;
    res["version"] = version;
    res["timestamp"] = timestamp;
    res["serialNumber"] = serial_number;
    res["manufacturer"] = manufacturer;
    res["actions"] = jsoncons::json::array();
    for (auto& x : actions) {
      res["actions"].push_back(x.to_json());
    }
    return res;
  }
};

}  // namespace instantaction
}  // namespace vda5050
#endif