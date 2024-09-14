#ifndef ACTION_HGPP
#define ACTION_HGPP
#include <optional>
#include <regex>
#include <variant>

#include "../../../3rdparty/jsoncons/basic_json.hpp"
#include "../../util/tools.hpp"

// #include "../order/driverorder.hpp"
namespace data::model {
class Actions {
public:
  enum class OpType {
    NOP,    // 啥也不干
    LOAD,   // 去某location load
    UNLOAD, // 去某location unload
    MOVE,   // 去某point
    CHARGE, // 充电
    CLOSE,
    OPEN,
    LIFT,
    DROP,
    PICK,
    PARK, // 停靠在某个location
    STARTPAUSE,
    STOPPAUSE
  };
  enum class ActionBlockingType { NONE = 1, SOFT = 2, HARD = 3 };
  class ActionParam {
  public:
    std::string key;
    std::variant<std::string, bool, double, std::vector<std::string>> value;
  };
  // 自定义
  enum class ActionWhen { ORDER_START = 1, ORDER_END = 2 };
  static std::optional<OpType> get_optype(const std::string &op) {
    auto str1 = op;
    std::transform(str1.begin(), str1.end(), str1.begin(), ::tolower);
    if (str1 == "nop") {
      return OpType::NOP;
    } else if (str1 == "pick") {
      return OpType::PICK;
    } else if (str1 == "load") {
      return OpType::LOAD;
    } else if (str1 == "unload") {
      return OpType::UNLOAD;
    } else if (str1 == "drop") {
      return OpType::DROP;
    } else if (str1 == "close") {
      return OpType::CLOSE;
    } else if (str1 == "open") {
      return OpType::OPEN;
    } else if (str1 == "lift") {
      return OpType::LIFT;
    } else if (str1 == "charge") {
      return OpType::CHARGE;
    } else if (str1 == "move") {
      return OpType::MOVE;
    } else if (str1 == "startpause") {
      return OpType::STARTPAUSE;
    } else if (str1 == "stoppause") {
      return OpType::STOPPAUSE;
    } else if (str1 == "park") {
      return OpType::PARK;
    } else {
      return std::nullopt;
    }
  }
  static std::string get_type(OpType t) {
    if (t == OpType::MOVE) {
      return "Move";
    } else if (t == OpType::PICK) {
      return "Pick";
    } else if (t == OpType::DROP) {
      return "Drop";
    } else if (t == OpType::LOAD) {
      return "Load";
    } else if (t == OpType::UNLOAD) {
      return "Unload";
    } else if (t == OpType::CHARGE) {
      return "Charge";
    } else if (t == OpType::OPEN) {
      return "Open";
    } else if (t == OpType::CLOSE) {
      return "Close";
    } else if (t == OpType::LIFT) {
      return "Lift";
    } else if (t == OpType::STARTPAUSE) {
      return "startPause";
    } else if (t == OpType::STOPPAUSE) {
      return "stopPause";
    } else if (t == OpType::PARK) {
      return "Park";
    } else {
      return "Nop";
    }
  }
  // vda操作
  class Action {
  public:
    virtual ~Action() = default;
    Action();
    OpType action_type{OpType::NOP};
    std::string action_id;
    std::optional<std::string> action_description;
    ActionBlockingType blocking_type{ActionBlockingType::HARD};
    std::optional<std::vector<ActionParam>> action_parameters;
    ActionWhen when;
    bool valid{false};
    std::string name;
    virtual void init(jsoncons::json &){};
    virtual jsoncons::json to_json() { return jsoncons::json::object(); };
  };
  explicit Actions(const std::map<std::string, std::string> &pro) {
    std::regex N{R"(^vda5050:action.([^.]+)$)"};
    std::regex T{R"(^vda5050:action.([^.]+).blockingType$)"};
    std::regex P{R"(^vda5050:action.([^.]+).parameter.([^.]+)$)"};
    std::regex W{R"(^vda5050:action.([^.]+).when$)"};

    for (auto &x : pro) {
      std::smatch mt;
      if (std::regex_match(x.first, N)) {
        std::regex_search(x.first, mt, N);
        auto id = (mt.end() - 1)->str();
        auto name = x.second;
        bool has{false};
        for (auto &a : actions) {
          if (a.action_id == id) {
            a.name = name;
            has = true;
          }
        }
        if (!has) {
          Action act;
          act.name = name;
          act.action_id = id;
          actions.push_back(act);
        }
      } else if (std::regex_match(x.first, T)) {
        std::regex_search(x.first, mt, T);
        auto id = (mt.end() - 1)->str();
        bool has{false};
        for (auto &a : actions) {
          if (a.action_id == id) {
            if (x.second == "NONE") {
              a.blocking_type = ActionBlockingType::NONE;
            } else if (x.second == "SOFT") {
              a.blocking_type = ActionBlockingType::SOFT;
            } else {
              a.blocking_type = ActionBlockingType::HARD;
            }
            has = true;
          }
        }
        if (!has) {
          Action act;
          act.action_id = id;
          if (x.second == "NONE") {
            act.blocking_type = ActionBlockingType::NONE;
          } else if (x.second == "SOFT") {
            act.blocking_type = ActionBlockingType::SOFT;
          } else {
            act.blocking_type = ActionBlockingType::HARD;
          }
          actions.push_back(act);
        }
      } else if (std::regex_match(x.first, W)) {
        std::regex_search(x.first, mt, W);
        auto id = (mt.end() - 1)->str();
        bool has{false};
        for (auto &a : actions) {
          if (a.action_id == id) {
            if (x.second == "ORDER_START") {
              a.when = ActionWhen::ORDER_START;
            } else {
              a.when = ActionWhen::ORDER_END;
            }
            has = true;
          }
        }
        if (!has) {
          Action act;
          act.action_id = id;
          if (x.second == "ORDER_START") {
            act.when = ActionWhen::ORDER_START;
          } else {
            act.when = ActionWhen::ORDER_END;
          }
          actions.push_back(act);
        }
      } else if (std::regex_match(x.first, P)) {
        std::regex_search(x.first, mt, P);
        auto p1 = std::pair<std::string, std::string>((mt.end() - 1)->str(),
                                                      x.second);
        auto p = ActionParam();
        p.key = (mt.end() - 1)->str();
        p.value = x.second;
        auto id = (mt.end() - 2)->str();
        bool has{false};
        for (auto &a : actions) {
          if (a.action_id == id) {
            if (!a.action_parameters.has_value()) {
              a.action_parameters = std::vector<ActionParam>();
            }
            a.action_parameters->push_back(p);
            has = true;
          }
        }
        if (!has) {
          Action act;
          act.action_id = id;
          act.action_parameters->push_back(p);
          actions.push_back(act);
        }
      }
    }
    for (auto &x : actions) {
      if (x.action_id.empty()) {
        x.valid = false;
        continue;
      }
      x.valid = true;
    }
    for (auto &x : actions) {
      if (!x.action_parameters.has_value()) {
        continue;
      }
      for (auto &param : x.action_parameters.value()) {
        if (param.key == "type") {
          x.action_type = get_optype(std::get<std::string>(param.value))
                              .value_or(OpType::NOP);
        }
      }
      x.action_id = "Action-" + uuids::to_string(get_uuid());
    }
  }
  Actions() = default;
  void append(const Action &act) { actions.push_back(act); }

public:
  std::vector<Action> actions;
};
inline Actions::Action::Action() : when(ActionWhen::ORDER_START) {}
class PeripheralActions {
public:
  class PeripheralAction {
  public:
    std::string op_name;
    std::string location_name;
    bool completion_required{false};
    std::string execution_trigger;
  };
  std::vector<PeripheralAction> acts;
};
}

#endif