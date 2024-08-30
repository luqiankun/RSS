#ifndef VDA5050ORDER_HPP
#define VDA5050ORDER_HPP
#include <optional>
#include <variant>

#include "../data/order/driverorder.hpp"
namespace vda5050 {
namespace order {
using ActionType = data::order::DriverOrder::Destination::OpType;
using ActionBlockingType = data::model::Actions::ActionBlockingType;

// 自定义
using ActionWhen = data::model::Actions::ActionWhen;

// action

using ActionParam = data::model::Actions::ActionParam;
inline vda5050::order::ActionType get_vda5050_type_from_str(
    const std::string& t) {
  vda5050::order::ActionType res{ActionType::NOP};
  return data::model::Actions::get_optype(t).value_or(res);
}

inline std::string vda5050_type_to_str(vda5050::order::ActionType t) {
  return data::model::Actions::get_type(t);
}
class Action : public data::model::Actions::Action {
 public:
  using data::model::Actions::Action ::Action;
  void init(jsoncons::json& obj) override {
    action_type = get_vda5050_type_from_str(obj["actionType"].as_string());
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
    res["actionType"] = vda5050_type_to_str(action_type);
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

// edge

class ControlPoint {
 public:
  double x;
  double y;
  double weight{1.0};
  double orientation{0.0};
};

class Trajectory {
 public:
  int degree;
  std::vector<double> knot_vector;
  std::vector<ControlPoint> control_points;
};
enum class orientationType { GLOBAL, TANGENTIAL };
class Edge {
 public:
  std::string edge_id{};
  int sequence_id;
  bool released;
  std::string start_node_id{};
  std::string end_node_id{};
  std::vector<Action> actions;
  //
  std::optional<orientationType> orientation_type;
  std::optional<std::string> direction;
  std::optional<std::string> edge_description;
  std::optional<double> max_height;
  std::optional<double> min_height;
  std::optional<double> orientation;
  std::optional<double> max_speed;
  std::optional<bool> rotation_allowed;
  std::optional<double> max_rotation_speed;
  std::optional<double> length;
  std::optional<Trajectory> trajectory;
  Edge() {}
  explicit Edge(jsoncons::json& obj)
      : edge_id(obj["edgeId"].as_string()),
        sequence_id(obj["sequenceId"].as_integer<int>()),
        released(obj["released"].as_bool()),
        start_node_id(obj["startNodeId"].as_string()),
        end_node_id(obj["endNodeId"].as_string()) {
    for (auto& x : obj["actions"].array_range()) {
      Action act;
      act.init(x);
      actions.push_back(act);
    }
    //
    if (obj.contains("maxSpeed")) {
      max_speed = obj["maxSpeed"].as_double();
    }
    if (obj.contains("maxHeight")) {
      max_height = obj["maxHeight"].as_double();
    }
    if (obj.contains("minHeight")) {
      min_height = obj["minHeight"].as_double();
    }
    if (obj.contains("orientation")) {
      orientation = obj["orientation"].as_double();
    }
    if (obj.contains("orientationType")) {
      auto str = obj["orientationType"].as_string();
      if (str == "GLOBAL") {
        orientation_type = orientationType::GLOBAL;
      } else {
        orientation_type = orientationType::TANGENTIAL;
      }
    }
    if (obj.contains("direction")) {
      direction = obj["direction"].as_string();
    }
    if (obj.contains("rotationAllowed")) {
      rotation_allowed = obj["rotationAllowed"].as_bool();
    }
    if (obj.contains("maxRotationSpeed")) {
      max_rotation_speed = obj["maxRotationSpeed"].as_double();
    }
    if (obj.contains("length")) {
      length = obj["length"].as_double();
    }
    if (obj.contains("trajectory")) {
      trajectory = Trajectory();
      trajectory->degree = obj["trajectory"]["degree"].as_integer<int>();
      for (auto& x : obj["trajectory"]["knotVector"].array_range()) {
        trajectory->knot_vector.push_back(x.as_double());
      }
      for (auto& x : obj["trajectory"]["controlPoints"].array_range()) {
        auto p = ControlPoint();
        p.x = x["x"].as_double();
        p.y = x["y"].as_double();
        if (x.contains("weight")) {
          p.weight = x["weight"].as_double();
        }
        trajectory->control_points.push_back(p);
      }
    }
  }  // namespace order
  jsoncons::json to_json() {
    jsoncons::json res;
    res["edgeId"] = edge_id;
    res["sequenceId"] = sequence_id;
    res["released"] = released;
    res["startNodeId"] = start_node_id;
    res["endNodeId"] = end_node_id;
    if (max_speed.has_value()) {
      res["maxSpeed"] = max_speed.value();
    }
    if (direction.has_value()) {
      res["direction"] = direction.value();
    }
    if (edge_description.has_value()) {
      res["edgeDescription"] = edge_description.value();
    }
    if (max_height.has_value()) {
      res["maxHeight"] = max_height.value();
    }
    if (min_height.has_value()) {
      res["minHeight"] = min_height.value();
    }
    if (orientation.has_value()) {
      res["orientation"] = orientation.value();
    }
    res["actions"] = jsoncons::json::array();
    for (auto& x : actions) {
      res["actions"].push_back(x.to_json());
    }
    if (orientation_type.has_value()) {
      if (orientation_type.value() == orientationType::GLOBAL) {
        res["orientationType"] = "GLOBAL";
      } else {
        res["orientationType"] = "TANGENTIAL";
      }
    }
    if (rotation_allowed.has_value()) {
      res["rotationAllowed"] = rotation_allowed.value();
    }
    if (max_rotation_speed.has_value()) {
      res["maxRotationSpeed"] = max_rotation_speed.value();
    }
    if (length.has_value()) {
      res["length"] = length.value();
    }
    if (trajectory.has_value()) {
      res["trajectory"]["degree"] = trajectory.value().degree;
      res["trajectory"]["knotVector"] = jsoncons::json::array();
      res["trajectory"]["controlPoints"] = jsoncons::json::array();
      for (auto& x : trajectory.value().knot_vector) {
        res["trajectory"]["knotVector"].push_back(x);
      }
      for (auto& x : trajectory.value().control_points) {
        jsoncons::json p;
        p["x"] = x.x;
        p["y"] = x.y;
        p["weight"] = x.weight;
        res["trajectory"]["controlPoints"].push_back(p);
      }
    }
    return res;
  }
};  // namespace vda5050

// node

class NodePosition {
 public:
  double x;
  double y;
  std::string map_id;
  std::optional<double> theta;
  std::optional<double> allowed_deviation_xy{0};
  std::optional<double> allowed_deviation_theta{0};
  std::optional<std::string> map_description;
};

class Node {
 public:
  std::string node_id{};
  int sequence_id;
  bool released;
  std::vector<Action> actions;
  std::optional<std::string> node_description;
  std::optional<NodePosition> node_position;
  Node() {}
  explicit Node(jsoncons::json& obj)
      : node_id(obj["nodeId"].as_string()),
        sequence_id(obj["sequenceId"].as_integer<int>()),
        released(obj["released"].as_bool()) {
    for (auto& x : obj["actions"].array_range()) {
      Action act;
      act.init(x);
      actions.push_back(act);
    }
    if (obj.contains("nodeDescription")) {
      node_description = obj["nodeDescription"].as_string();
    }
    if (obj.contains("nodePosition")) {
      node_position = NodePosition();
      node_position.value().x = obj["nodePosition"]["x"].as_double();
      node_position.value().y = obj["nodePosition"]["y"].as_double();
      node_position.value().theta = obj["nodePosition"]["theta"].as_double();
      node_position.value().map_id = obj["nodePosition"]["mapId"].as_string();
      if (obj["nodePosition"].contains("allowedDeviationXY")) {
        node_position.value().allowed_deviation_xy =
            obj["nodePosition"]["allowedDeviationXY"].as_double();
      }
      if (obj["nodePosition"].contains("allowedDeviationTheta")) {
        node_position.value().allowed_deviation_theta =
            obj["nodePosition"]["allowedDeviationTheta"].as_double();
      }
      if (obj["nodePosition"].contains("mapDescription")) {
        node_position.value().map_description =
            obj["nodePosition"]["mapDescription"].as_string();
      }
    }
  }
  jsoncons::json to_json() {
    jsoncons::json res;
    res["nodeId"] = node_id;
    res["sequenceId"] = sequence_id;
    res["released"] = released;
    if (node_description.has_value()) {
      res["nodeDescription"] = node_description;
    }
    res["actions"] = jsoncons::json::array();
    for (auto& x : actions) {
      res["actions"].push_back(x.to_json());
    }
    if (node_position.has_value()) {
      res["nodePosition"]["x"] = node_position.value().x;
      res["nodePosition"]["y"] = node_position.value().y;
      res["nodePosition"]["mapId"] = node_position.value().map_id;
      if (node_position.value().theta.has_value()) {
        res["nodePosition"]["theta"] = node_position.value().theta.value();
      }
      if (node_position.value().map_description.has_value()) {
        res["nodePosition"]["mapDescription"] =
            node_position.value().map_description;
      }
      if (node_position.value().allowed_deviation_xy.has_value()) {
        res["nodePosition"]["allowedDeviationXY"] =
            node_position.value().allowed_deviation_xy;
      }
      if (node_position.value().allowed_deviation_theta.has_value()) {
        res["nodePosition"]["allowedDeviationTheta"] =
            node_position.value().allowed_deviation_theta.value();
      }
    }
    return res;
  }
};

// order

class VDA5050Order {
 public:
  int header_id;
  std::string timestamp;
  std::string version;
  std::string manufacturer;
  std::string serial_number;
  std::string order_id;
  int order_update_id;
  std::vector<Node> nodes;
  std::vector<Edge> edges;
  std::optional<std::string> zoneset_id;
  VDA5050Order() {}
  explicit VDA5050Order(jsoncons::json& obj)
      : header_id(obj["headerId"].as_integer<int>()),
        timestamp(obj["timestamp"].as_string()),
        version(obj["version"].as_string()),
        manufacturer(obj["manufacturer"].as_string()),
        serial_number(obj["serialNumber"].as_string()),
        order_id(obj["orderId"].as_string()),
        order_update_id(obj["orderUpdateId"].as_integer<int>()) {
    if (obj.contains("zoneSetId")) {
      zoneset_id = obj["zoneSetId"].as_string();
    }
    for (auto& x : obj["nodes"].array_range()) {
      auto n = Node(x);
      nodes.push_back(n);
    }
    std::stable_sort(nodes.begin(), nodes.end(),
                     [](const Node& a, const Node& b) {
                       return a.sequence_id < b.sequence_id;
                     });
    for (auto& x : obj["edges"].array_range()) {
      auto n = Edge(x);
      edges.push_back(n);
    }
    std::stable_sort(edges.begin(), edges.end(),
                     [](const Edge& a, const Edge& b) {
                       return a.sequence_id < b.sequence_id;
                     });
  }
  jsoncons::json to_json() {
    jsoncons::json res;
    res["headerId"] = header_id;
    res["timestamp"] = timestamp;
    res["version"] = version;
    res["manufacturer"] = manufacturer;
    res["serialNumber"] = serial_number;
    res["orderId"] = order_id;
    res["orderUpdateId"] = order_update_id;
    res["nodes"] = jsoncons::json::array();
    res["edges"] = jsoncons::json::array();
    for (auto& x : nodes) {
      res["nodes"].push_back(x.to_json());
    }
    for (auto& x : edges) {
      res["edges"].push_back(x.to_json());
    }
    if (zoneset_id.has_value()) {
      res["zoneSetId"] = zoneset_id.value();
    }
    return res;
  }
};

}  // namespace order
}  // namespace vda5050

#endif