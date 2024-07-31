#ifndef VDA5050STATE_HPP
#define VDA5050STATE_HPP
#include <optional>

#include "../../3rdparty/jsoncons/json.hpp"
namespace vda5050 {
enum class VehicleMqttStatus { ONLINE = 1, OFFLINE = 2, CONNECTIONBROKEN = 3 };
enum class MasterMqttStatus { ONLINE = 1, OFFLINE = 2, CONNECTIONBROKEN = 3 };
namespace state {
enum class ActionStatus {
  WAITING = 1,
  INITIALIZING = 2,
  RUNNING = 3,
  FINISHED = 4,
  FAILED = 5
};
enum class ErrorLevel { WARNING = 1, FATAL = 2 };
enum class SafetyStatus { AUTOACK = 1, MANUAL = 2, REMOTE = 3, NONE = 4 };
enum class OperMode {
  AUTOMATIC = 1,
  SEMIAUTOMATIC = 2,
  MANUAL = 3,
  SERVICE = 4,
  TEACHIN = 5
};
enum InfoLevel { INFO = 1, DEBUG = 2 };

// data struct

// safety
class SafetyState {
 public:
  SafetyStatus estop{SafetyStatus::NONE};
  bool field_violation{false};
  SafetyState() {
    estop = SafetyStatus::NONE;
    field_violation = false;
  }
  explicit SafetyState(jsoncons::json& obj) {
    auto es = obj["eStop"].as_string();
    if (es == "AUTOACK") {
      estop = SafetyStatus::AUTOACK;
    } else if (es == "MANUAL") {
      estop = SafetyStatus::MANUAL;
    } else if (es == "REMOTE") {
      estop = SafetyStatus::REMOTE;
    } else {
      estop = SafetyStatus::NONE;
    }
    auto f = obj["fieldViolation"].as_bool();
    field_violation = f;
  }
  SafetyState(SafetyStatus s, bool f) {
    estop = s;
    field_violation = f;
  }
  jsoncons::json to_json() {
    jsoncons::json res;
    res["fieldViolation"] = field_violation;
    if (estop == SafetyStatus::AUTOACK) {
      res["eStop"] = "AUTOACK";
    } else if (estop == SafetyStatus::MANUAL) {
      res["eStop"] = "MANUAL";

    } else if (estop == SafetyStatus::REMOTE) {
      res["eStop"] = "REMOTE";
    } else {
      res["eStop"] = "NONE";
    }
    return res;
  }
};

// information
class InforRef {
 public:
  std::string reference_key{};
  std::string reference_value{};
};

class InforMation {
 public:
  std::string info_type{};
  InfoLevel info_level{InfoLevel::INFO};
  std::optional<std::string> info_description{std::nullopt};
  std::optional<std::vector<InforRef>> info_references{std::nullopt};
  InforMation(){};
  explicit InforMation(jsoncons::json& obj) {
    info_type.assign(obj["infoType"].as_string());
    auto level = obj["infoLevel"].as_string();
    if (level == "INFO") {
      info_level = InfoLevel::INFO;
    } else {
      info_level = InfoLevel::DEBUG;
    }
    if (obj.contains("infoReferences")) {
      info_references = std::vector<InforRef>();
      for (auto& x : obj["infoReferences"].array_range()) {
        auto t = InforRef();
        t.reference_key = x["referenceKey"].as_string();
        t.reference_value = x["referenceValue"].as_string();
        info_references->push_back(t);
      }
    }
    if (obj.contains("infoDescription")) {
      info_description = obj["infoDescription"].as_string();
    }
  }
  jsoncons::json to_json() {
    jsoncons::json res;
    res["infoType"] = info_type;
    if (info_level == InfoLevel::INFO) {
      res["infoLevel"] = "INFO";
    } else {
      res["infoLevel"] = "DEBUG";
    }
    if (info_description.has_value()) {
      res["infoDescription"] = info_description.value();
    }
    if (info_references.has_value()) {
      res["infoReferences"] = jsoncons::json::array();
      for (auto& x : info_references.value()) {
        jsoncons::json ref;
        ref["referenceKey"] = x.reference_key;
        ref["referenceValue"] = x.reference_value;
        res["infoReferences"].push_back(ref);
      }
    }
    return res;
  }
};

// errors

class ErrorRef {
 public:
  std::string reference_key{};
  std::string reference_value{};
};
class Error {
 public:
  ErrorLevel error_level;
  std::string error_type{};
  std::optional<std::string> error_description{std::nullopt};
  std::optional<std::vector<ErrorRef>> error_references{std::nullopt};
  Error() {}
  explicit Error(jsoncons::json& obj)
      : error_type(obj["errorType"].as_string()) {
    auto level = obj["errorLevel"].as_string();
    if (level == "WARNING") {
      error_level = ErrorLevel::WARNING;
    } else {
      error_level = ErrorLevel::FATAL;
    }
    if (obj.contains("errorDescription")) {
      error_description = obj["errorDescription"].as_string();
    }
    if (obj.contains("errorReferences")) {
      error_references = std::vector<ErrorRef>();
      for (auto& x : obj["errorReferences"].array_range()) {
        ErrorRef ref;
        ref.reference_key = x["referenceKey"].as_string();
        ref.reference_value = x["referenceValue"].as_string();
        error_references->push_back(ref);
      }
    }
  }
  jsoncons::json to_json() {
    jsoncons::json res;
    res["errorType"] = error_type;
    if (error_level == ErrorLevel::WARNING) {
      res["errorLevel"] = "WARNING";
    } else {
      res["errorLevel"] = "FATAL";
    }
    if (error_description.has_value()) {
      res["errorDescription"] = error_description.value();
    }
    if (error_references.has_value()) {
      res["errorReferences"] = jsoncons::json::array();
      for (auto& x : error_references.value()) {
        jsoncons::json ref;
        ref["referenceKey"] = x.reference_key;
        ref["referenceValue"] = x.reference_value;
        res["errorReferences"].push_back(ref);
      }
    }
    return res;
  }
};

// battery

class BatteryState {
 public:
  float battery_charge;
  bool charging;
  std::optional<float> battery_voltage{std::nullopt};
  std::optional<float> battery_health{std::nullopt};
  std::optional<float> reach{std::nullopt};
  BatteryState() {}
  explicit BatteryState(jsoncons::json& obj) {
    battery_charge = obj["batteryCharge"].as_double();
    charging = obj["charging"].as_bool();
    if (obj.contains("batteryHealth")) {
      battery_health = obj["batteryHealth"].as_double();
    }
    if (obj.contains("batteryVoltage")) {
      battery_voltage = obj["batteryVoltage"].as_double();
    }
    if (obj.contains("reach")) {
      reach = obj["reach"].as_double();
    }
  }
  jsoncons::json to_json() {
    jsoncons::json res;
    res["batteryCharge"] = battery_charge;
    res["charging"] = charging;
    if (battery_voltage.has_value()) {
      res["batteryVoltage"] = battery_voltage.value();
    }
    if (battery_health.has_value()) {
      res["batteryHealth"] = battery_health.value();
    }
    if (reach.has_value()) {
      res["reach"] = reach.value();
    }
    return res;
  }
};

// actionstate

class ActionState {
 public:
  std::string action_id{};
  std::optional<std::string> action_type{std::nullopt};
  std::optional<std::string> action_description{std::nullopt};
  std::optional<std::string> result_description{std::nullopt};
  ActionStatus action_status;
  ActionState(){};
  explicit ActionState(jsoncons::json& obj)
      : action_id(obj["actionId"].as_string()) {
    auto status = obj["actionStatus"].as_string();
    if (status == "WAITING") {
      action_status = ActionStatus::WAITING;
    } else if (status == "INITIALIZING") {
      action_status = ActionStatus::INITIALIZING;
    } else if (status == "RUNNING") {
      action_status = ActionStatus::RUNNING;
    } else if (status == "FINISHED") {
      action_status = ActionStatus::FINISHED;
    } else {
      action_status = ActionStatus::FAILED;
    }
    if (obj.contains("actionType")) {
      action_type = obj["actionType"].as_string();
    }
    if (obj.contains("actionDescription")) {
      action_description = obj["actionDescription"].as_string();
    }
    if (obj.contains("resultDescription")) {
      result_description = obj["resultDescription"].as_string();
    }
  };
  jsoncons::json to_json() {
    jsoncons::json res;
    res["actionId"] = action_id;
    if (action_status == ActionStatus::WAITING) {
      res["actionStatus"] = "WAITING";
    } else if (action_status == ActionStatus::INITIALIZING) {
      res["actionStatus"] = "INITIALIZING";
    } else if (action_status == ActionStatus::RUNNING) {
      res["actionStatus"] = "RUNNING";
    } else if (action_status == ActionStatus::FINISHED) {
      res["actionStatus"] = "FINISHED";
    } else {
      res["actionStatus"] = "FAILED";
    }
    if (action_type.has_value()) {
      res["actionType"] = action_type.value();
    }
    if (action_description.has_value()) {
      res["actionDescription"] = action_description.value();
    }
    if (result_description.has_value()) {
      res["resultDescription"] = result_description.value();
    }
    return res;
  }
};

// loads
class BoundingBox {
 public:
  float x;
  float y;
  float z;
  std::optional<float> theta;
};
class LoadDimensions {
 public:
  float length;
  float width;
  std::optional<float> height;
};

class Load {
 public:
  std::optional<std::string> load_id{std::nullopt};
  std::optional<std::string> load_type{std::nullopt};
  std::optional<std::string> load_position{std::nullopt};
  std::optional<float> weight{std::nullopt};
  std::optional<BoundingBox> bounding_boxReference{std::nullopt};
  std::optional<LoadDimensions> Load_dimensions{std::nullopt};
  Load(){};
  explicit Load(jsoncons::json& obj) {
    if (obj.contains("loadId")) {
      load_id = obj["loadId"].as_string();
    }
    if (obj.contains("loadType")) {
      load_type = obj["loadType"].as_string();
    }
    if (obj.contains("loadPosition")) {
      load_position = obj["loadPosition"].as_string();
    }
    if (obj.contains("weight")) {
      weight = obj["weight"].as_double();
    }
    if (obj.contains("boundingBoxReference")) {
      BoundingBox box;
      box.x = obj["boundingBoxReference"]["x"].as_double();
      box.y = obj["boundingBoxReference"]["y"].as_double();
      box.z = obj["boundingBoxReference"]["z"].as_double();
      if (obj["boundingBoxReference"].contains("theta")) {
        box.theta = obj["boundingBoxReference"]["theta"].as_double();
      }
      bounding_boxReference = box;
    }
    if (obj.contains("loadDimensions")) {
      LoadDimensions box;
      box.width = obj["loadDimensions"]["width"].as_double();
      box.length = obj["loadDimensions"]["length"].as_double();
      if (obj["loadDimensions"].contains("height")) {
        box.height = obj["loadDimensions"]["height"].as_double();
      }
      Load_dimensions = box;
    }
  };
  jsoncons::json to_json() {
    jsoncons::json res;
    if (load_id.has_value()) {
      res["loadId"] = load_id.value();
    }
    if (load_type.has_value()) {
      res["loadType"] = load_type.value();
    }
    if (load_position.has_value()) {
      res["loadPosition"] = load_position.value();
    }
    if (weight.has_value()) {
      res["weight"] = weight.value();
    }
    if (bounding_boxReference.has_value()) {
      res["boundingBoxReference"]["x"] = bounding_boxReference.value().x;
      res["boundingBoxReference"]["y"] = bounding_boxReference.value().y;
      res["boundingBoxReference"]["z"] = bounding_boxReference.value().z;
      if (bounding_boxReference.value().theta.has_value()) {
        res["boundingBoxReference"]["theta"] =
            bounding_boxReference.value().theta.value();
      }
    }
    if (Load_dimensions.has_value()) {
      res["loadDimensions"]["length"] = Load_dimensions.value().length;
      res["loadDimensions"]["width"] = Load_dimensions.value().width;
      if (Load_dimensions->height.has_value()) {
        res["loadDimensions"]["height"] =
            Load_dimensions.value().height.value();
      }
    }
    return res;
  }
};

// velocity

class Velocity {
 public:
  std::optional<float> vx{std::nullopt};
  std::optional<float> vy{std::nullopt};
  std::optional<float> omega{std::nullopt};
  Velocity(){};
  explicit Velocity(jsoncons::json& obj) {
    if (obj.contains("vx")) {
      vx = obj["vx"].as_double();
    }
    if (obj.contains("vy")) {
      vy = obj["vy"].as_double();
    }
    if (obj.contains("omega")) {
      omega = obj["omega"].as_double();
    }
  }
  jsoncons::json to_json() {
    jsoncons::json res;
    if (vx.has_value()) {
      res["vx"] = vx.value();
    }
    if (vy.has_value()) {
      res["vy"] = vy.value();
    }
    if (omega.has_value()) {
      res["omega"] = omega.value();
    }
    return res;
  }
};

// agvpositon

class AgvPosition {
 public:
  float x;
  float y;
  float theta;
  std::string map_id;
  bool position_initialized;
  std::optional<std::string> map_description;
  std::optional<float> deviation_range;
  std::optional<float> localization_score;
  AgvPosition(){};
  explicit AgvPosition(jsoncons::json obj) {
    x = obj["x"].as_double();
    y = obj["y"].as_double();
    theta = obj["theta"].as_double();
    map_id = obj["mapId"].as_string();
    position_initialized = obj["positionInitialized"].as_bool();
    if (obj.contains("mapDescription")) {
      map_description = obj["mapDescription"].as_string();
    }
    if (obj.contains("localizationScore")) {
      localization_score = obj["localizationScore"].as_double();
    }
    if (obj.contains("deviationRange")) {
      deviation_range = obj["deviationRange"].as_double();
    }
  }

  jsoncons::json to_json() {
    jsoncons::json res;
    res["x"] = x;
    res["y"] = y;
    res["theta"] = theta;
    res["mapId"] = map_id;
    res["positionInitialized"] = position_initialized;
    if (map_description.has_value()) {
      res["mapDescription"] = map_description.value();
    }
    if (deviation_range.has_value()) {
      res["deviationRange"] = deviation_range.value();
    }
    if (localization_score.has_value()) {
      res["localizationScore"] = localization_score.value();
    }
    return res;
  }
};

// edgeStates

class ControlPoint {
 public:
  float x;
  float y;
  std::optional<float> weight;
};

class Trajectory {
 public:
  int degree;
  std::vector<float> knot_vector;
  std::vector<ControlPoint> control_points;
};

class EdgeState {
 public:
  std::string edge_id{};
  int sequence_id;
  bool released;
  std::optional<std::string> edge_description;
  std::optional<Trajectory> trajectory;
  EdgeState() {}
  explicit EdgeState(jsoncons::json& obj)
      : edge_id(obj["edgeId"].as_string()),
        sequence_id(obj["sequenceId"].as_integer<int>()),
        released(obj["released"].as_bool()) {
    if (obj.contains("edgeDescription")) {
      edge_description = obj["edgeDescription"].as_string();
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
  }

  jsoncons::json to_json() {
    jsoncons::json res;
    res["edgeId"] = edge_id;
    res["sequenceId"] = sequence_id;
    res["released"] = released;
    if (edge_description.has_value()) {
      res["edgeDescription"] = edge_description.value();
    }
    if (trajectory.has_value()) {
      res["trajectory"]["degree"] = trajectory.value().degree;
      res["knotVector"] = jsoncons::json::array();
      res["controlPoints"] = jsoncons::json::array();
      for (auto& x : trajectory.value().knot_vector) {
        res["knotVector"].push_back(x);
      }
      for (auto& x : trajectory.value().control_points) {
        jsoncons::json p;
        p["x"] = x.x;
        p["y"] = x.y;
        if (x.weight.has_value()) {
          p["weight"] = x.weight.value();
        }
        res["controlPoints"].push_back(p);
      }
    }
    return res;
  }
};

// nodestates

class NodePosition {
 public:
  float x;
  float y;
  float theta;
  std::string map_id;
};

class NodeState {
 public:
  std::string node_id;
  int sequence_id;
  bool released;
  std::optional<std::string> node_description;
  std::optional<NodePosition> node_position;
  NodeState(){};
  explicit NodeState(jsoncons::json& obj)
      : node_id(obj["nodeId"].as_string()),
        sequence_id(obj["sequenceId"].as_integer<int>()),
        released(obj["released"].as_bool()) {
    if (obj.contains("nodeDescription")) {
      node_description = obj["nodeDescription"].as_string();
    }
    if (obj.contains("nodePosition")) {
      node_position = NodePosition();
      node_position->x = obj["nodePosition"]["x"].as_double();
      node_position->y = obj["nodePosition"]["y"].as_double();
      node_position->theta = obj["nodePosition"]["theta"].as_double();
      node_position->map_id = obj["nodePosition"]["mapId"].as_string();
    }
  }
  jsoncons::json to_json() {
    jsoncons::json res;
    res["nodeId"] = node_id;
    res["released"] = released;
    res["sequenceId"] = sequence_id;
    if (node_description.has_value()) {
      res["nodeDescription"] = node_description.value();
    }
    if (node_position.has_value()) {
      jsoncons::json p;
      p["x"] = node_position.value().x;
      p["y"] = node_position.value().y;
      p["theta"] = node_position.value().theta;
      p["mapId"] = node_position.value().map_id;
      res["nodePosition"] = p;
    }
    return res;
  }
};

//

class VDA5050State {
 public:
  int header_id;
  std::string timestamp;
  std::string version;
  std::string manufacturer;
  std::string serial_number;
  std::string order_id;
  int order_update_id;
  std::string last_node_id;
  int last_node_seq_id;
  std::vector<NodeState> nodestates;
  std::vector<EdgeState> edgestates;
  bool driving;
  std::vector<ActionState> actionstates;
  BatteryState battery_state;
  OperMode operating_mode;
  std::vector<Error> errors;
  SafetyState safetystate;
  // optional
  std::optional<bool> paused;
  std::optional<bool> newbase_request;
  std::optional<float> distance_since_last_node;
  std::optional<std::string> zoneset_id;
  std::optional<AgvPosition> agv_position;
  std::optional<Velocity> velocity;
  std::optional<std::vector<Load>> loads;
  std::optional<InforMation> information;
  VDA5050State() {}
  explicit VDA5050State(jsoncons::json& obj)
      : header_id(obj["headerId"].as_integer<int>()),
        timestamp(obj["timestamp"].as_string()),
        version(obj["version"].as_string()),
        manufacturer(obj["manufacturer"].as_string()),
        serial_number(obj["serialNumber"].as_string()),
        order_id(obj["orderId"].as_string()),
        order_update_id(obj["orderUpdateId"].as_integer<int>()),
        last_node_seq_id(obj["lastNodeSequenceId"].as_integer<int>()),
        last_node_id(obj["lastNodeId"].as_string()) {
    for (auto& x : obj["nodeStates"].array_range()) {
      auto s = NodeState(x);
      nodestates.push_back(s);
    }

    for (auto& x : obj["edgeStates"].array_range()) {
      auto e = EdgeState(x);
      edgestates.push_back(e);
    }

    driving = obj["driving"].as_bool();
    for (auto& x : obj["actionStates"].array_range()) {
      auto s = ActionState(x);
      actionstates.push_back(s);
    }

    battery_state = BatteryState(obj["batteryState"]);
    if (obj["operatingMode"].as_string() == "AUTOMATIC") {
      operating_mode = OperMode::AUTOMATIC;
    } else if (obj["operatingMode"].as_string() == "SEMIAUTOMATIC") {
      operating_mode = OperMode::SEMIAUTOMATIC;
    } else if (obj["operatingMode"].as_string() == "MANUAL") {
      operating_mode = OperMode::MANUAL;
    } else if (obj["operatingMode"].as_string() == "SERVICE") {
      operating_mode = OperMode::SERVICE;
    } else {
      operating_mode = OperMode::TEACHIN;
    }

    for (auto& x : obj["errors"].array_range()) {
      auto e = Error(x);
      errors.push_back(e);
    }

    safetystate = SafetyState(obj["safetyState"]);
    // optional

    if (obj.contains("paused")) {
      paused = obj["paused"].as_bool();
    }
    if (obj.contains("newBaseRequest")) {
      newbase_request = obj["newBaseRequest"].as_bool();
    }
    if (obj.contains("distanceSinceLastNode")) {
      distance_since_last_node = obj["distanceSinceLastNode"].as_bool();
    }
    if (obj.contains("zoneSetId")) {
      zoneset_id = obj["zoneSetId"].as_string();
    }
    if (obj.contains("agvPosition")) {
      agv_position = AgvPosition(obj["agvPosition"]);
    }
    if (obj.contains("velocity")) {
      velocity = Velocity(obj["velocity"]);
    }
    if (obj.contains("loads")) {
      loads = std::vector<Load>();
      for (auto& x : obj["loads"].array_range()) {
        auto l = Load(x);
        loads.value().push_back(l);
      }
    }
    if (obj.contains("information")) {
      information = InforMation(obj["information"]);
    }
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
    res["lastNodeId"] = last_node_id;
    res["lastNodeSequenceId"] = last_node_seq_id;
    res["nodeStates"] = jsoncons::json::array();
    for (auto& x : nodestates) {
      res["nodeStates"].push_back(x.to_json());
    }
    res["edgeStates"] = jsoncons::json::array();
    for (auto& x : edgestates) {
      res["edgeStates"].push_back(x.to_json());
    }
    res["driving"] = driving;
    res["actionStates"] = jsoncons::json::array();
    for (auto& x : actionstates) {
      res["actionStates"].push_back(x.to_json());
    }
    res["batteryState"] = battery_state.to_json();
    if (operating_mode == OperMode::AUTOMATIC) {
      res["operatingMode"] = "AUTOMATIC";
    } else if (operating_mode == OperMode::SEMIAUTOMATIC) {
      res["operatingMode"] = "SEMIAUTOMATIC";
    } else if (operating_mode == OperMode::MANUAL) {
      res["operatingMode"] = "MANUAL";
    } else if (operating_mode == OperMode::SERVICE) {
      res["operatingMode"] = "SERVICE";
    } else {
      res["operatingMode"] = "TEACHIN";
    }
    res["errors"] = jsoncons::json::array();
    for (auto& x : errors) {
      res["errors"].push_back(x.to_json());
    }
    res["safetyState"] = safetystate.to_json();

    // optinal
    if (paused.has_value()) {
      res["paused"] = paused.value();
    }
    if (newbase_request.has_value()) {
      res["newBaseRequest"] = newbase_request.value();
    }
    if (distance_since_last_node.has_value()) {
      res["distanceSinceLastNode"] = distance_since_last_node.value();
    }
    if (zoneset_id.has_value()) {
      res["zoneSetId"] = zoneset_id.value();
    }
    if (agv_position.has_value()) {
      res["agvPosition"] = agv_position.value().to_json();
    }
    if (velocity.has_value()) {
      res["velocity"] = velocity.value().to_json();
    }
    if (loads.has_value()) {
      res["loads"] = jsoncons::json::array();
      for (auto& x : loads.value()) {
        res["loads"].push_back(x.to_json());
      }
    }
    if (information.has_value()) {
      res["information"] = information.value().to_json();
    }
    return res;
  }
};
}  // namespace state

}  // namespace vda5050
#endif