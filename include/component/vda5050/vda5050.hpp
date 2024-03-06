#ifndef VDA5050_HPP
#define VDA5050_HPP
#include <optional>

#include "../tools/mqtt/mqtt.hpp"
namespace vda5050 {
enum class VehicleMqttStatus { ONLINE = 1, OFFLINE = 2, CONNECTIONBROKEN = 3 };
enum class MasterMqttStatus { ONLINE = 1, OFFLINE = 2, CONNECTIONBROKEN = 3 };
enum class ActionStatus {
  WAITING = 1,
  INITIALIZING = 2,
  RUNNING = 3,
  FINISHED = 4,
  FAILED = 5
};
enum class ActionType {
  NOP,     // 啥也不干
  LOAD,    // 去某location load
  UNLOAD,  // 去某location unload
  MOVE     // 去某point
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
enum ActionBlockingType { NONE = 1, SOFT = 2, HARD = 3 };

}  // namespace vda5050
#endif