#ifndef LOCATIONTYYPE_HPP
#define LOCATIONTYYPE_HPP
#include "../../tcsresource.hpp"
namespace data {
namespace model {
struct LocationTypeLayout {
  enum class LocationRepresentation {

    /**
     * A (empty) location without any representation.
     */
    NONE,
    /**
     * The default representation inherited from the assigned location type.
     */
    DEFAULT,
    /**
     * A location for vehicle load transfer, generic variant.
     */
    LOAD_TRANSFER_GENERIC,
    /**
     * A location for vehicle load transfers, alternative variant 1.
     */
    LOAD_TRANSFER_ALT_1,
    /**
     * A location for vehicle load transfers, alternative variant 2.
     */
    LOAD_TRANSFER_ALT_2,
    /**
     * A location for vehicle load transfers, alternative variant 3.
     */
    LOAD_TRANSFER_ALT_3,
    /**
     * A location for vehicle load transfers, alternative variant 4.
     */
    LOAD_TRANSFER_ALT_4,
    /**
     * A location for vehicle load transfers, alternative variant 5.
     */
    LOAD_TRANSFER_ALT_5,
    /**
     * A location for some generic processing, generic variant.
     */
    WORKING_GENERIC,
    /**
     * A location for some generic processing, alternative variant 1.
     */
    WORKING_ALT_1,
    /**
     * A location for some generic processing, alternative variant 2.
     */
    WORKING_ALT_2,
    /**
     * A location for recharging a vehicle, generic variant.
     */
    RECHARGE_GENERIC,
    /**
     * A location for recharging a vehicle, alternative variant 1.
     */
    RECHARGE_ALT_1,
    /**
     * A location for recharging a vehicle, alternative variant 2.
     */
    RECHARGE_ALT_2
  };
  LocationRepresentation location_representation{LocationRepresentation::NONE};
};
class LocationType : public TCSResource {
 public:
  using TCSResource::TCSResource;
  void get_param() {
    std::regex P{R"(^vda5050:destinationAction.([^.]+).parameter.([^.]+)$)"};
    for (auto& x : properties) {
      if (std::regex_match(x.first, P)) {
        std::smatch sm;
        std::regex_search(x.first, sm, P);
        auto name = (*(sm.end() - 2)).str();
        auto key = (*(sm.end() - 1)).str();
        auto value = x.second;
        for (auto& op : allowed_ops) {
          if (op.first == name) {
            op.second.insert(std::pair<std::string, std::string>(key, value));
            break;
          }
        }
        for (auto& op : allowrd_per_ops) {
          if (op.first == name) {
            op.second.insert(std::pair<std::string, std::string>(key, value));
            break;
          }
        }
      }
    }
  }

 public:
  std::map<std::string, std::map<std::string, std::string>>
      allowed_ops;  // 本地操作，优先通过vdaaction启动，类型hard
  std::map<std::string, std::map<std::string, std::string>>
      allowrd_per_ops;  // 外围操作，在其他点触发，通过vdaaction来启动
  LocationTypeLayout layout;
};

static std::string get_Representation(
    LocationTypeLayout::LocationRepresentation l) {
  if (l == LocationTypeLayout::LocationRepresentation::LOAD_TRANSFER_ALT_1) {
    return "LOAD_TRANSFER_ALT_1";
  } else if (l ==
             LocationTypeLayout::LocationRepresentation::LOAD_TRANSFER_ALT_2) {
    return "LOAD_TRANSFER_ALT_2";
  } else if (l ==
             LocationTypeLayout::LocationRepresentation::LOAD_TRANSFER_ALT_3) {
    return "LOAD_TRANSFER_ALT_3";
  } else if (l ==
             LocationTypeLayout::LocationRepresentation::LOAD_TRANSFER_ALT_4) {
    return "LOAD_TRANSFER_ALT_4";
  } else if (l == LocationTypeLayout::LocationRepresentation::
                      LOAD_TRANSFER_GENERIC) {
    return "LOAD_TRANSFER_GENERIC";
  } else if (l ==
             LocationTypeLayout::LocationRepresentation::LOAD_TRANSFER_ALT_5) {
    return "LOAD_TRANSFER_ALT_5";
  } else if (l == LocationTypeLayout::LocationRepresentation::RECHARGE_ALT_1) {
    return "RECHARGE_ALT_1";
  } else if (l == LocationTypeLayout::LocationRepresentation::RECHARGE_ALT_2) {
    return "RECHARGE_ALT_2";
  } else if (l == LocationTypeLayout::LocationRepresentation::WORKING_ALT_1) {
    return "WORKING_ALT_1";
  } else if (l == LocationTypeLayout::LocationRepresentation::WORKING_ALT_2) {
    return "WORKING_ALT_2";
  } else if (l ==
             LocationTypeLayout::LocationRepresentation::RECHARGE_GENERIC) {
    return "RECHARGE_GENERIC";
  } else if (l == LocationTypeLayout::LocationRepresentation::WORKING_GENERIC) {
    return "WORKING_GENERIC";
  } else if (l == LocationTypeLayout::LocationRepresentation::DEFAULT) {
    return "DEFAULT";
  } else {
    return "NONE";
  }
}
static LocationTypeLayout::LocationRepresentation new_location_type(
    const std::string& name) {
  if (name == "DEFAULT") {
    return LocationTypeLayout::LocationRepresentation::DEFAULT;
  } else if (name == "WORKING_GENERIC") {
    return LocationTypeLayout::LocationRepresentation::WORKING_GENERIC;
  } else if (name == "RECHARGE_GENERIC") {
    return LocationTypeLayout::LocationRepresentation::RECHARGE_GENERIC;
  } else if (name == "WORKING_ALT_2") {
    return LocationTypeLayout::LocationRepresentation::WORKING_ALT_2;
  } else if (name == "WORKING_ALT_1") {
    return LocationTypeLayout::LocationRepresentation::WORKING_ALT_1;
  } else if (name == "RECHARGE_ALT_2") {
    return LocationTypeLayout::LocationRepresentation::RECHARGE_ALT_2;
  } else if (name == "RECHARGE_ALT_1") {
    return LocationTypeLayout::LocationRepresentation::RECHARGE_ALT_1;
  } else if (name == "LOAD_TRANSFER_ALT_5") {
    return LocationTypeLayout::LocationRepresentation::LOAD_TRANSFER_ALT_5;
  } else if (name == "LOAD_TRANSFER_GENERIC") {
    return LocationTypeLayout::LocationRepresentation::LOAD_TRANSFER_GENERIC;
  } else if (name == "LOAD_TRANSFER_ALT_4") {
    return LocationTypeLayout::LocationRepresentation::LOAD_TRANSFER_ALT_4;
  } else if (name == "LOAD_TRANSFER_ALT_3") {
    return LocationTypeLayout::LocationRepresentation::LOAD_TRANSFER_ALT_3;
  } else if (name == "LOAD_TRANSFER_ALT_2") {
    return LocationTypeLayout::LocationRepresentation::LOAD_TRANSFER_ALT_2;
  } else if (name == "LOAD_TRANSFER_ALT_1") {
    return LocationTypeLayout::LocationRepresentation::LOAD_TRANSFER_ALT_1;
  } else {
    return LocationTypeLayout::LocationRepresentation::NONE;
  }
}
}  // namespace model
}  // namespace data

#endif