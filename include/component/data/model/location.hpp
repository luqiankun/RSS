#ifndef LOCATION_HPP
#define LOCATION_HPP
#include "../../tcsresource.hpp"
#include "point.hpp"
namespace data {
namespace model {

class Location : public TCSResource {
 public:
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
  struct Layout {
    Eigen::Vector2i position;
    Eigen::Vector2i label_offset;
    int layer_id;
  };
  struct LocationTypeLayout {
    LocationRepresentation location_representation{
        LocationRepresentation::NONE};
  };
  struct LocationType {
    std::vector<std::string> allowrd_ops;
    std::vector<std::string> allowrd_per_ops;
    LocationTypeLayout layout;
  };
  using TCSResource::TCSResource;
  Eigen::Vector3i position{0, 0, 0};
  Layout layout;
  bool locked{false};
  std::weak_ptr<Point> link;
  LocationType type;
};
}  // namespace model
}  // namespace data
#endif
