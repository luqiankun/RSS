#include "../../util/timer.hpp"
#include "../jsoncons/json.hpp"
#include "../jsoncons_ext/jsonschema/jsonschema.hpp"

class SchemaValidator {
 public:
  static std::vector<std::string> validate(
      jsoncons::json obj,
      std::shared_ptr<jsoncons::jsonschema::json_schema<jsoncons::json>>
          schema) {
    std::vector<std::string> errors;
    try {
      auto reporter =
          [&errors](const jsoncons::jsonschema::validation_output& o) {
            auto t = o.instance_location() + ": " + o.message();
            errors.push_back(t);
          };
      jsoncons::jsonschema::json_validator<jsoncons::json> vad(schema);
      vad.validate(obj, reporter);
      return errors;
    } catch (std::exception& ec) {
      errors.push_back(std::string{"std::ecception: "} + ec.what());
      return errors;
    }
  }
};
