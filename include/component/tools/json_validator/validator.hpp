#include "../json/json.hpp"
#include "../jsoncons/json.hpp"
#include "../jsoncons_ext/jsonschema/jsonschema.hpp"
class SchemaValidator {
 public:
  static std::vector<std::string> validate(nlohmann::json obj,
                                           nlohmann::json schema) {
    auto sch = jsoncons::json::parse(schema.dump());
    auto reg = jsoncons::jsonschema::make_schema(sch);
    auto src = jsoncons::json::parse(obj.dump());
    std::vector<std::string> errors;
    try {
      auto reporter =
          [&errors](const jsoncons::jsonschema::validation_output& o) {
            auto t = o.instance_location() + ": " + o.message();
            errors.push_back(t);
          };
      jsoncons::jsonschema::json_validator<jsoncons::json> vad(reg);
      vad.validate(src, reporter);
      return errors;
    } catch (std::exception& ec) {
      errors.push_back(std::string{"std::ecception: "} + ec.what());
      return errors;
    }
  }
};
