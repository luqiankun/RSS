/// Copyright 2013-2023 Daniel Parker
// Distributed under the Boost license, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

// See https://github.com/danielaparker/jsoncons for latest version

#ifndef JSONCONS_JSONSCHEMA_JSONSCHEMA_ERROR_HPP
#define JSONCONS_JSONSCHEMA_JSONSCHEMA_ERROR_HPP

#include <system_error>

#include "../../jsoncons/json_exception.hpp"
#include "../../jsoncons/uri.hpp"
#include "../../jsoncons_ext/jsonpointer/jsonpointer.hpp"

namespace jsoncons {
namespace jsonschema {

class schema_error : public std::runtime_error, public virtual json_exception {
 public:
  schema_error(const std::string& message) : std::runtime_error(message) {}

  const char* what() const noexcept override {
    return std::runtime_error::what();
  }
};

class validation_error : public std::runtime_error,
                         public virtual json_exception {
 public:
  validation_error(const std::string& message) : std::runtime_error(message) {}

  const char* what() const noexcept override {
    return std::runtime_error::what();
  }
};

class validation_output {
  std::string keyword_;
  jsonpointer::json_pointer eval_path_;
  uri schema_path_;
  std::string instance_location_;
  std::string message_;
  std::vector<validation_output> nested_errors_;

 public:
  validation_output(std::string keyword, jsonpointer::json_pointer eval_path,
                    uri schema_path, std::string instance_location,
                    std::string message)
      : keyword_(std::move(keyword)),
        eval_path_(std::move(eval_path)),
        schema_path_(std::move(schema_path)),
        instance_location_(std::move(instance_location)),
        message_(std::move(message)) {}

  validation_output(const std::string& keyword,
                    const jsonpointer::json_pointer& eval_path,
                    const uri& schema_path,
                    const std::string& instance_location,
                    const std::string& message,
                    const std::vector<validation_output>& nested_errors)
      : keyword_(keyword),
        eval_path_(eval_path),
        schema_path_(schema_path),
        instance_location_(instance_location),
        message_(message),
        nested_errors_(nested_errors) {}

  const std::string& instance_location() const { return instance_location_; }

  const std::string& message() const { return message_; }

  const jsonpointer::json_pointer& eval_path() const { return eval_path_; }

  const uri& schema_path() const { return schema_path_; }

  const std::string keyword_location() const { return eval_path_.to_string(); }

  const std::string absolute_keyword_location() const {
    return schema_path_.string();
  }

  const std::string& keyword() const { return keyword_; }

  const std::vector<validation_output>& nested_errors() const {
    return nested_errors_;
  }
};

}  // namespace jsonschema
}  // namespace jsoncons

#endif  // JSONCONS_JSONSCHEMA_JSONSCHEMA_ERROR_HPP
