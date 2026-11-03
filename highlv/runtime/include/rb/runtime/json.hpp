// Minimal JSON value: enough for runtime.json, models/manifest.json, the status file
// and the operator command reply. No external dependency.
#pragma once
#include <cmath>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace rb::runtime {

class Json {
 public:
  enum class Type { Null, Bool, Int, Real, String, Array, Object };
  using Array = std::vector<Json>;
  using Object = std::vector<std::pair<std::string, Json>>;  // keeps insertion order

  Json() = default;
  Json(std::nullptr_t) {}
  Json(bool v) : type_(Type::Bool), int_(v) {}
  Json(int v) : type_(Type::Int), int_(v) {}
  Json(unsigned v) : type_(Type::Int), int_(v) {}
  Json(std::int64_t v) : type_(Type::Int), int_(v) {}
  Json(std::uint64_t v) : type_(Type::Int), int_(static_cast<std::int64_t>(v)) {}
  Json(double v) : type_(Type::Real), real_(v) {}
  Json(const char *v) : type_(Type::String), string_(v) {}
  Json(std::string v) : type_(Type::String), string_(std::move(v)) {}
  Json(Array v) : type_(Type::Array), array_(std::move(v)) {}
  static Json object() {
    Json j;
    j.type_ = Type::Object;
    return j;
  }
  static Json array() {
    Json j;
    j.type_ = Type::Array;
    return j;
  }

  Type type() const { return type_; }
  bool isNull() const { return type_ == Type::Null; }
  bool isNumber() const { return type_ == Type::Int || type_ == Type::Real; }
  double number() const;
  std::int64_t integer() const;
  bool boolean() const;
  const std::string &string() const;
  const Array &items() const;
  const Object &members() const;

  bool has(const std::string &key) const { return find(key) != nullptr; }
  const Json *find(const std::string &key) const;
  const Json &at(const std::string &key) const;  // throws std::out_of_range

  // Object builders; set() replaces an existing key.
  Json &set(const std::string &key, Json value);
  void push(Json value);

  // indent < 0: compact. Non-finite reals are written as null (valid JSON).
  std::string dump(int indent = -1) const;
  static Json parse(const std::string &text);  // throws std::invalid_argument

 private:
  void write(std::string &out, int indent, int depth) const;
  Type type_ = Type::Null;
  std::int64_t int_ = 0;
  double real_ = 0;
  std::string string_;
  Array array_;
  Object object_;
};

}  // namespace rb::runtime
