#include "rb/runtime/json.hpp"

#include <cerrno>
#include <charconv>
#include <cstdio>
#include <cstdlib>

namespace rb::runtime {

double Json::number() const {
  if (type_ == Type::Int) return static_cast<double>(int_);
  if (type_ == Type::Real) return real_;
  throw std::invalid_argument("json: number expected");
}
std::int64_t Json::integer() const {
  if (type_ == Type::Int) return int_;
  if (type_ == Type::Real && std::floor(real_) == real_) return static_cast<std::int64_t>(real_);
  throw std::invalid_argument("json: integer expected");
}
bool Json::boolean() const {
  if (type_ != Type::Bool) throw std::invalid_argument("json: bool expected");
  return int_ != 0;
}
const std::string &Json::string() const {
  if (type_ != Type::String) throw std::invalid_argument("json: string expected");
  return string_;
}
const Json::Array &Json::items() const {
  if (type_ != Type::Array) throw std::invalid_argument("json: array expected");
  return array_;
}
const Json::Object &Json::members() const {
  if (type_ != Type::Object) throw std::invalid_argument("json: object expected");
  return object_;
}
const Json *Json::find(const std::string &key) const {
  if (type_ != Type::Object) return nullptr;
  for (const auto &m : object_)
    if (m.first == key) return &m.second;
  return nullptr;
}
const Json &Json::at(const std::string &key) const {
  if (const Json *v = find(key)) return *v;
  throw std::out_of_range("json: missing key " + key);
}
Json &Json::set(const std::string &key, Json value) {
  if (type_ != Type::Object) throw std::invalid_argument("json: object expected");
  for (auto &m : object_)
    if (m.first == key) {
      m.second = std::move(value);
      return m.second;
    }
  object_.emplace_back(key, std::move(value));
  return object_.back().second;
}
void Json::push(Json value) {
  if (type_ != Type::Array) throw std::invalid_argument("json: array expected");
  array_.push_back(std::move(value));
}

static void quote(std::string &out, const std::string &s) {
  out += '"';
  for (unsigned char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof buf, "\\u%04x", c);
          out += buf;
        } else {
          out += static_cast<char>(c);  // UTF-8 passes through
        }
    }
  }
  out += '"';
}

void Json::write(std::string &out, int indent, int depth) const {
  auto newline = [&](int level) {
    if (indent < 0) return;
    out += '\n';
    out.append(static_cast<std::size_t>(indent * level), ' ');
  };
  switch (type_) {
    case Type::Null: out += "null"; break;
    case Type::Bool: out += int_ ? "true" : "false"; break;
    case Type::Int: out += std::to_string(int_); break;
    case Type::Real: {
      if (!std::isfinite(real_)) {
        out += "null";
        break;
      }
      char buf[40];
      auto r = std::to_chars(buf, buf + sizeof buf, real_);
      out.append(buf, r.ptr);
      break;
    }
    case Type::String: quote(out, string_); break;
    case Type::Array:
      out += '[';
      for (std::size_t i = 0; i < array_.size(); ++i) {
        if (i) out += ',';
        newline(depth + 1);
        array_[i].write(out, indent, depth + 1);
      }
      if (!array_.empty()) newline(depth);
      out += ']';
      break;
    case Type::Object:
      out += '{';
      for (std::size_t i = 0; i < object_.size(); ++i) {
        if (i) out += ',';
        newline(depth + 1);
        quote(out, object_[i].first);
        out += indent < 0 ? ":" : ": ";
        object_[i].second.write(out, indent, depth + 1);
      }
      if (!object_.empty()) newline(depth);
      out += '}';
      break;
  }
}

std::string Json::dump(int indent) const {
  std::string out;
  write(out, indent, 0);
  return out;
}

namespace {
struct Parser {
  const std::string &s;
  std::size_t i = 0;
  [[noreturn]] void fail(const char *what) const {
    throw std::invalid_argument(std::string("json: ") + what + " at " + std::to_string(i));
  }
  void ws() {
    while (i < s.size() && (s[i] == ' ' || s[i] == '\n' || s[i] == '\t' || s[i] == '\r')) ++i;
  }
  bool eat(char c) {
    ws();
    if (i < s.size() && s[i] == c) {
      ++i;
      return true;
    }
    return false;
  }
  void expect(char c) {
    if (!eat(c)) fail("unexpected character");
  }
  static void utf8(std::string &out, unsigned cp) {
    if (cp < 0x80) out += static_cast<char>(cp);
    else if (cp < 0x800) {
      out += static_cast<char>(0xC0 | (cp >> 6));
      out += static_cast<char>(0x80 | (cp & 0x3F));
    } else if (cp < 0x10000) {
      out += static_cast<char>(0xE0 | (cp >> 12));
      out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
      out += static_cast<char>(0x80 | (cp & 0x3F));
    } else {
      out += static_cast<char>(0xF0 | (cp >> 18));
      out += static_cast<char>(0x80 | ((cp >> 12) & 0x3F));
      out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
      out += static_cast<char>(0x80 | (cp & 0x3F));
    }
  }
  unsigned hex4() {
    if (i + 4 > s.size()) fail("bad escape");
    unsigned v = 0;
    for (int k = 0; k < 4; ++k) {
      char c = s[i++];
      v <<= 4;
      if (c >= '0' && c <= '9') v |= static_cast<unsigned>(c - '0');
      else if (c >= 'a' && c <= 'f') v |= static_cast<unsigned>(c - 'a' + 10);
      else if (c >= 'A' && c <= 'F') v |= static_cast<unsigned>(c - 'A' + 10);
      else fail("bad escape");
    }
    return v;
  }
  std::string str() {
    expect('"');
    std::string out;
    while (true) {
      if (i >= s.size()) fail("unterminated string");
      char c = s[i++];
      if (c == '"') return out;
      if (c != '\\') {
        out += c;
        continue;
      }
      if (i >= s.size()) fail("bad escape");
      switch (char e = s[i++]) {
        case '"': case '\\': case '/': out += e; break;
        case 'b': out += '\b'; break;
        case 'f': out += '\f'; break;
        case 'n': out += '\n'; break;
        case 'r': out += '\r'; break;
        case 't': out += '\t'; break;
        case 'u': {
          unsigned cp = hex4();
          if (cp >= 0xD800 && cp < 0xDC00 && i + 1 < s.size() && s[i] == '\\' && s[i + 1] == 'u') {
            i += 2;
            unsigned low = hex4();
            cp = 0x10000 + ((cp - 0xD800) << 10) + (low - 0xDC00);
          }
          utf8(out, cp);
          break;
        }
        default: fail("bad escape");
      }
    }
  }
  Json value(int depth) {
    if (depth > 64) fail("nesting too deep");
    ws();
    if (i >= s.size()) fail("unexpected end");
    char c = s[i];
    if (c == '{') {
      ++i;
      Json obj = Json::object();
      if (eat('}')) return obj;
      do {
        ws();
        std::string key = str();
        expect(':');
        obj.set(key, value(depth + 1));
      } while (eat(','));
      expect('}');
      return obj;
    }
    if (c == '[') {
      ++i;
      Json arr = Json::array();
      if (eat(']')) return arr;
      do arr.push(value(depth + 1));
      while (eat(','));
      expect(']');
      return arr;
    }
    if (c == '"') return Json(str());
    if (s.compare(i, 4, "true") == 0) { i += 4; return Json(true); }
    if (s.compare(i, 5, "false") == 0) { i += 5; return Json(false); }
    if (s.compare(i, 4, "null") == 0) { i += 4; return Json(); }
    std::size_t start = i;
    bool real = false;
    if (i < s.size() && (s[i] == '-' || s[i] == '+')) ++i;
    while (i < s.size()) {
      char d = s[i];
      if (d >= '0' && d <= '9') ++i;
      else if (d == '.' || d == 'e' || d == 'E' || d == '-' || d == '+') { real = true; ++i; }
      else break;
    }
    if (i == start) fail("unexpected token");
    std::string token = s.substr(start, i - start);
    char *end = nullptr;
    if (!real) {
      errno = 0;
      long long v = std::strtoll(token.c_str(), &end, 10);
      if (*end == 0 && errno == 0) return Json(static_cast<std::int64_t>(v));
    }
    double v = std::strtod(token.c_str(), &end);
    if (*end != 0 || !std::isfinite(v)) fail("bad number");
    return Json(v);
  }
};
}  // namespace

Json Json::parse(const std::string &text) {
  Parser p{text};
  Json v = p.value(0);
  p.ws();
  if (p.i != text.size()) p.fail("trailing characters");
  return v;
}

}  // namespace rb::runtime
