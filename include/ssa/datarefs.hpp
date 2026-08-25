#pragma once

#include <XPLMDataAccess.h>
#include <memory>
#include <string>
#include <unordered_map>

namespace ssa {

class FloatDataRef {
public:
  FloatDataRef(std::string name, float initial = 0.0f, bool writable = true);
  ~FloatDataRef();
  FloatDataRef(const FloatDataRef&) = delete;
  FloatDataRef& operator=(const FloatDataRef&) = delete;
  float value() const { return value_; }
  void set(float value);
  const std::string& name() const { return name_; }

private:
  static float read(void* refcon);
  static void write(void* refcon, float value);
  std::string name_;
  float value_{};
  bool writable_{};
  XPLMDataRef handle_{};
};

class DataRefRegistry {
public:
  FloatDataRef& create(const std::string& name, float initial = 0.0f, bool writable = false);
  FloatDataRef* find(const std::string& name);
  void clear();

private:
  std::unordered_map<std::string, std::unique_ptr<FloatDataRef>> refs_;
};

} // namespace ssa

