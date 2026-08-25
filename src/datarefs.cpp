#include "ssa/datarefs.hpp"
#include <algorithm>
#include <stdexcept>

namespace ssa {

FloatDataRef::FloatDataRef(std::string name, float initial, bool writable)
    : name_(std::move(name)), value_(initial), writable_(writable) {
  handle_ = XPLMRegisterDataAccessor(
      name_.c_str(), xplmType_Float, writable_ ? 1 : 0,
      nullptr, nullptr, read, writable_ ? write : nullptr,
      nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
      this, this);
  if (!handle_) throw std::runtime_error("Cannot register dataref: " + name_);
}

FloatDataRef::~FloatDataRef() {
  if (handle_) XPLMUnregisterDataAccessor(handle_);
}

float FloatDataRef::read(void* refcon) {
  return static_cast<FloatDataRef*>(refcon)->value_;
}

void FloatDataRef::write(void* refcon, float value) {
  auto* self = static_cast<FloatDataRef*>(refcon);
  if (self->writable_) self->set(value);
}

void FloatDataRef::set(float value) {
  value_ = std::clamp(value, 0.0f, 1.0f);
}

FloatDataRef& DataRefRegistry::create(const std::string& name, float initial, bool writable) {
  if (auto* existing = find(name)) return *existing;
  auto item = std::make_unique<FloatDataRef>(name, initial, writable);
  auto* result = item.get();
  refs_.emplace(name, std::move(item));
  return *result;
}

FloatDataRef* DataRefRegistry::find(const std::string& name) {
  const auto it = refs_.find(name);
  return it == refs_.end() ? nullptr : it->second.get();
}

void DataRefRegistry::clear() { refs_.clear(); }

} // namespace ssa

