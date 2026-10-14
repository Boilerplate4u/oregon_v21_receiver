#pragma once
// Minimal host stub of esphome/core/log.h for tests/test_component.cpp.
#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>

#define ESPHOME_LOG_LEVEL_NONE 0
#define ESPHOME_LOG_LEVEL_ERROR 1
#define ESPHOME_LOG_LEVEL_WARN 2
#define ESPHOME_LOG_LEVEL_INFO 3
#define ESPHOME_LOG_LEVEL_CONFIG 4
#define ESPHOME_LOG_LEVEL_DEBUG 5
#define ESPHOME_LOG_LEVEL_VERBOSE 6
#define ESPHOME_LOG_LEVEL_VERY_VERBOSE 7
#ifndef ESPHOME_LOG_LEVEL
#define ESPHOME_LOG_LEVEL ESPHOME_LOG_LEVEL_VERBOSE
#endif

namespace esphome {
struct LogLine {
  char level;
  std::string text;
};
inline std::vector<LogLine> &test_log() {
  static std::vector<LogLine> lines;
  return lines;
}
__attribute__((format(printf, 2, 3))) inline void test_log_printf(char level, const char *fmt, ...) {
  char buf[512];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  test_log().push_back({level, buf});
}
}  // namespace esphome

#define ESP_LOGE(tag, ...) ::esphome::test_log_printf('E', __VA_ARGS__)
#define ESP_LOGW(tag, ...) ::esphome::test_log_printf('W', __VA_ARGS__)
#define ESP_LOGI(tag, ...) ::esphome::test_log_printf('I', __VA_ARGS__)
#define ESP_LOGCONFIG(tag, ...) ::esphome::test_log_printf('C', __VA_ARGS__)
#define ESP_LOGD(tag, ...) ::esphome::test_log_printf('D', __VA_ARGS__)
#define ESP_LOGV(tag, ...) ::esphome::test_log_printf('V', __VA_ARGS__)
#define YESNO(b) ((b) ? "YES" : "NO")
#define LOG_SENSOR(prefix, type, obj) \
  if ((obj) != nullptr) \
  ESP_LOGCONFIG("", "%s%s", prefix, type)
#define LOG_BINARY_SENSOR(prefix, type, obj) \
  if ((obj) != nullptr) \
  ESP_LOGCONFIG("", "%s%s", prefix, type)
#define LOG_UPDATE_INTERVAL(this) ESP_LOGCONFIG("", "  Update interval")
