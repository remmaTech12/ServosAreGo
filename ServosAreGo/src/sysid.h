#pragma once
#include <Arduino.h>

namespace sysid {
struct Sample {
  uint32_t commandUs, sampleUs;
  int32_t position, speed;
  int16_t pwm, current;
  uint16_t voltage;
  uint8_t temperature;
};
struct Adapter {
  const char *model;
  int maxPosition, maxGain;
  const char *channels;
  bool torqueSwitch;
  bool (*inspect)(int id, unsigned long &kp);
  bool (*setGain)(int id, int value);
  bool (*setGoal)(int id, int ticks);
  bool (*setTorque)(int id, bool enabled);
  bool (*sample)(int id, Sample &sample);
};
// One trial owns the bus and its buffers until the host releases it.
class Trial {
  const Adapter &device;
  struct Point {
    int target;
    bool enabled;
  };
  Point *points = nullptr;
  Sample *samples = nullptr;
  int low, high;
  int id, count, period, gain, uploaded = 0, recorded = 0;
  unsigned long oldGain, startTime, runId = 0;
  bool running = false, torque = false, finished = false;
  const char *reason = "ready";

  void reply(bool ok, const char *message) {
    Serial.printf("@{\"re\":\"sysid\",\"ok\":%s,\"message\":\"%s\",\"run\":%lu,"
                  "\"count\":%d,\"done\":%s}\n",
                  ok ? "true" : "false", message, runId, recorded,
                  finished ? "true" : "false");
  }
  void finish(const char *result) {
    bool off = device.setTorque(id, 0);
    bool restored = device.setGain(id, oldGain);
    reason = !off        ? "torque off unconfirmed"
             : !restored ? "gain restore failed"
                         : result;
    running = false;
    finished = true;
  }

public:
  explicit Trial(const Adapter &adapter) : device(adapter) {}
  bool ownsBus() const { return points != nullptr; }
  void tick() {
    if (!running)
      return;
    int32_t elapsed = micros() - startTime;
    if (elapsed < recorded * period)
      return;
    if (elapsed >= (recorded + 1) * period) {
      finish("missed deadline");
      return;
    }
    Point point = points[recorded];
    Sample &sample = samples[recorded];
    if (point.enabled && !device.setGoal(id, point.target)) {
      finish("goal write failed");
      return;
    }
    if (point.enabled != torque && !device.setTorque(id, point.enabled)) {
      finish("torque write failed");
      return;
    }
    torque = point.enabled;
    sample.commandUs = micros() - startTime;
    uint32_t before = micros();
    if (!device.sample(id, sample) || sample.position < low ||
        sample.position > high) {
      finish("invalid feedback");
      return;
    }
    sample.sampleUs = before - startTime + (micros() - before) / 2;
    if (++recorded == count)
      finish("complete");
  }

  void command(const String &line) {
    char op[12] = {};
    unsigned long token = 0;
    int offset = 0;
    sscanf(line.c_str(), "@sysid %11s %lu %d", op, &token, &offset);
    String action(op);
    if (action == "inspect" && !running) {
      unsigned long kp;
      if (token > 252 || !device.inspect(token, kp)) {
        reply(false, "unsupported model or bench configuration");
        return;
      }
      Serial.printf("@{\"re\":\"sysid\",\"ok\":true,\"model\":\"%s\",\"kp\":%"
                    "lu,\"channels\":%s,\"torqueSwitch\":%s}\n",
                    device.model, kp, device.channels,
                    device.torqueSwitch ? "true" : "false");
      return;
    }
    if (action == "prepare" && !ownsBus()) {
      if (sscanf(line.c_str(), "@sysid prepare %d %d %d %d %d %d", &id, &count,
                 &period, &gain, &low, &high) != 6 ||
          low < 0 || high > device.maxPosition || low >= high || id < 0 ||
          id > 252 || count < 2 || count > 2400 || period < 5000 ||
          period > 50000 || gain < 1 || gain > device.maxGain ||
          !device.inspect(id, oldGain)) {
        reply(false, "invalid configuration");
        return;
      }
      points = (Point *)calloc(count, sizeof(Point));
      samples = (Sample *)calloc(count, sizeof(Sample));
      if (!points || !samples) {
        free(points);
        free(samples);
        points = nullptr;
        samples = nullptr;
        reply(false, "insufficient RAM");
        return;
      }
      ++runId;
      uploaded = recorded = 0;
      finished = false;
      reason = "ready";
      reply(true, reason);
      return;
    }
    if (!ownsBus() || token != runId) {
      reply(false, "unknown or busy trial");
      return;
    }
    if (action == "upload" && !running && !finished) {
      int pos =
          line.indexOf(' ', line.indexOf(' ', line.indexOf(' ', 7) + 1) + 1);
      if (offset < 0 || offset > uploaded || pos < 0) {
        reply(false, "invalid offset");
        return;
      }
      const char *p = line.c_str() + pos;
      while (*p) {
        int target, enabled, used;
        if (sscanf(p, " %d,%d%n", &target, &enabled, &used) != 2 ||
            target < low || target > high ||
            (!device.torqueSwitch && enabled == 0) ||
            (enabled != 0 && enabled != 1) || offset >= count) {
          reply(false, "invalid point");
          return;
        }
        if (offset < uploaded && (points[offset].target != target ||
                                  points[offset].enabled != bool(enabled))) {
          reply(false, "conflicting retry");
          return;
        }
        points[offset++] = {target, bool(enabled)};
        if (offset > uploaded)
          uploaded = offset;
        p += used;
      }
    } else if (action == "start" && !running && !finished &&
               uploaded == count) {
      if (!device.inspect(id, oldGain)) {
        reply(false, "preflight failed");
        return;
      }
      Sample initial{};
      if (!device.sample(id, initial) ||
          abs(initial.position - points[0].target) > 50) {
        reply(false, "place the relaxed arm at the calibrated start pose");
        return;
      }
      torque = points[0].enabled;
      if (!device.setGain(id, gain) || !device.setGoal(id, points[0].target) ||
          !device.setTorque(id, torque)) {
        finish("initialization failed");
        reply(false, reason);
        return;
      }
      startTime = micros() + 1000000;
      running = true;
      reason = "running";
    } else if (action == "abort" && running)
      finish("aborted");
    else if (action == "release" && !running) {
      free(points);
      free(samples);
      points = nullptr;
      samples = nullptr;
    } else if (action == "read" && finished && offset >= 0 &&
               offset < recorded) {
      Serial.print("@{\"re\":\"sysid\",\"ok\":true,\"rows\":[");
      for (int i = offset; i < recorded && i < offset + 24; ++i) {
        auto &s = samples[i];
        auto &q = points[i];
        Serial.printf("%s[%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d]",
                      i == offset ? "" : ",", s.commandUs, s.sampleUs, q.target,
                      q.enabled, s.position, s.speed, s.pwm, s.current,
                      s.voltage, s.temperature, i);
      }
      Serial.println("]}");
      return;
    } else if (action != "status") {
      reply(false, "invalid state");
      return;
    }
    reply(true, reason);
  }
};
} // namespace sysid
