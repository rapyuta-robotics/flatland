// Playback schedule for the validation replay: no ROS, no flatland. A schedule
// is a list of drives, each with a start time in simulated time, a length, and
// time-stamped commands relative to the start of the drive.
//
//   {"version": 1, "drives": [{"name": "d1", "start_s": 2.0, "length_s": 41.5,
//   "events": [[0.0, 0.2, 0.0], ...]}]}
//
// The command of a step is the last event with time <= (step start - drive
// start), as in the offline stepper's `events_to_steps`. A drive records the
// pose from its start until `length_s` after it.
#ifndef FLATLAND_PLUGINS_COMMAND_PLAYBACK_SCHEDULE_H
#define FLATLAND_PLUGINS_COMMAND_PLAYBACK_SCHEDULE_H

#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

namespace flatland_plugins {
namespace base_model {

struct PlaybackEvent {
  double t, v, w;
};

struct PlaybackDrive {
  std::string name;
  double start_s = 0.0;
  double length_s = 0.0;
  std::vector<PlaybackEvent> events;
};

class PlaybackError : public std::runtime_error {
 public:
  explicit PlaybackError(const std::string& m) : std::runtime_error(m) {}
};

inline std::vector<PlaybackDrive> ParseSchedule(const YAML::Node& doc) {
  if (!doc.IsMap() || !doc["version"].IsDefined() ||
      doc["version"].as<int>() != 1) {
    throw PlaybackError("playback schedule: unsupported version");
  }
  std::vector<PlaybackDrive> out;
  double previous_end = -1.0;
  for (const auto& d : doc["drives"]) {
    PlaybackDrive drive;
    drive.name = d["name"].as<std::string>();
    drive.start_s = d["start_s"].as<double>();
    drive.length_s = d["length_s"].as<double>();
    if (drive.start_s < previous_end)
      throw PlaybackError("playback schedule: drives overlap at " + drive.name);
    previous_end = drive.start_s + drive.length_s;
    double last_t = -1e300;
    for (const auto& e : d["events"]) {
      PlaybackEvent ev{e[0].as<double>(), e[1].as<double>(), e[2].as<double>()};
      if (ev.t < last_t)
        throw PlaybackError("playback schedule: events of " + drive.name +
                            " are not sorted");
      last_t = ev.t;
      drive.events.push_back(ev);
    }
    out.push_back(drive);
  }
  return out;
}

// Index of the drive that is running at simulated time `t` (start_s <= t <
// start_s + length_s), or -1. A step that starts at the end of a drive belongs
// to no drive: the drive finished in the step before.
inline int ActiveDrive(const std::vector<PlaybackDrive>& drives, double t) {
  for (size_t i = 0; i < drives.size(); ++i) {
    if (t >= drives[i].start_s - 1e-9 &&
        t < drives[i].start_s + drives[i].length_s - 1e-9)
      return static_cast<int>(i);
  }
  return -1;
}

// Command at `rel_t` seconds into the drive: the last event with time <= rel_t
// (+1e-9), zero before the first.
inline PlaybackEvent CommandAt(const PlaybackDrive& d, double rel_t) {
  PlaybackEvent out{rel_t, 0.0, 0.0};
  for (const PlaybackEvent& e : d.events) {
    if (e.t <= rel_t + 1e-9) {
      out = e;
    } else {
      break;
    }
  }
  out.t = rel_t;
  return out;
}

struct PoseSample {
  double t, x, y, yaw;
};

// One track file per drive, as validate.load_track reads it: t from the start
// of the drive.
inline void WriteTrack(const std::string& path, const std::string& name,
                       const std::vector<PoseSample>& s) {
  std::FILE* f = std::fopen(path.c_str(), "w");
  if (!f) throw PlaybackError("cannot write " + path);
  const char* keys[4] = {"t", "x", "y", "yaw"};
  std::fprintf(f, "{\"name\":\"%s\"", name.c_str());
  for (int k = 0; k < 4; ++k) {
    std::fprintf(f, ",\"%s\":[", keys[k]);
    for (size_t i = 0; i < s.size(); ++i) {
      const double v = k == 0   ? s[i].t
                       : k == 1 ? s[i].x
                       : k == 2 ? s[i].y
                                : s[i].yaw;
      std::fprintf(f, i ? ",%.17g" : "%.17g", v);
    }
    std::fprintf(f, "]");
  }
  std::fprintf(f, "}\n");
  std::fclose(f);
}

}  // namespace base_model
}  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_COMMAND_PLAYBACK_SCHEDULE_H
