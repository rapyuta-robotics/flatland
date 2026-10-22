#include "flatland_plugins/command_playback_schedule.h"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <sstream>

using namespace flatland_plugins::base_model;

namespace {
const char* kSchedule = R"({"version": 1, "drives": [
  {"name": "a", "start_s": 2.0, "length_s": 10.0, "events": [[0.0, 0.5, 0.1], [1.0, 1.0, 0.0], [3.0, 0.0, 0.0]]},
  {"name": "b", "start_s": 20.0, "length_s": 5.0, "events": [[0.5, -0.3, 0.2]]}]})";
}

TEST(Playback, ParsesAndFindsTheActiveDrive) {
  const auto d = ParseSchedule(YAML::Load(kSchedule));
  ASSERT_EQ(d.size(), 2u);
  EXPECT_EQ(d[0].name, "a");
  EXPECT_EQ(ActiveDrive(d, 1.98), -1);
  EXPECT_EQ(ActiveDrive(d, 2.0), 0);
  // the step that starts at the end of a drive belongs to no drive: the drive finished in the step before
  EXPECT_EQ(ActiveDrive(d, 11.98), 0);
  EXPECT_EQ(ActiveDrive(d, 12.0), -1);
  EXPECT_EQ(ActiveDrive(d, 12.02), -1);
  EXPECT_EQ(ActiveDrive(d, 22.0), 1);
}

TEST(Playback, CommandIsTheLastEventAtOrBeforeTheStepStart) {
  const auto d = ParseSchedule(YAML::Load(kSchedule));
  EXPECT_EQ(CommandAt(d[0], 0.0).v, 0.5);
  EXPECT_EQ(CommandAt(d[0], 0.98).v, 0.5);
  EXPECT_EQ(CommandAt(d[0], 1.0).v,
            1.0);  // an event applies at the step that starts at its time
  EXPECT_EQ(CommandAt(d[0], 1.02).v, 1.0);
  EXPECT_EQ(CommandAt(d[0], 3.0).v, 0.0);
  EXPECT_EQ(CommandAt(d[1], 0.2).v, 0.0);  // before the first event: zero
  EXPECT_EQ(CommandAt(d[1], 0.5).v, -0.3);
}

TEST(Playback, RefusesOverlapUnsortedAndWrongVersion) {
  EXPECT_THROW(ParseSchedule(YAML::Load(R"({"version": 2, "drives": []})")),
               PlaybackError);
  EXPECT_THROW(ParseSchedule(YAML::Load(R"({"version": 1, "drives": [
    {"name": "a", "start_s": 0.0, "length_s": 10.0, "events": []},
    {"name": "b", "start_s": 5.0, "length_s": 1.0, "events": []}]})")),
               PlaybackError);
  EXPECT_THROW(ParseSchedule(YAML::Load(R"({"version": 1, "drives": [
    {"name": "a", "start_s": 0.0, "length_s": 10.0, "events": [[1.0, 0, 0], [0.5, 0, 0]]}]})")),
               PlaybackError);
}

TEST(Playback, TrackFileRoundTrips) {
  const std::string path = ::testing::TempDir() + "track.json";
  WriteTrack(
      path, "a",
      {{0.0, 0.0, 0.0, 0.0}, {0.02, 0.012345678901234567, -1e-9, 3.14159}});
  const YAML::Node n = YAML::LoadFile(path);
  EXPECT_EQ(n["name"].as<std::string>(), "a");
  ASSERT_EQ(n["t"].size(), 2u);
  EXPECT_DOUBLE_EQ(n["x"][1].as<double>(), 0.012345678901234567);
  EXPECT_DOUBLE_EQ(n["y"][1].as<double>(), -1e-9);
  EXPECT_DOUBLE_EQ(n["yaw"][1].as<double>(), 3.14159);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
