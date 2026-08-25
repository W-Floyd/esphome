#include <gtest/gtest.h>

#include "esphome/components/audio/audio.h"

namespace esphome::audio::testing {

// Duration conversions must hold for a whole buffer's worth of audio, not just one chunk of it.
// `frames * 1000000` in uint32 arithmetic overflows at 4295 frames -- 97.4 ms at 44.1 kHz, 89.5 ms
// at 48 kHz -- returning a value short by 2^32 / sample_rate per wrap, with no diagnostic. These
// pin the boundary, since nothing about a uint32_t frame count says a caller may not exceed it.

TEST(AudioStreamInfoFramesToMicroseconds, ExactForWholeSeconds) {
  EXPECT_EQ(AudioStreamInfo(16, 2, 44100).frames_to_microseconds(44100), 1000000u);
  EXPECT_EQ(AudioStreamInfo(16, 2, 48000).frames_to_microseconds(48000), 1000000u);
}

TEST(AudioStreamInfoFramesToMicroseconds, HoldsAcrossThe32BitProductBoundary) {
  const AudioStreamInfo info(16, 2, 44100);
  // 4294 frames is the last count whose 32-bit product does not overflow; 4295 is the first that
  // does. Before the 64-bit intermediate, the second of these returned 0 rather than 97392.
  EXPECT_EQ(info.frames_to_microseconds(4294), 97370u);
  EXPECT_EQ(info.frames_to_microseconds(4295), 97392u);
}

TEST(AudioStreamInfoFramesToMicroseconds, HoldsBeyondTwoWraps) {
  const AudioStreamInfo info(16, 2, 44100);
  EXPECT_EQ(info.frames_to_microseconds(8589), 194762u);
  EXPECT_EQ(info.frames_to_microseconds(8590), 194785u);
  // Ten seconds of audio: 2300 wraps of the old 32-bit product.
  EXPECT_EQ(info.frames_to_microseconds(441000), 10000000u);
}

TEST(AudioStreamInfoFramesToMicroseconds, BoundaryScalesWithSampleRate) {
  // The overflow threshold is a frame count, so it lands at a different DURATION per rate.
  const AudioStreamInfo info(16, 2, 48000);
  EXPECT_EQ(info.frames_to_microseconds(4294), 89458u);
  EXPECT_EQ(info.frames_to_microseconds(4295), 89479u);
  EXPECT_EQ(info.frames_to_microseconds(441000), 9187500u);
}

TEST(AudioStreamInfoFramesToMicroseconds, MonotonicAcrossTheBoundary) {
  // The clearest statement of the bug: a longer queue must never report a shorter duration.
  const AudioStreamInfo info(16, 2, 44100);
  uint32_t previous = 0;
  for (uint32_t frames = 4290; frames <= 8600; ++frames) {
    const uint32_t current = info.frames_to_microseconds(frames);
    EXPECT_GE(current, previous) << "not monotonic at " << frames << " frames";
    previous = current;
  }
}

TEST(AudioStreamInfoFramesToMicroseconds, ZeroAndOneFrame) {
  const AudioStreamInfo info(16, 2, 44100);
  EXPECT_EQ(info.frames_to_microseconds(0), 0u);
  EXPECT_EQ(info.frames_to_microseconds(1), 23u);  // 22.68 us, rounded
}

// Same 32-bit product, same fix. Reachable only past ~97 s of audio in a single call, so this is a
// latent case rather than an observed one, but the arithmetic is identical.
TEST(AudioStreamInfoFramesToMilliseconds, HoldsForVeryLargeFrameCounts) {
  const AudioStreamInfo info(16, 2, 44100);
  uint32_t frames = 441000000;  // 10000 s
  EXPECT_EQ(info.frames_to_milliseconds_with_remainder(&frames), 10000000u);
  EXPECT_EQ(frames, 0u);
}

TEST(AudioStreamInfoMsToBytes, HoldsForLongDurations) {
  // 16-bit stereo at 44.1 kHz: the 32-bit product wraps past ~24 s.
  EXPECT_EQ(AudioStreamInfo(16, 2, 44100).ms_to_bytes(60000), 10584000u);
}

}  // namespace esphome::audio::testing
