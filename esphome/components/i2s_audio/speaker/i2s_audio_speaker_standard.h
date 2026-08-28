#pragma once

#ifdef USE_ESP32

#include "i2s_audio_speaker.h"

namespace esphome::i2s_audio {

enum class I2SCommFmt : uint8_t {
  STANDARD,  // Philips / I2S standard
  PCM,       // PCM short
  MSB,       // MSB / left-justified
};

/// @brief Standard I2S speaker implementation.
/// Outputs PCM audio data directly to an I2S DAC using the standard I2S protocol.
class I2SAudioSpeaker final : public I2SAudioSpeakerBase {
 public:
  void dump_config() override;

  void set_i2s_comm_fmt(I2SCommFmt fmt) { this->i2s_comm_fmt_ = fmt; }

  /// @brief This writer carries identity end to end, so it reports tagged renders.
  ///
  /// It never changes frame counts -- narrowing 32-bit samples to a 16-bit slot changes byte widths,
  /// not frames -- and it composes descriptors from the ring strictly in order, so a tag's position
  /// arithmetic survives from ``play()`` to the DMA completion event that reports the audio rendered.
  ///
  /// Unconditional, including while stopped: this answers whether the PLATFORM can report, exactly as
  /// render_latency() does, so one-shot feature detection on a not-yet-started speaker does not
  /// conclude the feature is missing.
  bool supports_render_tags() const override { return true; }

 protected:
  void run_speaker_task() override;
  esp_err_t start_i2s_driver(audio::AudioStreamInfo &audio_stream_info) override;

  I2SCommFmt i2s_comm_fmt_{I2SCommFmt::STANDARD};
};

}  // namespace esphome::i2s_audio

#endif  // USE_ESP32
