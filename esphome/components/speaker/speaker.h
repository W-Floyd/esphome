#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#ifdef USE_ESP32
#include <freertos/FreeRTOS.h>
#endif

#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"

#include "esphome/components/audio/audio.h"
#ifdef USE_AUDIO_DAC
#include "esphome/components/audio_dac/audio_dac.h"
#endif

namespace esphome::speaker {

enum State : uint8_t {
  STATE_STOPPED = 0,
  STATE_STARTING,
  STATE_RUNNING,
  STATE_STOPPING,
};

class Speaker {
 public:
#ifdef USE_ESP32
  /// @brief Plays the provided audio data.
  /// If the speaker component doesn't implement this method, it falls back to the play method without this parameter.
  /// @param data Audio data in the format specified by ``set_audio_stream_info`` method.
  /// @param length The length of the audio data in bytes.
  /// @param ticks_to_wait The FreeRTOS ticks to wait before writing as much data as possible to the ring buffer.
  /// @return The number of bytes that were actually written to the speaker's internal buffer.
  virtual size_t play(const uint8_t *data, size_t length, TickType_t ticks_to_wait) {
    return this->play(data, length);
  };
#endif

  /// @brief Plays the provided audio data.
  /// If the audio stream is not the default defined in "esphome/core/audio.h" and the speaker component implements it,
  /// then this should be called after calling ``set_audio_stream_info``.
  /// @param data Audio data in the format specified by ``set_audio_stream_info`` method.
  /// @param length The length of the audio data in bytes.
  /// @return The number of bytes that were actually written to the speaker's internal buffer.
  virtual size_t play(const uint8_t *data, size_t length) = 0;

  size_t play(const std::vector<uint8_t> &data) { return this->play(data.data(), data.size()); }

  virtual void start() = 0;
  virtual void stop() = 0;
  // In compare between *STOP()* and *FINISH()*; *FINISH()* will stop after emptying the play buffer,
  // while *STOP()* will break directly.
  // When finish() is not implemented on the platform component it should just do a normal stop.
  virtual void finish() { this->stop(); }

  // Pauses processing incoming audio. Needs to be implemented specifically per speaker component
  virtual void set_pause_state(bool pause_state) {}
  virtual bool get_pause_state() const { return false; }

  virtual bool has_buffered_data() const = 0;

  /// @brief How long from now until audio handed to this speaker would be rendered, if the platform
  /// can report it.
  ///
  /// has_buffered_data() answers "any or none", which is enough to drain but not enough to schedule.
  /// A synchronised consumer can count what it pushed and be told what was played, but the fill
  /// sitting between those two points is otherwise invisible, so a pipeline restart at an unobserved
  /// fill level leaves playback offset by that amount with every other metric reading nominal.
  ///
  /// This is LATENCY, not "audio remaining". It includes buffering that holds no caller audio at all
  /// -- notably i2s DMA descriptors preloaded with silence, which still take time to clock out -- so
  /// while a speaker is running it does NOT fall to zero as the queue empties. Use has_buffered_data()
  /// to drain; a loop waiting for this to reach zero would never terminate.
  ///
  /// Reported as a duration because bytes and frames do not compose across a chain. A mixer may widen
  /// a mono source to stereo, a resampler changes the frame rate outright, and the i2s slot width may
  /// be narrower than the incoming stream, so bytes at one stage cannot be added to bytes at the next
  /// and neither can frames across a resampler. Each stage converts its own buffers with its own
  /// stream info and sums the results.
  ///
  /// Implementations that buffer on a task publish a snapshot rather than reading their queues live,
  /// so the value is internally consistent -- never a sum of terms taken at different instants -- but
  /// may lag by up to one iteration of that task. In steady state the latency is near-constant and
  /// the lag costs nothing; during a transient, such as a refill after starvation, the value can
  /// trail the truth by roughly one buffer period.
  ///
  /// @param microseconds Set to the latency on success. Untouched on failure. Bounded by the buffer
  /// sizes involved, so a uint32_t is ample; it is not a general-purpose timer.
  /// @return false only when the platform CANNOT report -- never merely because it is empty or
  /// stopped, both of which report true with a real value. A caller doing one-shot feature detection
  /// on a not-yet-started speaker must not conclude the platform is unsupported.
  virtual bool render_latency(uint32_t & /*microseconds*/) const { return false; }

  /// @brief How much of the CALLER'S OWN audio this speaker still holds, as a duration, if the
  /// platform can report it.
  ///
  /// Distinct from render_latency() and the distinction matters. render_latency() answers "when will
  /// audio handed over now be heard", so it counts every delay ahead of that audio -- including i2s
  /// DMA descriptors padded with silence, which hold none of the caller's audio but still take time
  /// to clock out. This answers "how much of what I gave you is left", which excludes that padding.
  ///
  /// A caller that tracks what it wrote and is told what was played needs THIS one to check its own
  /// accounting: comparing its outstanding count against render_latency() differences two different
  /// quantities and yields the padding as a spurious residue. It needs render_latency() to schedule.
  /// Both, for the two different questions.
  ///
  /// @param microseconds Set to the duration on success. Untouched on failure.
  /// @return false when the platform cannot report -- distinct from reporting zero.
  virtual bool buffered_audio(uint32_t & /*microseconds*/) const { return false; }

  bool is_running() const { return this->state_ == STATE_RUNNING; }
  bool is_stopped() const { return this->state_ == STATE_STOPPED; }

  // Volume control is handled by a configured audio dac component. Individual speaker components can
  // override and implement in software if an audio dac isn't available.
  virtual void set_volume(float volume) {
    this->volume_ = volume;
#ifdef USE_AUDIO_DAC
    if (this->audio_dac_ != nullptr) {
      this->audio_dac_->set_volume(volume);
    }
#endif
  };
  virtual float get_volume() { return this->volume_; }

  virtual void set_mute_state(bool mute_state) {
    this->mute_state_ = mute_state;
#ifdef USE_AUDIO_DAC
    if (this->audio_dac_) {
      if (mute_state) {
        this->audio_dac_->set_mute_on();
      } else {
        this->audio_dac_->set_mute_off();
      }
    }
#endif
  }
  virtual bool get_mute_state() { return this->mute_state_; }

#ifdef USE_AUDIO_DAC
  void set_audio_dac(audio_dac::AudioDac *audio_dac) { this->audio_dac_ = audio_dac; }
#endif

  void set_audio_stream_info(const audio::AudioStreamInfo &audio_stream_info) {
    this->audio_stream_info_ = audio_stream_info;
  }

  audio::AudioStreamInfo &get_audio_stream_info() { return this->audio_stream_info_; }

  /// Callback function for sending the duration of the audio written to the speaker since the last callback.
  /// Parameters:
  ///   - Frames played
  ///   - System time in microseconds when the frames were written to the DAC
  template<typename F> void add_audio_output_callback(F &&callback) {
    this->audio_output_callback_.add(std::forward<F>(callback));
  }

 protected:
  State state_{STATE_STOPPED};
  audio::AudioStreamInfo audio_stream_info_;
  float volume_{1.0f};
  bool mute_state_{false};

#ifdef USE_AUDIO_DAC
  audio_dac::AudioDac *audio_dac_{nullptr};
#endif

  CallbackManager<void(uint32_t, int64_t)> audio_output_callback_{};
};

}  // namespace esphome::speaker
