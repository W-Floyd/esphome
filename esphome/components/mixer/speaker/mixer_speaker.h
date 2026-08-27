#pragma once

#ifdef USE_ESP32

#include "esphome/components/audio/audio.h"
#include "esphome/components/audio/audio_transfer_buffer.h"
#include "esphome/components/ring_buffer/ring_buffer.h"
#include "esphome/components/speaker/speaker.h"

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/static_task.h"

#include <ducking.h>  // esp-audio-libs

#include <freertos/event_groups.h>

#include <atomic>

namespace esphome::mixer_speaker {

/* Classes for mixing several source speaker audio streams and writing it to another speaker component.
 *  - Volume controls are passed through to the output speaker
 *  - Source speaker commands are signaled via event group bits and processed in its loop function to ensure thread
 * safety
 *  - Directly handles pausing at the SourceSpeaker level; pause state is not passed through to the output speaker.
 *  - Audio sent to the SourceSpeaker can have 8, 16, 24, or 32 bits per sample. Each source is converted to the output
 *    speaker's bit depth as it is mixed (or copied) into the output buffer.
 *  - Audio sent to the SourceSpeaker can have any number of channels. They are duplicated or ignored as needed to match
 *    the number of channels required for the output speaker.
 *  - In queue mode, the audio sent to the SourceSpeakers can have different sample rates.
 *  - In non-queue mode, the audio sent to the SourceSpeakers must have the same sample rates.
 *  - SourceSpeaker has an internal ring buffer. It also allocates a shared_ptr for an AudioTranserBuffer object.
 *  - Audio Data Flow:
 *      - Audio data played on a SourceSpeaker first writes to its internal ring buffer.
 *      - MixerSpeaker task temporarily takes shared ownership of each SourceSpeaker's AudioTransferBuffer.
 *      - MixerSpeaker calls SourceSpeaker's `process_data_from_source`, which transfers audio from the SourceSpeaker's
 *        ring buffer to its AudioTransferBuffer. Audio ducking is applied at this step.
 *      - In queue mode, MixerSpeaker prioritizes the earliest configured SourceSpeaker with audio data. Audio data is
 *        sent to the output speaker.
 *      - In non-queue mode, MixerSpeaker adds all the audio data in each SourceSpeaker into one stream that is written
 *        to the output speaker.
 */

class MixerSpeaker;

class SourceSpeaker final : public speaker::Speaker, public Component {
 public:
  void dump_config() override;
  void setup() override;
  void loop() override;

  size_t play(const uint8_t *data, size_t length, TickType_t ticks_to_wait) override;
  size_t play(const uint8_t *data, size_t length) override { return this->play(data, length, 0); }

  void start() override;
  void stop() override;
  void finish() override;

  bool has_buffered_data() const override;
  bool render_latency(audio::AudioDepth &depth) const override;
  bool buffered_audio(audio::AudioDepth &depth) const override;

  /// @brief Mute state changes are passed to the parent's output speaker
  void set_mute_state(bool mute_state) override;
  bool get_mute_state() override;

  /// @brief Volume state changes are passed to the parent's output speaker
  void set_volume(float volume) override;
  float get_volume() override;

  void set_pause_state(bool pause_state) override { this->pause_state_ = pause_state; }
  bool get_pause_state() const override { return this->pause_state_; }

  /// @brief Exposes the next ring buffer chunk (zero-copy) and ducks the freshly exposed bytes in place.
  /// If the source still has bytes from a prior partial consume, this is a no-op (those bytes were already
  /// ducked on the fill that exposed them).
  /// @param audio_source Locked shared_ptr to the audio source (must be valid, not null)
  /// @param ticks_to_wait FreeRTOS ticks to wait while waiting to read from the ring buffer.
  /// @return Number of bytes newly exposed from the ring buffer.
  size_t process_data_from_source(std::shared_ptr<audio::RingBufferAudioSource> &audio_source,
                                  TickType_t ticks_to_wait);

  /// @brief Sets the ducking level for the source speaker.
  /// @param decibel_reduction The dB reduction level. For example, 0 is no change, 10 is a reduction by 10 dB
  /// @param duration The number of milliseconds to transition from the current level to the new level
  void apply_ducking(uint8_t decibel_reduction, uint32_t duration);

  void set_buffer_duration(uint32_t buffer_duration_ms) { this->buffer_duration_ms_ = buffer_duration_ms; }
  void set_parent(MixerSpeaker *parent) { this->parent_ = parent; }
  void set_timeout(uint32_t ms) { this->timeout_ms_ = ms; }

  std::weak_ptr<audio::RingBufferAudioSource> get_audio_source() { return this->audio_source_; }

 protected:
  friend class MixerSpeaker;
  esp_err_t start_();
  void enter_stopping_state_();
  void send_command_(uint32_t command_bit, bool wake_loop = false);

  MixerSpeaker *parent_;

  // This source's ENTIRE depth -- its own queue plus everything downstream -- PUBLISHED AS ONE
  // SNAPSHOT BY THE MIXER TASK. Deliberately one seqlock rather than terms the reader sums with the
  // parent's: two loads could straddle a mixer iteration and produce a total that was never true at
  // any instant. RingBufferAudioSource is also single-consumer-thread by contract, so a reader must
  // not compute its own term.
  //
  // Staleness is bounded by one iteration (TASK_DELAY_MS) and is REPORTED. That mattered in practice:
  // a consumer differencing this against its own live written-minus-played counter disagreed by whole
  // audio chunks, because the chunks it had pushed since the snapshot were in its accumulator and not
  // in here. The instant is what lets it line the two up.
  audio::DepthPublisher depth_;
  int64_t depth_debug_last_us_{0};  // TEMPORARY: time-throttles the DEPTH diagnostic
  // TEMPORARY DIAGNOSTIC: cumulative frames into and out of this source's ring, for the conservation
  // check received == consumed + still-held.
  std::atomic<uint32_t> dbg_received_frames_{0};
  std::atomic<uint32_t> dbg_consumed_frames_{0};
  std::shared_ptr<audio::RingBufferAudioSource> audio_source_;
  std::weak_ptr<ring_buffer::RingBuffer> ring_buffer_;

  uint32_t buffer_duration_ms_;
  uint32_t last_seen_data_ms_{0};
  /// Throttle for the not-accepting-audio diagnostic; see SourceSpeaker::play().
  uint32_t dbg_state_log_ms_{0};
  optional<uint32_t> timeout_ms_;
  bool stop_gracefully_{false};

  bool pause_state_{false};

  esp_audio_libs::ducking::DuckingState ducking_state_{};

  std::atomic<uint32_t> pending_playback_frames_{0};
  std::atomic<uint32_t> playback_delay_frames_{0};  // Frames in output pipeline when this source started contributing
  std::atomic<bool> has_contributed_{false};        // Tracks if source has contributed during this session

  EventGroupHandle_t event_group_{nullptr};
  uint32_t stopping_start_ms_{0};
};

class MixerSpeaker final : public Component {
 public:
  /// @brief Latency of everything past the source rings -- the task-local output transfer buffer
  /// plus the output speaker -- in microseconds. Published by the mixer task once per iteration.
  uint32_t get_downstream_latency_us() const { return this->downstream_latency_us_.load(std::memory_order_acquire); }
  uint32_t get_downstream_audio_us() const { return this->downstream_audio_us_.load(std::memory_order_acquire); }
  /// @brief The part of the downstream LATENCY that does not decay with age -- the sink's DMA span.
  /// See AudioDepth::render_nondraining_us. Passed straight through: this mixer's transfer buffer
  /// drains like any queue, so it contributes nothing of its own.
  uint32_t get_downstream_nondraining_us() const {
    return this->downstream_nondraining_us_.load(std::memory_order_acquire);
  }
  /// @brief The OLDEST instant contributing to the downstream terms above -- the sink's own snapshot
  /// instant, which is older than the transfer buffer read that accompanies it. A total is only as
  /// current as its stalest term, and the drain has to be measured from that one.
  /// @note Mixer task only, like the setter. Never crosses a task boundary, so it needs no atomic.
  int64_t get_downstream_as_of_us() const { return this->downstream_as_of_us_; }
  uint32_t get_dbg_xfer_us() const { return this->dbg_xfer_us_; }
  uint32_t get_dbg_sink_queued_us() const { return this->dbg_sink_queued_us_; }
  uint32_t get_dbg_sink_dma_us() const { return this->dbg_sink_dma_us_; }
  uint32_t get_dbg_sink_received() const { return this->dbg_sink_received_; }
  /// @brief Audio that has left the transfer buffer but is not yet in the sink's published
  /// snapshot, in us. See where it is computed for why the total is wrong without it.
  /// @note Mixer task only.
  uint32_t get_dbg_sink_inflight_us() const { return this->dbg_sink_inflight_us_; }
  /// @brief The sink's cumulative padded-silence frames, forwarded unchanged. @note Mixer task only.
  uint32_t get_dbg_sink_padded_frames() const { return this->dbg_sink_padded_frames_; }

  void dump_config() override;
  void setup() override;
  void loop() override;

  void init_source_speakers(size_t count) { this->source_speakers_.init(count); }
  void add_source_speaker(SourceSpeaker *source_speaker) { this->source_speakers_.push_back(source_speaker); }

  /// @brief Starts the mixer task. Called by a source speaker giving the current audio stream information
  /// @param stream_info The calling source speaker's audio stream information
  /// @return ESP_ERR_INVALID_ARG if the incoming stream is incompatible to be mixed with the other input audio stream
  ///         ESP_OK if the incoming stream is compatible and the mixer task starts
  esp_err_t start(audio::AudioStreamInfo &stream_info);

  void set_output_channels(uint8_t output_channels) { this->output_channels_ = output_channels; }
  void set_output_bits_per_sample(uint8_t output_bits_per_sample) {
    this->output_bits_per_sample_ = output_bits_per_sample;
  }
  void set_output_speaker(speaker::Speaker *speaker) { this->output_speaker_ = speaker; }
  void set_queue_mode(bool queue_mode) { this->queue_mode_ = queue_mode; }
  void set_task_stack_in_psram(bool task_stack_in_psram) { this->task_stack_in_psram_ = task_stack_in_psram; }

  speaker::Speaker *get_output_speaker() const { return this->output_speaker_; }

  /// @brief Returns the current number of frames in the output pipeline (written but not yet played)
  uint32_t get_frames_in_pipeline() const { return this->frames_in_pipeline_.load(std::memory_order_acquire); }

 protected:
  static void audio_mixer_task(void *params);

  EventGroupHandle_t event_group_{nullptr};

  FixedVector<SourceSpeaker *> source_speakers_;
  speaker::Speaker *output_speaker_{nullptr};

  uint8_t output_bits_per_sample_;
  uint8_t output_channels_;
  bool queue_mode_;
  bool task_stack_in_psram_{false};

  StaticTask task_;

  // Everything past the source rings -- the task-local output transfer buffer plus the output
  // speaker -- as a duration, published by the mixer task. Held here rather than computed on demand
  // because the transfer buffer is task-local and unreachable from any other thread.
  std::atomic<uint32_t> downstream_latency_us_{0};
  std::atomic<uint32_t> downstream_nondraining_us_{0};
  std::atomic<uint32_t> downstream_audio_us_{0};
  // Written by the mixer task alongside the two terms above and read back by the mixer task one call
  // later, in process_data_from_source(). It never crosses a task boundary, so it is a plain member --
  // a 64-bit atomic is not lock-free on these targets and there is nothing here to make lock-free.
  int64_t downstream_as_of_us_{0};
  // TEMPORARY DIAGNOSTIC: the sink's own split and this mixer's transfer buffer, captured with the
  // reading above so a consumer sees every term at one instant. Mixer task only, like the rest.
  uint32_t dbg_xfer_us_{0};
  uint32_t dbg_sink_queued_us_{0};
  uint32_t dbg_sink_dma_us_{0};
  uint32_t dbg_sink_received_{0};
  uint32_t dbg_sink_inflight_us_{0};
  uint32_t dbg_sink_padded_frames_{0};
  /// @brief Cumulative output-format frames this mixer has handed to the sink. The sink publishes
  /// the matching count of what it has ACCEPTED (``dbg_sink_received``), so the difference is
  /// exactly the audio in flight between the two -- which is the term the composite depth used to
  /// omit. Wraps at 2^32 frames (~27 h at 44.1 kHz) in step with the sink's counter, and unsigned
  /// subtraction is correct across the wrap. Mixer task only.
  uint32_t dbg_written_to_sink_frames_{0};
  int64_t depth_debug_last_us_{0};  // TEMPORARY: time-throttles the DEPTH diagnostic
  optional<audio::AudioStreamInfo> audio_stream_info_;

  std::atomic<uint32_t> frames_in_pipeline_{0};  // Frames written to output but not yet played
  uint32_t all_stopped_since_ms_{0};             // Debounce transient all-stopped windows before stopping task
};

}  // namespace esphome::mixer_speaker

#endif
