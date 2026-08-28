#include "mixer_speaker.h"

#ifdef USE_ESP32

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include "esp_timer.h"

#include <cinttypes>

#include <mixer.h>        // esp-audio-libs
#include <pcm_convert.h>  // esp-audio-libs

#include <algorithm>
#include <cstring>

namespace esphome::mixer_speaker {

static const UBaseType_t MIXER_TASK_PRIORITY = 10;

static const uint32_t STOPPING_TIMEOUT_MS = 5000;
static const uint32_t TRANSFER_BUFFER_DURATION_MS = 50;
static const uint32_t TASK_DELAY_MS = 25;
static const uint32_t MIXER_AUTO_STOP_DEBOUNCE_MS = 200;

static const size_t TASK_STACK_SIZE = 4096;

static const char *const TAG = "speaker_mixer";

// Event bits for SourceSpeaker command processing
enum SourceSpeakerEventBits : uint32_t {
  SOURCE_SPEAKER_COMMAND_START = (1 << 0),
  SOURCE_SPEAKER_COMMAND_STOP = (1 << 1),
  SOURCE_SPEAKER_COMMAND_FINISH = (1 << 2),
};

// Event bits for mixer task control and state
enum MixerTaskEventBits : uint32_t {
  MIXER_TASK_COMMAND_START = (1 << 0),
  MIXER_TASK_COMMAND_STOP = (1 << 1),
  MIXER_TASK_STATE_STARTING = (1 << 10),
  MIXER_TASK_STATE_RUNNING = (1 << 11),
  MIXER_TASK_STATE_STOPPING = (1 << 12),
  MIXER_TASK_STATE_STOPPED = (1 << 13),
  MIXER_TASK_ERR_ESP_NO_MEM = (1 << 19),
  MIXER_TASK_ALL_BITS = 0x00FFFFFF,  // All valid FreeRTOS event group bits
};

static inline uint32_t atomic_subtract_clamped(std::atomic<uint32_t> &var, uint32_t amount) {
  uint32_t current = var.load(std::memory_order_acquire);
  uint32_t subtracted = 0;
  if (current > 0) {
    uint32_t new_value;
    do {
      subtracted = std::min(amount, current);
      new_value = current - subtracted;
    } while (!var.compare_exchange_weak(current, new_value, std::memory_order_release, std::memory_order_acquire));
  }
  return subtracted;
}

static bool create_event_group(EventGroupHandle_t &event_group, Component *component) {
  event_group = xEventGroupCreate();
  if (event_group == nullptr) {
    ESP_LOGE(TAG, "Failed to create event group");
    component->mark_failed();
    return false;
  }
  return true;
}

void SourceSpeaker::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Mixer Source Speaker\n"
                "  Buffer Duration: %" PRIu32 " ms",
                this->buffer_duration_ms_);
  if (this->timeout_ms_.has_value()) {
    ESP_LOGCONFIG(TAG, "  Timeout: %" PRIu32 " ms", this->timeout_ms_.value());
  } else {
    ESP_LOGCONFIG(TAG, "  Timeout: never");
  }
}

void SourceSpeaker::setup() {
  if (!create_event_group(this->event_group_, this)) {
    return;
  }

  // Start with loop disabled since we begin in STATE_STOPPED with no pending commands
  this->disable_loop();

  this->parent_->get_output_speaker()->add_audio_output_callback([this](uint32_t new_frames, int64_t write_timestamp) {
    // First, drain the playback delay (frames in pipeline before this source started contributing)
    uint32_t delay_to_drain = atomic_subtract_clamped(this->playback_delay_frames_, new_frames);
    uint32_t remaining_frames = new_frames - delay_to_drain;

    // Then, count towards this source's pending playback frames
    if (remaining_frames > 0) {
      uint32_t speakers_playback_frames = atomic_subtract_clamped(this->pending_playback_frames_, remaining_frames);
      if (speakers_playback_frames > 0) {
        this->audio_output_callback_(speakers_playback_frames, write_timestamp);
      }
    }
  });
}

void SourceSpeaker::loop() {
  uint32_t event_bits = xEventGroupGetBits(this->event_group_);

  // Process commands with priority: STOP > FINISH > START
  // This ensures stop commands take precedence over conflicting start commands
  if (event_bits & SOURCE_SPEAKER_COMMAND_STOP) {
    if (this->state_ == speaker::STATE_RUNNING) {
      // Clear both STOP and START bits - stop takes precedence
      xEventGroupClearBits(this->event_group_, SOURCE_SPEAKER_COMMAND_STOP | SOURCE_SPEAKER_COMMAND_START);
      this->enter_stopping_state_();
    } else if (this->state_ == speaker::STATE_STOPPED) {
      // Already stopped, just clear the command bits
      xEventGroupClearBits(this->event_group_, SOURCE_SPEAKER_COMMAND_STOP | SOURCE_SPEAKER_COMMAND_START);
    }
    // Leave bits set if transitioning states (STARTING/STOPPING) - will be processed once state allows
  } else if (event_bits & SOURCE_SPEAKER_COMMAND_FINISH) {
    if (this->state_ == speaker::STATE_RUNNING) {
      xEventGroupClearBits(this->event_group_, SOURCE_SPEAKER_COMMAND_FINISH);
      this->stop_gracefully_ = true;
    } else if (this->state_ == speaker::STATE_STOPPED) {
      // Already stopped, just clear the command bit
      xEventGroupClearBits(this->event_group_, SOURCE_SPEAKER_COMMAND_FINISH);
    }
    // Leave bit set if transitioning states - will be processed once state allows
  } else if (event_bits & SOURCE_SPEAKER_COMMAND_START) {
    if (this->state_ == speaker::STATE_STOPPED) {
      xEventGroupClearBits(this->event_group_, SOURCE_SPEAKER_COMMAND_START);
      this->state_ = speaker::STATE_STARTING;
    } else if (this->state_ == speaker::STATE_RUNNING) {
      // Already running, just clear the command bit
      xEventGroupClearBits(this->event_group_, SOURCE_SPEAKER_COMMAND_START);
    }
    // Leave bit set if transitioning states - will be processed once state allows
  }
  // Process state machine
  switch (this->state_) {
    case speaker::STATE_STARTING: {
      esp_err_t err = this->start_();
      if (err == ESP_OK) {
        this->pending_playback_frames_.store(0, std::memory_order_release);  // reset pending playback frames
        this->playback_delay_frames_.store(0, std::memory_order_release);    // reset playback delay
        this->has_contributed_.store(false, std::memory_order_release);      // reset contribution tracking
        this->state_ = speaker::STATE_RUNNING;
        this->stop_gracefully_ = false;
        this->last_seen_data_ms_ = millis();
        this->status_clear_error();
      } else {
        switch (err) {
          case ESP_ERR_NO_MEM:
            this->status_set_error(LOG_STR("Not enough memory"));
            break;
          case ESP_ERR_NOT_SUPPORTED:
            this->status_set_error(LOG_STR("Unsupported bit depth"));
            break;
          case ESP_ERR_INVALID_ARG:
            this->status_set_error(LOG_STR("Incompatible audio streams"));
            break;
          case ESP_ERR_INVALID_STATE:
            this->status_set_error(LOG_STR("Task failed"));
            break;
          default:
            this->status_set_error(LOG_STR("Failed"));
            break;
        }

        this->enter_stopping_state_();
      }
      break;
    }
    case speaker::STATE_RUNNING:
      if (!this->audio_source_->has_buffered_data() &&
          (this->pending_playback_frames_.load(std::memory_order_acquire) == 0)) {
        // No audio data in buffer waiting to get mixed and no frames are pending playback
        if ((this->timeout_ms_.has_value() && ((millis() - this->last_seen_data_ms_) > this->timeout_ms_.value())) ||
            this->stop_gracefully_) {
          // Timeout exceeded or graceful stop requested
          this->enter_stopping_state_();
        }
      }
      break;
    case speaker::STATE_STOPPING: {
      if ((this->parent_->get_output_speaker()->get_pause_state()) ||
          ((millis() - this->stopping_start_ms_) > STOPPING_TIMEOUT_MS)) {
        // If parent speaker is paused or if the stopping timeout is exceeded, force stop the output speaker
        this->parent_->get_output_speaker()->stop();
      }

      if (this->parent_->get_output_speaker()->is_stopped() ||
          (this->pending_playback_frames_.load(std::memory_order_acquire) == 0)) {
        // Output speaker is stopped OR all pending playback frames have played
        this->pending_playback_frames_.store(0, std::memory_order_release);
        this->stop_gracefully_ = false;

        this->state_ = speaker::STATE_STOPPED;
      }
      break;
    }
    case speaker::STATE_STOPPED:
      // Re-check event bits for any new commands that may have arrived
      event_bits = xEventGroupGetBits(this->event_group_);
      if (!(event_bits &
            (SOURCE_SPEAKER_COMMAND_START | SOURCE_SPEAKER_COMMAND_STOP | SOURCE_SPEAKER_COMMAND_FINISH))) {
        // No pending commands, disable loop to save CPU cycles
        this->disable_loop();
      }
      break;
  }
}

size_t SourceSpeaker::play(const uint8_t *data, size_t length, TickType_t ticks_to_wait) {
  // WEDGE DIAGNOSTIC. A write requests a start only when the speaker is STOPPED. In a transitional
  // state it neither starts nor writes -- the weak ring-buffer reference below has expired, so the
  // write returns 0 with no error reported anywhere, and the caller cannot tell "not started yet"
  // from "refusing data".
  //
  // Measured on a forced reconnect: when the speaker reached STOPPED the very next play() started
  // it and the board recovered in ~5 s ("START requested" in the log). When it wedged, no start was
  // EVER requested -- so it never reached STOPPED, and the mixer task that would advance it had
  // already been deallocated. This says which state it is actually sitting in.
  if (this->is_stopped()) {
    this->start();
  } else if (this->state_ != speaker::STATE_RUNNING) {
    const uint32_t now = millis();
    if (now - this->dbg_state_log_ms_ >= 1000) {
      this->dbg_state_log_ms_ = now;
      ESP_LOGW(TAG, "SourceSpeaker not accepting audio: state=%d ring_valid=%d -- no start will be requested",
               static_cast<int>(this->state_), this->ring_buffer_.lock().use_count() > 0 ? 1 : 0);
    }
  }
  size_t bytes_written = 0;
  std::shared_ptr<ring_buffer::RingBuffer> temp_ring_buffer = this->ring_buffer_.lock();
  if (temp_ring_buffer.use_count() > 0) {
    // Only write to the ring buffer if the reference is valid
    bytes_written = temp_ring_buffer->write_without_replacement(data, length, ticks_to_wait);
    // TEMPORARY DIAGNOSTIC: what the ring actually took.
    this->dbg_received_frames_.fetch_add(this->audio_stream_info_.bytes_to_frames(bytes_written),
                                         std::memory_order_relaxed);
    // Bind any pending tag at the position the audio actually landed at, not the position it was
    // offered at: the ring may take less than was offered, and a short write counted in full would
    // slide every later tag forward by the shortfall.
    this->tag_track_.note_written(this->audio_stream_info_.bytes_to_frames(bytes_written));
    if (bytes_written > 0) {
      this->last_seen_data_ms_ = millis();
    }
  } else {
    // Delay to avoid repeatedly hammering while waiting for the speaker to start
    vTaskDelay(ticks_to_wait);
  }
  return bytes_written;
}

void SourceSpeaker::send_command_(uint32_t command_bit, bool wake_loop) {
  this->enable_loop_soon_any_context();
  uint32_t event_bits = xEventGroupGetBits(this->event_group_);
  if (!(event_bits & command_bit)) {
    xEventGroupSetBits(this->event_group_, command_bit);
    if (wake_loop) {
      App.wake_loop_threadsafe();
    }
  }
}

void SourceSpeaker::start() { this->send_command_(SOURCE_SPEAKER_COMMAND_START, true); }

esp_err_t SourceSpeaker::start_() {
  const size_t bytes_per_frame = this->audio_stream_info_.frames_to_bytes(1);
  // Round the ring buffer size down to a multiple of bytes_per_frame so the wrap boundary stays frame-aligned and
  // avoids unnecessary single-frame splices.
  const size_t ring_buffer_size =
      (this->audio_stream_info_.ms_to_bytes(this->buffer_duration_ms_) / bytes_per_frame) * bytes_per_frame;
  if (this->audio_source_.use_count() == 0) {
    std::shared_ptr<ring_buffer::RingBuffer> temp_ring_buffer = this->ring_buffer_.lock();
    if (!temp_ring_buffer) {
      temp_ring_buffer = ring_buffer::RingBuffer::create(ring_buffer_size);
      // Restart both ends of the tag stream before the ring is reachable. play() can only find the
      // ring through this weak_ptr, so until it is assigned there is no producer to race.
      this->tag_track_.reset();
      this->tag_consumed_frames_ = 0;
      this->ring_buffer_ = temp_ring_buffer;
    }

    if (!temp_ring_buffer) {
      return ESP_ERR_NO_MEM;
    }

    std::unique_ptr<audio::RingBufferAudioSource> source = audio::RingBufferAudioSource::create(
        temp_ring_buffer, this->audio_stream_info_.ms_to_bytes(TRANSFER_BUFFER_DURATION_MS),
        static_cast<uint8_t>(bytes_per_frame));
    if (source == nullptr) {
      return ESP_ERR_NO_MEM;
    }
    this->audio_source_ = std::move(source);
  }

  return this->parent_->start(this->audio_stream_info_);
}

void SourceSpeaker::stop() { this->send_command_(SOURCE_SPEAKER_COMMAND_STOP); }

void SourceSpeaker::finish() { this->send_command_(SOURCE_SPEAKER_COMMAND_FINISH); }

bool SourceSpeaker::has_buffered_data() const {
  return ((this->audio_source_.use_count() > 0) && this->audio_source_->has_buffered_data());
}

void SourceSpeaker::set_mute_state(bool mute_state) {
  this->mute_state_ = mute_state;
  this->parent_->get_output_speaker()->set_mute_state(mute_state);
}

bool SourceSpeaker::get_mute_state() { return this->parent_->get_output_speaker()->get_mute_state(); }

void SourceSpeaker::set_volume(float volume) {
  this->volume_ = volume;
  this->parent_->get_output_speaker()->set_volume(volume);
}

float SourceSpeaker::get_volume() { return this->parent_->get_output_speaker()->get_volume(); }

bool SourceSpeaker::render_latency(audio::AudioDepth &depth) const {
  // One coherent read of a total the mixer task published in a single seqlock section, so the reader
  // cannot observe a mix of instants, and never reaches into the source (single-consumer-thread) or
  // the task-local transfer buffer.
  //
  // Durations are also the only thing that CAN be combined here: this source's queue is in its own
  // stream format while everything downstream is in the mixer's output format, and queue mode allows
  // the two to differ in channel count and sample rate. A mono source feeding a stereo mixer would
  // double-weight the downstream term if bytes were added.
  if (this->parent_ == nullptr) {
    return false;
  }
  return this->depth_.read_render(depth);
}

bool SourceSpeaker::buffered_audio(audio::AudioDepth &depth) const {
  if (this->parent_ == nullptr) {
    return false;
  }
  return this->depth_.read_audio(depth);
}

size_t SourceSpeaker::process_data_from_source(std::shared_ptr<audio::RingBufferAudioSource> &audio_source,
                                               TickType_t ticks_to_wait) {
  // Publish the WHOLE latency in one store, before anything else in this function can return: this
  // source's queue plus everything downstream, the latter already computed once for this iteration by
  // the mixer task just above the loop that calls us. A reader then does a single load and cannot
  // observe a mix of instants -- summing two atomics could catch this source after a consume but the
  // downstream term before the matching transfer, under-reporting by up to one mix chunk, which is
  // exactly the kind of error a one-shot re-baseline cannot recover from.
  //
  // Published from the thread that owns the source, because RingBufferAudioSource is
  // single-consumer-thread by contract. Staleness is bounded by one mixer iteration.
  const uint32_t own_us = this->audio_stream_info_.frames_to_microseconds(
      this->audio_stream_info_.bytes_to_frames(audio_source->buffered_bytes()));
  // Stamped with the OLDEST instant in the total, which is the sink's snapshot instant rather than
  // now: this source's ring is read here, but the downstream terms describe a moment already past, and
  // a total is only as current as its stalest term. Reporting `now` would tell a consumer the reading
  // is fresh when the part of it that DRAINS is not, and the drain is what has to be corrected for.
  //
  // Audio moving BETWEEN stages does not change the total, so a mixed-age sum is not itself an error:
  // only what enters at the top or renders at the bottom moves the number, and the bottom is what this
  // instant describes.
  this->depth_.publish(own_us + this->parent_->get_downstream_latency_us(),
                       own_us + this->parent_->get_downstream_audio_us(),
                       this->parent_->get_downstream_as_of_us(), own_us, this->parent_->get_dbg_xfer_us(),
                       this->parent_->get_dbg_sink_queued_us(), this->parent_->get_dbg_sink_dma_us(),
                       this->dbg_received_frames_.load(std::memory_order_relaxed),
                       this->dbg_consumed_frames_.load(std::memory_order_relaxed),
                       this->parent_->get_dbg_sink_received(), this->parent_->get_downstream_nondraining_us(),
                       this->parent_->get_dbg_sink_inflight_us(), this->parent_->get_dbg_sink_padded_frames());

  // TEMPORARY DIAGNOSTIC: see the matching line in the mixer task. Remove once explained.
  //
  // `pending` is the referee. pending_playback_frames_ is the mixer's OWN count of frames it has
  // consumed from this source's ring but not yet reported as played, maintained independently of
  // every duration in this chain: incremented where the mix happens, decremented by the output
  // speaker's callback. It is therefore what `xfer + sink` OUGHT to equal, in frames.
  //
  // If pending exceeds the reported xfer + sink, the reported depths are missing audio the mixer
  // knows it is holding, and the fault is on the measurement side. If they agree, the consumer's
  // own pushed-minus-played is the side that is high. One line settles it either way.
  const int64_t depth_debug_now = esp_timer_get_time();
  if (depth_debug_now - this->depth_debug_last_us_ >= 1000000) {
    this->depth_debug_last_us_ = depth_debug_now;
    const uint32_t pending_us =
        this->audio_stream_info_.frames_to_microseconds(this->pending_playback_frames_.load(std::memory_order_acquire));
    const uint32_t delay_us =
        this->audio_stream_info_.frames_to_microseconds(this->playback_delay_frames_.load(std::memory_order_acquire));
    ESP_LOGD(TAG,
             "DEPTH own=%" PRIu32 " down_audio=%" PRIu32 " total_audio=%" PRIu32 " pending=%" PRIu32 " delay=%" PRIu32
             " age=%" PRId64,
             own_us, this->parent_->get_downstream_audio_us(), own_us + this->parent_->get_downstream_audio_us(),
             pending_us, delay_us, esp_timer_get_time() - this->parent_->get_downstream_as_of_us());
  }

  if (audio_source->available() > 0) {
    // Existing exposure was ducked when fill() promoted it; do not re-duck on partial-consume re-entry.
    return 0;
  }

  size_t bytes_read = audio_source->fill(ticks_to_wait, false);

  uint32_t samples_to_duck = this->audio_stream_info_.bytes_to_samples(bytes_read);
  if (samples_to_duck > 0) {
    esp_audio_libs::ducking::apply(audio_source->mutable_data(),
                                   static_cast<uint8_t>(this->audio_stream_info_.get_bits_per_sample() / 8),
                                   samples_to_duck, this->ducking_state_);
  }

  return bytes_read;
}

bool SourceSpeaker::supports_render_tags() const {
  return this->parent_->get_output_speaker()->supports_render_tags() && this->parent_->running_source_count() <= 1;
}

// THREAD CONTEXT: mixer task
audio::RenderTag SourceSpeaker::take_render_tag(uint32_t frames) {
  const audio::RenderTag tag = this->tag_track_.tag_at(this->tag_consumed_frames_);
  this->tag_consumed_frames_ += frames;
  return tag;
}

void SourceSpeaker::apply_ducking(uint8_t decibel_reduction, uint32_t duration) {
  const uint32_t transition_samples = duration > 0 ? this->audio_stream_info_.ms_to_samples(duration) : 0;
  esp_audio_libs::ducking::set_target(this->ducking_state_, decibel_reduction, transition_samples);
}

void SourceSpeaker::enter_stopping_state_() {
  this->state_ = speaker::STATE_STOPPING;
  this->stopping_start_ms_ = millis();
  this->audio_source_.reset();
}

void MixerSpeaker::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Speaker Mixer:\n"
                "  Number of output channels: %" PRIu8 "\n"
                "  Output bits per sample: %" PRIu8,
                this->output_channels_, this->output_bits_per_sample_);
}

void MixerSpeaker::setup() {
  if (!create_event_group(this->event_group_, this)) {
    return;
  }

  // Register callback to track frames in the output pipeline
  this->output_speaker_->add_audio_output_callback([this](uint32_t new_frames, int64_t write_timestamp) {
    atomic_subtract_clamped(this->frames_in_pipeline_, new_frames);
  });

  // Tagged renders are delivered to the source that tagged the audio, and only that one. Registered
  // once here rather than per source: the tag comes back as opaque bytes with no sender attached, so
  // fanning it out to every source would hand each of them another source's identity.
  this->output_speaker_->add_tagged_output_callback(
      [this](uint32_t frames, int64_t adjusted_ts, audio::RenderTag tag) {
        SourceSpeaker *owner = this->tag_owner_.load(std::memory_order_acquire);
        if (owner != nullptr) {
          owner->tagged_output_callback_(frames, adjusted_ts, tag);
        }
      });

  // Start with loop disabled since no task is running and no commands are pending
  this->disable_loop();
}

size_t MixerSpeaker::running_source_count() const {
  size_t running = 0;
  for (auto &speaker : this->source_speakers_) {
    if (speaker->is_running()) {
      running++;
    }
  }
  return running;
}

void MixerSpeaker::loop() {
  uint32_t event_group_bits = xEventGroupGetBits(this->event_group_);

  // Handle pending start request
  if (event_group_bits & MIXER_TASK_COMMAND_START) {
    // Only start the task if it's fully stopped and cleaned up
    if (!this->status_has_error() && !this->task_.is_created()) {
      if (this->task_.create(audio_mixer_task, "mixer", TASK_STACK_SIZE, (void *) this, MIXER_TASK_PRIORITY,
                             this->task_stack_in_psram_)) {
        xEventGroupClearBits(this->event_group_, MIXER_TASK_COMMAND_START);
      } else {
        ESP_LOGE(TAG, "Failed to start; retrying in 1 second");
        this->status_momentary_error("failure", 1000);
        return;
      }
    }
  }

  if (event_group_bits & MIXER_TASK_STATE_STARTING) {
    ESP_LOGD(TAG, "Starting");
    xEventGroupClearBits(this->event_group_, MIXER_TASK_STATE_STARTING);
  }
  if (event_group_bits & MIXER_TASK_ERR_ESP_NO_MEM) {
    this->status_set_error(LOG_STR("Not enough memory"));
    xEventGroupClearBits(this->event_group_, MIXER_TASK_ERR_ESP_NO_MEM);
  }
  if (event_group_bits & MIXER_TASK_STATE_RUNNING) {
    ESP_LOGV(TAG, "Started");
    this->status_clear_error();
    xEventGroupClearBits(this->event_group_, MIXER_TASK_STATE_RUNNING);
  }
  if (event_group_bits & MIXER_TASK_STATE_STOPPING) {
    ESP_LOGV(TAG, "Stopping");
    xEventGroupClearBits(this->event_group_, MIXER_TASK_STATE_STOPPING);
  }
  if (event_group_bits & MIXER_TASK_STATE_STOPPED) {
    this->task_.deallocate();
    // WEDGE DIAGNOSTIC (1 of 2). This handler clears ALL bits, so a MIXER_TASK_COMMAND_START set
    // between the top of loop() and here is discarded along with the state bits -- after which the
    // task is deallocated, nothing re-requests a start, and the speaker is silent until a replug.
    // Observed three times in one afternoon, always entered the same way: a supply outage, a
    // reconnect, "Stopped", then a session that reports PLAYING with dma_real=0 forever.
    //
    // The bits are RE-READ here rather than taken from the top of loop(): a start arriving in that
    // window is exactly the race in question, and the stale copy cannot show it.
    const uint32_t bits_at_clear = xEventGroupGetBits(this->event_group_);
    ESP_LOGD(TAG, "Stopped (bits=0x%06" PRIX32 "%s)", bits_at_clear,
             (bits_at_clear & MIXER_TASK_COMMAND_START) ? " -- CLEARING A PENDING START" : "");
    xEventGroupClearBits(this->event_group_, MIXER_TASK_ALL_BITS);
    this->all_stopped_since_ms_ = 0;
  }

  if (this->task_.is_created()) {
    // If the mixer task is running, check if all source speakers are stopped

    bool all_stopped = true;

    for (auto &speaker : this->source_speakers_) {
      all_stopped &= speaker->is_stopped();
    }

    if (all_stopped) {
      if (this->all_stopped_since_ms_ == 0) {
        this->all_stopped_since_ms_ = millis();
      } else if ((millis() - this->all_stopped_since_ms_) >= MIXER_AUTO_STOP_DEBOUNCE_MS) {
        // Send stop command only after a short debounce to avoid stop/start thrash during rapid seeks.
        xEventGroupSetBits(this->event_group_, MIXER_TASK_COMMAND_STOP);
      }
    } else {
      this->all_stopped_since_ms_ = 0;
      // New activity detected; clear any stale auto-stop request before it can stop the running task.
      if (event_group_bits & MIXER_TASK_COMMAND_STOP) {
        xEventGroupClearBits(this->event_group_, MIXER_TASK_COMMAND_STOP);
      }
    }
  } else {
    // Task is fully stopped and cleaned up, check if we can disable loop
    event_group_bits = xEventGroupGetBits(this->event_group_);
    if (event_group_bits == 0) {
      // No pending events, disable loop to save CPU cycles
      this->disable_loop();
    }
  }
}

esp_err_t MixerSpeaker::start(audio::AudioStreamInfo &stream_info) {
  if (!this->audio_stream_info_.has_value()) {
    this->audio_stream_info_ =
        audio::AudioStreamInfo(this->output_bits_per_sample_, this->output_channels_, stream_info.get_sample_rate());
    this->output_speaker_->set_audio_stream_info(this->audio_stream_info_.value());
  } else {
    if (!this->queue_mode_ && (stream_info.get_sample_rate() != this->audio_stream_info_.value().get_sample_rate())) {
      // The two audio streams must have the same sample rate to mix properly if not in queue mode
      return ESP_ERR_INVALID_ARG;
    }
  }

  this->enable_loop_soon_any_context();  // ensure loop processes command

  // Starting a new stream supersedes any previously queued stop request.
  xEventGroupClearBits(this->event_group_, MIXER_TASK_COMMAND_STOP);

  uint32_t event_bits = xEventGroupGetBits(this->event_group_);
  // WEDGE DIAGNOSTIC (2 of 2). The open question is whether a start is ISSUED AND LOST or NEVER
  // ISSUED: both end with no "Starting" line, and no fix can be chosen until they are told apart.
  // This is the only place a start is requested, so its absence in a log is the second answer.
  ESP_LOGD(TAG, "START requested (bits=0x%06" PRIX32 ", task %s, already pending %s)", event_bits,
           this->task_.is_created() ? "created" : "not created",
           (event_bits & MIXER_TASK_COMMAND_START) ? "yes" : "no");
  if (!(event_bits & MIXER_TASK_COMMAND_START)) {
    // Set MIXER_TASK_COMMAND_START bit if not already set, and then immediately wake for low latency
    xEventGroupSetBits(this->event_group_, MIXER_TASK_COMMAND_START);
    App.wake_loop_threadsafe();
  }

  return ESP_OK;
}

// NOLINTBEGIN(bugprone-unchecked-optional-access) -- audio_stream_info_ always set before this task is created
void MixerSpeaker::audio_mixer_task(void *params) {
  MixerSpeaker *this_mixer = static_cast<MixerSpeaker *>(params);

  xEventGroupSetBits(this_mixer->event_group_, MIXER_TASK_STATE_STARTING);

  {  // Ensure C++ objects fall out of scope to ensure proper cleanup before stopping the task
    std::unique_ptr<audio::AudioSinkTransferBuffer> output_transfer_buffer = audio::AudioSinkTransferBuffer::create(
        this_mixer->audio_stream_info_.value().ms_to_bytes(TRANSFER_BUFFER_DURATION_MS));

    if (output_transfer_buffer == nullptr) {
      xEventGroupSetBits(this_mixer->event_group_, MIXER_TASK_STATE_STOPPED | MIXER_TASK_ERR_ESP_NO_MEM);

      vTaskSuspend(nullptr);  // Suspend this task indefinitely until the loop method deletes it
    }

    output_transfer_buffer->set_sink(this_mixer->output_speaker_);

    // Both ends of the mixer->sink tag stream restart with the transfer buffer they describe.
    this_mixer->out_tag_track_.reset();
    this_mixer->tag_sent_frames_ = 0;
    this_mixer->tag_owner_.store(nullptr, std::memory_order_release);

    xEventGroupSetBits(this_mixer->event_group_, MIXER_TASK_STATE_RUNNING);

    bool sent_finished = false;

    // Pre-allocate vectors to avoid heap allocation in the loop (max 8 source speakers per schema)
    FixedVector<SourceSpeaker *> speakers_with_data;
    FixedVector<std::shared_ptr<audio::RingBufferAudioSource>> audio_sources_with_data;
    speakers_with_data.init(this_mixer->source_speakers_.size());
    audio_sources_with_data.init(this_mixer->source_speakers_.size());

    while (true) {
      uint32_t event_group_bits = xEventGroupGetBits(this_mixer->event_group_);
      if (event_group_bits & MIXER_TASK_COMMAND_STOP) {
        break;
      }

      // Transfer buffer (output format) plus whatever the output speaker holds, as one duration.
      //
      // READ BEFORE THE TRANSFER BELOW, and the order is load-bearing. The sink reports a snapshot
      // published on ITS task's cadence, so it describes an instant already past; the transfer buffer
      // is read here and now. Reading the transfer buffer AFTER handing audio to the sink put the two
      // terms either side of that hand-off: the audio just moved was gone from the transfer buffer and
      // not yet in the sink's snapshot, so it was counted in NEITHER and the total dropped by whatever
      // had moved -- about one mixer iteration's worth -- until the sink published again.
      //
      // Measured on hardware before this was reordered: the reported depth dipped ~30 ms on roughly a
      // seventh of samples while the consumer's own accounting did not move at all (-30.6 ms and
      // -28.2 ms of reported depth against -0.1 ms and -1.0 ms of accounted queue, on two clients).
      // That read as a steady accounting split for as long as it dwelt, and the consumer's self-repair
      // acted on it.
      //
      // Reading both before the transfer makes them coherent: nothing moves between stages in the
      // window between the sink's publish and this read, because the mixer is the only thing that
      // feeds the sink and its previous transfer predates that publish.
      uint32_t sink_us = 0, sink_audio_us = 0, sink_nondraining_us = 0;
      uint32_t sink_received_frames = 0;
      bool have_sink_received = false;
      int64_t downstream_as_of_us = esp_timer_get_time();
      if (this_mixer->output_speaker_ != nullptr) {
        audio::AudioDepth sink_latency, sink_audio;
        // Guard the instant as well as the value: only a POSITIVE as_of is a real sampling instant,
        // and taking the min against a zero would stamp the composite with the epoch.
        if (this_mixer->output_speaker_->render_latency(sink_latency)) {
          sink_us = sink_latency.microseconds;
          // Carried through unchanged: this mixer's own transfer buffer drains normally, so it adds
          // nothing to the held term, and without propagating the sink's value every chain with a
          // mixer in it would report zero held and a consumer would age the DMA span along with the
          // rest. Zero must mean "nothing is held", not "nobody asked the sink".
          sink_nondraining_us = sink_latency.render_nondraining_us;
          if (sink_latency.as_of_us > 0) {
            downstream_as_of_us = std::min(downstream_as_of_us, sink_latency.as_of_us);
          }
        }
        if (this_mixer->output_speaker_->buffered_audio(sink_audio)) {
          sink_audio_us = sink_audio.microseconds;
          if (sink_audio.as_of_us > 0) {
            downstream_as_of_us = std::min(downstream_as_of_us, sink_audio.as_of_us);
          }
          this_mixer->dbg_sink_queued_us_ = sink_audio.dbg_queued_us;
          this_mixer->dbg_sink_dma_us_ = sink_audio.dbg_dma_us;
          this_mixer->dbg_sink_received_ = sink_audio.dbg_sink_received;
          this_mixer->dbg_sink_padded_frames_ = sink_audio.dbg_padded_frames;
          have_sink_received = true;
          sink_received_frames = sink_audio.dbg_sink_received;
        }
      }
      this_mixer->downstream_as_of_us_ = downstream_as_of_us;
      this_mixer->dbg_xfer_us_ = this_mixer->audio_stream_info_.value().frames_to_microseconds(
          this_mixer->audio_stream_info_.value().bytes_to_frames(output_transfer_buffer->available()));

      // AUDIO IN FLIGHT TO THE SINK. Reading the transfer buffer and the sink's snapshot both before
      // the transfer below is not enough to make them coherent, and the comment above overstated the
      // case: it assumed this mixer's last transfer predates the sink's last publish. It usually does
      // not. The sink publishes on its own DMA cadence -- tens of milliseconds -- while this loop
      // iterates far faster, so most iterations transfer audio AFTER the sink's snapshot was taken.
      // That audio is gone from the transfer buffer read here and absent from the snapshot read here,
      // so the composite counted it in NEITHER, under-reporting by up to one publish interval.
      //
      // Measured downstream, where it does real damage: a consumer differencing its own accounting
      // against this total saw a steady 25.5 ms split -- 1125 frames, matching the conservation
      // residual to 1 us -- held long enough to pass every steadiness test and trigger its
      // self-repair, which then created an equal split of the opposite sign that a second repair
      // answered. Two corrections, neither needed, and the audio ended up ~85 us out on a logic
      // analyser against a 1-3 us noise floor.
      //
      // The bridge needs no timing assumption. Both sides of this boundary keep a CUMULATIVE frame
      // count -- what this mixer has handed over, and what the sink says it has accepted -- so their
      // difference is precisely the audio between them, whatever the relative age of the two reads.
      // Unsigned subtraction stays correct across the 2^32 wrap because both counters wrap together.
      uint32_t sink_inflight_us = 0;
      if (have_sink_received) {
        const uint32_t inflight_frames = this_mixer->dbg_written_to_sink_frames_ - sink_received_frames;
        // A sink restart zeroes its counter while ours keeps running, which would read as an enormous
        // in-flight term. Nothing legitimate is more than a second deep between these two stages, so
        // treat anything larger as a counter mismatch and contribute nothing until they line up again.
        if (inflight_frames <= this_mixer->audio_stream_info_.value().get_sample_rate()) {
          sink_inflight_us = this_mixer->audio_stream_info_.value().frames_to_microseconds(inflight_frames);
        }
      }
      this_mixer->dbg_sink_inflight_us_ = sink_inflight_us;

      // The transfer buffer holds only real mixed audio, so it counts toward both. So does the audio
      // in flight to the sink: it is real mixed audio too, and it is still going to be played.
      this_mixer->downstream_audio_us_.store(
          this_mixer->audio_stream_info_.value().frames_to_microseconds(
              this_mixer->audio_stream_info_.value().bytes_to_frames(output_transfer_buffer->available())) +
              sink_inflight_us + sink_audio_us,
          std::memory_order_release);
      this_mixer->downstream_latency_us_.store(
          this_mixer->audio_stream_info_.value().frames_to_microseconds(
              this_mixer->audio_stream_info_.value().bytes_to_frames(output_transfer_buffer->available())) +
              sink_inflight_us + sink_us,
          std::memory_order_release);
      this_mixer->downstream_nondraining_us_.store(sink_nondraining_us, std::memory_order_release);

      // TEMPORARY DIAGNOSTIC: attribute the composite to its terms. A consumer sees only the sum, so a
      // constant offset in it cannot be pinned to a stage from the outside.
      //
      // Throttled BY TIME, not by iteration count. This loop has no fixed cadence: when the sink stops
      // accepting audio and the transfer buffer is already full there is nothing to wait on, so it
      // spins at hundreds of iterations per second. A per-N-iterations throttle then emits dozens of
      // lines per millisecond, which floods the log and can stall an OTA. Remove once explained.
      const int64_t depth_debug_now = esp_timer_get_time();
      if (depth_debug_now - this_mixer->depth_debug_last_us_ >= 1000000) {
        this_mixer->depth_debug_last_us_ = depth_debug_now;
        const uint32_t xfer_us = this_mixer->audio_stream_info_.value().frames_to_microseconds(
            this_mixer->audio_stream_info_.value().bytes_to_frames(output_transfer_buffer->available()));
        ESP_LOGD(TAG,
                 "DEPTH xfer=%" PRIu32 " inflight=%" PRIu32 " sink_audio=%" PRIu32 " sink_lat=%" PRIu32
                 " age=%" PRId64,
                 xfer_us, sink_inflight_us, sink_audio_us, sink_us, esp_timer_get_time() - downstream_as_of_us);
      }

      // Hand audio to the sink only AFTER publishing the pair above, so the two terms describe the
      // same instant. Never shift the data in the output transfer buffer to avoid unnecessary, slow
      // data moves.
      //
      // The sink is told the identity of the FIRST frame this transfer will hand it, which is the
      // frame at the current read position -- the transfer buffer sits between the mixing above and
      // this write, so the tag attached here is generally not the one attached most recently. An
      // untagged answer is passed on as untagged, which is what suppresses a reading for blended or
      // producer-inserted audio.
      if (this_mixer->output_speaker_->supports_render_tags()) {
        this_mixer->output_speaker_->set_next_render_tag(
            this_mixer->out_tag_track_.tag_at(this_mixer->tag_sent_frames_));
      }
      const size_t transferred_bytes = output_transfer_buffer->transfer_data_to_sink(pdMS_TO_TICKS(TASK_DELAY_MS), false);
      this_mixer->tag_sent_frames_ += this_mixer->audio_stream_info_.value().bytes_to_frames(transferred_bytes);
      this_mixer->dbg_written_to_sink_frames_ +=
          this_mixer->audio_stream_info_.value().bytes_to_frames(transferred_bytes);

      // Free space is read after the transfer on purpose: this one wants the post-transfer figure,
      // since it bounds how much this iteration may mix in.
      const uint32_t output_frames_free =
          this_mixer->audio_stream_info_.value().bytes_to_frames(output_transfer_buffer->free());

      // Nothing can be mixed into a full buffer, so iterating again immediately is a spin. The
      // no-data case below already yields; this is the opposite one -- sources have audio and the
      // SINK is not draining -- and it had no yield at all. transfer_data_to_sink() only blocks
      // while the sink is accepting; a stopped sink refuses at once, so the loop ran flat out.
      //
      // Measured consequence: a speaker whose sink stopped after a stream teardown starved its own
      // main loop. The device stayed on wifi and answered pings, but declared healthy API clients
      // "unresponsive" and disconnected them, and refused OTA -- so it could not be recovered over
      // the network at all and needed the power pulled. A silent speaker is a bug; an unreachable
      // one is a much worse bug, and it turned a recoverable routing fault into a site visit.
      if (output_frames_free == 0) {
        delay(TASK_DELAY_MS);
        continue;
      }

      speakers_with_data.clear();
      audio_sources_with_data.clear();

      for (auto &speaker : this_mixer->source_speakers_) {
        if (speaker->is_running() && !speaker->get_pause_state()) {
          // Speaker is running and not paused, so it possibly can provide audio data
          std::shared_ptr<audio::RingBufferAudioSource> audio_source = speaker->get_audio_source().lock();
          if (audio_source.use_count() == 0) {
            // No audio source allocated, so skip processing this speaker
            continue;
          }
          speaker->process_data_from_source(audio_source, 0);  // Exposes and ducks audio from source ring buffers

          if (audio_source->available() > 0) {
            // Retain shared ownership across the mixing pass so the source isn't released mid-mix
            audio_sources_with_data.push_back(audio_source);
            speakers_with_data.push_back(speaker);
          }
        }
      }

      if (audio_sources_with_data.empty()) {
        // No audio available for transferring, block task temporarily
        delay(TASK_DELAY_MS);
        continue;
      }

      uint32_t frames_to_mix = output_frames_free;

      const audio::AudioStreamInfo &output_info = this_mixer->audio_stream_info_.value();
      const uint8_t output_bps = output_info.get_bits_per_sample() / 8;
      const uint8_t output_channels = output_info.get_channels();

      if ((audio_sources_with_data.size() == 1) || this_mixer->queue_mode_) {
        // Only one speaker has audio data, just copy samples over

        audio::AudioStreamInfo active_stream_info = speakers_with_data[0]->get_audio_stream_info();

        if (active_stream_info.get_sample_rate() ==
            this_mixer->output_speaker_->get_audio_stream_info().get_sample_rate()) {
          // Speaker's sample rate matches the output speaker's, convert directly into the output buffer

          const uint32_t frames_available_in_buffer =
              active_stream_info.bytes_to_frames(audio_sources_with_data[0]->available());
          frames_to_mix = std::min(frames_to_mix, frames_available_in_buffer);
          esp_audio_libs::pcm_convert::copy_frames(
              audio_sources_with_data[0]->data(), output_transfer_buffer->get_buffer_end(),
              static_cast<uint8_t>(active_stream_info.get_bits_per_sample() / 8), active_stream_info.get_channels(),
              output_bps, output_channels, frames_to_mix);

          // Set playback delay for newly contributing source
          if (!speakers_with_data[0]->has_contributed_.load(std::memory_order_acquire)) {
            const uint32_t dbg_delay = this_mixer->frames_in_pipeline_.load(std::memory_order_acquire);
            // TEMPORARY DIAGNOSTIC: fires exactly once per contribution start, so it needs no
            // throttle. The theory it was added to test is DISPROVEN, and the number is kept only
            // because it is free: this was believed to be where a consumer acquires a permanent
            // offset, on the argument that the first `dbg_delay` frames the sink plays are charged to
            // the delay and never credited to this source, and that a start leaves 2-4 DMA buffers
            // uncredited by a varying amount.
            //
            // Measured across 18 starts on two boards: playback_delay was ZERO every single time.
            // Whatever plants a per-start offset in a consumer, it is not this. The per-start offsets
            // that prompted the theory (30-130 us on a logic analyser) survive with this at zero, and
            // padded silence was eliminated as the mechanism separately -- two devices differing by
            // 877 ms of accumulated padding sat 133 us apart, so the sink's per-descriptor real-frame
            // bookkeeping is handling that correctly.
            ESP_LOGD(TAG,
                     "STARTDBG single: playback_delay=%" PRIu32 " frames (%" PRIu32 " us) pending=%" PRIu32
                     " frames_to_mix=%" PRIu32,
                     dbg_delay, speakers_with_data[0]->get_audio_stream_info().frames_to_microseconds(dbg_delay),
                     speakers_with_data[0]->pending_playback_frames_.load(std::memory_order_acquire), frames_to_mix);
            speakers_with_data[0]->playback_delay_frames_.store(dbg_delay, std::memory_order_release);
            speakers_with_data[0]->has_contributed_.store(true, std::memory_order_release);
          }

          // One source, so its identity IS the output's identity: frames cross unblended and the
          // conversion above changes sample width and channel count, never the frame count the tag's
          // offset is measured in.
          this_mixer->tag_owner_.store(speakers_with_data[0], std::memory_order_release);
          this_mixer->out_tag_track_.set_next(speakers_with_data[0]->take_render_tag(frames_to_mix));
          this_mixer->out_tag_track_.note_written(frames_to_mix);

          // Update source speaker pending frames
          speakers_with_data[0]->pending_playback_frames_.fetch_add(frames_to_mix, std::memory_order_release);
          speakers_with_data[0]->dbg_consumed_frames_.fetch_add(frames_to_mix, std::memory_order_relaxed);
          audio_sources_with_data[0]->consume(active_stream_info.frames_to_bytes(frames_to_mix));

          // Update output transfer buffer length and pipeline frame count
          output_transfer_buffer->increase_buffer_length(output_info.frames_to_bytes(frames_to_mix));
          this_mixer->frames_in_pipeline_.fetch_add(frames_to_mix, std::memory_order_release);
        } else {
          // Speaker's stream info doesn't match the output speaker's, so it's a new source speaker
          if (!this_mixer->output_speaker_->is_stopped()) {
            if (!sent_finished) {
              this_mixer->output_speaker_->finish();
              sent_finished = true;  // Avoid repeatedly sending the finish command
            }
          } else {
            // Speaker has finished writing the current audio, update the stream information and restart the speaker
            this_mixer->audio_stream_info_ =
                audio::AudioStreamInfo(this_mixer->output_bits_per_sample_, this_mixer->output_channels_,
                                       active_stream_info.get_sample_rate());
            this_mixer->output_speaker_->set_audio_stream_info(this_mixer->audio_stream_info_.value());
            this_mixer->output_speaker_->start();
            // Reset pipeline frame count since we're starting fresh with a new sample rate
            this_mixer->frames_in_pipeline_.store(0, std::memory_order_release);
            // The sink was restarted, so its accepted-frames counter starts over and ours has to as
            // well or the in-flight bridge above compares two unrelated origins.
            this_mixer->dbg_written_to_sink_frames_ = 0;
            sent_finished = false;
          }
        }
      } else {
        // Determine how many frames to mix
        for (size_t i = 0; i < audio_sources_with_data.size(); ++i) {
          const uint32_t frames_available_in_buffer =
              speakers_with_data[i]->get_audio_stream_info().bytes_to_frames(audio_sources_with_data[i]->available());
          frames_to_mix = std::min(frames_to_mix, frames_available_in_buffer);
        }
        const uint8_t *primary_buffer = audio_sources_with_data[0]->data();
        audio::AudioStreamInfo primary_stream_info = speakers_with_data[0]->get_audio_stream_info();

        // Mix two streams together at a time, accumulating into the output buffer.
        for (size_t i = 1; i < audio_sources_with_data.size(); ++i) {
          esp_audio_libs::mixer::mix_frames(
              primary_buffer, static_cast<uint8_t>(primary_stream_info.get_bits_per_sample() / 8),
              primary_stream_info.get_channels(), audio_sources_with_data[i]->data(),
              static_cast<uint8_t>(speakers_with_data[i]->get_audio_stream_info().get_bits_per_sample() / 8),
              speakers_with_data[i]->get_audio_stream_info().get_channels(), output_transfer_buffer->get_buffer_end(),
              output_bps, output_channels, frames_to_mix);

          if (i != audio_sources_with_data.size() - 1) {
            // Need to mix more streams together, point primary buffer and stream info to the already mixed output
            primary_buffer = output_transfer_buffer->get_buffer_end();
            primary_stream_info = output_info;
          }
        }

        // Get current pipeline depth for delay calculation (before incrementing)
        uint32_t current_pipeline_frames = this_mixer->frames_in_pipeline_.load(std::memory_order_acquire);

        // Update source audio source consumption and add new audio durations to the source speaker pending playbacks
        for (size_t i = 0; i < audio_sources_with_data.size(); ++i) {
          // Set playback delay for newly contributing sources
          if (!speakers_with_data[i]->has_contributed_.load(std::memory_order_acquire)) {
            // TEMPORARY DIAGNOSTIC: as above, on the multi-source path.
            ESP_LOGD(TAG,
                     "STARTDBG mixed[%u]: playback_delay=%" PRIu32 " frames (%" PRIu32 " us) pending=%" PRIu32
                     " frames_to_mix=%" PRIu32,
                     (unsigned) i, current_pipeline_frames,
                     speakers_with_data[i]->get_audio_stream_info().frames_to_microseconds(current_pipeline_frames),
                     speakers_with_data[i]->pending_playback_frames_.load(std::memory_order_acquire), frames_to_mix);
            speakers_with_data[i]->playback_delay_frames_.store(current_pipeline_frames, std::memory_order_release);
            speakers_with_data[i]->has_contributed_.store(true, std::memory_order_release);
          }

          speakers_with_data[i]->pending_playback_frames_.fetch_add(frames_to_mix, std::memory_order_release);
          speakers_with_data[i]->dbg_consumed_frames_.fetch_add(frames_to_mix, std::memory_order_relaxed);
          // Advance every contributor's read position even though the tag is discarded: the position
          // tracks the RING, and a source skipped here would have every later lookup naming audio
          // this many frames too early.
          speakers_with_data[i]->take_render_tag(frames_to_mix);
          audio_sources_with_data[i]->consume(
              speakers_with_data[i]->get_audio_stream_info().frames_to_bytes(frames_to_mix));
        }

        // A blend has no single identity. Marking the run explicitly untagged is what makes that
        // structural rather than advisory -- the sink then reports nothing for any descriptor
        // starting in it, whatever supports_render_tags() happened to say when the caller asked.
        this_mixer->out_tag_track_.set_next(audio::RenderTag{});
        this_mixer->out_tag_track_.note_written(frames_to_mix);

        // Update output transfer buffer length and pipeline frame count (once, not per source)
        output_transfer_buffer->increase_buffer_length(output_info.frames_to_bytes(frames_to_mix));
        this_mixer->frames_in_pipeline_.fetch_add(frames_to_mix, std::memory_order_release);
      }
    }

    xEventGroupSetBits(this_mixer->event_group_, MIXER_TASK_STATE_STOPPING);
  }

  // Reset pipeline frame count since the task is stopping
  this_mixer->frames_in_pipeline_.store(0, std::memory_order_release);

  xEventGroupSetBits(this_mixer->event_group_, MIXER_TASK_STATE_STOPPED);

  vTaskSuspend(nullptr);  // Suspend this task indefinitely until the loop method deletes it
}
// NOLINTEND(bugprone-unchecked-optional-access)

}  // namespace esphome::mixer_speaker

#endif
