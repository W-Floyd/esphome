#pragma once

#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"  // for ESPDEPRECATED

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace esphome::audio {

/// @brief An opaque IDENTITY a producer attaches to audio it hands down the chain, returned to it when
/// THAT audio renders.
///
/// The played-frames callback reports a QUANTITY -- how many frames rendered, and when -- never WHICH
/// audio. A caller that wants "when did this particular frame render" has to infer it from its own
/// pushed-minus-played ledger, and a device cannot detect that its own counter is biased by consulting
/// that counter: once a servo repairs a ledger bias, the bias and the audio displacement it caused are
/// equal and opposite inside the subtraction and cancel exactly. Measured against a known 1000 us
/// displacement, an inferred phase moved by 0.003 of it; an externally planted one by 1.000.
///
/// So the identity is CAPTURED rather than inferred: the producer states which audio it is handing over,
/// and the sink hands that statement back with the render timestamp of that audio. A ledger bias is then
/// visible IN the measurement rather than invisible TO it.
///
/// The contents mean nothing to anything between producer and sink -- they are carried, never
/// interpreted. For the snapcast client they are the server timestamp of a chunk and the frame offset
/// into it, because a DMA descriptor (441 frames) is smaller than a chunk (1152 frames) and descriptors
/// therefore straddle chunks: a bare chunk id could not say WHERE in the chunk a descriptor started.
struct RenderTag {
  /// Producer-defined timestamp of the frame at ``offset_frames`` zero. Zero means UNTAGGED.
  uint64_t server_ts{0};
  /// Frames from that timestamp's frame to the frame this tag is attached to.
  uint32_t offset_frames{0};

  /// @brief Whether this tag identifies anything.
  ///
  /// Untagged audio is not an error and must not be treated as one: silence padding, servo splices and
  /// repeat frames are all audio the producer deliberately inserted that corresponds to no server time
  /// at all. A sink must simply not report a render for audio that starts in such a run -- a wrong tag
  /// is far worse than no tag, because a wrong one is acted on.
  bool valid() const { return this->server_ts != 0; }
};

/// @brief Binds RenderTags to positions in a single ordered frame stream, so a consumer reading that
/// stream some time later can recover the tag covering any frame it reaches.
///
/// One instance describes ONE queue: the producer states a tag and then reports how many frames it
/// handed over, the consumer asks which tag covers the frame it is about to render. Frames between two
/// tagged points are contiguous by construction, so a tag covers everything after it until the next
/// one, with ``offset_frames`` advanced by the distance -- which is why a chunk boundary and a DMA
/// descriptor boundary need not line up.
///
/// A stage that changes the frame count (a resampler) cannot use this: the distance arithmetic is in
/// frames of one stream. A stage that changes the SAMPLE layout (mono widened to stereo, 32-bit
/// narrowed to 16) can, because frames are preserved.
///
/// THREADING: one producer thread and one consumer thread, serialised by an internal mutex. Both sides
/// are called at a buffer cadence -- tens of times a second, not per frame -- so the mutex costs
/// nothing measurable and buys an obviously-correct implementation instead of a hand-rolled seqlock
/// over a 24-byte entry.
class RenderTagTrack {
 public:
  /// @brief Forget everything and restart both positions at zero.
  /// @note Call only while the queue this describes is empty and neither side is running -- a
  /// reset with audio in flight would re-map surviving frames onto new positions.
  void reset() {
    LockGuard guard(this->mutex_);
    this->count_ = 0;
    this->next_ = 0;
    this->write_pos_ = 0;
    this->pending_ = RenderTag{};
    this->pending_set_ = false;
  }

  /// @brief Attach ``tag`` to the first frame of the next non-empty ``note_written()``.
  ///
  /// Stateful, and therefore racy if two producers write the same queue. That is accepted rather than
  /// designed around: the tagging path exists for a single synchronised producer, and every other
  /// caller simply never tags, which leaves the audio untagged rather than mistagged.
  void set_next(const RenderTag &tag) {
    LockGuard guard(this->mutex_);
    this->pending_ = tag;
    this->pending_set_ = true;
  }

  /// @brief Report ``frames`` accepted into the queue, consuming any tag set since the last report.
  /// @note Pass what the queue ACTUALLY took, never what was offered: a short write that counted in
  /// full would slide every later tag forward by the shortfall.
  void note_written(uint32_t frames) {
    if (frames == 0) {
      // A refused write leaves the pending tag pending: it still describes the next frame to land.
      return;
    }
    LockGuard guard(this->mutex_);
    if (this->pending_set_ && !this->continues_last_(this->pending_)) {
      // Only a DISCONTINUITY needs its own entry. A producer that re-states the identity of every
      // write -- the normal case, since a source chunk is usually split across several -- would
      // otherwise fill the ring with restatements of one fact and evict tags the sink has not
      // reached yet, turning valid readings into skipped ones.
      this->entries_[this->next_] = Entry{this->write_pos_, this->pending_};
      this->next_ = (this->next_ + 1) % ENTRIES;
      if (this->count_ < ENTRIES) {
        this->count_++;
      }
    }
    this->pending_set_ = false;
    this->write_pos_ += frames;
  }

  /// @brief Total frames reported through ``note_written()`` since the last reset.
  uint64_t written() const {
    LockGuard guard(this->mutex_);
    return this->write_pos_;
  }

  /// @brief The tag covering stream frame ``read_pos``, with ``offset_frames`` advanced to that frame.
  /// @return An INVALID tag when that frame is not covered -- it predates the oldest retained tag, or
  /// the producer explicitly marked the run untagged. Never a guess.
  RenderTag tag_at(uint64_t read_pos) const {
    LockGuard guard(this->mutex_);
    const Entry *best = nullptr;
    for (size_t i = 0; i < this->count_; i++) {
      const Entry &entry = this->entries_[i];
      if (entry.pos <= read_pos && (best == nullptr || entry.pos > best->pos)) {
        best = &entry;
      }
    }
    if (best == nullptr || !best->tag.valid()) {
      return RenderTag{};
    }
    const uint64_t advance = read_pos - best->pos;
    RenderTag tag = best->tag;
    tag.offset_frames += static_cast<uint32_t>(advance);
    return tag;
  }

 protected:
  struct Entry {
    uint64_t pos;
    RenderTag tag;
  };

  /// @brief Whether ``tag`` says exactly what the newest entry already implies about ``write_pos_``.
  /// @note Caller holds the mutex.
  bool continues_last_(const RenderTag &tag) const {
    if (this->count_ == 0) {
      // Nothing recorded yet, so nothing to continue -- not even an untagged run, since a consumer
      // reading before the first entry already gets an invalid tag.
      return !tag.valid();
    }
    const Entry &last = this->entries_[(this->next_ + ENTRIES - 1) % ENTRIES];
    if (!tag.valid() || !last.tag.valid()) {
      // Two untagged runs are one untagged run; a tagged run after an untagged one is a change.
      return !tag.valid() && !last.tag.valid();
    }
    return tag.server_ts == last.tag.server_ts &&
           static_cast<uint64_t>(tag.offset_frames) ==
               static_cast<uint64_t>(last.tag.offset_frames) + (this->write_pos_ - last.pos);
  }

  /// Sized to outlive the deepest queue a tag has to survive: a 100 ms speaker ring plus a 50 ms DMA
  /// span is ~6 chunks of audio, and a producer may split a chunk across several writes. Overrun is
  /// not a correctness hazard -- ``tag_at`` returns invalid for a frame whose tag has been evicted, so
  /// the reading is skipped rather than fabricated.
  static constexpr size_t ENTRIES = 32;

  mutable Mutex mutex_;
  Entry entries_[ENTRIES]{};
  size_t count_{0};  // entries currently retained, saturating at ENTRIES
  size_t next_{0};   // ring cursor for the next entry written
  uint64_t write_pos_{0};
  RenderTag pending_{};
  bool pending_set_{false};
};

/// @brief A depth reading and the instant it describes.
///
/// The instant is not decoration. A speaker that buffers on a task publishes a snapshot, so it stamps
/// when it SAMPLED, which is in the past by up to one iteration of that task. A consumer differencing
/// that against its own written-minus-played accumulator is then comparing a value from ``as_of_us``
/// with a value from now, and every frame that entered or left in between shows up as a disagreement
/// that is not one. Measured on a client with a mixer in the chain, the artefact was quantised in whole
/// audio chunks -- 26 ms steps -- with a mean that wandered tens of milliseconds over hours, which is
/// far larger than the accounting errors the comparison exists to catch.
///
/// So a consumer must evaluate its own accounting AT ``as_of_us`` rather than at read time. Reporting
/// the instant is what makes that possible; without it the comparison cannot be made correctly at all.
struct AudioDepth {
  /// The duration. Untouched unless the read succeeded.
  uint32_t microseconds{0};
  /// esp_timer time the reading describes. A live reader stamps now; a snapshot publisher stamps when
  /// it sampled. A stage that sums its own buffers with a downstream reading reports the OLDEST instant
  /// contributing to the total, since that is the one the drain has to be measured from.
  int64_t as_of_us{0};

  /// @brief How much of a render-latency reading does NOT decay as this snapshot ages.
  ///
  /// Ageing a stale reading is only valid for the parts that actually drain. An i2s DMA under an
  /// always-fill writer does not: every task iteration writes a whole buffer, padding with silence as
  /// needed, so its span stays full and costs the same time to clock out however old the reading is.
  /// A consumer that aged the whole latency drove hard resyncs at 350 ms, 2297 ms and 3581 ms within
  /// four seconds; one that aged the wrong term instead -- taking real-audio-in-DMA for the span --
  /// under-anchored by the padding and left a pair 67 ms apart. Hence a term for it, published rather
  /// than guessed.
  ///
  /// A consumer ageing a reading of age A wants:  held + max(0, (microseconds - held) - A).
  ///
  /// Qualifies the RENDER reading (read_render), which is the padding-inclusive one; the own-audio
  /// reading counts no padding and every part of it drains. A composing stage reports its downstream's
  /// value plus any of its own buffers that refill themselves -- normally none, an ordinary queue
  /// drains.
  ///
  /// 0 means "nothing held", which is also what a stage unable to distinguish reports. That is the
  /// safe direction only for the consumer that treats it as "do not age": ageing everything is the
  /// failure above. Callers must decide explicitly which they mean.
  uint32_t render_nondraining_us{0};

  // TEMPORARY DIAGNOSTIC: the total broken into the stage that holds each part, carried under the same
  // seqlock so a consumer reads every term at ONE instant. Attributing a total to a stage by logging
  // the stages separately does not work when the quantity sought is 20 ms and the samples are hundreds
  // of milliseconds apart. Remove once the offset is explained.
  uint32_t dbg_own_us{0};     // the source ring feeding a mixer
  uint32_t dbg_xfer_us{0};    // a mixer's output transfer buffer
  uint32_t dbg_queued_us{0};  // the sink's own ring
  // Handed to the sink but not yet in the snapshot the sink published: the two stages keep cumulative
  // frame counts, so this is their difference rather than a duration anyone measured. Without it the
  // composite omits whatever moved between the sink's publish and the mixer's read of its own buffer.
  uint32_t dbg_inflight_us{0};
  uint32_t dbg_dma_us{0};     // real audio resident in the sink's DMA descriptors
  // Cumulative FRAME counts at each boundary, for a conservation check. Every boundary must satisfy
  // received == passed-on + still-held; the boundary where that fails is the one losing audio.
  uint32_t dbg_src_received{0};   // frames accepted into the source ring
  uint32_t dbg_src_consumed{0};   // frames the mixer took out of it
  uint32_t dbg_sink_received{0};  // frames accepted into the sink's own ring
  // Cumulative silence frames the sink has padded into DMA descriptors behind real audio. Not a
  // conservation term: this audio was never ours. It is here because it DISPLACES ours -- each padded
  // frame pushes everything after it one frame later, for good, and an accounting that counts only
  // real frames cannot see that it happened.
  uint32_t dbg_padded_frames{0};
};

/// @brief Single-writer, multi-reader store for the two depth readings a speaker publishes.
///
/// A seqlock rather than a handful of independent atomics, because the values must be observed as ONE
/// instant. Pairing a duration sampled after a consume with an ``as_of_us`` sampled before it would
/// under-state the age, which is precisely the error AudioDepth exists to remove -- so a reader is
/// given a coherent snapshot or told it failed, never a mixture.
///
/// The writer must be a single task. That is already true of every speaker publishing here: the values
/// are computed from ring buffers and audio sources whose contract is single-consumer-thread, so there
/// is exactly one thread that may compute them.
class DepthPublisher {
 public:
  /// Publishes both readings and the instant they describe. Call from the owning task only.
  void publish(uint32_t render_us, uint32_t audio_us, int64_t as_of_us, uint32_t dbg_own_us = 0,
               uint32_t dbg_xfer_us = 0, uint32_t dbg_queued_us = 0, uint32_t dbg_dma_us = 0,
               uint32_t dbg_src_received = 0, uint32_t dbg_src_consumed = 0, uint32_t dbg_sink_received = 0,
               uint32_t render_nondraining_us = 0, uint32_t dbg_inflight_us = 0,
               uint32_t dbg_padded_frames = 0) {
    const uint32_t seq = this->seq_.load(std::memory_order_relaxed);
    this->seq_.store(seq + 1, std::memory_order_release);  // odd: publish in progress
    this->render_us_.store(render_us, std::memory_order_relaxed);
    this->audio_us_.store(audio_us, std::memory_order_relaxed);
    this->dbg_own_us_.store(dbg_own_us, std::memory_order_relaxed);
    this->dbg_xfer_us_.store(dbg_xfer_us, std::memory_order_relaxed);
    this->dbg_queued_us_.store(dbg_queued_us, std::memory_order_relaxed);
    this->dbg_dma_us_.store(dbg_dma_us, std::memory_order_relaxed);
    this->dbg_src_received_.store(dbg_src_received, std::memory_order_relaxed);
    this->dbg_src_consumed_.store(dbg_src_consumed, std::memory_order_relaxed);
    this->dbg_sink_received_.store(dbg_sink_received, std::memory_order_relaxed);
    this->render_nondraining_us_.store(render_nondraining_us, std::memory_order_relaxed);
    this->dbg_inflight_us_.store(dbg_inflight_us, std::memory_order_relaxed);
    this->dbg_padded_frames_.store(dbg_padded_frames, std::memory_order_relaxed);
    this->as_of_lo_.store(static_cast<uint32_t>(static_cast<uint64_t>(as_of_us)), std::memory_order_relaxed);
    this->as_of_hi_.store(static_cast<uint32_t>(static_cast<uint64_t>(as_of_us) >> 32), std::memory_order_relaxed);
    this->seq_.store(seq + 2, std::memory_order_release);  // even: stable
  }

  /// Publishes a true zero. A stopped speaker holds nothing, which is an answer rather than a refusal,
  /// so callers probing a not-yet-started speaker must not conclude the platform cannot report.
  void reset(int64_t as_of_us) { this->publish(0, 0, as_of_us); }

  bool read_render(AudioDepth &depth) const { return this->read_(depth, this->render_us_); }
  bool read_audio(AudioDepth &depth) const { return this->read_(depth, this->audio_us_); }

  /// @brief Whether anything has ever been published. A speaker that has not started yet holds a
  /// zero-initialised snapshot, and reporting that as a real reading hands the caller an as_of of 0
  /// -- which a composing stage will happily adopt as "the oldest instant in the total", producing a
  /// timestamp stale by the entire uptime. Observed as an age of 10 s on a freshly started mixer.
  bool has_published() const { return this->seq_.load(std::memory_order_acquire) != 0; }

 private:
  bool read_(AudioDepth &depth, const std::atomic<uint32_t> &field) const {
    // Bounded retry. A publisher runs on a task cadence measured in tens of milliseconds, so a reader
    // that loses four races in a row is not racing -- it is looking at a writer stopped mid-publish,
    // and reporting failure is more useful than spinning.
    for (int attempt = 0; attempt < 4; attempt++) {
      const uint32_t before = this->seq_.load(std::memory_order_acquire);
      if (before == 0) {
        return false;  // never published: a zero snapshot is not a reading
      }
      if (before & 1u) {
        continue;  // publish in progress
      }
      const uint32_t us = field.load(std::memory_order_relaxed);
      const uint64_t lo = this->as_of_lo_.load(std::memory_order_relaxed);
      const uint64_t hi = this->as_of_hi_.load(std::memory_order_relaxed);
      const uint32_t d_own = this->dbg_own_us_.load(std::memory_order_relaxed);
      const uint32_t d_xfer = this->dbg_xfer_us_.load(std::memory_order_relaxed);
      const uint32_t d_queued = this->dbg_queued_us_.load(std::memory_order_relaxed);
      const uint32_t d_dma = this->dbg_dma_us_.load(std::memory_order_relaxed);
      const uint32_t d_sr = this->dbg_src_received_.load(std::memory_order_relaxed);
      const uint32_t d_sc = this->dbg_src_consumed_.load(std::memory_order_relaxed);
      const uint32_t d_kr = this->dbg_sink_received_.load(std::memory_order_relaxed);
      const uint32_t nd = this->render_nondraining_us_.load(std::memory_order_relaxed);
      const uint32_t d_if = this->dbg_inflight_us_.load(std::memory_order_relaxed);
      const uint32_t d_pad = this->dbg_padded_frames_.load(std::memory_order_relaxed);
      if (this->seq_.load(std::memory_order_acquire) == before) {
        depth.microseconds = us;
        depth.as_of_us = static_cast<int64_t>((hi << 32) | lo);
        depth.dbg_own_us = d_own;
        depth.dbg_xfer_us = d_xfer;
        depth.dbg_queued_us = d_queued;
        depth.dbg_dma_us = d_dma;
        depth.dbg_src_received = d_sr;
        depth.dbg_src_consumed = d_sc;
        depth.dbg_sink_received = d_kr;
        depth.render_nondraining_us = nd;
        depth.dbg_inflight_us = d_if;
        depth.dbg_padded_frames = d_pad;
        return true;
      }
    }
    return false;
  }

  std::atomic<uint32_t> seq_{0};
  // Plain values guarded by seq_, split into 32-bit halves because a 64-bit atomic is not lock-free on
  // the targets this runs on and a hidden libatomic mutex has no business in a speaker task.
  std::atomic<uint32_t> render_us_{0};
  std::atomic<uint32_t> audio_us_{0};
  std::atomic<uint32_t> as_of_lo_{0};
  std::atomic<uint32_t> as_of_hi_{0};
  // TEMPORARY DIAGNOSTIC, see AudioDepth.
  std::atomic<uint32_t> dbg_own_us_{0};
  std::atomic<uint32_t> dbg_inflight_us_{0};
  std::atomic<uint32_t> dbg_padded_frames_{0};
  std::atomic<uint32_t> dbg_xfer_us_{0};
  std::atomic<uint32_t> dbg_queued_us_{0};
  std::atomic<uint32_t> dbg_dma_us_{0};
  std::atomic<uint32_t> render_nondraining_us_{0};
  std::atomic<uint32_t> dbg_src_received_{0};
  std::atomic<uint32_t> dbg_src_consumed_{0};
  std::atomic<uint32_t> dbg_sink_received_{0};
};

class AudioStreamInfo {
  /* Class to respresent important parameters of the audio stream that also provides helper function to convert between
   * various audio related units.
   *
   *  - An audio sample represents a unit of audio for one channel.
   *  - A frame represents a unit of audio with a sample for every channel.
   *
   * In general, converting between bytes, samples, and frames shouldn't result in rounding errors so long as frames
   * are used as the main unit when transferring audio data. Durations may result in rounding for certain sample rates;
   * e.g., 44.1 KHz. The ``frames_to_milliseconds_with_remainder`` function should be used for accuracy, as it takes
   * into account the remainder rather than just ignoring any rounding.
   */
 public:
  AudioStreamInfo()
      : AudioStreamInfo(16, 1, 16000){};  // Default values represent ESPHome's audio components historical values
  AudioStreamInfo(uint8_t bits_per_sample, uint8_t channels, uint32_t sample_rate);

  uint8_t get_bits_per_sample() const { return this->bits_per_sample_; }
  uint8_t get_channels() const { return this->channels_; }
  uint32_t get_sample_rate() const { return this->sample_rate_; }

  /// @brief Convert bytes to duration in milliseconds.
  /// @param bytes Number of bytes to convert
  /// @return Duration in milliseconds that will store `bytes` bytes of audio. May round down for certain sample rates
  ///         or values of `bytes`.
  uint32_t bytes_to_ms(size_t bytes) const {
    return bytes * 1000 / (this->sample_rate_ * this->bytes_per_sample_ * this->channels_);
  }

  /// @brief Convert bytes to frames.
  /// @param bytes Number of bytes to convert
  /// @return Audio frames that will store `bytes` bytes.
  uint32_t bytes_to_frames(size_t bytes) const { return (bytes / (this->bytes_per_sample_ * this->channels_)); }

  /// @brief Convert bytes to samples.
  /// @param bytes Number of bytes to convert
  /// @return Audio samples that will store `bytes` bytes.
  uint32_t bytes_to_samples(size_t bytes) const { return (bytes / this->bytes_per_sample_); }

  /// @brief Converts frames to bytes.
  /// @param frames Number of frames to convert.
  /// @return Number of bytes that will store `frames` frames of audio.
  size_t frames_to_bytes(uint32_t frames) const { return frames * this->bytes_per_sample_ * this->channels_; }

  /// @brief Converts samples to bytes.
  /// @param samples Number of samples to convert.
  /// @return Number of bytes that will store `samples` samples of audio.
  size_t samples_to_bytes(uint32_t samples) const { return samples * this->bytes_per_sample_; }

  /// @brief Converts duration to frames.
  /// @param ms Duration in milliseconds
  /// @return Audio frames that will store `ms` milliseconds of audio.  May round down for certain sample rates.
  uint32_t ms_to_frames(uint32_t ms) const {
    return static_cast<uint32_t>((static_cast<uint64_t>(ms) * this->sample_rate_) / 1000);
  }

  /// @brief Converts duration to samples.
  /// @param ms Duration in milliseconds
  /// @return Audio samples that will store `ms` milliseconds of audio.  May round down for certain sample rates.
  uint32_t ms_to_samples(uint32_t ms) const {
    return static_cast<uint32_t>((static_cast<uint64_t>(ms) * this->channels_ * this->sample_rate_) / 1000);
  }

  /// @brief Converts duration to bytes. May round down for certain sample rates.
  /// @param ms Duration in milliseconds
  /// @return Bytes that will store `ms` milliseconds of audio.  May round down for certain sample rates.
  // 64-bit intermediate: the 32-bit product wraps past ~24 s for 16-bit stereo at 44.1 kHz. Latent
  // rather than observed -- callers pass buffer durations -- but the same defect as
  // frames_to_microseconds(), which was not latent.
  size_t ms_to_bytes(uint32_t ms) const {
    return static_cast<size_t>((static_cast<uint64_t>(ms) * this->bytes_per_sample_ * this->channels_ *
                                this->sample_rate_) /
                               1000);
  }

  /// @brief Computes the duration, in microseconds, the given amount of frames represents.
  /// @param frames Number of audio frames
  /// @return Duration in microseconds `frames` represents. May be slightly inaccurate due to integer division rounding
  ///         for certain sample rates.
  uint32_t frames_to_microseconds(uint32_t frames) const;

  /// @brief Computes the duration, in milliseconds, the given amount of frames represents. Avoids
  /// accumulating rounding errors by updating `frames` with the remainder after converting.
  /// @param frames Pointer to uint32_t with the number of audio frames. Replaced with the remainder.
  /// @return Duration in milliseconds `frames` represents. Always less than or equal to the actual value due to
  ///         rounding.
  uint32_t frames_to_milliseconds_with_remainder(uint32_t *frames) const;

  // Class comparison operators
  bool operator==(const AudioStreamInfo &rhs) const;
  bool operator!=(const AudioStreamInfo &rhs) const { return !operator==(rhs); }

 protected:
  uint8_t bits_per_sample_;
  uint8_t channels_;
  uint32_t sample_rate_;

  // The greatest common divisor between 1000 ms = 1 second and the sample rate. Used to avoid accumulating error when
  // converting from frames to duration. Computed at construction.
  uint32_t ms_sample_rate_gcd_;

  // Conversion factor derived from the number of bits per sample. Assumes audio data is aligned to the byte. Computed
  // at construction.
  size_t bytes_per_sample_;
};

enum class AudioFileType : uint8_t {
  NONE = 0,
#ifdef USE_AUDIO_FLAC_SUPPORT
  FLAC,
#endif
#ifdef USE_AUDIO_MP3_SUPPORT
  MP3,
#endif
#ifdef USE_AUDIO_OPUS_SUPPORT
  OPUS,
#endif
#ifdef USE_AUDIO_WAV_SUPPORT
  WAV,
#endif
};

struct AudioFile {
  const uint8_t *data;
  size_t length;
  AudioFileType file_type;
};

/// @brief Helper function to convert file type to a const char string
/// @param file_type
/// @return const char pointer to the readable file type
const char *audio_file_type_to_string(AudioFileType file_type);

/// @brief Detect audio file type from a Content-Type header value and/or URL extension.
/// Tries Content-Type first, then falls back to URL extension. Either parameter may be null.
/// @param content_type Content-Type header value (may be null or empty)
/// @param url URL to inspect for file extension (may be null or empty)
/// @return The detected AudioFileType, or NONE if unknown
AudioFileType detect_audio_file_type(const char *content_type, const char *url);

/// @brief Scales Q15 fixed point audio samples. Scales in place if audio_samples == output_buffer.
/// @param audio_samples PCM int16 audio samples
/// @param output_buffer Buffer to store the scaled samples
/// @param scale_factor Q15 fixed point scaling factor
/// @param samples_to_scale Number of samples to scale
// Remove before 2026.12.0
ESPDEPRECATED("Use esp_audio_libs::gain::apply() (from <gain.h>) instead. Removed in 2026.12.0.", "2026.6.0")
void scale_audio_samples(const int16_t *audio_samples, int16_t *output_buffer, int16_t scale_factor,
                         size_t samples_to_scale);

/// @brief Unpacks a quantized audio sample into a Q31 fixed-point number.
/// @param data Pointer to uint8_t array containing the audio sample
/// @param bytes_per_sample The number of bytes per sample
/// @return Q31 sample
inline int32_t unpack_audio_sample_to_q31(const uint8_t *data, size_t bytes_per_sample) {
  int32_t sample = 0;
  if (bytes_per_sample == 1) {
    sample |= data[0] << 24;
  } else if (bytes_per_sample == 2) {
    sample |= data[0] << 16;
    sample |= data[1] << 24;
  } else if (bytes_per_sample == 3) {
    sample |= data[0] << 8;
    sample |= data[1] << 16;
    sample |= data[2] << 24;
  } else if (bytes_per_sample == 4) {
    sample |= data[0];
    sample |= data[1] << 8;
    sample |= data[2] << 16;
    sample |= data[3] << 24;
  }

  return sample;
}

/// @brief Packs a Q31 fixed-point number as an audio sample with the specified number of bytes per sample.
/// Packs the most significant bits - no dithering is applied.
/// @param sample Q31 fixed-point number to pack
/// @param data Pointer to data array to store
/// @param bytes_per_sample The audio data's bytes per sample
inline void pack_q31_as_audio_sample(int32_t sample, uint8_t *data, size_t bytes_per_sample) {
  if (bytes_per_sample == 1) {
    data[0] = static_cast<uint8_t>(sample >> 24);
  } else if (bytes_per_sample == 2) {
    data[0] = static_cast<uint8_t>(sample >> 16);
    data[1] = static_cast<uint8_t>(sample >> 24);
  } else if (bytes_per_sample == 3) {
    data[0] = static_cast<uint8_t>(sample >> 8);
    data[1] = static_cast<uint8_t>(sample >> 16);
    data[2] = static_cast<uint8_t>(sample >> 24);
  } else if (bytes_per_sample == 4) {
    data[0] = static_cast<uint8_t>(sample);
    data[1] = static_cast<uint8_t>(sample >> 8);
    data[2] = static_cast<uint8_t>(sample >> 16);
    data[3] = static_cast<uint8_t>(sample >> 24);
  }
}

}  // namespace esphome::audio
