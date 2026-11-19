#include "rf_catalog_reader.h"

#include <ctype.h>
#include <esp_heap_caps.h>
#include <limits.h>


static uint32_t rf_catalog_crc32_update(uint32_t crc, const uint8_t *data,
                                        size_t length) {
  while (length--) {
    crc ^= *data++;
    for (uint8_t bit = 0; bit < 8; bit++)
      crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1UL)));
  }
  return crc;
}


RipperRfCatalogReader::RipperRfCatalogReader()
    : storage_(nullptr), ready_(false), strings_cache_(nullptr),
      strings_cache_bytes_(0) {
  memset(&header_, 0, sizeof(header_));
  error_[0] = '\0';
  index_path_[0] = '\0';
  archive_path_[0] = '\0';
}


void RipperRfCatalogReader::setError(const char *message) {
  strlcpy(error_, message, sizeof(error_));
}


void RipperRfCatalogReader::closeIndex() {
  if (index_) index_.close();
}


void RipperRfCatalogReader::closeArchive() {
  if (archive_) archive_.close();
}


void RipperRfCatalogReader::clearStringCache() {
  if (strings_cache_ != nullptr) heap_caps_free(strings_cache_);
  strings_cache_ = nullptr;
  strings_cache_bytes_ = 0;
}


bool RipperRfCatalogReader::loadStringCache() {
  clearStringCache();
  // The cache is metadata only. Signal payloads and profile collections stay
  // on SD. Cap it so a future oversized catalog cannot consume unbounded RAM.
  constexpr uint32_t MAX_STRING_CACHE_BYTES = 512UL * 1024UL;
  if (header_.strings_bytes == 0 ||
      header_.strings_bytes > MAX_STRING_CACHE_BYTES) return true;
  strings_cache_ = (uint8_t *)heap_caps_malloc(
    header_.strings_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (strings_cache_ == nullptr && header_.strings_bytes <= 65536UL) {
    strings_cache_ = (uint8_t *)heap_caps_malloc(
      header_.strings_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  }
  if (strings_cache_ == nullptr) return true;
  if (!index_.seek(header_.strings_offset)) {
    clearStringCache();
    return false;
  }
  uint32_t copied = 0;
  while (copied < header_.strings_bytes) {
    const size_t chunk = min((uint32_t)4096,
                             header_.strings_bytes - copied);
    if (index_.read(strings_cache_ + copied, chunk) != chunk) {
      clearStringCache();
      return false;
    }
    copied += chunk;
  }
  strings_cache_bytes_ = header_.strings_bytes;
  return true;
}


void RipperRfCatalogReader::end() {
  closeIndex();
  closeArchive();
  clearStringCache();
  storage_ = nullptr;
  ready_ = false;
  memset(&header_, 0, sizeof(header_));
  index_path_[0] = '\0';
  archive_path_[0] = '\0';
}


bool RipperRfCatalogReader::openIndex() {
  if (index_) return true;
  if (storage_ == nullptr || index_path_[0] == '\0') return false;
  index_ = storage_->open(index_path_, FILE_READ);
  return (bool)index_;
}


bool RipperRfCatalogReader::openArchive() {
  if (archive_) return true;
  if (storage_ == nullptr || archive_path_[0] == '\0') return false;
  archive_ = storage_->open(archive_path_, FILE_READ);
  return (bool)archive_;
}


bool RipperRfCatalogReader::verifyIndexRegion(uint32_t offset, uint32_t bytes,
                                              uint32_t expected_crc) {
  if (!index_.seek(offset)) return false;
  uint8_t buffer[256];
  uint32_t remaining = bytes;
  uint32_t crc = 0xFFFFFFFFUL;
  while (remaining > 0) {
    const size_t wanted = remaining < sizeof(buffer) ? remaining : sizeof(buffer);
    if (index_.read(buffer, wanted) != wanted) return false;
    crc = rf_catalog_crc32_update(crc, buffer, wanted);
    remaining -= wanted;
  }
  return (~crc) == expected_crc;
}


bool RipperRfCatalogReader::begin(fs::FS &storage, const char *index_path,
                                  const char *archive_path,
                                  bool verify_archive_size) {
  end();
  storage_ = &storage;
  auto fail = [&](const char *message) {
    setError(message);
    closeIndex();
    closeArchive();
    clearStringCache();
    return false;
  };
  if (index_path == nullptr || archive_path == nullptr ||
      strlcpy(index_path_, index_path, sizeof(index_path_)) >= sizeof(index_path_) ||
      strlcpy(archive_path_, archive_path, sizeof(archive_path_)) >= sizeof(archive_path_))
    return fail("RF CATALOG PATH ERROR");
  if (!openIndex()) return fail("RF CATALOG INDEX NOT FOUND");
  if (index_.read((uint8_t *)&header_, sizeof(header_)) != sizeof(header_))
    return fail("RF CATALOG HEADER SHORT");
  RipperRfCatalogIndexHeaderV1 checked = header_;
  checked.header_crc32 = 0;
  const uint32_t header_crc = ~rf_catalog_crc32_update(
      0xFFFFFFFFUL, (const uint8_t *)&checked, sizeof(checked));
  if (header_.magic != RIPPER_RF_CATALOG_INDEX_MAGIC ||
      header_.version != RIPPER_RF_CATALOG_INDEX_VERSION ||
      header_.header_size != sizeof(header_) ||
      header_.profile_record_size != sizeof(RipperRfCatalogProfileRecordV1) ||
      header_.signal_record_size != sizeof(RipperRfCatalogSignalRecordV1) ||
      header_.profile_count == 0 || header_.signal_count == 0 ||
      header_.profiles_offset != sizeof(header_) ||
      header_.header_crc32 != header_crc)
    return fail("RF CATALOG INDEX FORMAT");

  const uint64_t profile_bytes =
      (uint64_t)header_.profile_count * sizeof(RipperRfCatalogProfileRecordV1);
  const uint64_t signal_bytes =
      (uint64_t)header_.signal_count * sizeof(RipperRfCatalogSignalRecordV1);
  if (profile_bytes > UINT32_MAX || signal_bytes > UINT32_MAX ||
      header_.signals_offset != header_.profiles_offset + profile_bytes ||
      header_.strings_offset != header_.signals_offset + signal_bytes ||
      (uint64_t)header_.strings_offset + header_.strings_bytes != index_.size())
    return fail("RF CATALOG INDEX BOUNDS");
  if (!verifyIndexRegion(header_.profiles_offset, (uint32_t)profile_bytes,
                         header_.profiles_crc32) ||
      !verifyIndexRegion(header_.signals_offset, (uint32_t)signal_bytes,
                         header_.signals_crc32) ||
      !verifyIndexRegion(header_.strings_offset, header_.strings_bytes,
                         header_.strings_crc32))
    return fail("RF CATALOG INDEX CRC");
  if (!loadStringCache()) return fail("RF CATALOG STRING CACHE READ");
  if (!openArchive()) return fail("RF CATALOG ARCHIVE NOT FOUND");
  if (verify_archive_size && archive_.size() != header_.archive_bytes)
    return fail("RF CATALOG ARCHIVE SIZE");

  ready_ = true;
  error_[0] = '\0';
  // File handles consume scarce internal heap. Retain validated metadata and
  // reopen only for a specific query or streamed signal parse.
  closeIndex();
  closeArchive();
  return true;
}


bool RipperRfCatalogReader::readProfileRecord(
    uint32_t ordinal, RipperRfCatalogProfileRecordV1 &record) {
  if (!ready_ || ordinal >= header_.profile_count) return false;
  const uint64_t offset = header_.profiles_offset +
      (uint64_t)ordinal * sizeof(record);
  return offset <= UINT32_MAX &&
      (index_.position() == (size_t)offset || index_.seek((uint32_t)offset)) &&
      index_.read((uint8_t *)&record, sizeof(record)) == sizeof(record);
}


bool RipperRfCatalogReader::readSignalRecord(
    uint32_t ordinal, RipperRfCatalogSignalRecordV1 &record) {
  if (!ready_ || ordinal >= header_.signal_count) return false;
  const uint64_t offset = header_.signals_offset +
      (uint64_t)ordinal * sizeof(record);
  return offset <= UINT32_MAX &&
      (index_.position() == (size_t)offset || index_.seek((uint32_t)offset)) &&
      index_.read((uint8_t *)&record, sizeof(record)) == sizeof(record);
}


bool RipperRfCatalogReader::readString(uint32_t offset, char *target,
                                       size_t target_size) {
  if (target == nullptr || target_size == 0 ||
      offset >= header_.strings_bytes) return false;
  if (strings_cache_ != nullptr &&
      strings_cache_bytes_ == header_.strings_bytes) {
    const uint8_t *start = strings_cache_ + offset;
    const size_t remaining = strings_cache_bytes_ - offset;
    const uint8_t *terminator =
      (const uint8_t *)memchr(start, 0, remaining);
    if (terminator == nullptr) return false;
    const size_t length = terminator - start;
    if (length + 1 > target_size) return false;
    memcpy(target, start, length + 1);
    return true;
  }
  if (!ready_ || !index_.seek(header_.strings_offset + offset)) return false;
  size_t length = 0;
  while (offset++ < header_.strings_bytes) {
    const int value = index_.read();
    if (value < 0) return false;
    if (value == 0) {
      target[length] = '\0';
      return true;
    }
    if (length + 1 >= target_size) return false;
    target[length++] = (char)value;
  }
  return false;
}


bool RipperRfCatalogReader::readProfileOpen(uint32_t ordinal,
                                            RipperRfCatalogProfile &profile) {
  memset(&profile, 0, sizeof(profile));
  RipperRfCatalogProfileRecordV1 record = {};
  if (!readProfileRecord(ordinal, record) ||
      record.first_signal + record.signal_count > header_.signal_count ||
      !readString(record.category_offset, profile.category, sizeof(profile.category)) ||
      !readString(record.manufacturer_offset, profile.manufacturer,
                  sizeof(profile.manufacturer)) ||
      !readString(record.model_offset, profile.model, sizeof(profile.model)) ||
      !readString(record.source_path_offset, profile.source_path,
                  sizeof(profile.source_path))) return false;
  profile.ordinal = ordinal;
  profile.first_signal = record.first_signal;
  profile.signal_count = record.signal_count;
  profile.importable_count = record.importable_count;
  profile.raw_count = record.raw_count;
  profile.bin_raw_count = record.bin_raw_count;
  profile.parsed_count = record.parsed_count;
  profile.unsupported_count = record.unsupported_count;
  profile.flags = record.flags;
  return true;
}


bool RipperRfCatalogReader::readProfile(uint32_t ordinal,
                                        RipperRfCatalogProfile &profile) {
  if (!ready_ || !openIndex()) {
    setError("RF CATALOG INDEX OPEN");
    return false;
  }
  const bool result = readProfileOpen(ordinal, profile);
  closeIndex();
  if (!result) setError("RF CATALOG PROFILE READ");
  return result;
}


bool RipperRfCatalogReader::readSignalOpen(uint32_t profile_ordinal,
                                           uint16_t local_ordinal,
                                           RipperRfCatalogSignal &signal) {
  memset(&signal, 0, sizeof(signal));
  RipperRfCatalogProfileRecordV1 profile = {};
  if (!readProfileRecord(profile_ordinal, profile) ||
      local_ordinal >= profile.signal_count) return false;
  const uint32_t signal_ordinal = profile.first_signal + local_ordinal;
  RipperRfCatalogSignalRecordV1 record = {};
  if (!readSignalRecord(signal_ordinal, record) ||
      record.tar_data_offset + record.tar_data_bytes > header_.archive_bytes ||
      record.encoding > RIPPER_RF_CATALOG_ENCODING_PARSED ||
      !readString(record.label_offset, signal.label, sizeof(signal.label)) ||
      !readString(record.protocol_offset, signal.protocol, sizeof(signal.protocol)) ||
      !readString(record.preset_offset, signal.preset, sizeof(signal.preset)) ||
      !readString(record.source_path_offset, signal.source_path,
                  sizeof(signal.source_path))) return false;
  signal.profile_ordinal = profile_ordinal;
  signal.signal_ordinal = signal_ordinal;
  signal.local_ordinal = local_ordinal;
  signal.tar_data_offset = record.tar_data_offset;
  signal.tar_data_bytes = record.tar_data_bytes;
  signal.payload_crc32 = record.payload_crc32;
  signal.frequency_hz = record.frequency_hz;
  signal.raw_timing_count = record.raw_timing_count;
  signal.te_us = record.te_us;
  signal.bit_count = record.bit_count;
  signal.encoding = record.encoding;
  signal.flags = record.flags;
  return true;
}


bool RipperRfCatalogReader::readSignal(uint32_t profile_ordinal,
                                       uint16_t local_ordinal,
                                       RipperRfCatalogSignal &signal) {
  if (!ready_ || !openIndex()) {
    setError("RF CATALOG INDEX OPEN");
    return false;
  }
  const bool result = readSignalOpen(profile_ordinal, local_ordinal, signal);
  closeIndex();
  if (!result) setError("RF CATALOG SIGNAL READ");
  return result;
}


static bool rf_catalog_text_equal(const char *left, const char *right) {
  if (left == nullptr || right == nullptr) return false;
  while (*left != '\0' && *right != '\0') {
    const unsigned char a = (unsigned char)*left++;
    const unsigned char b = (unsigned char)*right++;
    if (tolower(a) != tolower(b)) return false;
  }
  return *left == '\0' && *right == '\0';
}


static bool rf_catalog_text_contains(const char *text, const char *wanted) {
  if (wanted == nullptr || wanted[0] == '\0') return true;
  if (text == nullptr) return false;
  const size_t wanted_length = strlen(wanted);
  for (const char *start = text; *start != '\0'; start++) {
    size_t matched = 0;
    while (matched < wanted_length && start[matched] != '\0' &&
           tolower((unsigned char)start[matched]) ==
               tolower((unsigned char)wanted[matched])) matched++;
    if (matched == wanted_length) return true;
  }
  return false;
}


static bool rf_catalog_text_matches_any(const char *text,
                                        const char *const *options,
                                        uint8_t option_count) {
  if (options == nullptr || option_count == 0) return true;
  for (uint8_t index = 0; index < option_count; index++) {
    if (options[index] != nullptr &&
        rf_catalog_text_equal(text, options[index])) return true;
  }
  return false;
}


static uint16_t rf_catalog_test_label_score(const char *label) {
  if (label == nullptr || label[0] == '\0') return 1;
  char normalized[48] = {};
  size_t used = 0;
  for (size_t index = 0; label[index] != '\0' && used + 1 < sizeof(normalized);
       index++) {
    const unsigned char value = (unsigned char)label[index];
    normalized[used++] = isalnum(value) ? (char)toupper(value) : ' ';
  }
  normalized[used] = '\0';
  if (rf_catalog_text_contains(normalized, "POWER") ||
      rf_catalog_text_equal(normalized, "ON") ||
      rf_catalog_text_equal(normalized, "OFF")) return 1000;
  if (rf_catalog_text_contains(normalized, "OPEN") ||
      rf_catalog_text_contains(normalized, "CLOSE") ||
      rf_catalog_text_contains(normalized, "LOCK") ||
      rf_catalog_text_contains(normalized, "UNLOCK")) return 900;
  if (rf_catalog_text_contains(normalized, "TOGGLE") ||
      rf_catalog_text_contains(normalized, "START") ||
      rf_catalog_text_contains(normalized, "STOP")) return 800;
  if (rf_catalog_text_contains(normalized, "UP") ||
      rf_catalog_text_contains(normalized, "DOWN") ||
      rf_catalog_text_contains(normalized, "LIGHT") ||
      rf_catalog_text_contains(normalized, "FAN")) return 700;
  return 100;
}


static bool rf_catalog_signal_supported_now(
    const RipperRfCatalogSignal &signal) {
  if ((signal.flags & RIPPER_RF_CATALOG_SIGNAL_IMPORTABLE_NOW) != 0) return true;
  // Older on-card indexes marked RAW signals above the original 4096/16-bit
  // repository limit as too large. The V2 repository can stream those
  // records as uint32_t timings, so recognize them without requiring users
  // to rebuild an otherwise valid catalog index.
  if (signal.encoding == RIPPER_RF_CATALOG_ENCODING_RAW &&
      (signal.flags & RIPPER_RF_CATALOG_SIGNAL_PARSEABLE) != 0 &&
      signal.raw_timing_count >= 2 && signal.raw_timing_count <= 524288UL &&
      signal.frequency_hz != 0 && strcmp(signal.protocol, "RAW") == 0 &&
      (strcmp(signal.preset, "FuriHalSubGhzPresetOok270Async") == 0 ||
       strcmp(signal.preset, "FuriHalSubGhzPresetOok650Async") == 0)) return true;
  // KeeLoq catalog support is deliberately limited to replaying the exact
  // stored 64-bit frame. It does not derive manufacturer keys, predict a
  // counter, or manufacture a new rolling code.
  return signal.encoding == RIPPER_RF_CATALOG_ENCODING_PARSED &&
      signal.bit_count == 64 && signal.frequency_hz != 0 &&
      strcmp(signal.protocol, "KeeLoq") == 0 &&
      strcmp(signal.preset, "FuriHalSubGhzPresetOok650Async") == 0;
}


static bool rf_catalog_signal_testable_now(
    const RipperRfCatalogSignal &signal) {
  return signal.encoding == RIPPER_RF_CATALOG_ENCODING_RAW &&
      strcmp(signal.protocol, "RAW") == 0 &&
      (signal.flags & RIPPER_RF_CATALOG_SIGNAL_PARSEABLE) != 0 &&
      signal.raw_timing_count >= 2 && signal.raw_timing_count <= 32768UL &&
      signal.frequency_hz != 0 &&
      (strcmp(signal.preset, "FuriHalSubGhzPresetOok270Async") == 0 ||
       strcmp(signal.preset, "FuriHalSubGhzPresetOok650Async") == 0);
}


bool RipperRfCatalogReader::selectTestSignalOpen(
    const RipperRfCatalogProfile &profile, uint16_t &local_ordinal,
    char *label, size_t label_size, uint16_t &score) {
  local_ordinal = 0;
  score = 0;
  if (label != nullptr && label_size > 0) label[0] = '\0';
  for (uint16_t local = 0; local < profile.signal_count; local++) {
    const uint32_t signal_ordinal = profile.first_signal + local;
    RipperRfCatalogSignalRecordV1 record = {};
    if (!readSignalRecord(signal_ordinal, record) ||
        record.tar_data_offset > header_.archive_bytes ||
        record.tar_data_bytes > header_.archive_bytes - record.tar_data_offset ||
        record.encoding > RIPPER_RF_CATALOG_ENCODING_PARSED) return false;

    // A discovery query only needs the signal metadata used for support and
    // label ranking. Avoid re-reading the owning profile and source path for
    // every command; those random FATFS seeks dominated category load time.
    RipperRfCatalogSignal signal = {};
    signal.profile_ordinal = profile.ordinal;
    signal.signal_ordinal = signal_ordinal;
    signal.local_ordinal = local;
    signal.tar_data_offset = record.tar_data_offset;
    signal.tar_data_bytes = record.tar_data_bytes;
    signal.payload_crc32 = record.payload_crc32;
    signal.frequency_hz = record.frequency_hz;
    signal.raw_timing_count = record.raw_timing_count;
    signal.te_us = record.te_us;
    signal.bit_count = record.bit_count;
    signal.encoding = record.encoding;
    signal.flags = record.flags;
    if (!readString(record.label_offset, signal.label, sizeof(signal.label)))
      return false;
    if (!readString(record.protocol_offset, signal.protocol,
                    sizeof(signal.protocol)) ||
        !readString(record.preset_offset, signal.preset,
                    sizeof(signal.preset))) return false;
    if (!rf_catalog_signal_testable_now(signal)) continue;
    const uint16_t candidate_score = rf_catalog_test_label_score(signal.label);
    if (candidate_score <= score) continue;
    local_ordinal = local;
    score = candidate_score;
    if (label != nullptr && label_size > 0)
      strlcpy(label, signal.label, label_size);
  }
  return score > 0;
}


bool RipperRfCatalogReader::findProfileBatch(
    const RipperRfCatalogQuery &query, RipperRfCatalogCandidateRef *results,
    size_t capacity, size_t &result_count, bool &has_more) {
  result_count = 0;
  has_more = false;
  if (!ready_ || results == nullptr || capacity == 0 || !openIndex()) {
    setError("RF CATALOG QUERY OPEN");
    return false;
  }

  struct CategoryMatchCacheEntry {
    uint32_t offset;
    bool matches;
    char text[32];
  };
  constexpr size_t CATEGORY_CACHE_CAPACITY = 24;
  constexpr size_t PROFILE_BLOCK_COUNT = 16;
  CategoryMatchCacheEntry category_cache[CATEGORY_CACHE_CAPACITY] = {};
  size_t category_cache_count = 0;
  RipperRfCatalogProfileRecordV1 profile_block[PROFILE_BLOCK_COUNT] = {};

  bool ok = true;
  uint32_t block_start = query.after_ordinal == UINT32_MAX ? 0 :
    query.after_ordinal + 1;
  while (block_start < header_.profile_count && ok && !has_more) {
    const size_t block_count = min(
      (uint32_t)PROFILE_BLOCK_COUNT, header_.profile_count - block_start);
    const uint64_t block_offset = header_.profiles_offset +
      (uint64_t)block_start * sizeof(RipperRfCatalogProfileRecordV1);
    const size_t block_bytes = block_count * sizeof(profile_block[0]);
    if (block_offset > UINT32_MAX || !index_.seek((uint32_t)block_offset) ||
        index_.read((uint8_t *)profile_block, block_bytes) != block_bytes) {
      ok = false;
      break;
    }

    for (size_t block_index = 0; block_index < block_count; block_index++) {
      const uint32_t ordinal = block_start + block_index;
      const RipperRfCatalogProfileRecordV1 &record = profile_block[block_index];
      if (record.first_signal + record.signal_count > header_.signal_count) {
        ok = false;
        break;
      }
      if (!query.include_exact_duplicates &&
          (record.flags & RIPPER_RF_CATALOG_PROFILE_EXACT_DUPLICATE) != 0)
        continue;

      char category[32] = {};
      bool category_matches = true;
      const bool category_filter =
        (query.category_options != nullptr && query.category_option_count > 0) ||
        (query.category != nullptr && query.category[0] != '\0');
      if (category_filter) {
        bool cached = false;
        for (size_t cache_index = 0; cache_index < category_cache_count;
             cache_index++) {
          if (category_cache[cache_index].offset != record.category_offset)
            continue;
          category_matches = category_cache[cache_index].matches;
          strlcpy(category, category_cache[cache_index].text,
                  sizeof(category));
          cached = true;
          break;
        }
        if (!cached) {
          if (!readString(record.category_offset, category, sizeof(category))) {
            ok = false;
            break;
          }
          if (query.category_options != nullptr &&
              query.category_option_count > 0) {
            category_matches = rf_catalog_text_matches_any(
              category, query.category_options, query.category_option_count);
          } else {
            category_matches = rf_catalog_text_equal(category, query.category);
          }
          if (category_cache_count < CATEGORY_CACHE_CAPACITY) {
            CategoryMatchCacheEntry &entry =
              category_cache[category_cache_count++];
            entry.offset = record.category_offset;
            entry.matches = category_matches;
            strlcpy(entry.text, category, sizeof(entry.text));
          }
        }
      } else if (!readString(record.category_offset, category,
                             sizeof(category))) {
        ok = false;
        break;
      }
      if (!category_matches) continue;

      RipperRfCatalogProfile profile = {};
      profile.ordinal = ordinal;
      profile.first_signal = record.first_signal;
      profile.signal_count = record.signal_count;
      profile.importable_count = record.importable_count;
      profile.raw_count = record.raw_count;
      profile.bin_raw_count = record.bin_raw_count;
      profile.parsed_count = record.parsed_count;
      profile.unsupported_count = record.unsupported_count;
      profile.flags = record.flags;
      strlcpy(profile.category, category, sizeof(profile.category));
      if (!readString(record.manufacturer_offset, profile.manufacturer,
                      sizeof(profile.manufacturer)) ||
          !readString(record.model_offset, profile.model,
                      sizeof(profile.model))) {
        ok = false;
        break;
      }
      if (query.manufacturer != nullptr && query.manufacturer[0] != '\0' &&
          !rf_catalog_text_equal(profile.manufacturer, query.manufacturer))
        continue;
      if (!rf_catalog_text_contains(profile.model, query.model_contains))
        continue;
      if (query.require_complete) {
        if (profile.signal_count == 0) continue;
        bool complete = true;
        for (uint16_t local = 0; local < profile.signal_count; local++) {
          RipperRfCatalogSignal signal = {};
          if (!readSignalOpen(profile.ordinal, local, signal)) {
            ok = false;
            complete = false;
            break;
          }
          if (!rf_catalog_signal_supported_now(signal)) {
            complete = false;
            break;
          }
        }
        if (!ok) break;
        if (!complete) continue;
      }
      uint16_t test_signal = 0;
      uint16_t test_score = 0;
      char test_label[48] = {};
      if (!selectTestSignalOpen(profile, test_signal, test_label,
                                sizeof(test_label), test_score)) continue;
      if (result_count >= capacity) {
        has_more = true;
        break;
      }
      RipperRfCatalogCandidateRef &candidate = results[result_count++];
      memset(&candidate, 0, sizeof(candidate));
      candidate.profile_ordinal = ordinal;
      candidate.signal_count = profile.signal_count;
      candidate.test_signal_ordinal = test_signal;
      candidate.score = test_score +
        (profile.signal_count > 1 ? 100 : 0) +
        (profile.manufacturer[0] != '\0' ? 50 : 0);
      candidate.flags = profile.flags;
      strlcpy(candidate.category, profile.category, sizeof(candidate.category));
      strlcpy(candidate.manufacturer, profile.manufacturer,
              sizeof(candidate.manufacturer));
      strlcpy(candidate.model, profile.model, sizeof(candidate.model));
      strlcpy(candidate.test_label, test_label, sizeof(candidate.test_label));
    }
    block_start += block_count;
  }
  closeIndex();
  if (!ok) setError("RF CATALOG QUERY READ");
  else error_[0] = '\0';
  return ok;
}


enum RipperRfCatalogStreamField : uint8_t {
  RF_STREAM_IGNORE = 0,
  RF_STREAM_RAW,
  RF_STREAM_BIN_RAW,
  RF_STREAM_PARSED_KEY,
};


static RipperRfCatalogStreamField rf_catalog_stream_field(const char *key) {
  if (strcmp(key, "RAW_Data") == 0) return RF_STREAM_RAW;
  if (strcmp(key, "Data_RAW") == 0) return RF_STREAM_BIN_RAW;
  if (strcmp(key, "Key") == 0) return RF_STREAM_PARSED_KEY;
  return RF_STREAM_IGNORE;
}


static bool rf_catalog_emit_token(RipperRfCatalogStreamField field,
                                  char *token, uint8_t &token_length,
                                  RipperRfCatalogSignalVisitor &visitor,
                                  uint32_t &emitted) {
  if (token_length == 0) return true;
  token[token_length] = '\0';
  token_length = 0;
  char *end = nullptr;
  if (field == RF_STREAM_RAW) {
    const long value = strtol(token, &end, 10);
    if (end == token || *end != '\0' || value == 0 || value < INT32_MIN ||
        value > INT32_MAX || !visitor.onRawTiming((int32_t)value)) return false;
  } else {
    const unsigned long value = strtoul(token, &end, 16);
    if (end == token || *end != '\0' || value > 0xFF) return false;
    const bool accepted = field == RF_STREAM_BIN_RAW ?
        visitor.onBinRawByte((uint8_t)value) :
        visitor.onParsedKeyByte((uint8_t)value);
    if (!accepted) return false;
  }
  emitted++;
  return true;
}


bool RipperRfCatalogReader::parseSignal(
    uint32_t profile_ordinal, uint16_t local_ordinal,
    RipperRfCatalogSignalVisitor &visitor) {
  RipperRfCatalogSignal signal = {};
  if (!readSignal(profile_ordinal, local_ordinal, signal) || !openArchive()) {
    setError("RF CATALOG SIGNAL OPEN");
    return false;
  }
  if (!archive_.seek(signal.tar_data_offset)) {
    closeArchive();
    setError("RF CATALOG SIGNAL SEEK");
    return false;
  }

  // Validate before exposing any timing or key data to a caller.
  uint8_t buffer[256];
  uint32_t remaining = signal.tar_data_bytes;
  uint32_t crc = 0xFFFFFFFFUL;
  while (remaining > 0) {
    const size_t wanted = remaining < sizeof(buffer) ? remaining : sizeof(buffer);
    if (archive_.read(buffer, wanted) != wanted) {
      closeArchive();
      setError("RF CATALOG SIGNAL SHORT");
      return false;
    }
    crc = rf_catalog_crc32_update(crc, buffer, wanted);
    remaining -= wanted;
  }
  if ((~crc) != signal.payload_crc32 ||
      !archive_.seek(signal.tar_data_offset) || !visitor.onSignalBegin(signal)) {
    closeArchive();
    setError("RF CATALOG SIGNAL CRC");
    return false;
  }

  char key[20] = {};
  char token[24] = {};
  uint8_t key_length = 0;
  uint8_t token_length = 0;
  bool reading_key = true;
  bool key_overflow = false;
  RipperRfCatalogStreamField field = RF_STREAM_IGNORE;
  uint32_t emitted = 0;
  remaining = signal.tar_data_bytes;
  bool ok = true;
  while (remaining-- > 0 && ok) {
    const int input = archive_.read();
    if (input < 0) { ok = false; break; }
    const char value = (char)input;
    if (value == '\r') continue;
    if (value == '\n') {
      ok = rf_catalog_emit_token(field, token, token_length, visitor, emitted);
      key_length = 0;
      reading_key = true;
      key_overflow = false;
      field = RF_STREAM_IGNORE;
      continue;
    }
    if (reading_key) {
      if (value == ':') {
        key[key_length] = '\0';
        field = key_overflow ? RF_STREAM_IGNORE : rf_catalog_stream_field(key);
        reading_key = false;
      } else if (key_length + 1 < sizeof(key)) {
        key[key_length++] = value;
      } else {
        key_overflow = true;
      }
      continue;
    }
    if (field == RF_STREAM_IGNORE) continue;
    if (isspace((unsigned char)value)) {
      ok = rf_catalog_emit_token(field, token, token_length, visitor, emitted);
    } else if (token_length + 1 < sizeof(token)) {
      token[token_length++] = value;
    } else {
      ok = false;
    }
  }
  if (ok) ok = rf_catalog_emit_token(field, token, token_length, visitor, emitted);
  closeArchive();
  if (ok && signal.encoding == RIPPER_RF_CATALOG_ENCODING_RAW)
    ok = emitted == signal.raw_timing_count;
  if (ok && signal.encoding != RIPPER_RF_CATALOG_ENCODING_RAW)
    ok = emitted > 0;
  if (ok) ok = visitor.onSignalEnd(emitted);
  if (!ok) setError("RF CATALOG SIGNAL PARSE");
  return ok;
}
