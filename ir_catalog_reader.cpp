#include "ir_catalog_reader.h"

#include <ctype.h>
#include <strings.h>

static uint32_t catalog_crc32_update(uint32_t crc, const uint8_t *data, size_t length) {
  while (length--) {
    crc ^= *data++;
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc >> 1) ^ (0xEDB88320UL & (0UL - (crc & 1UL)));
    }
  }
  return crc;
}

static uint64_t parse_hex_bytes(const char *text) {
  uint64_t value = 0;
  uint8_t shift = 0;
  while (*text && shift < 64) {
    while (*text == ' ') text++;
    if (!isxdigit((unsigned char)text[0])) break;
    char pair[3] = {text[0], isxdigit((unsigned char)text[1]) ? text[1] : '0', 0};
    value |= ((uint64_t)strtoul(pair, nullptr, 16)) << shift;
    shift += 8;
    text += isxdigit((unsigned char)text[1]) ? 2 : 1;
  }
  return value;
}

static bool catalog_protocol_supported(const char *protocol) {
  static const char *supported[] = {
    "NEC", "NECext", "Samsung32", "SIRC", "SIRC15", "SIRC20", "RC5", "RC5X", "RC6"
  };
  for (const char *item : supported) {
    if (strcmp(protocol, item) == 0) return true;
  }
  return false;
}

// Catalog folder names are not always presented consistently (for example,
// "CurtisMathis" and "Curtis_Mathis"). Treat case and punctuation-only
// differences as the same maker while keeping genuinely different names
// separate. This same comparison is used when finding profiles so merging a
// display entry never makes the alternate spelling's profiles unreachable.
static bool catalog_name_equivalent(const char *left, const char *right) {
  if (left == nullptr || right == nullptr) return false;
  while (true) {
    while (*left && !isalnum((unsigned char)*left)) left++;
    while (*right && !isalnum((unsigned char)*right)) right++;
    const unsigned char left_char = (unsigned char)*left;
    const unsigned char right_char = (unsigned char)*right;
    if (tolower(left_char) != tolower(right_char)) return false;
    if (left_char == 0 || right_char == 0) return left_char == right_char;
    left++;
    right++;
  }
}

RipperIrCatalogReader::RipperIrCatalogReader()
    : storage_(nullptr), ready_(false) {
  memset(&header_, 0, sizeof(header_));
  error_[0] = '\0';
  index_path_[0] = '\0';
  archive_path_[0] = '\0';
}

void RipperIrCatalogReader::setError(const char *message) {
  strlcpy(error_, message, sizeof(error_));
}

void RipperIrCatalogReader::end() {
  closeIndex();
  closeArchive();
  storage_ = nullptr;
  ready_ = false;
  memset(&header_, 0, sizeof(header_));
  index_path_[0] = '\0';
  archive_path_[0] = '\0';
}

bool RipperIrCatalogReader::openIndex() {
  if (index_) return true;
  if (storage_ == nullptr || index_path_[0] == '\0') return false;
  index_ = storage_->open(index_path_, FILE_READ);
  return (bool)index_;
}

bool RipperIrCatalogReader::openArchive() {
  if (archive_) return true;
  if (storage_ == nullptr || archive_path_[0] == '\0') return false;
  archive_ = storage_->open(archive_path_, FILE_READ);
  return (bool)archive_;
}

void RipperIrCatalogReader::closeIndex() {
  if (index_) index_.close();
}

void RipperIrCatalogReader::closeArchive() {
  if (archive_) archive_.close();
}

bool RipperIrCatalogReader::verifyRegionCrc(uint32_t offset, uint32_t bytes,
                                            uint32_t expected) {
  if (!index_.seek(offset)) return false;
  uint8_t buffer[256];
  uint32_t remaining = bytes;
  uint32_t crc = 0xFFFFFFFFUL;
  while (remaining > 0) {
    const size_t wanted = remaining < sizeof(buffer) ? remaining : sizeof(buffer);
    const size_t count = index_.read(buffer, wanted);
    if (count != wanted) return false;
    crc = catalog_crc32_update(crc, buffer, count);
    remaining -= count;
  }
  return (~crc) == expected;
}

bool RipperIrCatalogReader::begin(fs::FS &storage, const char *index_path,
                                  const char *archive_path,
                                  bool verify_archive_size) {
  end();
  ready_ = false;
  error_[0] = '\0';
  storage_ = &storage;
  auto fail = [&](const char *message) {
    setError(message);
    closeIndex();
    closeArchive();
    return false;
  };
  if (strlcpy(index_path_, index_path, sizeof(index_path_)) >= sizeof(index_path_) ||
      strlcpy(archive_path_, archive_path, sizeof(archive_path_)) >= sizeof(archive_path_)) {
    return fail("CATALOG PATH TOO LONG");
  }
  index_ = storage.open(index_path_, FILE_READ);
  if (!index_) return fail("CATALOG INDEX NOT FOUND");
  if (index_.read((uint8_t *)&header_, sizeof(header_)) != sizeof(header_)) {
    return fail("CATALOG INDEX HEADER SHORT");
  }
  if (header_.magic != RIPPER_IR_CATALOG_INDEX_MAGIC ||
      header_.version != RIPPER_IR_CATALOG_INDEX_VERSION ||
      header_.header_size != sizeof(header_) ||
      header_.record_size != sizeof(RipperIrCatalogRecordV1) ||
      header_.records_offset != sizeof(header_) || header_.record_count == 0) {
    return fail("CATALOG INDEX FORMAT ERROR");
  }
  RipperIrCatalogIndexHeaderV1 checked = header_;
  checked.header_crc32 = 0;
  uint32_t crc = catalog_crc32_update(0xFFFFFFFFUL, (uint8_t *)&checked, sizeof(checked));
  if ((~crc) != header_.header_crc32) {
    return fail("CATALOG INDEX HEADER CRC");
  }
  const uint64_t records_bytes =
      (uint64_t)header_.record_count * sizeof(RipperIrCatalogRecordV1);
  if (records_bytes > UINT32_MAX ||
      header_.strings_offset != header_.records_offset + (uint32_t)records_bytes ||
      (uint64_t)header_.strings_offset + header_.strings_bytes > index_.size()) {
    return fail("CATALOG INDEX BOUNDS ERROR");
  }
  if (!verifyRegionCrc(header_.records_offset, (uint32_t)records_bytes,
                       header_.records_crc32) ||
      !verifyRegionCrc(header_.strings_offset, header_.strings_bytes,
                       header_.strings_crc32)) {
    return fail("CATALOG INDEX DATA CRC");
  }
  archive_ = storage.open(archive_path_, FILE_READ);
  if (!archive_) return fail("CATALOG ARCHIVE NOT FOUND");
  if (verify_archive_size && archive_.size() != header_.archive_bytes) {
    return fail("CATALOG ARCHIVE SIZE ERROR");
  }
  ready_ = true;
  // Keeping FATFS File objects open reserves internal heap. BLE/Wi-Fi also
  // require internal memory on ESP32-S3, so retain only paths and validated
  // metadata between catalog operations.
  closeIndex();
  closeArchive();
  return true;
}

bool RipperIrCatalogReader::readRecord(uint32_t ordinal,
                                       RipperIrCatalogRecordV1 &record) {
  if (!ready_ || ordinal >= header_.record_count) return false;
  const uint32_t offset = header_.records_offset + ordinal * sizeof(record);
  return index_.seek(offset) && index_.read((uint8_t *)&record, sizeof(record)) == sizeof(record);
}

bool RipperIrCatalogReader::readString(uint32_t offset, char *target,
                                       size_t target_size) {
  if (!ready_ || target == nullptr || target_size == 0 || offset >= header_.strings_bytes ||
      !index_.seek(header_.strings_offset + offset)) return false;
  size_t length = 0;
  while (offset++ < header_.strings_bytes) {
    const int value = index_.read();
    if (value < 0) return false;
    if (value == 0) { target[length] = '\0'; return true; }
    if (length + 1 >= target_size) return false;
    target[length++] = (char)value;
  }
  return false;
}

bool RipperIrCatalogReader::recordStringEquals(uint32_t offset, const char *wanted) {
  if (!ready_ || wanted == nullptr || offset >= header_.strings_bytes ||
      !index_.seek(header_.strings_offset + offset)) return false;
  while (offset++ < header_.strings_bytes) {
    const int value = index_.read();
    if (value < 0) return false;
    if ((char)value != *wanted) return false;
    if (value == 0) return true;
    wanted++;
  }
  return false;
}

bool RipperIrCatalogReader::readProfileOpen(uint32_t ordinal,
                                            RipperIrCatalogProfile &profile) {
  RipperIrCatalogRecordV1 record = {};
  memset(&profile, 0, sizeof(profile));
  if (!readRecord(ordinal, record) ||
      record.tar_data_offset + record.tar_data_bytes > header_.archive_bytes ||
      !readString(record.category_offset, profile.category, sizeof(profile.category)) ||
      !readString(record.manufacturer_offset, profile.manufacturer, sizeof(profile.manufacturer)) ||
      !readString(record.model_offset, profile.model, sizeof(profile.model)) ||
      !readString(record.source_path_offset, profile.source_path, sizeof(profile.source_path))) {
    setError("CATALOG PROFILE METADATA ERROR");
    return false;
  }
  profile.ordinal = ordinal;
  profile.tar_data_offset = record.tar_data_offset;
  profile.tar_data_bytes = record.tar_data_bytes;
  profile.profile_crc32 = record.profile_crc32;
  profile.command_count = record.command_count;
  profile.parsed_count = record.parsed_count;
  profile.raw_count = record.raw_count;
  profile.unsupported_count = record.unsupported_count;
  profile.flags = record.flags;
  return true;
}

bool RipperIrCatalogReader::readProfile(uint32_t ordinal,
                                        RipperIrCatalogProfile &profile) {
  if (!ready_ || !openIndex()) { setError("CATALOG INDEX OPEN ERROR"); return false; }
  const bool result = readProfileOpen(ordinal, profile);
  closeIndex();
  return result;
}

static bool catalog_emit_group(RipperIrCatalogNameVisitor &visitor,
                               const char *name, uint32_t count) {
  return count == 0 || visitor.onName(name, count);
}

static uint32_t catalog_candidate_score(const RipperIrCatalogRecordV1 &record,
                                        uint16_t usable) {
  // Completeness dominates. Power is only a convenient test signal, not
  // evidence that a one-button profile is better than a complete remote.
  uint32_t score = usable * 1024UL + record.command_count;
  if (record.flags & RIPPER_IR_CATALOG_FLAG_HAS_POWER_TEST) score += 256UL;
  if (record.raw_count == 0) score += 128UL;
  if (record.unsupported_count == 0) score += 64UL;
  return score;
}

bool RipperIrCatalogReader::visitCategories(RipperIrCatalogNameVisitor &visitor) {
  if (!ready_ || !openIndex()) { setError("CATALOG INDEX OPEN ERROR"); return false; }
  char current[24] = {};
  uint32_t count = 0;
  RipperIrCatalogRecordV1 record = {};
  for (uint32_t i = 0; i < header_.record_count; i++) {
    char name[24] = {};
    if (!readRecord(i, record) || !readString(record.category_offset, name, sizeof(name))) {
      setError("CATALOG CATEGORY READ ERROR"); closeIndex(); return false;
    }
    if (count > 0 && strcmp(name, current) != 0) {
      if (!catalog_emit_group(visitor, current, count)) { closeIndex(); return true; }
      count = 0;
    }
    if (count == 0) strlcpy(current, name, sizeof(current));
    count++;
  }
  const bool result = catalog_emit_group(visitor, current, count);
  closeIndex();
  return result;
}

bool RipperIrCatalogReader::visitManufacturers(const char *category,
                                                RipperIrCatalogNameVisitor &visitor) {
  if (!ready_ || category == nullptr || category[0] == '\0' || !openIndex()) {
    setError("CATALOG INDEX OPEN ERROR"); return false;
  }

  // Profiles are generated in case-insensitive category/manufacturer/path
  // order. Locate the requested category with two binary searches instead of
  // resolving a category string for every profile on the SD card. The old
  // full scan performed thousands of random string-table seeks inside an LVGL
  // callback and made Known Manufacturer appear frozen on the device.
  auto compareCategory = [&](uint32_t ordinal, int &comparison) -> bool {
    RipperIrCatalogRecordV1 record = {};
    char name[24] = {};
    if (!readRecord(ordinal, record) ||
        !readString(record.category_offset, name, sizeof(name))) return false;
    comparison = strcasecmp(name, category);
    return true;
  };

  uint32_t low = 0;
  uint32_t high = header_.record_count;
  while (low < high) {
    const uint32_t middle = low + (high - low) / 2;
    int comparison = 0;
    if (!compareCategory(middle, comparison)) {
      setError("CATALOG MAKER SEEK ERROR"); closeIndex(); return false;
    }
    if (comparison < 0) low = middle + 1;
    else high = middle;
  }
  const uint32_t category_begin = low;
  if (category_begin >= header_.record_count) {
    closeIndex(); return true;
  }
  int first_comparison = 0;
  if (!compareCategory(category_begin, first_comparison)) {
    setError("CATALOG MAKER SEEK ERROR"); closeIndex(); return false;
  }
  if (first_comparison != 0) {
    closeIndex(); return true;
  }

  high = header_.record_count;
  while (low < high) {
    const uint32_t middle = low + (high - low) / 2;
    int comparison = 0;
    if (!compareCategory(middle, comparison)) {
      setError("CATALOG MAKER SEEK ERROR"); closeIndex(); return false;
    }
    if (comparison <= 0) low = middle + 1;
    else high = middle;
  }
  const uint32_t category_end = low;

  char current[48] = {};
  char record_manufacturer[48] = {};
  uint32_t record_manufacturer_offset = UINT32_MAX;
  uint32_t count = 0;
  RipperIrCatalogRecordV1 record = {};
  for (uint32_t i = category_begin; i < category_end; i++) {
    if (!readRecord(i, record)) {
      setError("CATALOG MAKER READ ERROR"); closeIndex(); return false;
    }
    const uint16_t usable_count = (uint16_t)(
        (record.flags & RIPPER_IR_CATALOG_USABLE_COUNT_MASK) >>
        RIPPER_IR_CATALOG_USABLE_COUNT_SHIFT);
    if (usable_count == 0) continue;

    if (record.manufacturer_offset != record_manufacturer_offset) {
      if (!readString(record.manufacturer_offset, record_manufacturer,
                      sizeof(record_manufacturer))) {
        setError("CATALOG MAKER NAME ERROR"); closeIndex(); return false;
      }
      record_manufacturer_offset = record.manufacturer_offset;
    }
    if (count > 0 && !catalog_name_equivalent(current, record_manufacturer)) {
      if (!catalog_emit_group(visitor, current, count)) { closeIndex(); return true; }
      count = 0;
    }
    if (count == 0) strlcpy(current, record_manufacturer, sizeof(current));
    count++;
  }
  const bool result = count == 0 || catalog_emit_group(visitor, current, count);
  closeIndex();
  return result;
}

bool RipperIrCatalogReader::visitModels(const char *category, const char *manufacturer,
                                        RipperIrCatalogNameVisitor &visitor) {
  if (!ready_ || category == nullptr || manufacturer == nullptr || !openIndex()) {
    setError("CATALOG INDEX OPEN ERROR"); return false;
  }
  char current[72] = {};
  uint32_t count = 0;
  RipperIrCatalogRecordV1 record = {};
  for (uint32_t i = 0; i < header_.record_count; i++) {
    if (!readRecord(i, record)) {
      setError("CATALOG MODEL READ ERROR"); closeIndex(); return false;
    }
    char record_manufacturer[48] = {};
    if (!recordStringEquals(record.category_offset, category) ||
        !readString(record.manufacturer_offset, record_manufacturer,
                    sizeof(record_manufacturer)) ||
        !catalog_name_equivalent(record_manufacturer, manufacturer)) continue;
    char name[72] = {};
    if (!readString(record.model_offset, name, sizeof(name))) {
      setError("CATALOG MODEL NAME ERROR"); closeIndex(); return false;
    }
    if (count > 0 && strcmp(name, current) != 0) {
      if (!catalog_emit_group(visitor, current, count)) { closeIndex(); return true; }
      count = 0;
    }
    if (count == 0) strlcpy(current, name, sizeof(current));
    count++;
  }
  const bool result = catalog_emit_group(visitor, current, count);
  closeIndex();
  return result;
}

bool RipperIrCatalogReader::findNextCandidate(
    const char *category, const char *manufacturer, const char *model,
    const RipperIrCatalogCandidate *previous, RipperIrCatalogCandidate &candidate) {
  if (!ready_ || category == nullptr || category[0] == '\0' || !openIndex()) {
    setError("CATALOG INDEX OPEN ERROR"); return false;
  }
  bool found = false;
  uint32_t best_score = 0;
  uint32_t best_ordinal = 0;
  RipperIrCatalogRecordV1 record = {};
  for (uint32_t i = 0; i < header_.record_count; i++) {
    if (!readRecord(i, record)) {
      setError("CATALOG CANDIDATE READ ERROR"); closeIndex(); return false;
    }
    if ((record.flags & (RIPPER_IR_CATALOG_FLAG_EXACT_DUPLICATE |
                         RIPPER_IR_CATALOG_FLAG_INCOMPLETE_METADATA)) != 0) continue;
    if (!recordStringEquals(record.category_offset, category)) continue;
    if (manufacturer != nullptr && manufacturer[0] != '\0') {
      char record_manufacturer[48] = {};
      if (!readString(record.manufacturer_offset, record_manufacturer,
                      sizeof(record_manufacturer)) ||
          !catalog_name_equivalent(record_manufacturer, manufacturer)) continue;
    }
    if (model != nullptr && model[0] != '\0' &&
        !recordStringEquals(record.model_offset, model)) continue;
    const uint16_t usable = (record.flags & RIPPER_IR_CATALOG_USABLE_COUNT_MASK) >>
                            RIPPER_IR_CATALOG_USABLE_COUNT_SHIFT;
    const uint16_t test_plus_one = (record.flags & RIPPER_IR_CATALOG_TEST_ORDINAL_MASK) >>
                                   RIPPER_IR_CATALOG_TEST_ORDINAL_SHIFT;
    if (usable == 0 || test_plus_one == 0) continue;
    const uint32_t score = catalog_candidate_score(record, usable);
    if (previous != nullptr &&
        (score > previous->score ||
         (score == previous->score && i <= previous->profile.ordinal))) continue;
    if (!found || score > best_score || (score == best_score && i < best_ordinal)) {
      found = true;
      best_score = score;
      best_ordinal = i;
    }
  }
  if (!found || !readProfileOpen(best_ordinal, candidate.profile)) {
    closeIndex(); return false;
  }
  candidate.score = best_score;
  candidate.usable_command_count =
      (candidate.profile.flags & RIPPER_IR_CATALOG_USABLE_COUNT_MASK) >>
      RIPPER_IR_CATALOG_USABLE_COUNT_SHIFT;
  candidate.test_command_ordinal =
      ((candidate.profile.flags & RIPPER_IR_CATALOG_TEST_ORDINAL_MASK) >>
       RIPPER_IR_CATALOG_TEST_ORDINAL_SHIFT) - 1;
  candidate.test_is_power =
      (candidate.profile.flags & RIPPER_IR_CATALOG_FLAG_HAS_POWER_TEST) != 0;
  closeIndex();
  return true;
}

bool RipperIrCatalogReader::findCandidateBatch(
    const char *category, const char *manufacturer, const char *model,
    const RipperIrCatalogCandidateRef *after,
    RipperIrCatalogCandidateRef *batch, uint8_t capacity, uint8_t &count) {
  count = 0;
  if (!ready_ || category == nullptr || category[0] == '\0' || batch == nullptr ||
      capacity == 0 || capacity > RIPPER_IR_CATALOG_DISCOVERY_BATCH_SIZE || !openIndex()) {
    setError("CATALOG BATCH REQUEST ERROR"); return false;
  }

  // The index is sorted by category, manufacturer, then source path. Restrict
  // candidate ranking to the requested category instead of walking every
  // profile and performing thousands of random string-table reads.
  auto compareCategory = [&](uint32_t ordinal, int &comparison) -> bool {
    RipperIrCatalogRecordV1 probe = {};
    char name[24] = {};
    if (!readRecord(ordinal, probe) ||
        !readString(probe.category_offset, name, sizeof(name))) return false;
    comparison = strcasecmp(name, category);
    return true;
  };
  uint32_t low = 0;
  uint32_t high = header_.record_count;
  while (low < high) {
    const uint32_t middle = low + (high - low) / 2;
    int comparison = 0;
    if (!compareCategory(middle, comparison)) {
      setError("CATALOG BATCH SEEK ERROR"); closeIndex(); return false;
    }
    if (comparison < 0) low = middle + 1;
    else high = middle;
  }
  const uint32_t category_begin = low;
  if (category_begin >= header_.record_count) { closeIndex(); return true; }
  int first_comparison = 0;
  if (!compareCategory(category_begin, first_comparison) || first_comparison != 0) {
    closeIndex(); return first_comparison != 0;
  }
  high = header_.record_count;
  while (low < high) {
    const uint32_t middle = low + (high - low) / 2;
    int comparison = 0;
    if (!compareCategory(middle, comparison)) {
      setError("CATALOG BATCH SEEK ERROR"); closeIndex(); return false;
    }
    if (comparison <= 0) low = middle + 1;
    else high = middle;
  }
  const uint32_t category_end = low;

  RipperIrCatalogRecordV1 record = {};
  uint32_t cached_manufacturer_offset = UINT32_MAX;
  bool cached_manufacturer_matches = false;
  bool selected_manufacturer_seen = false;
  for (uint32_t ordinal = category_begin; ordinal < category_end; ordinal++) {
    if (!readRecord(ordinal, record)) {
      setError("CATALOG BATCH READ ERROR"); closeIndex(); return false;
    }
    if ((record.flags & (RIPPER_IR_CATALOG_FLAG_EXACT_DUPLICATE |
                         RIPPER_IR_CATALOG_FLAG_INCOMPLETE_METADATA)) != 0) continue;
    if (manufacturer != nullptr && manufacturer[0] != '\0') {
      if (record.manufacturer_offset != cached_manufacturer_offset) {
        char record_manufacturer[48] = {};
        if (!readString(record.manufacturer_offset, record_manufacturer,
                        sizeof(record_manufacturer))) {
          setError("CATALOG BATCH MAKER ERROR"); closeIndex(); return false;
        }
        cached_manufacturer_offset = record.manufacturer_offset;
        cached_manufacturer_matches =
          catalog_name_equivalent(record_manufacturer, manufacturer);
        if (cached_manufacturer_matches) selected_manufacturer_seen = true;
        else if (selected_manufacturer_seen) break;
      }
      if (!cached_manufacturer_matches) continue;
    }
    if (model != nullptr && model[0] != '\0' &&
        !recordStringEquals(record.model_offset, model)) continue;
    const uint16_t usable = (record.flags & RIPPER_IR_CATALOG_USABLE_COUNT_MASK) >>
                            RIPPER_IR_CATALOG_USABLE_COUNT_SHIFT;
    const uint16_t test_plus_one = (record.flags & RIPPER_IR_CATALOG_TEST_ORDINAL_MASK) >>
                                   RIPPER_IR_CATALOG_TEST_ORDINAL_SHIFT;
    if (usable == 0 || test_plus_one == 0) continue;
    const uint32_t score = catalog_candidate_score(record, usable);
    if (after != nullptr &&
        (score > after->score ||
         (score == after->score && ordinal <= after->profile_ordinal))) continue;

    RipperIrCatalogCandidateRef candidate = {
      ordinal,
      score,
      usable,
      (uint16_t)(test_plus_one - 1),
      (record.flags & RIPPER_IR_CATALOG_FLAG_HAS_POWER_TEST) != 0,
    };
    uint8_t position = 0;
    while (position < count &&
           (batch[position].score > score ||
            (batch[position].score == score &&
             batch[position].profile_ordinal < ordinal))) position++;
    if (position >= capacity) continue;
    if (count < capacity) count++;
    for (uint8_t move = count - 1; move > position; move--) batch[move] = batch[move - 1];
    batch[position] = candidate;
  }
  closeIndex();
  return true;
}

uint32_t RipperIrCatalogReader::generationUnsupportedCount() {
  if (!ready_ || !openIndex()) { setError("CATALOG INDEX OPEN ERROR"); return 0; }
  uint32_t total = 0;
  RipperIrCatalogRecordV1 record = {};
  for (uint32_t i = 0; i < header_.record_count; i++) {
    if (!readRecord(i, record)) { closeIndex(); return total; }
    total += record.unsupported_count;
  }
  closeIndex();
  return total;
}

bool RipperIrCatalogReader::parseProfile(uint32_t ordinal,
                                         RipperIrCatalogCommandVisitor &visitor,
                                         uint16_t *parsed_commands,
                                         uint16_t *raw_commands,
                                         uint16_t *unsupported_commands) {
  RipperIrCatalogProfile profile = {};
  if (!ready_ || !openIndex()) { setError("CATALOG INDEX OPEN ERROR"); return false; }
  const bool metadata_ok = readProfileOpen(ordinal, profile);
  closeIndex();
  if (!metadata_ok || !openArchive() || !archive_.seek(profile.tar_data_offset)) {
    closeArchive(); return false;
  }
  uint32_t remaining = profile.tar_data_bytes;
  uint32_t crc = 0xFFFFFFFFUL;
  uint16_t parsed = 0, raw = 0, unsupported = 0;
  RipperIrCatalogCommand command = {};
  char type[12] = {};
  char line[192] = {};
  size_t line_length = 0;
  bool raw_data_line = false;
  char number[12] = {};
  size_t number_length = 0;
  uint32_t raw_count = 0;
  bool raw_started = false;
  bool visitor_ok = true;

  auto finish_record = [&]() {
    if (type[0] == '\0') return;
    if (strcmp(type, "parsed") == 0) {
      command.parsed = true;
      command.protocol_supported = catalog_protocol_supported(command.protocol);
      if (!command.protocol_supported) unsupported++;
      if (visitor_ok) visitor_ok = visitor.onParsedCommand(command);
      parsed++;
    } else if (strcmp(type, "raw") == 0) {
      if (!raw_started && visitor_ok) visitor_ok = visitor.onRawCommandBegin(command);
      if (visitor_ok) visitor_ok = visitor.onRawCommandEnd(raw_count);
      raw++;
    }
    memset(&command, 0, sizeof(command));
    memset(type, 0, sizeof(type));
    raw_started = false;
    raw_count = 0;
  };

  auto finish_line = [&]() {
    while (line_length > 0 && (line[line_length - 1] == ' ' || line[line_length - 1] == '\t')) {
      line_length--;
    }
    line[line_length] = '\0';
    if (strcmp(line, "#") == 0) {
      finish_record();
    } else {
      char *separator = strchr(line, ':');
      if (separator != nullptr) {
        *separator = '\0';
        char *key_end = separator;
        while (key_end > line && (key_end[-1] == ' ' || key_end[-1] == '\t')) *--key_end = '\0';
        char *value = separator + 1;
        while (*value == ' ') value++;
        if (strcmp(line, "name") == 0) strlcpy(command.name, value, sizeof(command.name));
        else if (strcmp(line, "type") == 0) strlcpy(type, value, sizeof(type));
        else if (strcmp(line, "protocol") == 0) strlcpy(command.protocol, value, sizeof(command.protocol));
        else if (strcmp(line, "address") == 0) command.address = parse_hex_bytes(value);
        else if (strcmp(line, "command") == 0) command.command = parse_hex_bytes(value);
        else if (strcmp(line, "frequency") == 0) command.frequency = strtoul(value, nullptr, 10);
        else if (strcmp(line, "duty_cycle") == 0) command.duty_cycle = strtof(value, nullptr);
      }
    }
    line_length = 0;
  };

  while (remaining-- > 0 && visitor_ok) {
    const int input = archive_.read();
    if (input < 0) {
      setError("CATALOG PROFILE SHORT READ"); closeArchive(); return false;
    }
    const uint8_t byte = (uint8_t)input;
    crc = catalog_crc32_update(crc, &byte, 1);
    if (raw_data_line) {
      if (byte == '\n' || byte == '\r') {
        if (number_length > 0) {
          number[number_length] = '\0';
          if (visitor_ok) visitor_ok = visitor.onRawTiming(strtoul(number, nullptr, 10));
          raw_count++;
          number_length = 0;
        }
        raw_data_line = false;
      } else if (byte == ' ' || byte == '\t') {
        if (number_length > 0) {
          number[number_length] = '\0';
          if (visitor_ok) visitor_ok = visitor.onRawTiming(strtoul(number, nullptr, 10));
          raw_count++;
          number_length = 0;
        }
      } else if (isdigit(byte)) {
        if (number_length + 1 >= sizeof(number)) {
          setError("CATALOG RAW TIMING INVALID"); closeArchive(); return false;
        }
        number[number_length++] = (char)byte;
      } else { setError("CATALOG RAW DATA INVALID"); closeArchive(); return false; }
      continue;
    }
    if (byte == '\n' || byte == '\r') {
      if (line_length > 0) finish_line();
      continue;
    }
    if (line_length + 1 >= sizeof(line)) {
      setError("CATALOG PROFILE LINE TOO LONG"); closeArchive(); return false;
    }
    line[line_length++] = (char)byte;
    if (line_length == 6 && memcmp(line, "data: ", 6) == 0 && strcmp(type, "raw") == 0) {
      line_length = 0;
      raw_data_line = true;
      raw_started = true;
      if (visitor_ok) visitor_ok = visitor.onRawCommandBegin(command);
    }
  }
  if (raw_data_line && number_length > 0) {
    number[number_length] = '\0';
    if (visitor_ok) visitor_ok = visitor.onRawTiming(strtoul(number, nullptr, 10));
    raw_count++;
  }
  if (line_length > 0) finish_line();
  finish_record();
  if (!visitor_ok) { setError("CATALOG VISITOR STOPPED"); closeArchive(); return false; }
  if ((~crc) != profile.profile_crc32 || parsed != profile.parsed_count ||
      raw != profile.raw_count || unsupported != profile.unsupported_count) {
    setError("CATALOG PROFILE INTEGRITY ERROR"); closeArchive(); return false;
  }
  if (parsed_commands) *parsed_commands = parsed;
  if (raw_commands) *raw_commands = raw;
  if (unsupported_commands) *unsupported_commands = unsupported;
  closeArchive();
  return true;
}
