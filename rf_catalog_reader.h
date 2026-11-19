#pragma once

#include <Arduino.h>
#include <FS.h>

// Read-only, bounded-memory access to indexed Flipper .sub collections. The
// catalog stays separate from Ripper's writable saved-remote repository.

#define RIPPER_RF_CATALOG_INDEX_MAGIC 0x31464352UL  // "RCF1"
#define RIPPER_RF_CATALOG_INDEX_VERSION 1

#define RIPPER_RF_CATALOG_ENCODING_UNKNOWN 0
#define RIPPER_RF_CATALOG_ENCODING_RAW 1
#define RIPPER_RF_CATALOG_ENCODING_BIN_RAW 2
#define RIPPER_RF_CATALOG_ENCODING_PARSED 3

#define RIPPER_RF_CATALOG_SIGNAL_PARSEABLE 0x00000001UL
#define RIPPER_RF_CATALOG_SIGNAL_IMPORTABLE_NOW 0x00000002UL
#define RIPPER_RF_CATALOG_SIGNAL_RAW_TOO_LARGE 0x00000004UL
#define RIPPER_RF_CATALOG_SIGNAL_INCOMPLETE 0x00000008UL

#define RIPPER_RF_CATALOG_PROFILE_EXACT_DUPLICATE 0x00000001UL
#define RIPPER_RF_CATALOG_PROFILE_INCOMPLETE_METADATA 0x00000002UL
#define RIPPER_RF_CATALOG_PROFILE_MULTIBUTTON 0x00000004UL

#pragma pack(push, 1)
struct RipperRfCatalogIndexHeaderV1 {
  uint32_t magic;
  uint16_t version;
  uint16_t header_size;
  uint16_t profile_record_size;
  uint16_t signal_record_size;
  uint32_t profile_count;
  uint32_t signal_count;
  uint32_t profiles_offset;
  uint32_t signals_offset;
  uint32_t strings_offset;
  uint32_t strings_bytes;
  uint32_t profiles_crc32;
  uint32_t signals_crc32;
  uint32_t strings_crc32;
  uint64_t archive_bytes;
  uint32_t archive_crc32;
  uint32_t header_crc32;
};

struct RipperRfCatalogProfileRecordV1 {
  uint32_t category_offset;
  uint32_t manufacturer_offset;
  uint32_t model_offset;
  uint32_t source_path_offset;
  uint32_t first_signal;
  uint16_t signal_count;
  uint16_t importable_count;
  uint16_t raw_count;
  uint16_t bin_raw_count;
  uint16_t parsed_count;
  uint16_t unsupported_count;
  uint32_t flags;
};

struct RipperRfCatalogSignalRecordV1 {
  uint32_t label_offset;
  uint32_t protocol_offset;
  uint32_t preset_offset;
  uint32_t source_path_offset;
  uint64_t tar_data_offset;
  uint32_t tar_data_bytes;
  uint32_t payload_crc32;
  uint32_t frequency_hz;
  uint32_t raw_timing_count;
  uint32_t te_us;
  uint16_t bit_count;
  uint8_t encoding;
  uint8_t reserved;
  uint32_t flags;
};
#pragma pack(pop)

struct RipperRfCatalogProfile {
  uint32_t ordinal;
  char category[32];
  char manufacturer[48];
  char model[72];
  char source_path[192];
  uint32_t first_signal;
  uint16_t signal_count;
  uint16_t importable_count;
  uint16_t raw_count;
  uint16_t bin_raw_count;
  uint16_t parsed_count;
  uint16_t unsupported_count;
  uint32_t flags;
};

struct RipperRfCatalogSignal {
  uint32_t profile_ordinal;
  uint32_t signal_ordinal;
  uint16_t local_ordinal;
  char label[48];
  char protocol[24];
  char preset[48];
  char source_path[192];
  uint64_t tar_data_offset;
  uint32_t tar_data_bytes;
  uint32_t payload_crc32;
  uint32_t frequency_hz;
  uint32_t raw_timing_count;
  uint32_t te_us;
  uint16_t bit_count;
  uint8_t encoding;
  uint32_t flags;
};

struct RipperRfCatalogQuery {
  const char *category;
  const char *const *category_options;
  uint8_t category_option_count;
  const char *manufacturer;
  const char *model_contains;
  uint32_t after_ordinal;
  bool require_complete;
  bool include_exact_duplicates;
};

struct RipperRfCatalogCandidateRef {
  uint32_t profile_ordinal;
  uint16_t signal_count;
  uint16_t test_signal_ordinal;
  uint16_t score;
  uint32_t flags;
  char category[32];
  char manufacturer[48];
  char model[72];
  char test_label[48];
};

class RipperRfCatalogSignalVisitor {
 public:
  virtual ~RipperRfCatalogSignalVisitor() {}
  virtual bool onSignalBegin(const RipperRfCatalogSignal &signal) {
    (void)signal;
    return true;
  }
  // RAW timings retain the Flipper sign: positive is carrier/high and
  // negative is space/low. No array proportional to the signal is allocated.
  virtual bool onRawTiming(int32_t signed_duration_us) {
    (void)signed_duration_us;
    return true;
  }
  virtual bool onBinRawByte(uint8_t value) {
    (void)value;
    return true;
  }
  virtual bool onParsedKeyByte(uint8_t value) {
    (void)value;
    return true;
  }
  virtual bool onSignalEnd(uint32_t emitted_values) {
    (void)emitted_values;
    return true;
  }
};

class RipperRfCatalogReader {
 public:
  RipperRfCatalogReader();
  bool begin(fs::FS &storage, const char *index_path, const char *archive_path,
             bool verify_archive_size = true);
  void end();
  bool ready() const { return ready_; }
  const char *lastError() const { return error_; }
  uint32_t profileCount() const { return header_.profile_count; }
  uint32_t signalCount() const { return header_.signal_count; }
  uint32_t stringCacheBytes() const { return strings_cache_bytes_; }
  bool readProfile(uint32_t ordinal, RipperRfCatalogProfile &profile);
  bool readSignal(uint32_t profile_ordinal, uint16_t local_ordinal,
                  RipperRfCatalogSignal &signal);
  bool parseSignal(uint32_t profile_ordinal, uint16_t local_ordinal,
                   RipperRfCatalogSignalVisitor &visitor);
  // Scans the compact index with one file handle and retains only the caller's
  // bounded result page. Catalog payloads remain closed and no catalog-sized
  // array is allocated.
  bool findProfileBatch(const RipperRfCatalogQuery &query,
                        RipperRfCatalogCandidateRef *results,
                        size_t capacity, size_t &result_count,
                        bool &has_more);

 private:
  fs::FS *storage_;
  File index_;
  File archive_;
  RipperRfCatalogIndexHeaderV1 header_;
  bool ready_;
  char error_[64];
  char index_path_[112];
  char archive_path_[112];
  uint8_t *strings_cache_;
  uint32_t strings_cache_bytes_;

  void setError(const char *message);
  void clearStringCache();
  bool loadStringCache();
  bool openIndex();
  bool openArchive();
  void closeIndex();
  void closeArchive();
  bool verifyIndexRegion(uint32_t offset, uint32_t bytes, uint32_t expected_crc);
  bool readProfileRecord(uint32_t ordinal, RipperRfCatalogProfileRecordV1 &record);
  bool readSignalRecord(uint32_t ordinal, RipperRfCatalogSignalRecordV1 &record);
  bool readString(uint32_t offset, char *target, size_t target_size);
  bool readProfileOpen(uint32_t ordinal, RipperRfCatalogProfile &profile);
  bool readSignalOpen(uint32_t profile_ordinal, uint16_t local_ordinal,
                      RipperRfCatalogSignal &signal);
  bool selectTestSignalOpen(const RipperRfCatalogProfile &profile,
                            uint16_t &local_ordinal, char *label,
                            size_t label_size, uint16_t &score);
};
