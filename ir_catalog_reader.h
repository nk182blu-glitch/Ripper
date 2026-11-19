#pragma once

#include <Arduino.h>
#include <FS.h>

// Read-only, bounded-memory view of the IR catalog. The index contains byte
// offsets into one uncompressed TAR archive, so no catalog profile is extracted
// and only the selected profile is read.

#define RIPPER_IR_CATALOG_INDEX_MAGIC 0x31494352UL  // "RCI1"
#define RIPPER_IR_CATALOG_INDEX_VERSION 1
#define RIPPER_IR_CATALOG_FLAG_EXACT_DUPLICATE 0x00000001UL
#define RIPPER_IR_CATALOG_FLAG_INCOMPLETE_METADATA 0x00000002UL
#define RIPPER_IR_CATALOG_FLAG_HAS_POWER_TEST 0x00000004UL
#define RIPPER_IR_CATALOG_TEST_ORDINAL_SHIFT 8
#define RIPPER_IR_CATALOG_TEST_ORDINAL_MASK 0x000FFF00UL
#define RIPPER_IR_CATALOG_USABLE_COUNT_SHIFT 20
#define RIPPER_IR_CATALOG_USABLE_COUNT_MASK 0xFFF00000UL

#pragma pack(push, 1)
struct RipperIrCatalogIndexHeaderV1 {
  uint32_t magic;
  uint16_t version;
  uint16_t header_size;
  uint16_t record_size;
  uint16_t reserved;
  uint32_t record_count;
  uint32_t records_offset;
  uint32_t strings_offset;
  uint32_t strings_bytes;
  uint32_t records_crc32;
  uint32_t strings_crc32;
  uint64_t archive_bytes;
  uint32_t archive_crc32;
  uint32_t header_crc32;
};

struct RipperIrCatalogRecordV1 {
  uint32_t category_offset;
  uint32_t manufacturer_offset;
  uint32_t model_offset;
  uint32_t source_path_offset;
  uint64_t tar_data_offset;
  uint32_t tar_data_bytes;
  uint32_t profile_crc32;
  uint16_t command_count;
  uint16_t parsed_count;
  uint16_t raw_count;
  uint16_t unsupported_count;
  uint32_t flags;
};
#pragma pack(pop)

struct RipperIrCatalogProfile {
  uint32_t ordinal;
  char category[24];
  char manufacturer[48];
  char model[72];
  char source_path[192];
  uint64_t tar_data_offset;
  uint32_t tar_data_bytes;
  uint32_t profile_crc32;
  uint16_t command_count;
  uint16_t parsed_count;
  uint16_t raw_count;
  uint16_t unsupported_count;
  uint32_t flags;
};

struct RipperIrCatalogCandidate {
  RipperIrCatalogProfile profile;
  uint32_t score;
  uint16_t usable_command_count;
  uint16_t test_command_ordinal;
  bool test_is_power;
};

// Compact references let discovery retain a ten-profile test batch without
// loading ten profile names, commands, or raw timing arrays into RAM.
struct RipperIrCatalogCandidateRef {
  uint32_t profile_ordinal;
  uint32_t score;
  uint16_t usable_command_count;
  uint16_t test_command_ordinal;
  bool test_is_power;
};

#define RIPPER_IR_CATALOG_DISCOVERY_BATCH_SIZE 10

class RipperIrCatalogNameVisitor {
 public:
  virtual ~RipperIrCatalogNameVisitor() {}
  virtual bool onName(const char *name, uint32_t matching_profiles) = 0;
};

struct RipperIrCatalogCommand {
  char name[40];
  char protocol[20];
  bool parsed;
  bool protocol_supported;
  uint64_t address;
  uint64_t command;
  uint32_t frequency;
  float duty_cycle;
  uint32_t raw_timing_count;
};

class RipperIrCatalogCommandVisitor {
 public:
  virtual ~RipperIrCatalogCommandVisitor() {}
  virtual bool onParsedCommand(const RipperIrCatalogCommand &command) = 0;
  virtual bool onRawCommandBegin(const RipperIrCatalogCommand &command) = 0;
  virtual bool onRawTiming(uint32_t timing_us) = 0;
  virtual bool onRawCommandEnd(uint32_t timing_count) = 0;
};

class RipperIrCatalogReader {
 public:
  RipperIrCatalogReader();
  bool begin(fs::FS &storage, const char *index_path, const char *archive_path,
             bool verify_archive_size = true);
  void end();
  bool ready() const { return ready_; }
  const char *lastError() const { return error_; }
  uint32_t profileCount() const { return header_.record_count; }
  uint32_t generationUnsupportedCount();
  bool readProfile(uint32_t ordinal, RipperIrCatalogProfile &profile);
  bool visitCategories(RipperIrCatalogNameVisitor &visitor);
  bool visitManufacturers(const char *category, RipperIrCatalogNameVisitor &visitor);
  bool visitModels(const char *category, const char *manufacturer,
                   RipperIrCatalogNameVisitor &visitor);
  bool findNextCandidate(const char *category, const char *manufacturer,
                         const char *model, const RipperIrCatalogCandidate *previous,
                         RipperIrCatalogCandidate &candidate);
  bool findCandidateBatch(const char *category, const char *manufacturer,
                          const char *model,
                          const RipperIrCatalogCandidateRef *after,
                          RipperIrCatalogCandidateRef *batch, uint8_t capacity,
                          uint8_t &count);
  bool parseProfile(uint32_t ordinal, RipperIrCatalogCommandVisitor &visitor,
                    uint16_t *parsed_commands = nullptr,
                    uint16_t *raw_commands = nullptr,
                    uint16_t *unsupported_commands = nullptr);

 private:
  fs::FS *storage_;
  File index_;
  File archive_;
  RipperIrCatalogIndexHeaderV1 header_;
  bool ready_;
  char error_[64];
  char index_path_[96];
  char archive_path_[112];

  void setError(const char *message);
  bool openIndex();
  bool openArchive();
  void closeIndex();
  void closeArchive();
  bool readRecord(uint32_t ordinal, RipperIrCatalogRecordV1 &record);
  bool readString(uint32_t offset, char *target, size_t target_size);
  bool recordStringEquals(uint32_t offset, const char *wanted);
  bool readProfileOpen(uint32_t ordinal, RipperIrCatalogProfile &profile);
  bool verifyRegionCrc(uint32_t offset, uint32_t bytes, uint32_t expected);
};
