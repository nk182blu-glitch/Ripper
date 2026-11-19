# RipperZero Version History

## Current release

- Firmware: `2.1.3`
- Backup sequence: `89`
- Phone interface QA repair: 2026-11-14
- Source: `RIPPERV2.ino`

## Release history

- `2.1.3` / backup `89`: repaired narrow-phone overflow, remote-opening and
  tab-scroll navigation, visible offline/error states, touch-target sizing,
  received-signal duty display, and safe upload/busy feedback.
- `2.1.2` / backup `88`: simplified the generated local phone/AP password to
  the WPA-minimum eight-character `ripperNN` format and automatically migrates
  Ripper's previous generated password without overwriting future custom ones.
- `2.1.1` / backup `87`: delta-region screen mirroring, faster refresh,
  phone press/hold/drag/release input, automatic reconnection, and home-screen
  web-app metadata.
- `2.1` / backup `86`: authenticated local phone UI, live Ripper screen
  mirroring and touch, Saved Remotes control, received-signal details,
  transactional SD import transfer, and safe display settings.
- `2.0` / backup `85`: version display and backup-count baseline.

## Version rules

- A major feature or architecture checkpoint increments the middle number:
  `2.0` -> `2.1` -> `2.2`.
- A smaller feature, UI adjustment, or bug fix increments the patch number:
  `2.1` -> `2.1.1` -> `2.1.2`.
- Every verified pre-change backup increments the backup sequence, whether or
  not the firmware version changes.
- Compiling, flashing, or rebuilding unchanged source does not change the
  firmware version.
- The version and backup count displayed by the device are defined once near
  the top of `RIPPERV2.ino` as `RIPPER_VERSION` and `RIPPER_BACKUP_COUNT`.

## Backup baseline

- Existing project backups before this checkpoint: 82
- Existing separate RIPPERV2 backups before this checkpoint: 2
- Version-display pre-change backup: 85
- Mobile-control pre-change backup: 86
- Mobile-interaction pre-change backup: 87
- Backup path:
  `/Users/macbook/Documents/Arduino/RIPPERV2_backups/RIPPERV2_v2.0_backup_085_before_version_display_20261111`
- Mobile backup path:
  `/Users/macbook/Documents/Arduino/RIPPERV2_backups/RIPPERV2_v2.0_backup_086_before_mobile_web_20261111`
- Mobile interaction backup path:
  `/Users/macbook/Documents/Arduino/RIPPERV2_backups/RIPPERV2_v2.1_backup_087_before_mobile_interaction_20261111`
- Simple phone password backup path:
  `/Users/macbook/Documents/Arduino/RIPPERV2_backups/RIPPERV2_v2.1.1_backup_088_before_simple_phone_password_20261111`
- Phone interface QA backup path:
  `/Users/macbook/Documents/Arduino/RIPPERV2_backups/RIPPERV2_v2.1.2_backup_089_before_phone_interface_qa_fixes_20261114`
