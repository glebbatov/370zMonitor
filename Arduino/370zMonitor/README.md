# 370zMonitor - Setup Guide
**Board:** Waveshare ESP32-S3-Touch-LCD-7 (800×480)  
**SquareLine Project:** GaugeAndChartDesign  
**LVGL Version:** 9.1.0

## ✅ Setup Complete

The sketch has been configured for the Waveshare ESP32-S3 7" (800×480) board with the following:

### Hardware Configuration
- **Display:** 800×480 RGB666 panel
- **Touch:** GT911 capacitive touch controller
- **IO Expander:** CH422G (I2C address 0x24)
  - Controls backlight ON/OFF (no PWM brightness)
  - Controls LCD reset
  - Controls touch reset
- **I2C:** SDA=GPIO8, SCL=GPIO9
- **Touch INT:** GPIO4

### Key Differences from 7B Model
| Feature | 7" (800×480) | 7B (1024×600) |
|---------|--------------|---------------|
| Resolution | 800×480 | 1024×600 |
| IO Expander | CH422G | CH32V003 |
| Backlight | Digital ON/OFF | PWM (0-255) |
| Touch INT Pin | GPIO4 | GPIO18 |
| Pixel Clock | 14MHz | 16MHz |

### RGB Pin Mapping (800×480 specific)
```
DE (Data Enable): GPIO5
VSYNC: GPIO3
HSYNC: GPIO46
PCLK: GPIO7

Red:   R3=GPIO1,  R4=GPIO2,  R5=GPIO42, R6=GPIO41, R7=GPIO40
Green: G2=GPIO39, G3=GPIO0,  G4=GPIO45, G5=GPIO48, G6=GPIO47, G7=GPIO21
Blue:  B3=GPIO14, B4=GPIO38, B5=GPIO18, B6=GPIO17, B7=GPIO10
```

## Required Libraries

Make sure you have these installed via Arduino Library Manager:

1. **Arduino_GFX** (by moononournation)
2. **lvgl** (version 9.x)
3. **TAMC_GT911** (GT911 touch driver)
4. **SD** (built-in ESP32 SD library)
5. **SPI** (built-in)

## Upload Instructions

1. **Select Board:** ESP32S3 Dev Module
2. **USB CDC On Boot:** Enabled
3. **PSRAM:** OPI PSRAM
4. **Flash Mode:** QIO 80MHz
5. **Flash Size:** 16MB (or your board's size)
6. **Partition Scheme:** 16M Flash (3MB APP/9.9MB FATFS)
7. **Upload Speed:** 921600
8. **USB Mode:** Hardware CDC and JTAG

## Expected Serial Output

```
=== Waveshare ESP32-S3 7" 800x480 ===
SquareLine Studio UI: GaugeAndChartDesign
Initializing CH422G IO Expander...
Initializing Touch Controller...
Initializing RGB Display...
Turning on Backlight...
Initializing GT911 Touch...
Initializing LVGL...
Loading UI...

=== READY ===
✅ Display: Working
✅ Touch: Ready
✅ Backlight: ON
✅ UI: Loaded
```

## Troubleshooting

### Black Screen
- Check Serial output for errors
- Verify PSRAM is enabled in board settings
- Ensure backlight initialization succeeded
- Try increasing the delay after `setBacklight(true)`

### Touch Not Working
- Verify GT911 I2C address (should auto-detect 0x5D or 0x14)
- Check touch INT pin (GPIO4)
- Ensure touch reset sequence completed
- Touch the screen - Serial should show touch coordinates

### Display Artifacts
- Lower pixel clock if unstable: change `14000000` to `12000000`
- Check cable connections
- Verify RGB pin mappings match your board

### Compilation Errors
- Ensure lv_conf.h exists in `C:\Users\Gleb\Documents\Arduino\libraries\`
- Check LVGL version is 9.x
- Verify all required libraries are installed

## Notes

### Backlight Control
This board uses **digital ON/OFF control only** - no PWM brightness adjustment like the 7B model. If you need brightness control, you would need to implement software PWM or use a different approach.

### SquareLine Studio Export
The UI files (ui.c, ui.h, ui_Screen1.c, etc.) are already in your project folder and will be included automatically. Any changes made in SquareLine Studio should be re-exported to this folder.

### LVGL Configuration
The lv_conf.h file is already configured for:
- 128KB memory allocation (PSRAM)
- RGB565 color format
- Required fonts (Montserrat 14, 20, 26, 48)
- All necessary widgets enabled
- 60 FPS target (16ms refresh)

## SD Card Data Logging

The monitor includes SD card data logging functionality for recording all sensor data during track sessions.

### Features
- **Session-Based Files:** Creates `SESS_NNNNN.csv` files based on boot counter
- **Boot Counter Persistence:** Stored in `/.bootcount` on SD card
- **Configurable Write Rate:** Default 1Hz (1 sample/second), adjustable via `SD_WRITE_INTERVAL_MS`
- **Corruption Prevention:**
  - Immediate flush after every write
  - File opened in append mode
  - No buffering (direct writes)
  - Write retry on failure (3 attempts)
- **Automatic Space Management:**
  - Keeps at least 5% free space (configurable via `SD_FREE_SPACE_PERCENT`)
  - Automatically deletes oldest session files when space is low
  - Space check every 60 writes (~1 minute at 1Hz)
- **RTC Detection:** Auto-detects DS3231 RTC module for real timestamps (optional)

### CSV Data Format
Each log file contains:
```
timestamp_ms,elapsed_s,cpu_percent,mode,oil_press_psi,oil_press_valid,
oil_temp_pan_f,oil_temp_cooled_f,oil_temp_valid,water_temp_hot_f,water_temp_cooled_f,
water_temp_valid,trans_temp_hot_f,trans_temp_cooled_f,trans_temp_valid,
steer_temp_hot_f,steer_temp_cooled_f,steer_temp_valid,diff_temp_hot_f,diff_temp_cooled_f,
diff_temp_valid,fuel_trust_percent,fuel_trust_valid,rpm,rpm_valid
```

### File Naming
- **Without RTC:** Files are named `SESS_00001.csv`, `SESS_00002.csv`, etc. based on boot counter
- **With DS3231 RTC:** (Future) Files will be named `YYYY-MM-DD_HH-MM-SS.csv`

### SD Card Requirements
- **Format:** FAT32 (recommended) or FAT16
- **Size:** Any size supported, minimum 1MB free space required
- **Speed:** Class 10 or faster recommended for reliable writes

### SD Card Pins (Waveshare ESP32-S3 7")
| Function | Pin |
|----------|-----|
| SCK | GPIO 12 |
| MISO | GPIO 13 |
| MOSI | GPIO 11 |
| CS | IO Expander bit 4 (EXIO_SD_CS) |

**WARNING:** GPIO10 is used for display Blue channel - do NOT use for SD!

### Utility Box Display
The utility box shows SD card status:
- `SD:NONE` - No SD card detected
- `SD:READY` - Ready but not logging
- `SD:###K` or `SD:###M` - Actively logging (shows bytes written)
- `SD:PAUSE` - Logging paused
- `SD:E#` - Write errors occurred

### Adding Real Timestamps (DS3231 RTC)
To get real date/time on your log files:
1. Connect a DS3231 RTC module to I2C (SDA=GPIO8, SCL=GPIO9)
2. The module shares I2C with the touch controller and IO expander
3. Set the time once (code to be added)
4. Files will automatically use real timestamps

### Disabling SD Logging
Set `#define ENABLE_SD_LOGGING 0` in the code to disable SD card functionality.

---

## USB Mass Storage Mode

The ESP32-S3's native USB allows the SD card to appear as a USB flash drive when connected to a computer. This provides convenient access to log files without removing the SD card.

### How to Enter USB MSC Mode
1. **Connect USB-C** cable to power the board
2. **Wait** for the normal monitor UI to appear on display
3. **Press RESET button**, then **immediately hold BOOT button**
4. Keep holding BOOT until the green "USB MASS STORAGE MODE" screen appears
5. The SD card will appear as a removable drive (E:, F:, etc.) on your computer

**Note:** First access may take 15-20 seconds while Windows scans the filesystem.

### What You'll See
- **Green screen** with "USB MASS STORAGE MODE" text
- **Card info:** Type (SDHC) and size displayed
- **"USB Ready!"** message when ready for computer connection
- **Red blinking dot** in bottom-right corner (heartbeat indicator)

### Exiting USB MSC Mode
- **Power cycle** the board (unplug and replug USB-C)
- Normal monitor mode will start automatically
- There is no software exit - power cycle is required

### Important Notes
- **Exclusive mode:** Serial debugging is NOT available in USB MSC mode
- **Safely eject:** Always "Safely Remove Hardware" before unplugging
- **FAT32 format:** SD card must be FAT32 formatted
- **Full access:** Read/write access to all SD card contents
- **Data integrity:** All session files remain intact after USB access

### Troubleshooting
| Issue | Solution |
|-------|----------|
| Green screen shows "SD card not found" | Ensure SD card is inserted before entering USB mode |
| Drive not appearing on PC | Wait 15-20 seconds; try different USB-C cable (must be data-capable) |
| "Insert disk" or "Format disk" prompt | SD card read failed - check card is FAT32 and healthy |
| Windows Explorer freezes | Normal on first access - wait 15-20 seconds |
| Can't enter USB mode | Make sure to press RESET first, then hold BOOT immediately |
| Screen flickers green/black | Release BOOT button - mode entry failed, try again |

### Technical Details
- **BOOT button:** GPIO0 (directly on ESP32-S3, active LOW)
- **USB interface:** Native USB OTG on ESP32-S3
- **Sector access:** Uses ESP32 SD library's `sd_read_raw()` / `sd_write_raw()`
- **Drive label:** "370zMon" with product ID "SD Card"

---

## DS3231 RTC Module (Optional)

For real timestamps on log files (instead of boot-counter based names), add a DS3231 RTC module.

### Connection
| DS3231 Pin | ESP32-S3 Pin |
|------------|-------------|
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO 8 |
| SCL | GPIO 9 |

### Features
- Battery-backed real-time clock
- Automatic detection at boot
- Log files named: `YYYY-MM-DD_HH-MM-SS.csv`
- Temperature-compensated crystal (±2ppm accuracy)

### Cost & Overhead
- Module cost: ~$2
- Power: ~1.5mA (from coin cell, not ESP32)
- CPU overhead: Negligible (single I2C read at boot)

---

## CAN Bus Integration (Future)

Since this is a 370z monitor project, you'll likely want to add CAN bus functionality. Here's how to integrate it:

1. Add CAN library (ESP32-TWAI-CAN or similar)
2. Initialize CAN in `setup()` after display initialization
3. Read CAN messages in `loop()` or create a separate task
4. Update LVGL UI elements with CAN data
5. Use `lv_label_set_text()`, `lv_arc_set_value()`, etc. to update widgets

Example snippet:
```cpp
void updateGauges() {
    // Read CAN data
    int rpm = readRPMFromCAN();
    int speed = readSpeedFromCAN();
    
    // Update LVGL widgets (use appropriate widget from your UI)
    lv_label_set_text_fmt(ui_RPMLabel, "%d", rpm);
    lv_arc_set_value(ui_SpeedGauge, speed);
}
```

---

**Status:** ✅ Ready to upload and test!
