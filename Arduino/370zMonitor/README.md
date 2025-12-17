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
