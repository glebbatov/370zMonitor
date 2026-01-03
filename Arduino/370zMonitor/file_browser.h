//-----------------------------------------------------------------
/*
 * 370zMonitor - File Browser Module v2.0
 * 
 * OPTIMIZED: Uses boot_count to generate recent session filenames
 * instead of scanning the entire directory. This is O(25) instead of O(n)
 * and works instantly even with millions of files.
 * 
 * Entry: Tap "FILES" button in utility box
 * Exit: Navigate back from root directory
 */
//-----------------------------------------------------------------

#ifndef FILE_BROWSER_H
#define FILE_BROWSER_H

#include <Arduino.h>
#include <lvgl.h>
#include <SD.h>
#include <vector>

// External references (defined in 370zMonitor.ino)
extern volatile bool g_fb_pause_sd_writes;
extern uint32_t g_current_boot_count;  // Current boot count for file generation

//=================================================================
// CONFIGURATION
//=================================================================

#define FB_MAX_PATH_LEN             128
#define FB_MAX_FILENAME_LEN         64
#define FB_MAX_FILES_TO_DISPLAY     25      // Max files to show
#define FB_TEXT_CHUNK_SIZE          4096
#define FB_TEXT_MAX_SIZE            (100 * 1024)
#define FB_LIST_ITEM_HEIGHT         40      // Taller for easier touch

// CSV table viewer limits
#define FB_CSV_MAX_ROWS             1000    // Max rows to display
#define FB_CSV_MAX_COLS             30      // Max columns to display
#define FB_CSV_MAX_CELL_LEN         48      // Max chars per cell (truncate longer)
#define FB_CSV_COL_WIDTH            100     // Default column width in pixels

// Session filename format - supports BOTH legacy 5-digit and new 8-digit
#define FB_SESSION_FORMAT_8     "SESS_%08lu"   // New format
#define FB_SESSION_FORMAT_5     "SESS_%05lu"   // Legacy format

//=================================================================
// FILE ENTRY STRUCTURE
//=================================================================

struct FileEntry {
    char name[FB_MAX_FILENAME_LEN];
    uint32_t size;
    time_t modTime;  // Last modified time
    bool isDirectory;
};

//=================================================================
// FILE BROWSER STATE
//=================================================================

enum FileBrowserState {
    FB_STATE_INACTIVE,
    FB_STATE_BROWSING,
    FB_STATE_TEXT_VIEW,
    FB_STATE_CSV_VIEW
};

struct FileBrowserContext {
    FileBrowserState state;
    char currentPath[FB_MAX_PATH_LEN];
    std::vector<FileEntry> files;
    std::vector<FileEntry> cachedFolders;  // Cached folder list (scan once)
    bool foldersCached;                     // True after first scan
    int totalSessions;  // Total sessions found
    
    // LVGL objects
    lv_obj_t* screenBrowser;
    lv_obj_t* screenTextView;
    lv_obj_t* screenCsvView;
    
    lv_obj_t* pathLabel;
    lv_obj_t* fileListContainer;
    lv_obj_t* statusLabel;
    
    lv_obj_t* textArea;
    lv_obj_t* textPathLabel;
    
    lv_obj_t* csvTable;          // Scrollable data (excludes row 0 and col 0)
    lv_obj_t* csvHeaderTable;      // Frozen header row (excludes col 0)
    lv_obj_t* csvColumnTable;      // Frozen first column (excludes row 0)
    lv_obj_t* csvCornerCell;       // Frozen top-left cell
    lv_obj_t* csvPathLabel;
    lv_obj_t* csvInfoLabel;
    
    lv_obj_t* previousScreen;
};

static FileBrowserContext g_fb = { FB_STATE_INACTIVE };

//=================================================================
// FORWARD DECLARATIONS
//=================================================================

void fb_init();
bool fb_isActive();
void fb_enter();
void fb_exit();
void fb_update();  // Called from main loop (currently empty)

static void fb_createScreens();
static void fb_destroyScreens();
static void fb_loadRecentSessions();
static void fb_navigateTo(const char* path);
static void fb_openTextFile(const char* path);
static void fb_openCsvFile(const char* path);
static void fb_goBack();
static void fb_updateFileList();
static const char* fb_formatSize(uint32_t size);
static const char* fb_formatDate(time_t t);
static bool fb_isCsvFile(const char* filename);

static void fb_fileListClickCb(lv_event_t* e);
static void fb_backBtnCb(lv_event_t* e);
static void fb_textBackBtnCb(lv_event_t* e);
static void fb_csvBackBtnCb(lv_event_t* e);
static void fb_csvScrollCb(lv_event_t* e);  // Sync header scroll with data scroll

//=================================================================
// INITIALIZATION
//=================================================================

void fb_init() {
    memset(&g_fb, 0, sizeof(g_fb));
    g_fb.state = FB_STATE_INACTIVE;
    g_fb.foldersCached = false;
    strcpy(g_fb.currentPath, "/");
    Serial.println("[FB] File browser initialized");
}

void fb_update() {
    // File browser is event-driven via LVGL - nothing to poll
}

bool fb_isActive() {
    return g_fb.state != FB_STATE_INACTIVE;
}

//=================================================================
// ENTER/EXIT
//=================================================================

void fb_enter() {
    if (g_fb.state != FB_STATE_INACTIVE) return;
    
    Serial.println("[FB] Entering file browser mode");
    
    // Pause SD logging
    g_fb_pause_sd_writes = true;
    vTaskDelay(pdMS_TO_TICKS(200));
    Serial.println("[FB] SD logging paused");
    
    g_fb.previousScreen = lv_scr_act();
    fb_createScreens();
    lv_scr_load(g_fb.screenBrowser);
    g_fb.state = FB_STATE_BROWSING;
    
    // Show loading indicator
    lv_obj_clean(g_fb.fileListContainer);
    lv_obj_t* loadingLabel = lv_label_create(g_fb.fileListContainer);
    lv_label_set_text(loadingLabel, "Loading files...");
    lv_obj_set_style_text_color(loadingLabel, lv_color_hex(0xFFFF00), 0);  // Yellow
    lv_obj_set_style_text_font(loadingLabel, &lv_font_unscii_16, 0);
    lv_obj_center(loadingLabel);
    
    // Force render so user sees loading screen
    lv_refr_now(NULL);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // Now load files and show
    fb_navigateTo("/");
}

void fb_exit() {
    if (g_fb.state == FB_STATE_INACTIVE) return;
    
    Serial.println("[FB] Exiting file browser mode");
    
    if (g_fb.previousScreen) {
        lv_scr_load(g_fb.previousScreen);
    }
    
    g_fb.state = FB_STATE_INACTIVE;
    g_fb_pause_sd_writes = false;
    Serial.println("[FB] SD logging resumed");
    
    // Clear file list but keep cached folders
    g_fb.files.clear();
    g_fb.files.shrink_to_fit();
    // Note: g_fb.cachedFolders is kept for next time
    
    fb_destroyScreens();
}

//=================================================================
// SCREEN CREATION
//=================================================================

static void fb_createScreens() {
    fb_destroyScreens();
    
    lv_color_t bgColor = lv_color_hex(0x1a1a1a);
    lv_color_t textColor = lv_color_hex(0xffffff);
    lv_color_t accentColor = lv_color_hex(0xff4500);
    
    //-------------------------------------------------------------
    // BROWSER SCREEN
    //-------------------------------------------------------------
    g_fb.screenBrowser = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(g_fb.screenBrowser, bgColor, 0);
    lv_obj_clear_flag(g_fb.screenBrowser, LV_OBJ_FLAG_SCROLLABLE);
    
    // Header bar
    lv_obj_t* headerBar = lv_obj_create(g_fb.screenBrowser);
    lv_obj_set_size(headerBar, 800, 50);
    lv_obj_align(headerBar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(headerBar, lv_color_hex(0x333333), 0);
    lv_obj_set_style_border_width(headerBar, 0, 0);
    lv_obj_set_style_radius(headerBar, 0, 0);
    lv_obj_set_style_pad_all(headerBar, 0, 0);
    lv_obj_clear_flag(headerBar, LV_OBJ_FLAG_SCROLLABLE);
    
    // Back button - centered vertically in header
    lv_obj_t* backBtn = lv_btn_create(headerBar);
    lv_obj_set_size(backBtn, 110, 40);
    lv_obj_align(backBtn, LV_ALIGN_LEFT_MID, 5, 0);
    lv_obj_set_style_bg_color(backBtn, accentColor, 0);
    lv_obj_set_style_radius(backBtn, 0, 0);  // No rounded corners
    lv_obj_add_event_cb(backBtn, fb_backBtnCb, LV_EVENT_CLICKED, NULL);
    
    lv_obj_t* backLabel = lv_label_create(backBtn);
    lv_label_set_text(backLabel, "< BACK");
    lv_obj_set_style_text_font(backLabel, &lv_font_unscii_16, 0);
    lv_obj_center(backLabel);
    
    // Path label
    g_fb.pathLabel = lv_label_create(headerBar);
    lv_label_set_text(g_fb.pathLabel, "root/");
    lv_obj_set_style_text_font(g_fb.pathLabel, &lv_font_unscii_16, 0);
    lv_obj_set_style_text_color(g_fb.pathLabel, textColor, 0);
    lv_obj_align(g_fb.pathLabel, LV_ALIGN_LEFT_MID, 125, 0);
    lv_obj_set_width(g_fb.pathLabel, 660);
    
    // File list container
    g_fb.fileListContainer = lv_obj_create(g_fb.screenBrowser);
    lv_obj_set_size(g_fb.fileListContainer, 800, 400);
    lv_obj_align(g_fb.fileListContainer, LV_ALIGN_TOP_MID, 0, 50);
    lv_obj_set_style_bg_color(g_fb.fileListContainer, bgColor, 0);
    lv_obj_set_style_border_width(g_fb.fileListContainer, 0, 0);
    lv_obj_set_style_radius(g_fb.fileListContainer, 0, 0);  // No rounded corners
    lv_obj_set_style_pad_all(g_fb.fileListContainer, 2, 0);
    lv_obj_set_style_pad_row(g_fb.fileListContainer, 2, 0);
    lv_obj_set_flex_flow(g_fb.fileListContainer, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_scrollbar_mode(g_fb.fileListContainer, LV_SCROLLBAR_MODE_AUTO);
    
    // Status bar
    lv_obj_t* statusBar = lv_obj_create(g_fb.screenBrowser);
    lv_obj_set_size(statusBar, 800, 30);
    lv_obj_align(statusBar, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(statusBar, lv_color_hex(0x333333), 0);
    lv_obj_set_style_border_width(statusBar, 0, 0);
    lv_obj_set_style_radius(statusBar, 0, 0);  // No rounded corners
    lv_obj_set_style_pad_left(statusBar, 10, 0);
    lv_obj_clear_flag(statusBar, LV_OBJ_FLAG_SCROLLABLE);
    
    //-------------------------------------------------------------
    // TEXT VIEWER SCREEN
    //-------------------------------------------------------------
    g_fb.screenTextView = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(g_fb.screenTextView, bgColor, 0);
    lv_obj_clear_flag(g_fb.screenTextView, LV_OBJ_FLAG_SCROLLABLE);
    
    // Text header bar
    lv_obj_t* textHeaderBar = lv_obj_create(g_fb.screenTextView);
    lv_obj_set_size(textHeaderBar, 800, 50);
    lv_obj_align(textHeaderBar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(textHeaderBar, lv_color_hex(0x333333), 0);
    lv_obj_set_style_border_width(textHeaderBar, 0, 0);
    lv_obj_set_style_radius(textHeaderBar, 0, 0);  // No rounded corners
    lv_obj_set_style_pad_all(textHeaderBar, 0, 0);
    lv_obj_clear_flag(textHeaderBar, LV_OBJ_FLAG_SCROLLABLE);
    
    // Text back button - centered vertically
    lv_obj_t* textBackBtn = lv_btn_create(textHeaderBar);
    lv_obj_set_size(textBackBtn, 110, 40);
    lv_obj_align(textBackBtn, LV_ALIGN_LEFT_MID, 5, 0);
    lv_obj_set_style_bg_color(textBackBtn, accentColor, 0);
    lv_obj_set_style_radius(textBackBtn, 0, 0);  // No rounded corners
    lv_obj_add_event_cb(textBackBtn, fb_textBackBtnCb, LV_EVENT_CLICKED, NULL);
    
    lv_obj_t* textBackLabel = lv_label_create(textBackBtn);
    lv_label_set_text(textBackLabel, "< BACK");
    lv_obj_set_style_text_font(textBackLabel, &lv_font_unscii_16, 0);
    lv_obj_center(textBackLabel);
    
    // Text path label
    g_fb.textPathLabel = lv_label_create(textHeaderBar);
    lv_obj_set_style_text_font(g_fb.textPathLabel, &lv_font_unscii_16, 0);
    lv_obj_set_style_text_color(g_fb.textPathLabel, textColor, 0);
    lv_obj_align(g_fb.textPathLabel, LV_ALIGN_LEFT_MID, 125, 0);
    lv_obj_set_width(g_fb.textPathLabel, 660);
    
    // Text area - WHITE text
    g_fb.textArea = lv_textarea_create(g_fb.screenTextView);
    lv_obj_set_size(g_fb.textArea, 796, 426);
    lv_obj_align(g_fb.textArea, LV_ALIGN_TOP_MID, 0, 52);
    lv_obj_set_style_bg_color(g_fb.textArea, lv_color_hex(0x0a0a0a), 0);
    lv_obj_set_style_text_color(g_fb.textArea, lv_color_hex(0xffffff), 0);  // WHITE text
    lv_obj_set_style_text_font(g_fb.textArea, &lv_font_unscii_8, 0);
    lv_obj_set_style_border_width(g_fb.textArea, 1, 0);
    lv_obj_set_style_border_color(g_fb.textArea, lv_color_hex(0x444444), 0);
    lv_obj_set_style_radius(g_fb.textArea, 0, 0);  // No rounded corners
    lv_obj_clear_flag(g_fb.textArea, LV_OBJ_FLAG_CLICK_FOCUSABLE);
    
    // Enable horizontal scrolling (disable text wrap) and always show scrollbars
    lv_obj_t* ta_label = lv_textarea_get_label(g_fb.textArea);
    lv_obj_set_width(ta_label, LV_SIZE_CONTENT);  // Label width = content width (no wrap)
    lv_obj_set_scrollbar_mode(g_fb.textArea, LV_SCROLLBAR_MODE_ON);  // Always show scrollbars
    
    //-------------------------------------------------------------
    // CSV TABLE VIEWER SCREEN
    //-------------------------------------------------------------
    g_fb.screenCsvView = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(g_fb.screenCsvView, bgColor, 0);
    lv_obj_clear_flag(g_fb.screenCsvView, LV_OBJ_FLAG_SCROLLABLE);
    
    // CSV header bar
    lv_obj_t* csvHeaderBar = lv_obj_create(g_fb.screenCsvView);
    lv_obj_set_size(csvHeaderBar, 800, 50);
    lv_obj_align(csvHeaderBar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(csvHeaderBar, lv_color_hex(0x333333), 0);
    lv_obj_set_style_border_width(csvHeaderBar, 0, 0);
    lv_obj_set_style_radius(csvHeaderBar, 0, 0);
    lv_obj_set_style_pad_all(csvHeaderBar, 0, 0);
    lv_obj_clear_flag(csvHeaderBar, LV_OBJ_FLAG_SCROLLABLE);
    
    // CSV back button
    lv_obj_t* csvBackBtn = lv_btn_create(csvHeaderBar);
    lv_obj_set_size(csvBackBtn, 110, 40);
    lv_obj_align(csvBackBtn, LV_ALIGN_LEFT_MID, 5, 0);
    lv_obj_set_style_bg_color(csvBackBtn, accentColor, 0);
    lv_obj_set_style_radius(csvBackBtn, 0, 0);
    lv_obj_add_event_cb(csvBackBtn, fb_csvBackBtnCb, LV_EVENT_CLICKED, NULL);
    
    lv_obj_t* csvBackLabel = lv_label_create(csvBackBtn);
    lv_label_set_text(csvBackLabel, "< BACK");
    lv_obj_set_style_text_font(csvBackLabel, &lv_font_unscii_16, 0);
    lv_obj_center(csvBackLabel);
    
    // CSV path label
    g_fb.csvPathLabel = lv_label_create(csvHeaderBar);
    lv_obj_set_style_text_font(g_fb.csvPathLabel, &lv_font_unscii_16, 0);
    lv_obj_set_style_text_color(g_fb.csvPathLabel, textColor, 0);
    lv_obj_align(g_fb.csvPathLabel, LV_ALIGN_LEFT_MID, 125, 0);
    lv_obj_set_width(g_fb.csvPathLabel, 500);
    
    // CSV info label (row/col count) - right side of header
    g_fb.csvInfoLabel = lv_label_create(csvHeaderBar);
    lv_obj_set_style_text_font(g_fb.csvInfoLabel, &lv_font_unscii_8, 0);
    lv_obj_set_style_text_color(g_fb.csvInfoLabel, lv_color_hex(0x888888), 0);
    lv_obj_align(g_fb.csvInfoLabel, LV_ALIGN_RIGHT_MID, -10, 0);
    
    // CSV Table - frozen corner cell (top-left, completely frozen)
    g_fb.csvCornerCell = lv_table_create(g_fb.screenCsvView);
    lv_obj_set_size(g_fb.csvCornerCell, FB_CSV_COL_WIDTH + 2, 24);  // +2 for border
    lv_obj_align(g_fb.csvCornerCell, LV_ALIGN_TOP_LEFT, 2, 52);
    lv_obj_set_style_bg_color(g_fb.csvCornerCell, lv_color_hex(0x2a2a2a), 0);
    lv_obj_set_style_border_width(g_fb.csvCornerCell, 1, 0);
    lv_obj_set_style_border_color(g_fb.csvCornerCell, lv_color_hex(0x444444), 0);
    lv_obj_set_style_radius(g_fb.csvCornerCell, 0, 0);
    lv_obj_set_style_pad_all(g_fb.csvCornerCell, 0, 0);
    lv_obj_clear_flag(g_fb.csvCornerCell, LV_OBJ_FLAG_SCROLLABLE);
    
    // Corner cell styling - yellow text
    lv_obj_set_style_text_font(g_fb.csvCornerCell, &lv_font_unscii_8, LV_PART_ITEMS);
    lv_obj_set_style_text_color(g_fb.csvCornerCell, lv_color_hex(0xffff00), LV_PART_ITEMS);
    lv_obj_set_style_bg_color(g_fb.csvCornerCell, lv_color_hex(0x2a2a2a), LV_PART_ITEMS);
    lv_obj_set_style_border_width(g_fb.csvCornerCell, 1, LV_PART_ITEMS);
    lv_obj_set_style_border_color(g_fb.csvCornerCell, lv_color_hex(0x444444), LV_PART_ITEMS);
    lv_obj_set_style_pad_top(g_fb.csvCornerCell, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_bottom(g_fb.csvCornerCell, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_left(g_fb.csvCornerCell, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_right(g_fb.csvCornerCell, 4, LV_PART_ITEMS);
    lv_obj_set_style_min_height(g_fb.csvCornerCell, 24, LV_PART_ITEMS);  // Fixed height for 2 lines
    lv_obj_set_style_max_height(g_fb.csvCornerCell, 24, LV_PART_ITEMS);
    
    // CSV Table - frozen header row (scrolls horizontally only)
    g_fb.csvHeaderTable = lv_table_create(g_fb.screenCsvView);
    lv_obj_set_size(g_fb.csvHeaderTable, 694, 24);  // Width minus frozen column
    lv_obj_align(g_fb.csvHeaderTable, LV_ALIGN_TOP_LEFT, FB_CSV_COL_WIDTH + 4, 52);
    lv_obj_set_style_bg_color(g_fb.csvHeaderTable, lv_color_hex(0x2a2a2a), 0);
    lv_obj_set_style_border_width(g_fb.csvHeaderTable, 1, 0);
    lv_obj_set_style_border_color(g_fb.csvHeaderTable, lv_color_hex(0x444444), 0);
    lv_obj_set_style_radius(g_fb.csvHeaderTable, 0, 0);
    lv_obj_set_style_pad_all(g_fb.csvHeaderTable, 0, 0);
    lv_obj_clear_flag(g_fb.csvHeaderTable, LV_OBJ_FLAG_SCROLLABLE);  // No scroll - frozen!
    
    // Header table cell styling - yellow text on dark background
    lv_obj_set_style_text_font(g_fb.csvHeaderTable, &lv_font_unscii_8, LV_PART_ITEMS);
    lv_obj_set_style_text_color(g_fb.csvHeaderTable, lv_color_hex(0xffff00), LV_PART_ITEMS);  // Yellow headers
    lv_obj_set_style_bg_color(g_fb.csvHeaderTable, lv_color_hex(0x2a2a2a), LV_PART_ITEMS);
    lv_obj_set_style_border_width(g_fb.csvHeaderTable, 1, LV_PART_ITEMS);
    lv_obj_set_style_border_color(g_fb.csvHeaderTable, lv_color_hex(0x444444), LV_PART_ITEMS);
    lv_obj_set_style_pad_top(g_fb.csvHeaderTable, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_bottom(g_fb.csvHeaderTable, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_left(g_fb.csvHeaderTable, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_right(g_fb.csvHeaderTable, 4, LV_PART_ITEMS);
    lv_obj_set_style_min_height(g_fb.csvHeaderTable, 24, LV_PART_ITEMS);  // Fixed height
    lv_obj_set_style_max_height(g_fb.csvHeaderTable, 24, LV_PART_ITEMS);
    
    // CSV Table - frozen first column (scrolls vertically only)
    g_fb.csvColumnTable = lv_table_create(g_fb.screenCsvView);
    lv_obj_set_size(g_fb.csvColumnTable, FB_CSV_COL_WIDTH + 2, 402);  // +2 for border
    lv_obj_align(g_fb.csvColumnTable, LV_ALIGN_TOP_LEFT, 2, 76);
    lv_obj_set_style_bg_color(g_fb.csvColumnTable, lv_color_hex(0x2a2a2a), 0);
    lv_obj_set_style_border_width(g_fb.csvColumnTable, 1, 0);
    lv_obj_set_style_border_color(g_fb.csvColumnTable, lv_color_hex(0x444444), 0);
    lv_obj_set_style_radius(g_fb.csvColumnTable, 0, 0);
    lv_obj_set_style_pad_all(g_fb.csvColumnTable, 0, 0);
    lv_obj_set_scrollbar_mode(g_fb.csvColumnTable, LV_SCROLLBAR_MODE_OFF);  // Hide scrollbar, synced with data
    
    // Column table cell styling - yellow text (same as header)
    lv_obj_set_style_text_font(g_fb.csvColumnTable, &lv_font_unscii_8, LV_PART_ITEMS);
    lv_obj_set_style_text_color(g_fb.csvColumnTable, lv_color_hex(0xffff00), LV_PART_ITEMS);
    lv_obj_set_style_bg_color(g_fb.csvColumnTable, lv_color_hex(0x2a2a2a), LV_PART_ITEMS);
    lv_obj_set_style_border_width(g_fb.csvColumnTable, 1, LV_PART_ITEMS);
    lv_obj_set_style_border_color(g_fb.csvColumnTable, lv_color_hex(0x444444), LV_PART_ITEMS);
    lv_obj_set_style_pad_top(g_fb.csvColumnTable, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_bottom(g_fb.csvColumnTable, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_left(g_fb.csvColumnTable, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_right(g_fb.csvColumnTable, 4, LV_PART_ITEMS);
    lv_obj_set_style_min_height(g_fb.csvColumnTable, 24, LV_PART_ITEMS);  // Fixed height
    lv_obj_set_style_max_height(g_fb.csvColumnTable, 24, LV_PART_ITEMS);
    
    // Disable cell selection highlight for column table
    lv_obj_set_style_bg_color(g_fb.csvColumnTable, lv_color_hex(0x2a2a2a), LV_PART_ITEMS | LV_STATE_PRESSED);
    lv_obj_set_style_bg_color(g_fb.csvColumnTable, lv_color_hex(0x2a2a2a), LV_PART_ITEMS | LV_STATE_FOCUSED);
    
    // CSV Table - scrollable data rows (excludes row 0 and col 0)
    g_fb.csvTable = lv_table_create(g_fb.screenCsvView);
    lv_obj_set_size(g_fb.csvTable, 694, 402);  // Width minus frozen column
    lv_obj_align(g_fb.csvTable, LV_ALIGN_TOP_LEFT, FB_CSV_COL_WIDTH + 4, 76);  // Below header, right of column
    lv_obj_set_style_bg_color(g_fb.csvTable, lv_color_hex(0x0a0a0a), 0);
    lv_obj_set_style_border_width(g_fb.csvTable, 1, 0);
    lv_obj_set_style_border_color(g_fb.csvTable, lv_color_hex(0x444444), 0);
    lv_obj_set_style_radius(g_fb.csvTable, 0, 0);
    lv_obj_set_style_pad_all(g_fb.csvTable, 0, 0);
    
    // Table cell styling - normal cells
    lv_obj_set_style_text_font(g_fb.csvTable, &lv_font_unscii_8, LV_PART_ITEMS);
    lv_obj_set_style_text_color(g_fb.csvTable, lv_color_hex(0xffffff), LV_PART_ITEMS);  // White data
    lv_obj_set_style_bg_color(g_fb.csvTable, lv_color_hex(0x1a1a1a), LV_PART_ITEMS);
    lv_obj_set_style_border_width(g_fb.csvTable, 1, LV_PART_ITEMS);
    lv_obj_set_style_border_color(g_fb.csvTable, lv_color_hex(0x333333), LV_PART_ITEMS);
    lv_obj_set_style_pad_top(g_fb.csvTable, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_bottom(g_fb.csvTable, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_left(g_fb.csvTable, 4, LV_PART_ITEMS);
    lv_obj_set_style_pad_right(g_fb.csvTable, 4, LV_PART_ITEMS);
    lv_obj_set_style_min_height(g_fb.csvTable, 24, LV_PART_ITEMS);  // Fixed height
    lv_obj_set_style_max_height(g_fb.csvTable, 24, LV_PART_ITEMS);
    
    // Disable cell selection highlight (pressed state looks same as normal)
    lv_obj_set_style_bg_color(g_fb.csvTable, lv_color_hex(0x1a1a1a), LV_PART_ITEMS | LV_STATE_PRESSED);
    lv_obj_set_style_bg_color(g_fb.csvTable, lv_color_hex(0x1a1a1a), LV_PART_ITEMS | LV_STATE_FOCUSED);
    
    // Scrollbars always visible
    lv_obj_set_scrollbar_mode(g_fb.csvTable, LV_SCROLLBAR_MODE_ON);
    
    // Sync horizontal scroll between header and data tables
    lv_obj_add_event_cb(g_fb.csvTable, fb_csvScrollCb, LV_EVENT_SCROLL, NULL);
    
    Serial.println("[FB] Screens created");
}

static void fb_destroyScreens() {
    // Clear table data before destroying (prevents memory issues)
    if (g_fb.csvCornerCell) {
        lv_table_set_row_count(g_fb.csvCornerCell, 0);
        lv_table_set_column_count(g_fb.csvCornerCell, 0);
    }
    if (g_fb.csvHeaderTable) {
        lv_table_set_row_count(g_fb.csvHeaderTable, 0);
        lv_table_set_column_count(g_fb.csvHeaderTable, 0);
    }
    if (g_fb.csvColumnTable) {
        lv_table_set_row_count(g_fb.csvColumnTable, 0);
        lv_table_set_column_count(g_fb.csvColumnTable, 0);
    }
    if (g_fb.csvTable) {
        lv_table_set_row_count(g_fb.csvTable, 0);
        lv_table_set_column_count(g_fb.csvTable, 0);
    }
    
    // Clear textarea before destroying
    if (g_fb.textArea) {
        lv_textarea_set_text(g_fb.textArea, "");
    }
    
    if (g_fb.screenBrowser) { lv_obj_del(g_fb.screenBrowser); g_fb.screenBrowser = NULL; }
    if (g_fb.screenTextView) { lv_obj_del(g_fb.screenTextView); g_fb.screenTextView = NULL; }
    if (g_fb.screenCsvView) { lv_obj_del(g_fb.screenCsvView); g_fb.screenCsvView = NULL; }
    g_fb.pathLabel = NULL;
    g_fb.fileListContainer = NULL;
    g_fb.statusLabel = NULL;
    g_fb.textArea = NULL;
    g_fb.textPathLabel = NULL;
    g_fb.csvTable = NULL;
    g_fb.csvHeaderTable = NULL;
    g_fb.csvColumnTable = NULL;
    g_fb.csvCornerCell = NULL;
    g_fb.csvPathLabel = NULL;
    g_fb.csvInfoLabel = NULL;
}

//=================================================================
// OPTIMIZED SESSION LOADING - Uses boot_count, no scanning!
//=================================================================

static void fb_loadRecentSessions() {
    g_fb.files.clear();
    g_fb.files.reserve(FB_MAX_FILES_TO_DISPLAY + 10);  // Pre-allocate to reduce fragmentation
    
    uint32_t bootCount = g_current_boot_count;
    Serial.printf("[FB] Loading recent sessions from boot count %lu\n", bootCount);
    
    // FIRST: Add cached folders (or scan once if not cached)
    // Only scan first 50 entries - folders are typically at the beginning
    if (!g_fb.foldersCached) {
        Serial.println("[FB] Quick folder scan (first 50 entries)...");
        
        g_fb.cachedFolders.clear();
        g_fb.cachedFolders.reserve(10);
        
        File root = SD.open("/");
        if (root && root.isDirectory()) {
            File entry;
            int scanned = 0;
            const int MAX_SCAN = 50;  // Only scan first 50 entries
            
            while ((entry = root.openNextFile()) && scanned < MAX_SCAN) {
                if (entry.isDirectory()) {
                    const char* name = entry.name();
                    if (strcmp(name, "System Volume Information") != 0) {
                        FileEntry fe;
                        strncpy(fe.name, name, FB_MAX_FILENAME_LEN - 1);
                        fe.name[FB_MAX_FILENAME_LEN - 1] = '\0';
                        fe.size = 0;
                        fe.modTime = 0;
                        fe.isDirectory = true;
                        g_fb.cachedFolders.push_back(fe);
                        Serial.printf("[FB] Found folder: %s\n", name);
                    }
                }
                entry.close();
                scanned++;
            }
            root.close();
            Serial.printf("[FB] Scanned %d entries, found %d folders\n", scanned, (int)g_fb.cachedFolders.size());
        }
        g_fb.foldersCached = true;
    } else {
        Serial.printf("[FB] Using cached folders (%d)\n", (int)g_fb.cachedFolders.size());
    }
    
    // Add cached folders to file list
    for (const auto& folder : g_fb.cachedFolders) {
        g_fb.files.push_back(folder);
    }
    int foldersFound = g_fb.files.size();
    
    // SECOND: Add special files (if they exist)
    const char* specialFiles[] = { "wifi.cfg", "test_write.csv" };
    for (int i = 0; i < 2; i++) {
        char path[64];
        snprintf(path, sizeof(path), "/%s", specialFiles[i]);
        File f = SD.open(path);
        if (f) {
            FileEntry fe;
            strncpy(fe.name, specialFiles[i], FB_MAX_FILENAME_LEN - 1);
            fe.size = f.size();
            fe.modTime = f.getLastWrite();
            fe.isDirectory = false;
            g_fb.files.push_back(fe);
            f.close();
        }
    }
    
    // THIRD: Generate session filenames from current boot_count down
    int filesFound = g_fb.files.size();
    int consecutiveMisses = 0;
    const int MAX_CONSECUTIVE_MISSES = 10;
    
    for (uint32_t i = bootCount; i > 0 && filesFound < FB_MAX_FILES_TO_DISPLAY; i--) {
        bool foundAny = false;
        
        // Try CSV - 8-digit format first, then 5-digit
        char csvPath8[64], csvPath5[64];
        snprintf(csvPath8, sizeof(csvPath8), "/" FB_SESSION_FORMAT_8 ".csv", i);
        snprintf(csvPath5, sizeof(csvPath5), "/" FB_SESSION_FORMAT_5 ".csv", i);
        
        File csvFile = SD.open(csvPath8);
        const char* csvPath = csvPath8;
        if (!csvFile) {
            csvFile = SD.open(csvPath5);
            csvPath = csvPath5;
        }
        
        if (csvFile && filesFound < FB_MAX_FILES_TO_DISPLAY) {
            FileEntry fe;
            strncpy(fe.name, csvPath + 1, FB_MAX_FILENAME_LEN - 1);
            fe.size = csvFile.size();
            fe.modTime = csvFile.getLastWrite();
            fe.isDirectory = false;
            g_fb.files.push_back(fe);
            csvFile.close();
            filesFound++;
            foundAny = true;
        }
        
        // Try LOG - 8-digit format first, then 5-digit
        char logPath8[64], logPath5[64];
        snprintf(logPath8, sizeof(logPath8), "/" FB_SESSION_FORMAT_8 ".log", i);
        snprintf(logPath5, sizeof(logPath5), "/" FB_SESSION_FORMAT_5 ".log", i);
        
        File logFile = SD.open(logPath8);
        const char* logPath = logPath8;
        if (!logFile) {
            logFile = SD.open(logPath5);
            logPath = logPath5;
        }
        
        if (logFile && filesFound < FB_MAX_FILES_TO_DISPLAY) {
            FileEntry fe;
            strncpy(fe.name, logPath + 1, FB_MAX_FILENAME_LEN - 1);
            fe.size = logFile.size();
            fe.modTime = logFile.getLastWrite();
            fe.isDirectory = false;
            g_fb.files.push_back(fe);
            logFile.close();
            filesFound++;
            foundAny = true;
        }
        
        // Track consecutive misses to stop early
        if (foundAny) {
            consecutiveMisses = 0;
        } else {
            consecutiveMisses++;
            if (consecutiveMisses >= MAX_CONSECUTIVE_MISSES) {
                Serial.printf("[FB] Stopped after %d consecutive misses at session %lu\n", 
                              MAX_CONSECUTIVE_MISSES, i);
                break;
            }
        }
        
        vTaskDelay(1);  // Yield every iteration
    }
    
    g_fb.totalSessions = filesFound;
    Serial.printf("[FB] Total: %d items (%d folders, %d files)\n", 
                  (int)g_fb.files.size(), foldersFound, (int)g_fb.files.size() - foldersFound);
}

//=================================================================
// NAVIGATION
//=================================================================

static void fb_navigateTo(const char* path) {
    strncpy(g_fb.currentPath, path, FB_MAX_PATH_LEN - 1);
    
    if (strcmp(path, "/") == 0) {
        // Root directory - show loading if folders not cached yet
        if (!g_fb.foldersCached && g_fb.fileListContainer) {
            lv_obj_clean(g_fb.fileListContainer);
            lv_obj_t* loadLabel = lv_label_create(g_fb.fileListContainer);
            lv_label_set_text(loadLabel, "Loading...");
            lv_obj_set_style_text_color(loadLabel, lv_color_hex(0xFFFF00), 0);
            lv_obj_set_style_text_font(loadLabel, &lv_font_unscii_16, 0);
            lv_obj_center(loadLabel);
            lv_refr_now(NULL);
        }
        // Use optimized boot_count method
        fb_loadRecentSessions();
    } else {
        // Subdirectory - show loading indicator
        if (g_fb.fileListContainer) {
            lv_obj_clean(g_fb.fileListContainer);
            lv_obj_t* loadLabel = lv_label_create(g_fb.fileListContainer);
            lv_label_set_text(loadLabel, "Loading...");
            lv_obj_set_style_text_color(loadLabel, lv_color_hex(0xFFFF00), 0);
            lv_obj_set_style_text_font(loadLabel, &lv_font_unscii_16, 0);
            lv_obj_center(loadLabel);
            lv_refr_now(NULL);
        }
        
        // Subdirectory - use traditional scan
        g_fb.files.clear();
        File dir = SD.open(path);
        if (dir && dir.isDirectory()) {
            File entry;
            while ((entry = dir.openNextFile()) && (int)g_fb.files.size() < FB_MAX_FILES_TO_DISPLAY) {
                FileEntry fe;
                strncpy(fe.name, entry.name(), FB_MAX_FILENAME_LEN - 1);
                fe.size = entry.size();
                fe.modTime = entry.getLastWrite();
                fe.isDirectory = entry.isDirectory();
                g_fb.files.push_back(fe);
                entry.close();
            }
            dir.close();
        }
    }
    
    fb_updateFileList();
    
    if (g_fb.pathLabel) {
        lv_label_set_text(g_fb.pathLabel, g_fb.currentPath);
    }
    
    if (g_fb.statusLabel) {
        char status[40];
        snprintf(status, sizeof(status), "%d items", (int)g_fb.files.size());
        lv_label_set_text(g_fb.statusLabel, status);
    }
}

static void fb_goBack() {
    if (strcmp(g_fb.currentPath, "/") == 0) {
        fb_exit();
        return;
    }
    
    char parentPath[FB_MAX_PATH_LEN];
    strncpy(parentPath, g_fb.currentPath, FB_MAX_PATH_LEN);
    char* lastSlash = strrchr(parentPath, '/');
    if (lastSlash && lastSlash != parentPath) {
        *lastSlash = '\0';
    } else {
        strcpy(parentPath, "/");
    }
    fb_navigateTo(parentPath);
}

//=================================================================
// FILE LIST UI
//=================================================================

static lv_obj_t* fb_createListItem(const char* text, lv_color_t textColor, int index) {
    lv_obj_t* btn = lv_btn_create(g_fb.fileListContainer);
    lv_obj_set_size(btn, 790, FB_LIST_ITEM_HEIGHT);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x2a2a2a), 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x444444), LV_STATE_PRESSED);
    lv_obj_set_style_radius(btn, 0, 0);  // No rounded corners for performance
    lv_obj_set_style_pad_left(btn, 10, 0);
    lv_obj_add_flag(btn, LV_OBJ_FLAG_GESTURE_BUBBLE);
    
    lv_obj_t* label = lv_label_create(btn);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_font(label, &lv_font_unscii_16, 0);
    lv_obj_set_style_text_color(label, textColor, 0);
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 0, 0);
    
    lv_obj_add_event_cb(btn, fb_fileListClickCb, LV_EVENT_CLICKED, (void*)(intptr_t)index);
    
    return btn;
}

static void fb_updateFileList() {
    if (!g_fb.fileListContainer) return;
    lv_obj_clean(g_fb.fileListContainer);
    
    for (size_t i = 0; i < g_fb.files.size(); i++) {
        FileEntry& fe = g_fb.files[i];
        
        char displayText[160];
        if (fe.isDirectory) {
            // Directories: no indent, yellow
            snprintf(displayText, sizeof(displayText), "[DIR] %s/", fe.name);
        } else {
            // Files: 1-space indent, show size and date
            const char* dateStr = fb_formatDate(fe.modTime);
            if (dateStr[0] != '\0') {
                snprintf(displayText, sizeof(displayText), " %s  [%s] [%s]", 
                         fe.name, fb_formatSize(fe.size), dateStr);
            } else {
                snprintf(displayText, sizeof(displayText), " %s  [%s]", 
                         fe.name, fb_formatSize(fe.size));
            }
        }
        
        lv_color_t color;
        if (fe.isDirectory) {
            color = lv_color_hex(0xffff00);  // Yellow
        } else if (strstr(fe.name, ".csv")) {
            color = lv_color_hex(0x00ff00);  // Green for CSV
        } else if (strstr(fe.name, ".log")) {
            color = lv_color_hex(0x00ccff);  // Cyan for LOG
        } else {
            color = lv_color_hex(0xffffff);  // White for others
        }
        
        fb_createListItem(displayText, color, i);
    }
    
    lv_obj_scroll_to_y(g_fb.fileListContainer, 0, LV_ANIM_OFF);
}

//=================================================================
// FILE OPERATIONS
//=================================================================

static void fb_openTextFile(const char* path) {
    Serial.printf("[FB] Opening: %s\n", path);
    
    // Clear any previous text first
    if (g_fb.textArea) {
        lv_textarea_set_text(g_fb.textArea, "");
    }
    
    File file = SD.open(path, FILE_READ);
    if (!file) {
        Serial.printf("[FB] Failed to open: %s\n", path);
        return;
    }
    
    size_t fileSize = file.size();
    size_t readSize = min(fileSize, (size_t)FB_TEXT_MAX_SIZE);
    bool truncated = (fileSize > FB_TEXT_MAX_SIZE);
    
    char* buffer = (char*)heap_caps_malloc(readSize + 100, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buffer) buffer = (char*)malloc(readSize + 100);
    
    if (!buffer) {
        Serial.println("[FB] Memory allocation failed");
        file.close();
        return;
    }
    
    size_t bytesRead = file.read((uint8_t*)buffer, readSize);
    file.close();
    
    if (truncated) {
        snprintf(buffer + bytesRead, 100, "\n\n--- TRUNCATED (showing %uKB of %uKB) ---", 
                 (unsigned)(readSize/1024), (unsigned)(fileSize/1024));
        bytesRead = strlen(buffer);
    } else {
        buffer[bytesRead] = '\0';
    }
    
    lv_textarea_set_text(g_fb.textArea, buffer);
    free(buffer);
    
    lv_label_set_text(g_fb.textPathLabel, path);
    lv_textarea_set_cursor_pos(g_fb.textArea, 0);
    
    lv_scr_load(g_fb.screenTextView);
    g_fb.state = FB_STATE_TEXT_VIEW;
    
    Serial.printf("[FB] File loaded: %u bytes\n", bytesRead);
}

//=================================================================
// CSV FILE VIEWER
//=================================================================

static bool fb_isCsvFile(const char* filename) {
    const char* ext = strrchr(filename, '.');
    if (!ext) return false;
    return (strcasecmp(ext, ".csv") == 0);
}

static void fb_openCsvFile(const char* path) {
    Serial.printf("[FB] Opening CSV: %s\n", path);
    
    // Clear any previous table data first
    if (g_fb.csvCornerCell) {
        lv_table_set_row_count(g_fb.csvCornerCell, 0);
        lv_table_set_column_count(g_fb.csvCornerCell, 0);
    }
    if (g_fb.csvHeaderTable) {
        lv_table_set_row_count(g_fb.csvHeaderTable, 0);
        lv_table_set_column_count(g_fb.csvHeaderTable, 0);
    }
    if (g_fb.csvColumnTable) {
        lv_table_set_row_count(g_fb.csvColumnTable, 0);
        lv_table_set_column_count(g_fb.csvColumnTable, 0);
    }
    if (g_fb.csvTable) {
        lv_table_set_row_count(g_fb.csvTable, 0);
        lv_table_set_column_count(g_fb.csvTable, 0);
    }
    
    File file = SD.open(path, FILE_READ);
    if (!file) {
        Serial.printf("[FB] Failed to open CSV: %s\n", path);
        return;
    }
    
    // Read file into buffer (limit size)
    size_t fileSize = file.size();
    time_t fileTime = file.getLastWrite();  // Get file modification time
    size_t readSize = min(fileSize, (size_t)(FB_CSV_MAX_ROWS * 200));  // Rough estimate
    
    char* buffer = (char*)heap_caps_malloc(readSize + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buffer) buffer = (char*)malloc(readSize + 1);
    
    if (!buffer) {
        Serial.println("[FB] CSV memory allocation failed");
        file.close();
        return;
    }
    
    size_t bytesRead = file.read((uint8_t*)buffer, readSize);
    buffer[bytesRead] = '\0';
    file.close();
    
    // Parse CSV - first pass to count columns from header
    int numCols = 1;
    char* headerEnd = strchr(buffer, '\n');
    if (headerEnd) {
        for (char* p = buffer; p < headerEnd; p++) {
            if (*p == ',') numCols++;
        }
    }
    numCols = min(numCols, FB_CSV_MAX_COLS);
    
    // Count rows
    int numRows = 0;
    for (char* p = buffer; *p && numRows < FB_CSV_MAX_ROWS; p++) {
        if (*p == '\n') numRows++;
    }
    if (buffer[bytesRead-1] != '\n' && numRows < FB_CSV_MAX_ROWS) numRows++;  // Last line without newline
    
    Serial.printf("[FB] CSV: %d rows x %d cols\n", numRows, numCols);
    
    // Calculate data dimensions (excluding frozen row/col)
    int dataRows = (numRows > 1) ? (numRows - 1) : 0;
    int dataCols = (numCols > 1) ? (numCols - 1) : 0;
    
    // Configure corner cell (1x1, frozen top-left)
    lv_table_set_row_count(g_fb.csvCornerCell, 1);
    lv_table_set_column_count(g_fb.csvCornerCell, 1);
    lv_table_set_column_width(g_fb.csvCornerCell, 0, FB_CSV_COL_WIDTH);
    
    // Configure header table (1 row, cols 1+ for frozen header)
    lv_table_set_row_count(g_fb.csvHeaderTable, 1);
    lv_table_set_column_count(g_fb.csvHeaderTable, dataCols);
    
    // Configure column table (rows 1+, 1 col for frozen first column)
    lv_table_set_row_count(g_fb.csvColumnTable, dataRows);
    lv_table_set_column_count(g_fb.csvColumnTable, 1);
    lv_table_set_column_width(g_fb.csvColumnTable, 0, FB_CSV_COL_WIDTH);
    
    // Configure data table (rows 1+, cols 1+)
    lv_table_set_row_count(g_fb.csvTable, dataRows);
    lv_table_set_column_count(g_fb.csvTable, dataCols);
    
    // Set column widths for header and data tables (must match!)
    for (int c = 0; c < dataCols; c++) {
        lv_table_set_column_width(g_fb.csvHeaderTable, c, FB_CSV_COL_WIDTH);
        lv_table_set_column_width(g_fb.csvTable, c, FB_CSV_COL_WIDTH);
    }
    
    // Parse and populate cells
    char cell[FB_CSV_MAX_CELL_LEN + 1];
    char* lineStart = buffer;
    int row = 0;
    
    while (*lineStart && row < numRows) {
        char* lineEnd = strchr(lineStart, '\n');
        if (!lineEnd) lineEnd = lineStart + strlen(lineStart);
        
        // Parse columns in this line
        char* colStart = lineStart;
        int col = 0;
        bool inQuote = false;
        
        while (colStart < lineEnd && col < numCols) {
            char* cellEnd = colStart;
            inQuote = false;
            
            // Find end of cell (comma or end of line)
            while (cellEnd < lineEnd) {
                if (*cellEnd == '"') {
                    inQuote = !inQuote;
                } else if (*cellEnd == ',' && !inQuote) {
                    break;
                }
                cellEnd++;
            }
            
            // Extract cell content
            int cellLen = cellEnd - colStart;
            if (cellLen > FB_CSV_MAX_CELL_LEN) cellLen = FB_CSV_MAX_CELL_LEN;
            
            // Copy and clean up (remove quotes)
            int destIdx = 0;
            for (int i = 0; i < cellLen && destIdx < FB_CSV_MAX_CELL_LEN; i++) {
                char c = colStart[i];
                if (c != '"' && c != '\r') {
                    cell[destIdx++] = c;
                }
            }
            cell[destIdx] = '\0';
            
            // Distribute cells to appropriate table based on position:
            // [0,0] → corner, [0,1+] → header, [1+,0] → column, [1+,1+] → data
            if (row == 0 && col == 0) {
                lv_table_set_cell_value(g_fb.csvCornerCell, 0, 0, cell);
            } else if (row == 0 && col > 0) {
                lv_table_set_cell_value(g_fb.csvHeaderTable, 0, col - 1, cell);
            } else if (row > 0 && col == 0) {
                lv_table_set_cell_value(g_fb.csvColumnTable, row - 1, 0, cell);
            } else if (row > 0 && col > 0) {
                lv_table_set_cell_value(g_fb.csvTable, row - 1, col - 1, cell);
            }
            
            // Move to next column
            colStart = cellEnd + 1;  // Skip comma
            col++;
        }
        
        // Fill remaining columns with empty
        while (col < numCols) {
            if (row == 0 && col == 0) {
                lv_table_set_cell_value(g_fb.csvCornerCell, 0, 0, "");
            } else if (row == 0 && col > 0) {
                lv_table_set_cell_value(g_fb.csvHeaderTable, 0, col - 1, "");
            } else if (row > 0 && col == 0) {
                lv_table_set_cell_value(g_fb.csvColumnTable, row - 1, 0, "");
            } else if (row > 0 && col > 0) {
                lv_table_set_cell_value(g_fb.csvTable, row - 1, col - 1, "");
            }
            col++;
        }
        
        // Move to next row
        lineStart = (*lineEnd) ? lineEnd + 1 : lineEnd;
        row++;
        
        // Yield every 50 rows to prevent watchdog
        if (row % 50 == 0) {
            vTaskDelay(1);
        }
    }
    
    free(buffer);
    
    // Update UI labels
    lv_label_set_text(g_fb.csvPathLabel, path);
    
    // Format file size
    char sizeStr[16];
    if (fileSize >= 1024 * 1024) {
        snprintf(sizeStr, sizeof(sizeStr), "%.1f MB", fileSize / (1024.0 * 1024.0));
    } else if (fileSize >= 1024) {
        snprintf(sizeStr, sizeof(sizeStr), "%.1f KB", fileSize / 1024.0);
    } else {
        snprintf(sizeStr, sizeof(sizeStr), "%u B", (unsigned)fileSize);
    }
    
    // Format file date
    char dateStr[20];
    struct tm* tmInfo = localtime(&fileTime);
    if (tmInfo) {
        snprintf(dateStr, sizeof(dateStr), "%02d/%02d/%04d %02d:%02d",
                 tmInfo->tm_mon + 1, tmInfo->tm_mday, tmInfo->tm_year + 1900,
                 tmInfo->tm_hour, tmInfo->tm_min);
    } else {
        snprintf(dateStr, sizeof(dateStr), "Unknown");
    }
    
    // Combine all info
    char infoStr[80];
    snprintf(infoStr, sizeof(infoStr), "%d rows x %d cols\n%s | %s", numRows, numCols, sizeStr, dateStr);
    lv_label_set_text(g_fb.csvInfoLabel, infoStr);
    
    // Scroll to top-left
    lv_obj_scroll_to(g_fb.csvTable, 0, 0, LV_ANIM_OFF);
    
    // Switch to CSV view screen
    lv_scr_load(g_fb.screenCsvView);
    g_fb.state = FB_STATE_CSV_VIEW;
    
    Serial.printf("[FB] CSV loaded: %d rows x %d cols\n", row, numCols);
}

//=================================================================
// UTILITY FUNCTIONS
//=================================================================

static const char* fb_formatSize(uint32_t size) {
    static char buf[16];
    if (size < 1024) {
        snprintf(buf, sizeof(buf), "%uB", size);
    } else if (size < 1024 * 1024) {
        snprintf(buf, sizeof(buf), "%.1fK", size / 1024.0f);
    } else {
        snprintf(buf, sizeof(buf), "%.1fM", size / (1024.0f * 1024.0f));
    }
    return buf;
}

static const char* fb_formatDate(time_t t) {
    static char buf[20];
    if (t == 0) {
        return "";
    }
    struct tm* tm_info = localtime(&t);
    if (tm_info) {
        snprintf(buf, sizeof(buf), "%02d/%02d/%04d %02d:%02d",
                 tm_info->tm_mon + 1,
                 tm_info->tm_mday,
                 tm_info->tm_year + 1900,
                 tm_info->tm_hour,
                 tm_info->tm_min);
    } else {
        return "";
    }
    return buf;
}

//=================================================================
// EVENT CALLBACKS
//=================================================================

static void fb_fileListClickCb(lv_event_t* e) {
    int index = (int)(intptr_t)lv_event_get_user_data(e);
    if (index < 0 || index >= (int)g_fb.files.size()) return;
    
    FileEntry& fe = g_fb.files[index];
    
    char fullPath[FB_MAX_PATH_LEN];
    if (strcmp(g_fb.currentPath, "/") == 0) {
        snprintf(fullPath, sizeof(fullPath), "/%s", fe.name);
    } else {
        snprintf(fullPath, sizeof(fullPath), "%s/%s", g_fb.currentPath, fe.name);
    }
    
    if (fe.isDirectory) {
        fb_navigateTo(fullPath);
    } else if (fb_isCsvFile(fe.name)) {
        fb_openCsvFile(fullPath);
    } else {
        fb_openTextFile(fullPath);
    }
}

static void fb_backBtnCb(lv_event_t* e) {
    LV_UNUSED(e);
    fb_goBack();
}

static void fb_textBackBtnCb(lv_event_t* e) {
    LV_UNUSED(e);
    
    // Clear textarea to free memory
    if (g_fb.textArea) {
        lv_textarea_set_text(g_fb.textArea, "");
    }
    
    lv_scr_load(g_fb.screenBrowser);
    g_fb.state = FB_STATE_BROWSING;
}

// Sync header and column table scroll with data table
static void fb_csvScrollCb(lv_event_t* e) {
    LV_UNUSED(e);
    if (!g_fb.csvTable) return;
    
    // Get current scroll position of data table
    lv_coord_t scroll_x = lv_obj_get_scroll_x(g_fb.csvTable);
    lv_coord_t scroll_y = lv_obj_get_scroll_y(g_fb.csvTable);
    
    // Sync horizontal scroll to header table
    if (g_fb.csvHeaderTable) {
        lv_obj_scroll_to_x(g_fb.csvHeaderTable, scroll_x, LV_ANIM_OFF);
    }
    
    // Sync vertical scroll to column table
    if (g_fb.csvColumnTable) {
        lv_obj_scroll_to_y(g_fb.csvColumnTable, scroll_y, LV_ANIM_OFF);
    }
}

static void fb_csvBackBtnCb(lv_event_t* e) {
    LV_UNUSED(e);
    
    // Clear table data to free memory before switching screens
    if (g_fb.csvCornerCell) {
        lv_table_set_row_count(g_fb.csvCornerCell, 0);
        lv_table_set_column_count(g_fb.csvCornerCell, 0);
    }
    if (g_fb.csvHeaderTable) {
        lv_table_set_row_count(g_fb.csvHeaderTable, 0);
        lv_table_set_column_count(g_fb.csvHeaderTable, 0);
    }
    if (g_fb.csvColumnTable) {
        lv_table_set_row_count(g_fb.csvColumnTable, 0);
        lv_table_set_column_count(g_fb.csvColumnTable, 0);
    }
    if (g_fb.csvTable) {
        lv_table_set_row_count(g_fb.csvTable, 0);
        lv_table_set_column_count(g_fb.csvTable, 0);
    }
    
    lv_scr_load(g_fb.screenBrowser);
    g_fb.state = FB_STATE_BROWSING;
}

#endif // FILE_BROWSER_H
