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
    FB_STATE_TEXT_VIEW
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
    
    lv_obj_t* pathLabel;
    lv_obj_t* fileListContainer;
    lv_obj_t* statusLabel;
    
    lv_obj_t* textArea;
    lv_obj_t* textPathLabel;
    
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
static void fb_goBack();
static void fb_updateFileList();
static const char* fb_formatSize(uint32_t size);
static const char* fb_formatDate(time_t t);

static void fb_fileListClickCb(lv_event_t* e);
static void fb_backBtnCb(lv_event_t* e);
static void fb_textBackBtnCb(lv_event_t* e);

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
    lv_obj_set_style_text_font(loadingLabel, &lv_font_montserrat_20, 0);
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
    
    Serial.println("[FB] Screens created");
}

static void fb_destroyScreens() {
    if (g_fb.screenBrowser) { lv_obj_del(g_fb.screenBrowser); g_fb.screenBrowser = NULL; }
    if (g_fb.screenTextView) { lv_obj_del(g_fb.screenTextView); g_fb.screenTextView = NULL; }
    g_fb.pathLabel = NULL;
    g_fb.fileListContainer = NULL;
    g_fb.statusLabel = NULL;
    g_fb.textArea = NULL;
    g_fb.textPathLabel = NULL;
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
            lv_obj_set_style_text_font(loadLabel, &lv_font_montserrat_20, 0);
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
            lv_obj_set_style_text_font(loadLabel, &lv_font_montserrat_20, 0);
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
    lv_scr_load(g_fb.screenBrowser);
    g_fb.state = FB_STATE_BROWSING;
}

#endif // FILE_BROWSER_H
