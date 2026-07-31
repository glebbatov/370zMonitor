/**
 * 370zMonitor — cloud log receiver (Google Apps Script web app)
 * ---------------------------------------------------------------------------
 * Receives session log files from the ESP32 over HTTPS and saves each one into
 * a folder in YOUR Google Drive (which then syncs to your PC and phone).
 *
 * The ESP32 sends each file as the POST body, with metadata in the query string:
 *   POST https://script.google.com/macros/s/XXXX/exec
 *        ?secret=<SHARED_SECRET>&folder=370zMonitor_logs&name=SESS_00000601.csv
 *   body = the raw file text
 *
 * ---- ONE-TIME SETUP -------------------------------------------------------
 * 1. Go to https://script.google.com  ->  New project.
 * 2. Delete the sample code, paste THIS whole file in.
 * 3. Change SHARED_SECRET below to a long random string of your own.
 * 4. Deploy -> New deployment -> type "Web app".
 *      - Execute as:      Me (your account)
 *      - Who has access:  Anyone            <-- required so the box can POST
 *    Click Deploy, authorize when prompted, and COPY the "/exec" Web app URL.
 * 5. On the SD card, edit /wifi.cfg and add:
 *      CLOUD_URL=<the /exec URL you copied>
 *      CLOUD_SECRET=<the same SHARED_SECRET string>
 *      CLOUD_FOLDER=370zMonitor_logs
 * 6. Sanity check: paste the /exec URL into a browser — you should see
 *    "370zMonitor uploader alive".
 *
 * Files land in Drive > 370zMonitor_logs. Point the Fuel/Trims dashboard (or
 * anything else) at that synced folder. Uploads are idempotent: a name that
 * already exists is skipped, so re-sends never create duplicates.
 * ---------------------------------------------------------------------------
 */

var SHARED_SECRET = "CHANGE_ME_to_a_long_random_string";   // must equal CLOUD_SECRET in /wifi.cfg

function doPost(e) {
  try {
    var p = (e && e.parameter) || {};
    if (p.secret !== SHARED_SECRET) return out("ERR bad-secret");

    // sanitize the filename (defense-in-depth)
    var name = (p.name || ("upload_" + Date.now() + ".txt")).replace(/[^A-Za-z0-9._-]/g, "_");
    var folderName = (p.folder || "370zMonitor_logs").replace(/[^A-Za-z0-9._ -]/g, "_");
    var body = (e && e.postData && e.postData.contents) ? e.postData.contents : "";

    var folder = getFolder(folderName);

    // idempotent: skip if a file with this exact name already exists
    if (folder.getFilesByName(name).hasNext()) return out("OK exists " + name);

    var mime = /\.csv$/i.test(name) ? "text/csv" : "text/plain";
    folder.createFile(name, body, mime);
    return out("OK saved " + name + " (" + body.length + " bytes)");
  } catch (err) {
    return out("ERR " + err);
  }
}

// A browser GET is a handy health check.
function doGet(e) {
  return out("370zMonitor uploader alive");
}

function getFolder(name) {
  var it = DriveApp.getFoldersByName(name);
  return it.hasNext() ? it.next() : DriveApp.createFolder(name);
}

function out(s) {
  return ContentService.createTextOutput(s).setMimeType(ContentService.MimeType.TEXT);
}
