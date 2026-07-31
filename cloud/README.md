# 370zMonitor — Cloud Log Upload (v6.11)

Auto-uploads each completed session's `SESS_*.csv` + `SESS_*.log` from the box to
**your Google Drive**, so you stop pulling the SD card / plugging in USB. Files
sync to your PC and phone through Drive, and the Fuel/Trims dashboard can read them
straight from the synced folder.

## How it works

At boot, the box already brings WiFi up briefly to sync the clock (NTP). v6.11
reuses that window: right before WiFi powers back off, it scans the card and streams
any sessions **not yet uploaded** to your endpoint, then records them in
`/CLOUDUP.DAT` so they're never sent twice. WiFi then powers off exactly as before.

- **You get logs after the fact:** the session you're driving now uploads on the
  **next** boot (it's still being written during the current drive). So: drive → get
  home → next time you start the car in WiFi range, yesterday's sessions upload.
- Only uploads when the configured WiFi is in range (your garage/home). At the track
  with no WiFi, nothing uploads — it catches up next time you're home.
- Capped at 8 sessions per boot (it catches up over a few boots if a backlog builds).
- Idempotent on both ends (manifest on the box + dedupe-by-name in the script), so
  re-sends never create duplicates.

## One-time setup (~5 minutes)

**1. Deploy the receiver script**
   1. Open <https://script.google.com> → **New project**.
   2. Delete the sample, paste in `cloud/CloudUpload.gs`.
   3. Change `SHARED_SECRET` to a long random string of your own.
   4. **Deploy → New deployment → Web app**: *Execute as* **Me**, *Who has access*
      **Anyone**. Deploy, authorize when prompted, and copy the **`/exec` URL**.
   5. Paste that `/exec` URL into a browser — you should see
      `370zMonitor uploader alive`.

**2. Point the box at it** — edit `/wifi.cfg` on the SD card (same file as your WiFi
   credentials) and add:

   ```
   WIFI_SSID=YourGarageNetwork
   WIFI_PASSWORD=YourPassword
   CLOUD_URL=https://script.google.com/macros/s/XXXXXXXX/exec
   CLOUD_SECRET=the_same_long_random_string_from_step_1.3
   CLOUD_FOLDER=370zMonitor_logs
   ```

That's it. Files land in **Drive → `370zMonitor_logs`**. Open that folder in the Fuel/
Trims dashboard (or anywhere) and it's the same data, everywhere, no cable.

> Leave `CLOUD_URL` blank (or omit it) to disable uploading — everything else works
> exactly as before.

## Verifying it (first flash)

Watch the serial log (115200) at boot for the `[CLOUD]` tag:

```
[CLOUD] Scanning card for sessions to upload...
[CLOUD] SESS_00000605.csv       HTTP 200  OK  (412334 bytes)
[CLOUD] SESS_00000605.log       HTTP 200  OK  (338110 bytes)
[CLOUD] Uploaded 1 new session(s) this boot.
```

Two things are **not flash-tested yet** and are the likely first-flash snags:

1. **The Apps Script redirect.** A Google web-app POST answers with a 302 to
   `script.googleusercontent.com`; the file is created on that first hop and we follow
   the redirect for the `OK` confirmation (we accept HTTP 200/301/302). If you see the
   file appear in Drive but the log says `FAIL`, that's just the confirmation read —
   harmless; tell me and I'll relax the check.
2. **TLS memory.** The HTTPS handshake needs a chunk of RAM/stack (we already bumped
   the sync-task stack 4 KB→16 KB). If uploads fail with mbedTLS alloc/handshake or a
   stack-overflow reset, internal heap is the cause and we'll trim buffers or move the
   upload earlier in boot.

## Not Google Drive?

The firmware is endpoint-agnostic — it just HTTPS-POSTs the file with `secret`,
`folder`, and `name` in the query string. Point `CLOUD_URL` at Dropbox, S3 (presigned),
or your own server and only `/wifi.cfg` changes; no firmware edit.

## Files

- `cloud/CloudUpload.gs` — the Google Apps Script web app (deploy this).
- Firmware: `uploadPendingSessions()` / `cloudUploadFile()` in `370zMonitor.ino`
  (feature flag `ENABLE_CLOUD_UPLOAD`), config parsed in `loadWifiConfig()`.
