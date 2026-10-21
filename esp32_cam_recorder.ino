/* =====================================================================
 *  Freenove ESP32-WROVER CAM - change detector
 *
 *  The board watches the scene and sends almost nothing:
 *
 *    1. "Capture reference" freezes exposure and averages a few frames
 *       into a reference still.
 *    2. Ten times a second it compares the live frame to that reference,
 *       pixel by pixel, and keeps ONE number: the percentage of pixels
 *       that changed. /motion returns it in a few dozen bytes.
 *    3. When that percentage crosses the trigger, it captures a burst of
 *       20 mask frames, writes them to the SD card as a lossless .avi,
 *       and holds them in PSRAM for /burst to hand over in one go.
 *
 *  Nothing is streamed. An earlier version pushed every frame over Wi-Fi
 *  and fought the link the whole way: measured on the phone hotspot, the
 *  board had a -35 dBm signal with power save off and still saw 237 ms
 *  average ping with spikes past 1.3 s. Sending a number instead of a
 *  picture makes that latency irrelevant.
 *
 *  Board:   "ESP32 Wrover Module"  (Tools > Board > ESP32 Arduino)
 *           PSRAM: Enabled, Partition scheme: Huge APP (3MB)
 * ===================================================================== */

#include "esp_camera.h"
#include "img_converters.h"
#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>
#include "FS.h"
#include "SD_MMC.h"
#include "app_page.h"       // generated from phone_app.html by make_page.py

/* ------------------------- 1. Network ------------------------------- */
static const char* WIFI_SSID = "Philphone";   // 2.4 GHz hotspot
static const char* WIFI_PASS = "12345678";
static const char* HOSTNAME  = "esp32cam";

/* The board also runs its own access point, always at 192.168.4.1.
 * Connecting the phone straight to it takes the hotspot - and its
 * latency - out of the path. AP_ONLY skips the hotspot entirely. */
#define AP_ENABLED   1
#define AP_ONLY      0
static const char* AP_SSID = "ESP32-CAM";
static const char* AP_PASS = "camera123";     // at least 8 characters

/* ------------------------- 2. Detection ----------------------------- */
#define FRAME_SIZE        FRAMESIZE_QVGA  // 320x240; every pixel is compared
#define MOTION_HZ         10              // comparisons per second
#define MOTION_THRESHOLD  30              // per-pixel |live - reference|, 0..255
#define REF_AVERAGE       8               // frames averaged into the reference
#define MASK_DENOISE      1               // drop isolated pixels (sensor noise)

/* ------------------------- 3. Event shape --------------------------- *
 * A burst is NOT fired by movement alone - a hand left in frame would
 * then fire for ever. What fires it is a SEQUENCE: the picture moves a
 * lot for a while, then settles back down. The 20 frames handed over are
 * the last two seconds of that movement, kept in a ring buffer, not the
 * stillness that follows it.                                            */
#define MOVE_PERCENT      3.0f            // % of changed pixels that counts as movement
#define CALM_PERCENT      0.8f            // % below which the scene is considered settled
#define MIN_ACTIVE_MS     5000            // movement must last at least this long
#define CALM_CYCLES       8               // consecutive calm cycles that close an event
#define REARM_CYCLES      10              // calm cycles before a new event may start

/* ------------------------- 4. Burst --------------------------------- */
#define BURST_FRAMES      20              // frames kept per event
#define BURST_FPS         10
#define BURST_FRAME_MAX   24576           // PSRAM reserved per burst frame
#define BURST_QUALITY     12              // JPEG quality of the stored frames

/* ------------------------- 4. Camera pins --------------------------- */
/* Freenove ESP32-WROVER CAM (same map as the WROVER-KIT OV2640 board). */
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    21
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27
#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      19
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM       5
#define Y2_GPIO_NUM       4
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22
#define XCLK_FREQ_HZ     20000000
#define PREVIEW_QUALITY  12               // JPEG quality for /snapshot only

/* ------------------------- 5. Raw recording -------------------------- *
 * A continuous dump of exactly what the sensor produces: 8-bit grayscale,
 * no codec at all. The point is to replay the detection offline on real
 * footage and get the SAME numbers as live - a JPEG would quietly change
 * pixel values and move every threshold. Costs w*h bytes per frame, so
 * QVGA at 10 fps is 768 kB/s: roughly 20 minutes per gigabyte.          */
#define REC_FPS           10
static const char REC_MAGIC[4] = { 'E', 'S', 'P', 'G' };

/* ------------------------- 6. SD card ------------------------------- */
/* SDMMC pins are IO-MUX fixed on the classic ESP32: CLK 14, CMD 15, D0 2.
 * 1-bit mode frees GPIO4, which the Freenove board shares with its LED. */
#define SD_ONE_BIT_MODE   true
#define SD_MOUNT_POINT    "/sdcard"

/* ===================================================================== */

WebServer server(80);

/* --- frame buffers (PSRAM) --- */
static uint8_t*  refBuf   = nullptr;   // the reference still
static uint8_t*  maskBuf  = nullptr;   // working mask, 0x00 or 0xFF
static uint8_t*  workBuf  = nullptr;   // denoise scratch
static uint8_t*  prevBuf  = nullptr;   // previous frame, for inter-frame motion
static bool      havePrev = false;
static uint8_t*  rleBuf   = nullptr;   // encoder output
static size_t    rleCap   = 0;
static size_t    grayLen  = 0;
static uint16_t  grayW = 0, grayH = 0;

/* --- detection state --- */
static bool     haveRef        = false;
static bool     exposureLocked = false;
static uint8_t  threshold      = MOTION_THRESHOLD;
static float    movePct        = MOVE_PERCENT;
static float    calmPct        = CALM_PERCENT;
static uint32_t minActiveMs    = MIN_ACTIVE_MS;
static float    lastPct        = 0.0f;   // inter-frame movement
static float    refPct         = 0.0f;   // difference from the reference
static uint32_t nextMotionMs   = 0;

/* --- event state machine ---
 * WAITING : nothing happening, or still settling after an event.
 * ACTIVE  : movement is under way, frames are going into the ring.
 * READY   : an event closed, its last 20 frames are available.        */
enum EvtState { EVT_WAITING = 0, EVT_ACTIVE = 1, EVT_READY = 2 };
static EvtState evtState    = EVT_WAITING;
static uint32_t activeStart = 0;         // millis() when movement began
static uint32_t activeMs    = 0;         // duration of the event just closed
static uint16_t calmRun     = 0;         // consecutive calm cycles
static uint16_t rearmLeft   = 0;         // calm cycles still needed to re-arm

/* --- ring of the most recent masks --- */
static uint8_t*   burstBuf   = nullptr;          // BURST_FRAMES * BURST_FRAME_MAX
static uint32_t   burstLen[BURST_FRAMES];
static uint8_t    ringPos    = 0;                // next slot to write
static uint8_t    ringCount  = 0;                // slots filled, up to BURST_FRAMES
static uint8_t    burstCount = 0;                // frames in the frozen burst
static uint8_t    burstOrder[BURST_FRAMES];      // slots, oldest first
static uint32_t   eventCount = 0;
static char       burstFile[32] = {0};

static bool     sdReady  = false;
static String   lastError = "";

/* --- raw recording --- */
static File     recFile;
static bool     recOn     = false;
static uint32_t recFrames = 0;
static uint32_t recNextMs = 0;
static char     recPath[32] = {0};

/* ---------------------- little-endian helpers ------------------------ */
static inline void w32(uint8_t* p, uint32_t v) {
  p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}
static inline void w16(uint8_t* p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static inline void tag(uint8_t* p, const char* s) { memcpy(p, s, 4); }

/* ------------------------- AVI (MJPEG) ------------------------------- *
 * The burst holds real camera frames, already JPEG encoded, so they go
 * into the file untouched. Header is a fixed 224 bytes.                 */
static const uint32_t AVI_HEADER_BYTES = 224;

static void buildAviHeader(uint8_t* h, uint16_t w, uint16_t ht,
                           uint32_t fps, uint32_t frames,
                           uint32_t movi, uint32_t maxFrame) {
  memset(h, 0, AVI_HEADER_BYTES);
  if (fps == 0) fps = 1;
  const uint32_t usPerFrame = 1000000UL / fps;
  const uint32_t idxBytes   = 16UL * frames;

  tag(h + 0,  "RIFF"); w32(h + 4,  216 + movi + 8 + idxBytes); tag(h + 8, "AVI ");
  tag(h + 12, "LIST"); w32(h + 16, 192);                       tag(h + 20, "hdrl");

  tag(h + 24, "avih"); w32(h + 28, 56);
  w32(h + 32, usPerFrame);        // dwMicroSecPerFrame
  w32(h + 36, maxFrame * fps);    // dwMaxBytesPerSec
  w32(h + 44, 0x10);              // dwFlags = AVIF_HASINDEX
  w32(h + 48, frames);            // dwTotalFrames
  w32(h + 56, 1);                 // dwStreams
  w32(h + 60, maxFrame);          // dwSuggestedBufferSize
  w32(h + 64, w);
  w32(h + 68, ht);

  tag(h + 88,  "LIST"); w32(h + 92, 116); tag(h + 96, "strl");

  tag(h + 100, "strh"); w32(h + 104, 56);
  tag(h + 108, "vids");
  tag(h + 112, "MJPG");           // fccHandler
  w32(h + 128, 1);                // dwScale
  w32(h + 132, fps);              // dwRate
  w32(h + 140, frames);           // dwLength
  w32(h + 144, maxFrame);
  w32(h + 148, 0xFFFFFFFF);       // dwQuality = default
  w16(h + 156, 0); w16(h + 158, 0); w16(h + 160, w); w16(h + 162, ht);

  tag(h + 164, "strf"); w32(h + 168, 40);   // BITMAPINFOHEADER
  w32(h + 172, 40);                         // biSize
  w32(h + 176, w);
  w32(h + 180, ht);
  w16(h + 184, 1);                          // biPlanes
  w16(h + 186, 24);                         // biBitCount
  tag(h + 188, "MJPG");                     // biCompression
  w32(h + 192, (uint32_t)w * ht * 3);       // biSizeImage

  tag(h + 212, "LIST"); w32(h + 216, 4 + movi); tag(h + 220, "movi");
}

/* --------------------------- camera ---------------------------------- */
static bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;   c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;   c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;   c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;   c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk  = XCLK_GPIO_NUM;  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn  = PWDN_GPIO_NUM;  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = XCLK_FREQ_HZ;
  c.pixel_format = PIXFORMAT_GRAYSCALE;   // raw pixels: nothing to decode
  c.frame_size   = FRAME_SIZE;
  c.jpeg_quality = PREVIEW_QUALITY;
  c.grab_mode    = CAMERA_GRAB_LATEST;
  c.fb_location  = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  c.fb_count     = psramFound() ? 2 : 1;

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("[CAM] init failed: 0x%X\n", err);
    return false;
  }
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, 0);        // set to 1 if the picture is upside down
    s->set_hmirror(s, 0);
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    /* Automatic exposure stays ON until a reference is taken: forcing it
     * manually at boot only produces a picture too dark to work with. */
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_gain_ctrl(s, 1);
  }
  exposureLocked = false;
  return true;
}

/* Freeze the sensor at whatever auto-exposure settled on. Without this the
 * sensor re-exposes between the reference and the live frame, every pixel
 * shifts at once, and the mask turns solid black. */
static void lockExposure(bool on) {
  sensor_t* s = esp_camera_sensor_get();
  if (!s) return;
  if (on) {
    const int ae = s->status.aec_value;
    const int ag = s->status.agc_gain;
    s->set_exposure_ctrl(s, 0);
    s->set_gain_ctrl(s, 0);
    s->set_whitebal(s, 0);
    s->set_awb_gain(s, 0);
    s->set_aec_value(s, ae);
    s->set_agc_gain(s, ag);
    Serial.printf("[CAM] exposure locked (aec=%d gain=%d)\n", ae, ag);
  } else {
    s->set_exposure_ctrl(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
  }
  exposureLocked = on;
}

/* ------------------------- buffers ----------------------------------- */
static bool ensureBuffers(camera_fb_t* fb) {
  if (grayLen == fb->len && refBuf) return true;

  free(refBuf); free(maskBuf); free(workBuf); free(prevBuf); free(rleBuf); free(burstBuf);
  refBuf = maskBuf = workBuf = prevBuf = rleBuf = burstBuf = nullptr;
  haveRef = havePrev = false;

  grayLen = fb->len; grayW = fb->width; grayH = fb->height;
  rleCap  = grayLen * 2 + 16;              // worst case: alternating pixels

  refBuf   = (uint8_t*)ps_malloc(grayLen);
  maskBuf  = (uint8_t*)ps_malloc(grayLen);
  workBuf  = (uint8_t*)ps_malloc(grayLen);
  prevBuf  = (uint8_t*)ps_malloc(grayLen);
  rleBuf   = (uint8_t*)ps_malloc(rleCap);
  burstBuf = (uint8_t*)ps_malloc((size_t)BURST_FRAMES * BURST_FRAME_MAX);

  if (!refBuf || !maskBuf || !workBuf || !prevBuf || !rleBuf || !burstBuf) {
    lastError = "Out of PSRAM";
    Serial.println("[MEM] allocation failed");
    grayLen = 0;
    return false;
  }
  Serial.printf("[MEM] %ux%u ready, %u kB PSRAM free\n",
                grayW, grayH, (unsigned)(ESP.getFreePsram() / 1024));
  return true;
}

/* Average several frames into the reference. One frame carries sensor
 * noise straight into the comparison, which speckles every later mask and
 * drowns the shape of whatever actually moved. */
static bool captureReference() {
  lockExposure(true);
  delay(150);                                    // let the locked settings land

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { lastError = "Camera capture failed"; return false; }
  if (!ensureBuffers(fb)) { esp_camera_fb_return(fb); return false; }

  /* The accumulator lives in PSRAM: 320x240 uint32 is 300 kB, which does
   * not fit in the ESP32's internal DRAM at all. */
  uint32_t* acc = (uint32_t*)ps_malloc(grayLen * sizeof(uint32_t));
  if (!acc) { esp_camera_fb_return(fb); lastError = "Out of PSRAM"; return false; }

  for (size_t i = 0; i < grayLen; i++) acc[i] = fb->buf[i];
  esp_camera_fb_return(fb);

  uint8_t used = 1;
  for (int k = 1; k < REF_AVERAGE; k++) {
    delay(40);
    camera_fb_t* f = esp_camera_fb_get();
    if (!f) continue;
    if (f->len == grayLen) {
      for (size_t i = 0; i < grayLen; i++) acc[i] += f->buf[i];
      used++;
    }
    esp_camera_fb_return(f);
  }
  for (size_t i = 0; i < grayLen; i++) refBuf[i] = (uint8_t)(acc[i] / used);
  free(acc);

  haveRef   = true;
  evtState  = EVT_WAITING;
  rearmLeft = 0;
  ringCount = ringPos = 0;
  lastError = "";
  Serial.printf("[REF] reference averaged over %d frames (%ux%u)\n",
                REF_AVERAGE, grayW, grayH);
  return true;
}

/* Percentage of pixels that differ from the PREVIOUS frame.
 *
 * This is what drives the event logic, and it has to be inter-frame
 * rather than against the reference: a hand left in the picture keeps a
 * reference comparison permanently high, so the event would never close
 * and the burst would fire again and again. Compared frame to frame, the
 * same motionless hand reads 0% - movement is movement, presence is not. */
static float interFrameMotion(camera_fb_t* fb) {
  if (fb->len != grayLen) return 0.0f;
  if (!havePrev) {
    memcpy(prevBuf, fb->buf, grayLen);
    havePrev = true;
    return 0.0f;
  }
  const uint8_t thr = threshold;
  uint32_t moved = 0;
  for (size_t i = 0; i < grayLen; i++) {
    const int d = (int)fb->buf[i] - (int)prevBuf[i];
    if ((d < 0 ? -d : d) > thr) moved++;
  }
  memcpy(prevBuf, fb->buf, grayLen);
  return 100.0f * (float)moved / (float)grayLen;
}

/* Compare to the reference. Fills maskBuf with 0x00 (changed) / 0xFF
 * (unchanged) and returns the percentage of changed pixels. This is the
 * picture the burst carries: it shows the SHAPE of whatever is there. */
static float buildMask(camera_fb_t* fb) {
  if (!haveRef || fb->len != grayLen) return 0.0f;

  const uint8_t thr = threshold;
  uint32_t changed = 0;
  for (size_t i = 0; i < grayLen; i++) {
    const int d = (int)fb->buf[i] - (int)refBuf[i];
    const bool diff = ((d < 0 ? -d : d) > thr);
    maskBuf[i] = diff ? 0x00 : 0xFF;
  }

#if MASK_DENOISE
  /* A black pixel with fewer than two black neighbours is sensor noise,
   * not an object. Done into a separate buffer so the erosion does not
   * cascade into itself as it walks the image. */
  memcpy(workBuf, maskBuf, grayLen);
  for (uint16_t y = 1; y + 1 < grayH; y++) {
    const size_t row = (size_t)y * grayW;
    for (uint16_t x = 1; x + 1 < grayW; x++) {
      const size_t i = row + x;
      if (maskBuf[i]) continue;
      uint8_t n = 0;
      if (!maskBuf[i - 1])     n++;
      if (!maskBuf[i + 1])     n++;
      if (!maskBuf[i - grayW]) n++;
      if (!maskBuf[i + grayW]) n++;
      if (n < 2) workBuf[i] = 0xFF;
    }
  }
  memcpy(maskBuf, workBuf, grayLen);
#endif

  for (size_t i = 0; i < grayLen; i++) if (!maskBuf[i]) changed++;
  return 100.0f * (float)changed / (float)grayLen;
}

/* ---------------------- encoders ------------------------------------- */
/* For transfer: plain [count][0|1] pairs, top-down, no markers. A few
 * lines of JavaScript undo it. A still scene is one long white run. */
static size_t encodeRuns(const uint8_t* img) {
  size_t o = 0, i = 0;
  while (i < grayLen) {
    const uint8_t v = img[i] ? 1 : 0;
    size_t run = 1;
    while (i + run < grayLen && (img[i + run] ? 1 : 0) == v && run < 255) run++;
    rleBuf[o++] = (uint8_t)run;
    rleBuf[o++] = v;
    i += run;
  }
  return o;
}

/* ----------------------------- SD ------------------------------------ */
static bool initSD() {
  if (!SD_MMC.begin(SD_MOUNT_POINT, SD_ONE_BIT_MODE)) {
    lastError = "SD mount failed (card inserted? formatted FAT32?)";
    Serial.println("[SD] mount failed");
    return false;
  }
  if (SD_MMC.cardType() == CARD_NONE) {
    lastError = "No SD card detected";
    return false;
  }
  Serial.printf("[SD] mounted: %llu MB total, %llu MB used\n",
                SD_MMC.totalBytes() / (1024ULL * 1024ULL),
                SD_MMC.usedBytes()  / (1024ULL * 1024ULL));
  return true;
}

/* Writes the burst held in PSRAM as one lossless .avi. The burst frames
 * are stored run-length coded for transfer, so they are decoded back to a
 * mask and re-encoded in the AVI's own flavour of RLE. */
static void writeBurstAvi() {
  if (!sdReady || !burstCount) return;

  char path[32];
  for (uint16_t i = 1; i < 10000; i++) {
    snprintf(path, sizeof(path), "/evt_%04u.avi", i);
    if (!SD_MMC.exists(path)) break;
  }

  File f = SD_MMC.open(path, FILE_WRITE);
  if (!f) { lastError = String("Cannot create ") + path; return; }

  uint8_t hdr[AVI_HEADER_BYTES];
  buildAviHeader(hdr, grayW, grayH, BURST_FPS, 0, 0, 0);
  f.write(hdr, AVI_HEADER_BYTES);

  uint8_t* idx = (uint8_t*)ps_malloc(16UL * BURST_FRAMES);
  if (!idx) { f.close(); lastError = "Out of PSRAM"; return; }

  uint32_t movi = 0, maxFrame = 0;
  for (uint8_t k = 0; k < burstCount; k++) {
    const uint8_t  slot = burstOrder[k];
    const uint8_t* src  = burstBuf + (size_t)slot * BURST_FRAME_MAX;
    const uint32_t len  = burstLen[slot];
    const uint8_t  pad  = (len & 1) ? 1 : 0;      // RIFF chunks are word-aligned

    uint8_t ch[8];
    tag(ch, "00dc"); w32(ch + 4, len);
    f.write(ch, 8);
    f.write(src, len);
    if (pad) { uint8_t z = 0; f.write(&z, 1); }

    uint8_t* e = idx + 16UL * k;
    tag(e, "00dc"); w32(e + 4, 0x10); w32(e + 8, 4 + movi); w32(e + 12, len);

    movi += 8 + len + pad;
    if (len > maxFrame) maxFrame = len;
  }

  uint8_t ih[8];
  tag(ih, "idx1"); w32(ih + 4, 16UL * burstCount);
  f.write(ih, 8);
  f.write(idx, 16UL * burstCount);
  f.close();
  free(idx);

  File patch = SD_MMC.open(path, "r+");
  if (patch) {
    buildAviHeader(hdr, grayW, grayH, BURST_FPS, burstCount, movi, maxFrame);
    patch.seek(0);
    patch.write(hdr, AVI_HEADER_BYTES);
    patch.close();
  }

  strncpy(burstFile, path, sizeof(burstFile) - 1);
  Serial.printf("[EVT] %s written, %u frames\n", path, burstCount);
}

/* ------------------------ raw recording ------------------------------ *
 * File layout, little-endian, 16-byte header then frame after frame:
 *   0  "ESPG"   4 bytes
 *   4  width    uint16
 *   6  height   uint16
 *   8  fps      uint16
 *  10  (unused) uint16
 *  12  frames   uint32   - written as 0, patched when recording stops
 *  16  frame 0, frame 1, ... each width*height bytes of 8-bit grayscale
 * tools/replay.py reads exactly this.                                    */
static bool startRawRecording() {
  if (recOn) return true;
  if (!sdReady) { lastError = "SD card not ready"; return false; }
  if (!grayW)   { lastError = "camera not ready yet"; return false; }

  for (uint16_t i = 1; i < 10000; i++) {
    snprintf(recPath, sizeof(recPath), "/raw_%04u.gray", i);
    if (!SD_MMC.exists(recPath)) break;
  }
  recFile = SD_MMC.open(recPath, FILE_WRITE);
  if (!recFile) { lastError = String("Cannot create ") + recPath; return false; }

  uint8_t h[16] = {0};
  memcpy(h, REC_MAGIC, 4);
  w16(h + 4, grayW);
  w16(h + 6, grayH);
  w16(h + 8, REC_FPS);
  w32(h + 12, 0);                      // patched on stop
  if (recFile.write(h, 16) != 16) {
    recFile.close();
    lastError = "SD write failed on the header";
    return false;
  }

  recFrames = 0;
  recNextMs = millis();
  recOn = true;
  lastError = "";
  Serial.printf("[RAW] recording -> %s (%ux%u, %d fps)\n", recPath, grayW, grayH, REC_FPS);
  return true;
}

static void stopRawRecording() {
  if (!recOn) return;
  recOn = false;
  recFile.close();

  File patch = SD_MMC.open(recPath, "r+");
  if (patch) {
    uint8_t n[4];
    w32(n, recFrames);
    patch.seek(12);
    patch.write(n, 4);
    patch.close();
  }
  Serial.printf("[RAW] stopped: %s, %u frames, %.1f MB\n", recPath, (unsigned)recFrames,
                (16.0 + (double)recFrames * grayLen) / 1048576.0);
}

/* Writes the frame the detection has just looked at: one grab serves
 * both, so recording costs no extra capture. */
static void recordRaw(camera_fb_t* fb) {
  if (!recOn) return;
  const uint32_t now = millis();
  if ((int32_t)(now - recNextMs) < 0) return;
  recNextMs = now + 1000UL / REC_FPS;

  if (recFile.write(fb->buf, fb->len) != fb->len) {
    Serial.println("[RAW] SD write failed - card full?");
    lastError = "SD write failed (card full?)";
    stopRawRecording();
    return;
  }
  recFrames++;
  if ((recFrames & 0x1F) == 0) recFile.flush();
}

/* ------------------------ detection loop ----------------------------- */
/* Keeps the most recent PICTURE in the ring, overwriting the oldest.
 *
 * The mask is what decides when something happened; the burst is what you
 * actually look at afterwards, so it holds real camera frames. They are
 * JPEG encoded here and never touched again: the .avi stores these exact
 * bytes, and the page decodes them as ordinary images.
 *
 * Encoding only runs while an event is under way, not while waiting. */
static void ringPush(camera_fb_t* fb) {
  uint8_t* jpg = nullptr;
  size_t   len = 0;
  if (!fmt2jpg(fb->buf, fb->len, fb->width, fb->height,
               PIXFORMAT_GRAYSCALE, BURST_QUALITY, &jpg, &len)) return;

  if (len <= BURST_FRAME_MAX) {
    memcpy(burstBuf + (size_t)ringPos * BURST_FRAME_MAX, jpg, len);
    burstLen[ringPos] = len;
    ringPos = (ringPos + 1) % BURST_FRAMES;
    if (ringCount < BURST_FRAMES) ringCount++;
  }
  free(jpg);
}

/* Freezes the ring into a burst, oldest frame first. */
static void closeEvent() {
  burstCount = ringCount;
  const uint8_t first = (uint8_t)((ringPos + BURST_FRAMES - ringCount) % BURST_FRAMES);
  for (uint8_t i = 0; i < burstCount; i++)
    burstOrder[i] = (uint8_t)((first + i) % BURST_FRAMES);

  evtState = EVT_READY;
  eventCount++;
  rearmLeft = REARM_CYCLES;
  writeBurstAvi();
  Serial.printf("[EVT] event %u: %.1f s of movement, %u frames kept\n",
                (unsigned)eventCount, activeMs / 1000.0f, burstCount);
}

static void motionTick() {
  const uint32_t now = millis();
  if ((int32_t)(now - nextMotionMs) < 0) return;
  nextMotionMs = now + 1000UL / MOTION_HZ;

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return;
  if (!ensureBuffers(fb)) { esp_camera_fb_return(fb); return; }
  if (!haveRef) { esp_camera_fb_return(fb); return; }

  recordRaw(fb);                      // same grab feeds the recording
  lastPct = interFrameMotion(fb);     // drives the state machine
  refPct  = buildMask(fb);            // the mask, for the on-demand view

  switch (evtState) {

    case EVT_ACTIVE:
      ringPush(fb);                                // keep the movement itself
      if (lastPct < calmPct) {
        calmRun++;
        if (calmRun >= CALM_CYCLES) {
          /* The scene has settled. Only call it an event if the movement
           * lasted long enough - a brief flicker is not one. */
          activeMs = now - activeStart - CALM_CYCLES * (1000UL / MOTION_HZ);
          if (activeMs >= minActiveMs) {
            closeEvent();
          } else {
            Serial.printf("[EVT] ignored, only %.1f s of movement\n", activeMs / 1000.0f);
            evtState = EVT_WAITING;
            ringCount = ringPos = 0;
          }
          calmRun = 0;
        }
      } else {
        calmRun = 0;                               // still moving
      }
      break;

    case EVT_READY:
      /* Hold the burst until the scene has been calm for a while, so one
       * long movement cannot produce event after event. */
      if (lastPct < calmPct) {
        if (rearmLeft) rearmLeft--;
      } else {
        rearmLeft = REARM_CYCLES;
      }
      break;

    case EVT_WAITING:
    default:
      if (rearmLeft) {                             // settling after an event
        if (lastPct < calmPct) rearmLeft--; else rearmLeft = REARM_CYCLES;
        break;
      }
      if (lastPct >= movePct) {
        evtState    = EVT_ACTIVE;
        activeStart = now;
        calmRun     = 0;
        ringCount   = ringPos = 0;
        burstFile[0] = 0;
        Serial.printf("[EVT] movement started at %.1f%%\n", lastPct);
        ringPush(fb);
      }
      break;
  }

  esp_camera_fb_return(fb);
}

/* ----------------------------- web ----------------------------------- */
static void cors() { server.sendHeader("Access-Control-Allow-Origin", "*"); }

static void handleRoot() { server.send_P(200, "text/html", APP_PAGE); }

/* A few dozen bytes, polled several times a second even on a bad link. */
static void handleMotion() {
  char j[256];
  snprintf(j, sizeof(j),
           "{\"pct\":%.2f,\"refpct\":%.2f,\"move\":%.2f,\"calm\":%.2f,\"minms\":%u,\"thr\":%u,"
           "\"ref\":%s,\"lock\":%s,\"state\":%u,\"active\":%u,\"n\":%u,"
           "\"events\":%u,\"sd\":%s,\"rec\":%s,\"recn\":%u,\"recfile\":\"%s\",\"w\":%u,\"h\":%u,\"file\":\"%s\"}",
           lastPct, refPct, movePct, calmPct, (unsigned)minActiveMs, (unsigned)threshold,
           haveRef ? "true" : "false",
           exposureLocked ? "true" : "false",
           (unsigned)evtState,
           (unsigned)(evtState == EVT_ACTIVE ? (millis() - activeStart) : activeMs),
           (unsigned)burstCount, (unsigned)eventCount,
           sdReady ? "true" : "false",
           recOn ? "true" : "false", (unsigned)recFrames, recPath,
           (unsigned)grayW, (unsigned)grayH, burstFile);
  cors();
  server.send(200, "application/json", j);
}

/* The whole burst in one response: [u16 w][u16 h][u8 count], then per
 * frame [u32 length][run-length bytes]. One transfer per event instead of
 * a continuous stream. */
static void handleBurst() {
  if (evtState != EVT_READY || !burstCount) { cors(); server.send(409, "text/plain", "no burst"); return; }

  uint32_t total = 5;
  for (uint8_t k = 0; k < burstCount; k++) total += 4 + burstLen[burstOrder[k]];

  uint8_t head[5];
  w16(head, grayW); w16(head + 2, grayH); head[4] = burstCount;

  cors();
  server.setContentLength(total);
  server.send(200, "application/octet-stream", "");
  server.sendContent((const char*)head, 5);

  for (uint8_t k = 0; k < burstCount; k++) {
    const uint8_t slot = burstOrder[k];
    uint8_t len[4];
    w32(len, burstLen[slot]);
    server.sendContent((const char*)len, 4);
    server.sendContent((const char*)(burstBuf + (size_t)slot * BURST_FRAME_MAX), burstLen[slot]);
  }
}

static void handleRecord() {
  const bool on = (server.arg("on") == "1");
  if (on) {
    if (!startRawRecording()) { cors(); server.send(500, "text/plain", lastError); return; }
  } else {
    stopRawRecording();
  }
  cors();
  server.send(200, "text/plain", recOn ? "recording" : "stopped");
}

static void handleClear() {
  evtState   = EVT_WAITING;
  burstCount = 0;
  ringCount  = ringPos = 0;
  cors();
  server.send(200, "text/plain", "ok");
}

static void handleReference() {
  if (!captureReference()) { cors(); server.send(500, "text/plain", lastError); return; }
  cors();
  server.send(200, "text/plain", "ok");
}

static void handleThreshold() {
  const int v = server.arg("v").toInt();
  if (v >= 1 && v <= 254) threshold = (uint8_t)v;
  cors();
  server.send(200, "text/plain", String(threshold));
}

static void handleTrigger() {
  const float v = server.arg("v").toFloat();
  if (v > 0.0f && v <= 100.0f) movePct = v;
  cors();
  server.send(200, "text/plain", String(movePct, 2));
}

/* The percentage below which the scene counts as settled: it is what
 * closes an event, so a hand held still never keeps firing. */
static void handleCalm() {
  const float v = server.arg("v").toFloat();
  if (v >= 0.0f && v <= 100.0f) calmPct = v;
  cors();
  server.send(200, "text/plain", String(calmPct, 2));
}

/* How long the movement must last before it is worth an event. */
static void handleMinActive() {
  const long v = server.arg("v").toInt();
  if (v >= 0 && v <= 60000) minActiveMs = (uint32_t)v;
  cors();
  server.send(200, "text/plain", String(minActiveMs));
}

/* Alignment only: one JPEG, software encoded, meant to be asked for
 * rarely. The live picture is never streamed. */
static void handleSnapshot() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { server.send(503, "text/plain", "capture failed"); return; }
  uint8_t* jpg = nullptr; size_t len = 0;
  const bool ok = fmt2jpg(fb->buf, fb->len, fb->width, fb->height,
                          PIXFORMAT_GRAYSCALE, PREVIEW_QUALITY, &jpg, &len);
  esp_camera_fb_return(fb);
  if (!ok) { server.send(503, "text/plain", "encode failed"); return; }
  cors();
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(len);
  server.send(200, "image/jpeg", "");
  server.sendContent((const char*)jpg, len);
  free(jpg);
}

/* The live mask as run-length, for aiming and tuning the threshold.
 * Asked for on demand, never pushed. */
static void handleMask() {
  if (!haveRef) { cors(); server.send(409, "text/plain", "no reference"); return; }
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { server.send(503, "text/plain", "capture failed"); return; }
  if (!ensureBuffers(fb)) { esp_camera_fb_return(fb); server.send(500, "text/plain", "no memory"); return; }
  refPct = buildMask(fb);   // on-demand view: does not touch the motion figure
  esp_camera_fb_return(fb);

  const size_t len = encodeRuns(maskBuf);
  uint8_t head[5];
  w16(head, grayW); w16(head + 2, grayH); head[4] = 1;
  cors();
  server.setContentLength(5 + len);
  server.send(200, "application/octet-stream", "");
  server.sendContent((const char*)head, 5);
  server.sendContent((const char*)rleBuf, len);
}

static void handleFiles() {
  cors();
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  server.sendContent("[");
  if (sdReady) {
    File root = SD_MMC.open("/");
    bool first = true;
    File f = root.openNextFile();
    while (f) {
      String n = f.name();
      if (n.startsWith("/")) n = n.substring(1);
      if (n.endsWith(".avi") || n.endsWith(".gray")) {
        char row[96];
        snprintf(row, sizeof(row), "%s{\"n\":\"%s\",\"s\":%u}",
                 first ? "" : ",", n.c_str(), (unsigned)f.size());
        server.sendContent(row);
        first = false;
      }
      f = root.openNextFile();
    }
    root.close();
  }
  server.sendContent("]");
  server.sendContent("");
}

static void handleDownload() {
  String f = server.arg("f");
  if (!sdReady || !f.startsWith("/") || f.indexOf("..") >= 0 || !SD_MMC.exists(f)) {
    server.send(404, "text/plain", "not found");
    return;
  }
  File file = SD_MMC.open(f, FILE_READ);
  if (!file) { server.send(500, "text/plain", "cannot open"); return; }

  const size_t total = file.size();
  server.sendHeader("Content-Disposition", "attachment; filename=" + f.substring(1));
  server.sendHeader("Connection", "close");
  cors();
  server.setContentLength(total);
  server.send(200, "application/octet-stream", "");

  WiFiClient client = server.client();
  static uint8_t buf[4096];
  size_t sent = 0;
  while (sent < total && client.connected()) {
    const size_t n = file.read(buf, sizeof(buf));
    if (n == 0) break;
    size_t off = 0;
    while (off < n && client.connected()) {
      const size_t w = client.write(buf + off, n - off);
      if (w == 0) { delay(1); continue; }
      off += w;
    }
    sent += off;
    delay(0);
  }
  file.close();
  Serial.printf("[HTTP] %s: %u/%u bytes%s\n", f.c_str(), (unsigned)sent,
                (unsigned)total, sent == total ? "" : "  <-- TRUNCATED");
}

static void handleDelete() {
  String f = server.arg("f");
  if (sdReady && f.startsWith("/") && f.indexOf("..") < 0 && SD_MMC.exists(f)) SD_MMC.remove(f);
  cors();
  server.send(200, "text/plain", "ok");
}

/* ----------------------------- setup --------------------------------- */
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[BOOT] ESP32-WROVER change detector");

  if (!psramFound()) Serial.println("[BOOT] WARNING: no PSRAM detected");
  if (!initCamera()) {
    Serial.println("[BOOT] camera init failed - halting");
    while (true) delay(1000);
  }
  sdReady = initSD();

#if AP_ONLY
  WiFi.mode(WIFI_AP);
#elif AP_ENABLED
  WiFi.mode(WIFI_AP_STA);
#else
  WiFi.mode(WIFI_STA);
#endif
  WiFi.setSleep(false);
  WiFi.setHostname(HOSTNAME);

#if AP_ENABLED || AP_ONLY
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("[AP] \"%s\" / \"%s\"  ->  http://%s\n",
                AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
#endif

#if !AP_ONLY
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("[WIFI] connecting");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(400);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    esp_wifi_set_ps(WIFI_PS_NONE);     // must be after association to stick
    wifi_ps_type_t ps = WIFI_PS_NONE;
    esp_wifi_get_ps(&ps);
    Serial.print("[WIFI] open http://");
    Serial.println(WiFi.localIP());
    Serial.printf("[WIFI] RSSI %d dBm, ps=%d (0 = none)\n", WiFi.RSSI(), (int)ps);
  } else {
    Serial.println("[WIFI] hotspot not joined - the board's own AP still works");
  }
#endif

  server.on("/",          handleRoot);
  server.on("/motion",    handleMotion);
  server.on("/burst",     handleBurst);
  server.on("/record",    handleRecord);
  server.on("/clear",     handleClear);
  server.on("/mask",      handleMask);
  server.on("/snapshot",  handleSnapshot);
  server.on("/reference", handleReference);
  server.on("/threshold", handleThreshold);
  server.on("/trigger",   handleTrigger);
  server.on("/calm",      handleCalm);
  server.on("/minactive", handleMinActive);
  server.on("/files",     handleFiles);
  server.on("/download",  handleDownload);
  server.on("/delete",    handleDelete);
  server.onNotFound([]() { server.send(404, "text/plain", "404"); });
  server.begin();
  Serial.println("[HTTP] server started on port 80");
}

void loop() {
  server.handleClient();
  motionTick();

  static uint32_t lastWifiCheck = 0;
  if (millis() - lastWifiCheck > 10000) {
    lastWifiCheck = millis();
#if !AP_ONLY
    if (WiFi.status() != WL_CONNECTED) WiFi.reconnect();
    else esp_wifi_set_ps(WIFI_PS_NONE);   // a reassociation restores the default
#endif
  }
}
