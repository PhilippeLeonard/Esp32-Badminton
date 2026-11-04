/* Freenove ESP32-WROVER CAM - detecteur de changement.
 *
 * IDE : carte "ESP32 Wrover Module", PSRAM Enabled,
 *       Partition Scheme "Huge APP (3MB No OTA)".
 */

#include "esp_camera.h"
#include "img_converters.h"
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <esp_wifi.h>
#include <ESPmDNS.h>
#include "FS.h"
#include "SD_MMC.h"
#include "app_page.h"       // generated from phone_app.html by make_page.py

/* La page compare cette version a la sienne et affiche les deux quand
 * elles different : il faut alors relancer make_page.py. */
#define FW_VERSION "1.3.0"

/* ------------------------- 1. Reseau -------------------------------- */
static const char* WIFI_SSID = "Philphone";   // hotspot 2,4 GHz
static const char* WIFI_PASS = "12345678";
static const char* HOSTNAME  = "esp32cam";    // http://esp32cam.local

/* AP et station partagent la radio : des que la station s'associe, l'AP
 * suit le canal du hotspot et ejecte le telephone deja connecte. */
#define AP_ENABLED   0
#define AP_ONLY      0
#define STA_RETRY_MS 20000            // intervalle entre deux tentatives
static const char* AP_SSID = "ESP32-CAM";
static const char* AP_PASS = "camera123";     // au moins 8 caracteres

/* Vide ou injoignable : la carte retombe sur l'ecriture SD. */
static char SERVER_URL[96] = "";              // ex: http://192.168.4.2:8000

/* ------------------------- 2. Detection ----------------------------- */
#define FRAME_SIZE        FRAMESIZE_QVGA  // 320x240, chaque pixel est compare
#define MOTION_HZ         10              // comparaisons par seconde
#define MOTION_THRESHOLD  30              // |courant - reference| par pixel, 0..255
#define REF_AVERAGE       8               // images moyennees dans la reference

/* ------------------------- 3. Forme d'un evenement ------------------ *
 * Ce qui declenche est une SEQUENCE : du mouvement soutenu, PUIS un
 * retour au calme. Le mouvement seul declencherait sans fin sur une main
 * laissee dans le champ.                                               */
#define MOVE_PERCENT      3.0f            // % de pixels changes = mouvement
#define CALM_PERCENT      0.8f            // % sous lequel la scene est calme
#define MIN_ACTIVE_MS     5000            // duree minimale du mouvement
#define PCT_AVERAGE       3               // mesures moyennees dans le chiffre utilise
#define CALM_CYCLES       8               // cycles calmes qui ferment un evenement
#define REARM_CYCLES      10              // cycles calmes avant un nouvel evenement

/* ------------------------- 4. Rafale -------------------------------- */
#define BURST_FRAMES      20              // images gardees par evenement
#define BURST_FPS         10
#define BURST_FRAME_MAX   24576           // PSRAM reservee par image de rafale
#define BURST_QUALITY     8               // qualite JPEG des images stockees

/* ------------------------- 5. Broches camera ------------------------ */
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
/* 10 MHz et pas 20 : a 20 MHz le DMA ne suit pas les 150 ko d'une trame
 * RGB565 et le driver crache EV-EOF-OVF en continu. */
#define XCLK_FREQ_HZ     10000000
#define PREVIEW_QUALITY  12               // qualite JPEG de /snapshot seulement

/* ------------------------- 6. Enregistrement brut ------------------- *
 * Gris 8 bits sans codec : 768 ko/s en QVGA a 10 i/s.                  */
#define REC_FPS           10
static const char REC_MAGIC[4] = { 'E', 'S', 'P', 'G' };

/* ------------------------- 7. Carte SD ------------------------------ */
/* Broches fixees par l'IO-MUX : CLK 14, CMD 15, D0 2. Le mode 1 bit
 * libere GPIO4, partage avec la LED et Y2 de la camera. */
#define SD_ONE_BIT_MODE   true
#define SD_MOUNT_POINT    "/sdcard"

WebServer server(80);

static uint32_t  staLastTry = 0;       // date de la derniere tentative
static bool      staWasUp   = false;   // etat annonce au dernier passage

static uint8_t*  refBuf   = nullptr;   // l'image de reference
static uint8_t*  maskBuf  = nullptr;   // masque de travail, 0x00 ou 0xFF
static uint8_t*  lumaBuf  = nullptr;   // luminance de l'image courante
static uint8_t*  prevBuf  = nullptr;   // image precedente, pour le mouvement
static uint8_t*  rleBuf   = nullptr;   // sortie de l'encodeur
static size_t    rleCap   = 0;
static bool      havePrev = false;
static size_t    grayLen  = 0;
static uint16_t  grayW = 0, grayH = 0;

static int16_t  zoneX[4] = {0}, zoneY[4] = {0};
static uint16_t zoneBand = 40;
static bool     zoneSet  = false;
static uint8_t* zoneMap  = nullptr;    // 0 dehors, 1 terrain, 2 marge

/* Pic de l'evenement en cours, et centre des pixels changes a cet
 * instant : c'est lui qui situe l'impact. */
static float    peakIn = 0.0f, peakBand = 0.0f;
static int16_t  peakCx = -1, peakCy = -1;
/* Les memes, figes a la fermeture : ce sont ceux qui sont envoyes. */
static float    pctIn = 0.0f, pctBand = 0.0f;
static int16_t  maskCx = -1, maskCy = -1;

static bool     cameraOk       = false;
static bool     haveRef        = false;
static bool     exposureLocked = false;
static uint8_t  threshold      = MOTION_THRESHOLD;
static float    movePct        = MOVE_PERCENT;
static float    calmPct        = CALM_PERCENT;
static uint32_t minActiveMs    = MIN_ACTIVE_MS;
static float    lastPct        = 0.0f;   // mouvement lisse
static float    rawPct         = 0.0f;   // le meme chiffre, brut
static float    pctHist[PCT_AVERAGE] = {0};
static uint8_t  pctCount       = 0;
static uint8_t  pctIndex       = 0;
static uint32_t nextMotionMs   = 0;

/* --- machine a etats de l'evenement ---
 * WAITING : rien en cours, ou retour au calme apres un evenement.
 * ACTIVE  : du mouvement est en cours, les images entrent dans l'anneau.
 * READY   : un evenement s'est ferme, ses dernieres images sont la.    */
enum EvtState { EVT_WAITING = 0, EVT_ACTIVE = 1, EVT_READY = 2 };
static EvtState evtState    = EVT_WAITING;
static uint32_t activeStart = 0;         // millis() au debut du mouvement
static uint32_t activeMs    = 0;         // duree de l'evenement ferme
static uint16_t calmRun     = 0;         // cycles calmes consecutifs
static uint16_t rearmLeft   = 0;         // cycles calmes avant de rearmer

static uint8_t*   burstBuf   = nullptr;          // BURST_FRAMES * BURST_FRAME_MAX
static uint32_t   burstLen[BURST_FRAMES] = {0};
static uint8_t    ringPos    = 0;                // prochaine case a ecrire
static uint8_t    ringCount  = 0;                // cases remplies
static uint8_t    burstCount = 0;                // images de la rafale figee
static uint8_t    burstOrder[BURST_FRAMES] = {0};// cases, la plus vieille d'abord
static uint32_t   eventCount = 0;
static bool       lastSendOk = false;
static uint8_t    forceLeft  = 0;      // images restantes d'une rafale forcee
static char       burstFile[32] = {0};

static bool     sdReady  = false;
static String   lastError = "";

/* Prototypes : l'IDE Arduino en genere, mais pas toujours dans l'ordre. */
static void toLuma(camera_fb_t* fb);
static void buildMask();
static void buildZoneMap();
static bool ensureBuffers(camera_fb_t* fb);

static File     recFile;
static bool     recOn     = false;
static uint32_t recFrames = 0;
static uint32_t recNextMs = 0;
static char     recPath[32] = {0};

/* ---------------------- ecriture little-endian ----------------------- */
static inline void w32(uint8_t* p, uint32_t v) {
  p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}
static inline void w16(uint8_t* p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static inline void tag(uint8_t* p, const char* s) { memcpy(p, s, 4); }

/* ------------------------- AVI (MJPEG) ------------------------------- */
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
  c.pixel_format = PIXFORMAT_RGB565;      // couleur, et acces direct aux pixels
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
    /* L'automatique reste actif jusqu'a la prise de reference. */
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_gain_ctrl(s, 1);
  }
  exposureLocked = false;
  return true;
}

/* Fige le capteur sur ce que l'automatique a trouve : sinon il re-expose
 * entre la reference et l'image courante, tous les pixels bougent d'un
 * coup et le masque devient tout noir. */
static void lockExposure(bool on) {
  sensor_t* s = esp_camera_sensor_get();
  if (!s) return;
  if (on) {
    const int ae = s->status.aec_value;
    const int ag = s->status.agc_gain;

    /* Reimposer une exposition nulle coupe le temps de pose et rend
     * l'image entierement noire. */
    if (ae <= 0) {
      Serial.println("[CAM] exposition illisible, on reste en automatique");
      exposureLocked = false;
      return;
    }

    s->set_exposure_ctrl(s, 0);
    s->set_gain_ctrl(s, 0);
    s->set_whitebal(s, 0);
    s->set_awb_gain(s, 0);
    s->set_aec_value(s, ae);
    s->set_agc_gain(s, ag);

    /* Image noire apres verrouillage : rendre la main a l'automatique
     * plutot que d'enregistrer du vide. */
    delay(120);
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      uint32_t sum = 0;
      const size_t step = fb->len > 4096 ? fb->len / 4096 : 1;
      size_t n = 0;
      for (size_t i = 0; i < fb->len; i += step) { sum += fb->buf[i]; n++; }
      const uint32_t moy = n ? sum / n : 0;
      esp_camera_fb_return(fb);
      if (moy < 8) {
        Serial.printf("[CAM] image noire apres verrouillage (moy=%u), retour auto\n",
                      (unsigned)moy);
        s->set_exposure_ctrl(s, 1);
        s->set_gain_ctrl(s, 1);
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
        exposureLocked = false;
        return;
      }
    }
    Serial.printf("[CAM] exposure locked (aec=%d gain=%d)\n", ae, ag);
  } else {
    s->set_exposure_ctrl(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
  }
  exposureLocked = on;
}

/* ------------------------- tampons ---------------------------------- */
static void freeBuffers() {
  free(refBuf);   refBuf   = nullptr;
  free(maskBuf);  maskBuf  = nullptr;
  free(lumaBuf);  lumaBuf  = nullptr;
  free(prevBuf);  prevBuf  = nullptr;
  free(zoneMap);  zoneMap  = nullptr;
  free(rleBuf);   rleBuf   = nullptr;
  free(burstBuf); burstBuf = nullptr;
  haveRef = havePrev = false;
  /* zoneMap avait la geometrie d'avant : la garder "definie" ferait lire
   * une table qui ne correspond plus a l'image. */
  zoneSet   = false;
  ringCount = ringPos = burstCount = 0;
  evtState  = EVT_WAITING;
}

static bool ensureBuffers(camera_fb_t* fb) {
  if (grayLen == (size_t)fb->width * fb->height && refBuf) return true;

  freeBuffers();

  grayW = fb->width; grayH = fb->height;
  grayLen = (size_t)grayW * grayH;         // la luminance, pas la trame RGB565
  rleCap  = grayLen * 2 + 16;              // pire cas : un pixel sur deux

  refBuf   = (uint8_t*)ps_malloc(grayLen);
  maskBuf  = (uint8_t*)ps_malloc(grayLen);
  lumaBuf  = (uint8_t*)ps_malloc(grayLen);
  prevBuf  = (uint8_t*)ps_malloc(grayLen);
  zoneMap  = (uint8_t*)ps_malloc(grayLen);
  rleBuf   = (uint8_t*)ps_malloc(rleCap);
  burstBuf = (uint8_t*)ps_malloc((size_t)BURST_FRAMES * BURST_FRAME_MAX);

  if (!refBuf || !maskBuf || !lumaBuf || !prevBuf || !zoneMap || !rleBuf || !burstBuf) {
    /* Tout ou rien : une allocation partielle laisserait des pointeurs
     * nuls lus plus loin comme s'ils etaient valides. */
    freeBuffers();
    lastError = "PSRAM insuffisante";
    Serial.println("[MEM] allocation impossible");
    grayLen = 0; grayW = grayH = 0;
    return false;
  }
  memset(zoneMap, 0, grayLen);
  Serial.printf("[MEM] %ux%u pret, %u ko de PSRAM libre\n",
                grayW, grayH, (unsigned)(ESP.getFreePsram() / 1024));
  return true;
}

static bool captureReference() {
  lockExposure(true);
  delay(150);                              // laisser les reglages s'appliquer

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { lastError = "capture camera impossible"; return false; }
  if (!ensureBuffers(fb)) { esp_camera_fb_return(fb); return false; }
  /* RGB565 = deux octets par pixel ; plus court, toLuma lirait dehors. */
  if (fb->len < grayLen * 2) {
    esp_camera_fb_return(fb);
    lastError = "trame camera incomplete";
    return false;
  }

  /* En PSRAM : 320x240 uint32 fait 300 ko, hors de la DRAM interne. */
  uint32_t* acc = (uint32_t*)ps_malloc(grayLen * sizeof(uint32_t));
  if (!acc) { esp_camera_fb_return(fb); lastError = "PSRAM insuffisante"; return false; }

  toLuma(fb);
  for (size_t i = 0; i < grayLen; i++) acc[i] = lumaBuf[i];
  esp_camera_fb_return(fb);

  uint8_t used = 1;
  for (int k = 1; k < REF_AVERAGE; k++) {
    delay(40);
    camera_fb_t* f = esp_camera_fb_get();
    if (!f) continue;
    if ((size_t)f->width * f->height == grayLen && f->len >= grayLen * 2) {
      toLuma(f);
      for (size_t i = 0; i < grayLen; i++) acc[i] += lumaBuf[i];
      used++;
    }
    esp_camera_fb_return(f);
  }
  for (size_t i = 0; i < grayLen; i++) refBuf[i] = (uint8_t)(acc[i] / used);
  free(acc);

  haveRef   = true;
  pctCount  = pctIndex = 0;           // la moyenne ne doit pas se reporter
  havePrev  = false;                  // ni l'image precedente
  evtState  = EVT_WAITING;
  rearmLeft = 0;
  ringCount = ringPos = burstCount = 0;
  peakIn = peakBand = 0.0f;
  peakCx = peakCy = -1;
  lastError = "";
  Serial.printf("[REF] reference moyennee sur %u images (%ux%u)\n",
                (unsigned)used, grayW, grayH);
  return true;
}

/* RGB565 -> luminance : la detection ne travaille que la-dessus. */
static void toLuma(camera_fb_t* fb) {
  const uint8_t* p = fb->buf;
  for (size_t i = 0; i < grayLen; i++) {
    const uint16_t px = (uint16_t)(p[i * 2] << 8) | p[i * 2 + 1];
    const uint8_t r = (px >> 11) & 0x1F, g = (px >> 5) & 0x3F, b = px & 0x1F;
    lumaBuf[i] = (uint8_t)(((r << 3) * 77 + (g << 2) * 150 + (b << 3) * 29) >> 8);
  }
}

/* ------------------------------ terrain ------------------------------ *
 * Quatre coins quelconques, pas un rectangle : la camera voit le terrain
 * en perspective. zoneMap classe chaque pixel une fois par changement de
 * terrain : 0 dehors, 1 terrain, 2 marge.                              */

static bool pointInQuad(float px, float py) {
  bool in = false;
  for (int i = 0, j = 3; i < 4; j = i++) {
    const float xi = zoneX[i], yi = zoneY[i], xj = zoneX[j], yj = zoneY[j];
    if (((yi > py) != (yj > py)) &&
        (px < (xj - xi) * (py - yi) / (yj - yi + 1e-6f) + xi)) in = !in;
  }
  return in;
}

static float distToSegment(float px, float py, float ax, float ay, float bx, float by) {
  const float dx = bx - ax, dy = by - ay;
  const float len2 = dx * dx + dy * dy;
  float t = len2 > 0 ? ((px - ax) * dx + (py - ay) * dy) / len2 : 0.0f;
  t = t < 0 ? 0 : (t > 1 ? 1 : t);
  const float qx = ax + t * dx - px, qy = ay + t * dy - py;
  return sqrtf(qx * qx + qy * qy);
}

static void buildZoneMap() {
  if (!zoneMap || !grayW || !grayH) return;
  for (uint16_t y = 0; y < grayH; y++) {
    for (uint16_t x = 0; x < grayW; x++) {
      uint8_t cls = 0;
      if (pointInQuad(x, y)) {
        cls = 1;
      } else {
        float best = 1e9f;
        for (int i = 0, j = 3; i < 4; j = i++) {
          const float d = distToSegment(x, y, zoneX[j], zoneY[j], zoneX[i], zoneY[i]);
          if (d < best) best = d;
        }
        if (best <= zoneBand) cls = 2;
      }
      zoneMap[(size_t)y * grayW + x] = cls;
    }
  }
  Serial.printf("[ZONE] carte calculee, marge %u px\n", zoneBand);
}

/* Pixels qui different de l'image PRECEDENTE, en %. Compare a la
 * reference, une main immobile resterait haute et l'evenement ne se
 * fermerait jamais ; image a image elle lit 0 %. */
static float interFrameMotion() {
  if (!havePrev) {
    memcpy(prevBuf, lumaBuf, grayLen);
    havePrev = true;
    return 0.0f;
  }
  const uint8_t thr = threshold;
  uint32_t moved = 0;
  for (size_t i = 0; i < grayLen; i++) {
    const int d = (int)lumaBuf[i] - (int)prevBuf[i];
    if ((d < 0 ? -d : d) > thr) moved++;
  }
  memcpy(prevBuf, lumaBuf, grayLen);
  return 100.0f * (float)moved / (float)grayLen;
}

/* Remplit maskBuf : 0x00 change, 0xFF inchange. */
static void buildMask() {
  if (!haveRef || !maskBuf || !lumaBuf || !refBuf) return;

  const uint8_t thr = threshold;
  for (size_t i = 0; i < grayLen; i++) {
    const int d = (int)lumaBuf[i] - (int)refBuf[i];
    maskBuf[i] = ((d < 0 ? -d : d) > thr) ? 0x00 : 0xFF;
  }
}

/* Retient le pic de l'evenement. Les pourcentages sont rapportes aux
 * pixels de chaque zone, pas a l'image entiere, pour rester comparables
 * d'un cadrage a l'autre. */
static void measureZonePeak() {
  if (!zoneSet || !zoneMap || !maskBuf) return;

  uint32_t nIn = 0, nBand = 0, totIn = 0, totBand = 0;
  uint32_t sx = 0, sy = 0;
  for (size_t i = 0; i < grayLen; i++) {
    const uint8_t cls = zoneMap[i];
    if (!cls) continue;
    if (cls == 1) totIn++; else totBand++;
    if (maskBuf[i]) continue;                 // 0xFF = inchange
    if (cls == 1) nIn++; else nBand++;
    sx += i % grayW;
    sy += i / grayW;
  }
  const float in   = totIn   ? 100.0f * (float)nIn   / (float)totIn   : 0.0f;
  const float band = totBand ? 100.0f * (float)nBand / (float)totBand : 0.0f;
  if (in <= peakIn) return;

  peakIn   = in;
  peakBand = band;
  const uint32_t n = nIn + nBand;
  peakCx = n ? (int16_t)(sx / n) : -1;
  peakCy = n ? (int16_t)(sy / n) : -1;
}

/* ---------------------- encodage ------------------------------------- */
/* Paires [longueur][0|1], de haut en bas, sans marqueur. */
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
    lastError = "carte SD non montee (inseree ? formatee en FAT32 ?)";
    Serial.println("[SD] montage impossible");
    return false;
  }
  if (SD_MMC.cardType() == CARD_NONE) {
    lastError = "aucune carte SD detectee";
    SD_MMC.end();
    return false;
  }
  Serial.printf("[SD] mounted: %llu MB total, %llu MB used\n",
                SD_MMC.totalBytes() / (1024ULL * 1024ULL),
                SD_MMC.usedBytes()  / (1024ULL * 1024ULL));
  return true;
}

/* Premier nom libre /prefixe_NNNN.suffixe ; false si la serie est pleine,
 * sinon le dernier nom serait ecrase en silence. */
static bool freeName(char* out, size_t cap, const char* prefix, const char* suffix) {
  for (uint16_t i = 1; i <= 9999; i++) {
    snprintf(out, cap, "/%s_%04u.%s", prefix, i, suffix);
    if (!SD_MMC.exists(out)) return true;
  }
  out[0] = 0;
  return false;
}

/* Les JPEG de la rafale vont dans le fichier tels quels. L'entete est
 * ecrite une seule fois : les tailles sont connues avant d'ecrire. */
static void writeBurstAvi() {
  if (!sdReady || !burstCount || !burstBuf) return;

  uint32_t movi = 0, maxFrame = 0;
  for (uint8_t k = 0; k < burstCount; k++) {
    const uint32_t len = burstLen[burstOrder[k]];
    movi += 8 + len + (len & 1);
    if (len > maxFrame) maxFrame = len;
  }

  char path[32];
  if (!freeName(path, sizeof(path), "evt", "avi")) {
    lastError = "plus de nom libre pour un evt_NNNN.avi";
    return;
  }

  uint8_t* idx = (uint8_t*)ps_malloc(16UL * burstCount);
  if (!idx) { lastError = "PSRAM insuffisante"; return; }

  File f = SD_MMC.open(path, FILE_WRITE);
  if (!f) { free(idx); lastError = String("creation impossible : ") + path; return; }

  uint8_t hdr[AVI_HEADER_BYTES];
  buildAviHeader(hdr, grayW, grayH, BURST_FPS, burstCount, movi, maxFrame);
  bool ok = f.write(hdr, AVI_HEADER_BYTES) == AVI_HEADER_BYTES;

  uint32_t off = 0;
  for (uint8_t k = 0; k < burstCount && ok; k++) {
    const uint8_t  slot = burstOrder[k];
    const uint8_t* src  = burstBuf + (size_t)slot * BURST_FRAME_MAX;
    const uint32_t len  = burstLen[slot];
    const uint8_t  pad  = (len & 1) ? 1 : 0;      // un bloc RIFF est aligne

    uint8_t ch[8];
    tag(ch, "00dc"); w32(ch + 4, len);
    ok = f.write(ch, 8) == 8 && f.write(src, len) == len;
    if (ok && pad) { uint8_t z = 0; ok = f.write(&z, 1) == 1; }

    uint8_t* e = idx + 16UL * k;
    tag(e, "00dc"); w32(e + 4, 0x10); w32(e + 8, 4 + off); w32(e + 12, len);
    off += 8 + len + pad;
  }

  if (ok) {
    uint8_t ih[8];
    tag(ih, "idx1"); w32(ih + 4, 16UL * burstCount);
    ok = f.write(ih, 8) == 8
      && f.write(idx, 16UL * burstCount) == 16UL * burstCount;
  }
  f.close();
  free(idx);

  if (!ok) {
    SD_MMC.remove(path);
    lastError = "ecriture SD echouee (carte pleine ?)";
    Serial.printf("[EVT] %s abandonne, ecriture SD echouee\n", path);
    return;
  }
  strncpy(burstFile, path, sizeof(burstFile) - 1);
  burstFile[sizeof(burstFile) - 1] = 0;
  Serial.printf("[EVT] %s ecrit, %u images\n", path, burstCount);
}

/* ------------------------ enregistrement brut ------------------------ *
 * Entete de 16 octets, little-endian, puis les images bout a bout :
 *   0  "ESPG"    4 octets
 *   4  largeur   uint16
 *   6  hauteur   uint16
 *   8  images/s  uint16
 *  10  (inutilise) uint16
 *  12  images    uint32   - ecrit a 0, corrige a l'arret
 *  16  image 0, image 1, ... largeur*hauteur octets de gris 8 bits
 * tools/replay.py lit exactement ceci.                                   */
static bool startRawRecording() {
  if (recOn) return true;
  if (!sdReady)             { lastError = "carte SD non prete"; return false; }
  if (!grayW || !grayLen)   { lastError = "camera pas encore prete"; return false; }

  if (!freeName(recPath, sizeof(recPath), "raw", "gray")) {
    lastError = "plus de nom libre pour un raw_NNNN.gray";
    return false;
  }
  recFile = SD_MMC.open(recPath, FILE_WRITE);
  if (!recFile) { lastError = String("creation impossible : ") + recPath; return false; }

  uint8_t h[16] = {0};
  memcpy(h, REC_MAGIC, 4);
  w16(h + 4, grayW);
  w16(h + 6, grayH);
  w16(h + 8, REC_FPS);
  w32(h + 12, 0);                      // corrige a l'arret
  if (recFile.write(h, 16) != 16) {
    recFile.close();
    SD_MMC.remove(recPath);
    recPath[0] = 0;
    lastError = "ecriture SD echouee sur l'entete";
    return false;
  }
  recFile.flush();

  recFrames = 0;
  recNextMs = millis();
  recOn = true;
  lastError = "";
  Serial.printf("[RAW] enregistre -> %s (%ux%u, %d i/s)\n", recPath, grayW, grayH, REC_FPS);
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
  } else {
    Serial.println("[RAW] nombre d'images non corrige dans l'entete");
  }
  Serial.printf("[RAW] arret : %s, %u images, %.1f Mo\n", recPath, (unsigned)recFrames,
                (16.0 + (double)recFrames * grayLen) / 1048576.0);
}

static void recordRaw() {
  if (!recOn) return;
  const uint32_t now = millis();
  if ((int32_t)(now - recNextMs) < 0) return;
  recNextMs = now + 1000UL / REC_FPS;

  if (!lumaBuf || !grayLen) { stopRawRecording(); return; }
  if (recFile.write(lumaBuf, grayLen) != grayLen) {
    lastError = "ecriture SD echouee (carte pleine ?)";
    Serial.println("[RAW] ecriture SD echouee - carte pleine ?");
    stopRawRecording();
    return;
  }
  recFrames++;
  if ((recFrames & 0x1F) == 0) recFile.flush();
}

/* ------------------------ boucle de detection ------------------------ */
static void ringPush(camera_fb_t* fb) {
  if (!burstBuf) return;
  uint8_t* jpg = nullptr;
  size_t   len = 0;
  if (!fmt2jpg(fb->buf, fb->len, fb->width, fb->height,
               PIXFORMAT_RGB565, BURST_QUALITY, &jpg, &len)) return;

  if (len && len <= BURST_FRAME_MAX) {
    memcpy(burstBuf + (size_t)ringPos * BURST_FRAME_MAX, jpg, len);
    burstLen[ringPos] = len;
    ringPos = (ringPos + 1) % BURST_FRAMES;
    if (ringCount < BURST_FRAMES) ringCount++;
  }
  free(jpg);
}

/* Envoie la rafale en un seul POST : [u16 w][u16 h][u8 n] puis
 * [u32 longueur][jpeg] par image. Les mesures voyagent dans l'adresse. */
static bool postBurst() {
  if (!SERVER_URL[0] || !burstCount || !burstBuf) return false;
  /* Sans lien, HTTPClient bloquerait la boucle de detection pendant son
   * delai de connexion : on tombe tout de suite sur la SD. */
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[CLIP] pas de lien wifi, ecriture SD");
    return false;
  }

  uint32_t total = 5;
  for (uint8_t k = 0; k < burstCount; k++) total += 4 + burstLen[burstOrder[k]];

  uint8_t* body = (uint8_t*)ps_malloc(total);
  if (!body) return false;

  size_t o = 0;
  w16(body + o, grayW); o += 2;
  w16(body + o, grayH); o += 2;
  body[o++] = burstCount;
  for (uint8_t k = 0; k < burstCount; k++) {
    const uint8_t slot = burstOrder[k];
    w32(body + o, burstLen[slot]); o += 4;
    memcpy(body + o, burstBuf + (size_t)slot * BURST_FRAME_MAX, burstLen[slot]);
    o += burstLen[slot];
  }

  char url[256];
  snprintf(url, sizeof(url),
           "%s/clip?w=%u&h=%u&n=%u&in=%.2f&band=%.2f&cx=%d&cy=%d&ms=%u",
           SERVER_URL, (unsigned)grayW, (unsigned)grayH, (unsigned)burstCount,
           pctIn, pctBand, (int)maskCx, (int)maskCy, (unsigned)activeMs);

  HTTPClient http;
  http.setConnectTimeout(2500);
  http.setTimeout(8000);
  http.setReuse(false);
  bool ok = false;
  if (http.begin(url)) {
    http.addHeader("Content-Type", "application/octet-stream");
    const int code = http.POST(body, total);
    ok = (code >= 200 && code < 300);
    Serial.printf("[CLIP] POST %s -> %d (%u o)\n", url, code, (unsigned)total);
    http.end();
  } else {
    Serial.printf("[CLIP] adresse de serveur inutilisable : %s\n", url);
  }
  free(body);
  return ok;
}

static void closeEvent() {
  burstCount = ringCount;
  const uint8_t first = (uint8_t)((ringPos + BURST_FRAMES - ringCount) % BURST_FRAMES);
  for (uint8_t i = 0; i < burstCount; i++)
    burstOrder[i] = (uint8_t)((first + i) % BURST_FRAMES);

  /* Le pic de l'evenement, pas la scene calme qui vient de le fermer. */
  pctIn = peakIn; pctBand = peakBand;
  maskCx = peakCx; maskCy = peakCy;
  peakIn = peakBand = 0.0f;
  peakCx = peakCy = -1;

  evtState = EVT_READY;
  eventCount++;
  rearmLeft = REARM_CYCLES;
  burstFile[0] = 0;
  if (burstCount == 0) {
    lastSendOk = false;
    Serial.println("[EVT] aucune image a envoyer");
    return;
  }
  lastSendOk = postBurst();             // le serveur d'abord, sinon la SD
  if (!lastSendOk) writeBurstAvi();
  Serial.printf("[EVT] evenement %u : %.1f s de mouvement, %u images, pic %.2f %%\n",
                (unsigned)eventCount, activeMs / 1000.0f, burstCount, pctIn);
}

static void motionTick() {
  if (!cameraOk) return;
  const uint32_t now = millis();
  if ((int32_t)(now - nextMotionMs) < 0) return;
  nextMotionMs = now + 1000UL / MOTION_HZ;

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return;
  if (!ensureBuffers(fb)) { esp_camera_fb_return(fb); return; }
  if (fb->len < grayLen * 2) {        // trame RGB565 incomplete
    esp_camera_fb_return(fb);
    return;
  }

  toLuma(fb);
  recordRaw();                        // apres toLuma, qui remplit lumaBuf

  /* Avant le garde-fou : forcer un envoi sert a verifier la liaison sans
   * rien avoir regle. */
  if (forceLeft) {
    ringPush(fb);
    if (--forceLeft == 0) {
      activeMs = 0;
      closeEvent();
    }
    esp_camera_fb_return(fb);
    return;
  }

  /* Sans reference ET sans terrain la carte se contente d'enregistrer. */
  if (!haveRef || !zoneSet) {
    esp_camera_fb_return(fb);
    return;
  }

  /* Moyenne glissante : une seule mesure basse au milieu d'un echange
   * suffisait a commencer a fermer l'evenement. */
  rawPct = interFrameMotion();
  pctHist[pctIndex] = rawPct;
  pctIndex = (pctIndex + 1) % PCT_AVERAGE;
  if (pctCount < PCT_AVERAGE) pctCount++;
  float sum = 0.0f;
  for (uint8_t i = 0; i < pctCount; i++) sum += pctHist[i];
  lastPct = sum / pctCount;           // drives the state machine

  switch (evtState) {

    case EVT_ACTIVE:
      ringPush(fb);                      // garder le mouvement lui-meme
      buildMask();                       // puis relever le pic par zone
      measureZonePeak();
      if (lastPct < calmPct) {
        calmRun++;
        if (calmRun >= CALM_CYCLES) {
          /* Les cycles calmes ayant servi a constater la fin ne font pas
           * partie du mouvement. */
          const uint32_t calmMs = CALM_CYCLES * (1000UL / MOTION_HZ);
          const uint32_t ecoule = now - activeStart;
          activeMs = ecoule > calmMs ? ecoule - calmMs : 0;
          if (activeMs >= minActiveMs) {
            closeEvent();
          } else {
            Serial.printf("[EVT] ignore, seulement %.1f s de mouvement\n",
                          activeMs / 1000.0f);
            evtState  = EVT_WAITING;
            ringCount = ringPos = 0;
            peakIn = peakBand = 0.0f;
            peakCx = peakCy = -1;
          }
          calmRun = 0;
        }
      } else {
        calmRun = 0;                     // ca bouge encore
      }
      break;

    case EVT_READY:
      /* Attendre le retour au calme : un seul long mouvement ne doit pas
       * produire evenement apres evenement. */
      if (lastPct < calmPct) {
        if (rearmLeft) rearmLeft--;
      } else {
        rearmLeft = REARM_CYCLES;
      }
      break;

    case EVT_WAITING:
    default:
      if (rearmLeft) {
        if (lastPct < calmPct) rearmLeft--; else rearmLeft = REARM_CYCLES;
        break;
      }
      if (lastPct >= movePct) {
        evtState    = EVT_ACTIVE;
        activeStart = now;
        calmRun     = 0;
        ringCount   = ringPos = 0;
        burstFile[0] = 0;
        peakIn = peakBand = 0.0f;
        peakCx = peakCy = -1;
        Serial.printf("[EVT] mouvement a %.1f %%\n", lastPct);
        ringPush(fb);
      }
      break;
  }

  esp_camera_fb_return(fb);
}

/* ----------------------------- web ----------------------------------- */
static void cors() { server.sendHeader("Access-Control-Allow-Origin", "*"); }

static void handleRoot() { server.send_P(200, "text/html", APP_PAGE); }

/* Les guillemets d'un message d'erreur casseraient le JSON. */
static void jsonSafe(char* out, size_t cap, const char* in) {
  size_t o = 0;
  for (size_t i = 0; in[i] && o + 1 < cap; i++) {
    const char c = in[i];
    out[o++] = (c == '"' || c == '\\' || (unsigned char)c < 0x20) ? ' ' : c;
  }
  out[o] = 0;
}

static void handleMotion() {
  char err[96], rp[40], bf[40], srv[112];
  jsonSafe(err, sizeof(err), lastError.c_str());
  jsonSafe(rp,  sizeof(rp),  recPath);
  jsonSafe(bf,  sizeof(bf),  burstFile);
  jsonSafe(srv, sizeof(srv), SERVER_URL);

  /* Large : srv, recfile, file et err sont des chaines, et un JSON
   * tronque ferait echouer chaque interrogation de la page. */
  char j[900];
  snprintf(j, sizeof(j),
           "{\"ver\":\"" FW_VERSION "\",\"pct\":%.2f,\"rawpct\":%.2f,\"move\":%.2f,"
           "\"calm\":%.2f,\"minms\":%u,\"thr\":%u,\"ref\":%s,\"lock\":%s,"
           "\"zone\":%s,\"band\":%u,\"srv\":\"%s\",\"state\":%u,\"active\":%u,"
           "\"n\":%u,\"events\":%u,\"sent\":%s,\"in\":%.2f,\"band_pct\":%.2f,"
           "\"cx\":%d,\"cy\":%d,\"cam\":%s,\"sd\":%s,\"rec\":%s,\"recn\":%u,"
           "\"recfile\":\"%s\",\"w\":%u,\"h\":%u,\"file\":\"%s\",\"err\":\"%s\"}",
           lastPct, rawPct, movePct, calmPct, (unsigned)minActiveMs, (unsigned)threshold,
           haveRef ? "true" : "false",
           exposureLocked ? "true" : "false",
           zoneSet ? "true" : "false", (unsigned)zoneBand, srv,
           (unsigned)evtState,
           (unsigned)(evtState == EVT_ACTIVE ? (millis() - activeStart) : activeMs),
           (unsigned)burstCount, (unsigned)eventCount,
           lastSendOk ? "true" : "false",
           pctIn, pctBand, (int)maskCx, (int)maskCy,
           cameraOk ? "true" : "false",
           sdReady ? "true" : "false",
           recOn ? "true" : "false", (unsigned)recFrames, rp,
           (unsigned)grayW, (unsigned)grayH, bf, err);
  cors();
  server.send(200, "application/json", j);
}

/* [u16 w][u16 h][u8 n] puis [u32 longueur][jpeg] par image. */
static void handleBurst() {
  if (evtState != EVT_READY || !burstCount || !burstBuf) {
    cors(); server.send(409, "text/plain", "aucune rafale"); return;
  }

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
  rearmLeft  = 0;
  calmRun    = 0;
  forceLeft  = 0;
  burstFile[0] = 0;
  peakIn = peakBand = 0.0f;
  peakCx = peakCy = -1;
  cors();
  server.send(200, "text/plain", "ok");
}

static void handleReference() {
  if (!captureReference()) { cors(); server.send(500, "text/plain", lastError); return; }
  cors();
  server.send(200, "text/plain", "ok");
}

/* /zone?p=x0,y0,x1,y1,x2,y2,x3,y3&band=40 : les quatre coins dans l'ordre
 * du contour. Sans argument, renvoie le terrain courant. */
static void handleZone() {
  const String p = server.arg("p");
  if (p.length()) {
    if (!zoneMap || !grayW) {
      cors();
      server.send(409, "text/plain", "camera pas encore prete");
      return;
    }
    int v[8], n = 0, from = 0;
    while (n < 8) {
      const int c = p.indexOf(',', from);
      v[n++] = p.substring(from, c < 0 ? (int)p.length() : c).toInt();
      if (c < 0) break;
      from = c + 1;
    }
    if (n != 8) {
      cors();
      server.send(400, "text/plain", "il faut huit nombres");
      return;
    }
    /* Des coins hors cadre rendraient buildZoneMap faux sans rien dire. */
    for (int i = 0; i < 4; i++) {
      const int x = v[i * 2], y = v[i * 2 + 1];
      zoneX[i] = (int16_t)(x < 0 ? 0 : (x > grayW - 1 ? grayW - 1 : x));
      zoneY[i] = (int16_t)(y < 0 ? 0 : (y > grayH - 1 ? grayH - 1 : y));
    }
    if (server.hasArg("band")) {
      const int b = server.arg("band").toInt();
      if (b >= 0 && b <= 500) zoneBand = (uint16_t)b;
    }
    zoneSet = true;
    buildZoneMap();
  }
  char j[192];
  snprintf(j, sizeof(j),
           "{\"set\":%s,\"band\":%u,\"p\":[%d,%d,%d,%d,%d,%d,%d,%d]}",
           zoneSet ? "true" : "false", (unsigned)zoneBand,
           zoneX[0], zoneY[0], zoneX[1], zoneY[1],
           zoneX[2], zoneY[2], zoneX[3], zoneY[3]);
  cors();
  server.send(200, "application/json", j);
}

/* /force?n=10 : repond tout de suite, la capture se fait dans la boucle. */
static void handleForce() {
  if (!cameraOk || !burstBuf) {
    cors();
    server.send(409, "text/plain", "camera pas encore prete");
    return;
  }
  int n = server.hasArg("n") ? server.arg("n").toInt() : 10;
  if (n < 1) n = 1;
  if (n > BURST_FRAMES) n = BURST_FRAMES;
  ringCount = ringPos = 0;
  burstFile[0] = 0;
  forceLeft = (uint8_t)n;
  cors();
  server.send(200, "application/json", String("{\"n\":") + n + "}");
}

/* /server?url=http://... : vide pour revenir a l'ecriture SD seule. */
static void handleServer() {
  if (server.hasArg("url")) {
    String u = server.arg("url");
    u.trim();
    while (u.endsWith("/")) u.remove(u.length() - 1);
    /* postBurst colle "/clip" derriere. */
    if (u.length() && !u.startsWith("http://") && !u.startsWith("https://")) {
      cors();
      server.send(400, "text/plain", "adresse attendue sous la forme http://hote:port");
      return;
    }
    if (u.length() > (int)sizeof(SERVER_URL) - 6) {
      cors();
      server.send(400, "text/plain", "adresse trop longue");
      return;
    }
    strncpy(SERVER_URL, u.c_str(), sizeof(SERVER_URL) - 1);
    SERVER_URL[sizeof(SERVER_URL) - 1] = 0;
    Serial.printf("[CLIP] serveur : %s\n", SERVER_URL[0] ? SERVER_URL : "(aucun)");
  }
  cors();
  server.send(200, "text/plain", SERVER_URL);
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

static void handleCalm() {
  const float v = server.arg("v").toFloat();
  if (v >= 0.0f && v <= 100.0f) calmPct = v;
  cors();
  server.send(200, "text/plain", String(calmPct, 2));
}

static void handleMinActive() {
  const long v = server.arg("v").toInt();
  if (v >= 0 && v <= 60000) minActiveMs = (uint32_t)v;
  cors();
  server.send(200, "text/plain", String(minActiveMs));
}

static void handleSnapshot() {
  cors();
  if (!cameraOk) { server.send(409, "text/plain", "camera indisponible"); return; }
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { server.send(503, "text/plain", "capture impossible"); return; }
  uint8_t* jpg = nullptr; size_t len = 0;
  const bool ok = fmt2jpg(fb->buf, fb->len, fb->width, fb->height,
                          PIXFORMAT_RGB565, PREVIEW_QUALITY, &jpg, &len);
  esp_camera_fb_return(fb);
  if (!ok) { server.send(503, "text/plain", "encodage impossible"); return; }
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(len);
  server.send(200, "image/jpeg", "");
  server.sendContent((const char*)jpg, len);
  free(jpg);
}

static void handleRefImg() {
  cors();
  if (!haveRef || !refBuf) { server.send(409, "text/plain", "aucune reference"); return; }
  uint8_t* jpg = nullptr;
  size_t   len = 0;
  if (!fmt2jpg(refBuf, grayLen, grayW, grayH, PIXFORMAT_GRAYSCALE,
               PREVIEW_QUALITY, &jpg, &len)) {
    server.send(503, "text/plain", "encodage impossible");
    return;
  }
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(len);
  server.send(200, "image/jpeg", "");
  server.sendContent((const char*)jpg, len);
  free(jpg);
}

static void handleMask() {
  cors();
  if (!haveRef) { server.send(409, "text/plain", "aucune reference"); return; }
  if (!cameraOk) { server.send(409, "text/plain", "camera indisponible"); return; }
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { server.send(503, "text/plain", "capture impossible"); return; }
  if (!ensureBuffers(fb) || fb->len < grayLen * 2) {
    esp_camera_fb_return(fb);
    server.send(500, "text/plain", "trame ou memoire indisponible");
    return;
  }
  toLuma(fb);
  buildMask();              // a la demande : ne touche pas au chiffre de mouvement
  esp_camera_fb_return(fb);

  const size_t len = encodeRuns(maskBuf);
  uint8_t head[5];
  w16(head, grayW); w16(head + 2, grayH); head[4] = 1;
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
        char nom[72];
        jsonSafe(nom, sizeof(nom), n.c_str());
        char row[112];
        snprintf(row, sizeof(row), "%s{\"n\":\"%s\",\"s\":%u}",
                 first ? "" : ",", nom, (unsigned)f.size());
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

/* Un nom venu du navigateur ne doit designer qu'un fichier de la racine. */
static bool sdPathOk(const String& f) {
  if (!sdReady || f.length() < 2 || f.length() > 63) return false;
  if (!f.startsWith("/")) return false;
  if (f.indexOf("..") >= 0) return false;
  if (f.indexOf('/', 1) >= 0) return false;      // pas de sous-dossier
  return SD_MMC.exists(f);
}

static void handleDownload() {
  const String f = server.arg("f");
  if (!sdPathOk(f)) {
    cors();
    server.send(404, "text/plain", "introuvable");
    return;
  }
  File file = SD_MMC.open(f, FILE_READ);
  if (!file) { cors(); server.send(500, "text/plain", "ouverture impossible"); return; }
  if (recOn && f == String(recPath)) {
    /* Le fichier grandit pendant la lecture : la taille annoncee serait
     * fausse et le telechargement tronque. */
    file.close();
    cors();
    server.send(409, "text/plain", "enregistrement en cours sur ce fichier");
    return;
  }

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
  const String f = server.arg("f");
  cors();
  if (!sdPathOk(f)) { server.send(404, "text/plain", "introuvable"); return; }
  if (recOn && f == String(recPath)) {
    server.send(409, "text/plain", "enregistrement en cours sur ce fichier");
    return;
  }
  server.send(200, "text/plain", SD_MMC.remove(f) ? "ok" : "suppression impossible");
}

/* ----------------------------- demarrage ----------------------------- */
/* Le bail DHCP change a chaque redemarrage, le nom non. */
static void announceMdns() {
  static bool mdnsUp = false;
  if (mdnsUp) return;
  if (MDNS.begin(HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    mdnsUp = true;
    Serial.printf("[MDNS] http://%s.local\n", HOSTNAME);
  } else {
    Serial.println("[MDNS] demarrage impossible");
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[BOOT] detecteur de changement ESP32-WROVER " FW_VERSION);

  if (!psramFound()) Serial.println("[BOOT] ATTENTION : pas de PSRAM detectee");

  /* L'init echoue parfois au premier essai apres une coupure brutale. */
  for (int essai = 1; essai <= 3 && !cameraOk; essai++) {
    cameraOk = initCamera();
    if (!cameraOk) {
      Serial.printf("[CAM] essai %d/3 echoue\n", essai);
      esp_camera_deinit();
      delay(500);
    }
  }
  if (!cameraOk) {
    /* Ne pas bloquer : sans serveur web on ne peut pas lire la panne. */
    lastError = "initialisation camera impossible";
    Serial.println("[BOOT] camera indisponible - le serveur web demarre quand meme");
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
  staLastTry = millis();
  if (WiFi.status() == WL_CONNECTED) {
    esp_wifi_set_ps(WIFI_PS_NONE);     // apres l'association, sinon sans effet
    Serial.print("[WIFI] ouvrir http://");
    Serial.println(WiFi.localIP());
    announceMdns();
    staWasUp = true;
    Serial.printf("[WIFI] RSSI %d dBm\n", WiFi.RSSI());
  } else {
    /* Sans cet arret, la tentative suivante se superpose et le driver
     * repond "sta is connecting, return error". */
    WiFi.disconnect(false);
    Serial.printf("[WIFI] %s injoignable - nouvelle tentative toutes les %u s\n",
                  WIFI_SSID, (unsigned)(STA_RETRY_MS / 1000));
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
  server.on("/refimg",    handleRefImg);
  server.on("/zone",      handleZone);
  server.on("/server",    handleServer);
  server.on("/force",     handleForce);
  server.on("/threshold", handleThreshold);
  server.on("/trigger",   handleTrigger);
  server.on("/calm",      handleCalm);
  server.on("/minactive", handleMinActive);
  server.on("/files",     handleFiles);
  server.on("/download",  handleDownload);
  server.on("/delete",    handleDelete);
  server.onNotFound([]() { cors(); server.send(404, "text/plain", "404"); });
  server.begin();
  Serial.println("[HTTP] serveur demarre sur le port 80");
}

void loop() {
  server.handleClient();
  motionTick();

#if !AP_ONLY
  /* Le hotspot est souvent allume APRES la carte, et Android le coupe
   * quand plus aucun client n'est connecte. Tentatives espacees : chacune
   * fait scanner la radio, partagee avec la camera. */
  if (WiFi.status() == WL_CONNECTED) {
    if (!staWasUp) {
      staWasUp = true;
      esp_wifi_set_ps(WIFI_PS_NONE);
      Serial.print("[WIFI] connecte, http://");
      Serial.println(WiFi.localIP());
      announceMdns();
    }
  } else if (millis() - staLastTry > STA_RETRY_MS) {
    if (staWasUp) {
      staWasUp = false;
      Serial.println("[WIFI] lien perdu");
    }
    staLastTry = millis();
    Serial.printf("[WIFI] tentative vers %s\n", WIFI_SSID);
    WiFi.disconnect(false);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
#endif
}
