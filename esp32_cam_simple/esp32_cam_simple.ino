/* ESP32-WROVER CAM - version simple
 *
 * La carte rejoint le hotspot du telephone et sert une page a deux
 * boutons :
 *   - une image : /shot renvoie un JPEG pris a l'instant
 *   - un enregistrement : /record?on=1 ecrit un .avi MJPEG sur la carte SD
 *
 * L'enregistrement s'arrete tout seul quand il ne reste plus assez de
 * place sur la carte, ou quand la duree maximale est atteinte.
 *
 * IDE : carte "ESP32 Wrover Module", PSRAM Enabled,
 *       Partition Scheme "Huge APP (3MB No OTA)".
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include "FS.h"
#include "SD_MMC.h"

#define FW_VERSION "2.0.0-simple"

/* ------------------------- reglages ---------------------------------- */
static const char* WIFI_SSID = "Philphone";   // hotspot 2.4 GHz du telephone
static const char* WIFI_PASS = "12345678";
static const char* HOSTNAME  = "esp32cam";    // http://esp32cam.local

#define FRAME_SIZE       FRAMESIZE_VGA   // 640x480
#define JPEG_QUALITY     12              // 10 = mieux et plus gros, 20 = plus petit
#define XCLK_FREQ_HZ     10000000

#define REC_FPS          10
#define REC_MAX_FRAMES   18000           // 30 min a 10 i/s (index = 16 o/image)
#define SD_MIN_FREE_MB   20              // marge sous laquelle on coupe

/* ----------------------- brochage Freenove --------------------------- */
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

#define SD_ONE_BIT_MODE  true
#define SD_MOUNT_POINT   "/sdcard"

WebServer server(80);

static bool     sdReady   = false;
static String   lastMsg   = "";

static File     recFile;
static bool     recOn     = false;
static uint32_t recFrames = 0;
static uint32_t recNextMs = 0;
static uint32_t recMovi   = 0;        // octets de donnees deja ecrits
static uint32_t recMax    = 0;        // plus grande image du fichier
static uint16_t recW = 0, recH = 0;
static uint8_t* recIdx    = nullptr;  // index idx1, en PSRAM
static uint64_t recFree   = 0;        // place restante estimee, en octets
static char     recPath[32] = {0};

/* --------------------------- AVI MJPEG -------------------------------- *
 * Les images sortent deja compressees du capteur, elles entrent dans le
 * fichier telles quelles. L'entete fait 224 octets, ecrits une premiere
 * fois a zero puis corriges a la fermeture.                              */
static const uint32_t AVI_HEADER_BYTES = 224;

static inline void w32(uint8_t* p, uint32_t v) {
  p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}
static inline void w16(uint8_t* p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static inline void tag(uint8_t* p, const char* s) { memcpy(p, s, 4); }

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
  w32(h + 32, usPerFrame);
  w32(h + 36, maxFrame * fps);
  w32(h + 44, 0x10);                // AVIF_HASINDEX
  w32(h + 48, frames);
  w32(h + 56, 1);
  w32(h + 60, maxFrame);
  w32(h + 64, w);
  w32(h + 68, ht);

  tag(h + 88,  "LIST"); w32(h + 92, 116); tag(h + 96, "strl");

  tag(h + 100, "strh"); w32(h + 104, 56);
  tag(h + 108, "vids");
  tag(h + 112, "MJPG");
  w32(h + 128, 1);
  w32(h + 132, fps);
  w32(h + 140, frames);
  w32(h + 144, maxFrame);
  w32(h + 148, 0xFFFFFFFF);
  w16(h + 156, 0); w16(h + 158, 0); w16(h + 160, w); w16(h + 162, ht);

  tag(h + 164, "strf"); w32(h + 168, 40);
  w32(h + 172, 40);
  w32(h + 176, w);
  w32(h + 180, ht);
  w16(h + 184, 1);
  w16(h + 186, 24);
  tag(h + 188, "MJPG");
  w32(h + 192, (uint32_t)w * ht * 3);

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
  c.pixel_format = PIXFORMAT_JPEG;        // compression par le capteur
  c.frame_size   = FRAME_SIZE;
  c.jpeg_quality = JPEG_QUALITY;
  c.grab_mode    = CAMERA_GRAB_LATEST;
  c.fb_location  = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  c.fb_count     = psramFound() ? 2 : 1;

  const esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("[CAM] init failed: 0x%X\n", err);
    return false;
  }
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_vflip(s, 0);       // 1 si l'image est a l'envers
    s->set_hmirror(s, 0);
  }
  return true;
}

/* ----------------------------- SD ------------------------------------ */
static bool initSD() {
  if (!SD_MMC.begin(SD_MOUNT_POINT, SD_ONE_BIT_MODE)) {
    lastMsg = "carte SD non montee (inseree ? formatee FAT32 ?)";
    return false;
  }
  if (SD_MMC.cardType() == CARD_NONE) {
    lastMsg = "aucune carte SD detectee";
    return false;
  }
  Serial.printf("[SD] %llu Mo au total, %llu Mo utilises\n",
                SD_MMC.totalBytes() / (1024ULL * 1024ULL),
                SD_MMC.usedBytes()  / (1024ULL * 1024ULL));
  return true;
}

static uint64_t sdFreeBytes() {
  if (!sdReady) return 0;
  const uint64_t total = SD_MMC.totalBytes();
  const uint64_t used  = SD_MMC.usedBytes();
  return (total > used) ? (total - used) : 0;
}

/* ------------------------ enregistrement ------------------------------ */
static void stopRecording(const char* why);

static bool startRecording() {
  if (recOn) return true;
  if (!sdReady) { lastMsg = "carte SD non prete"; return false; }

  if (sdFreeBytes() < (uint64_t)SD_MIN_FREE_MB * 1024ULL * 1024ULL) {
    lastMsg = "carte SD pleine, enregistrement impossible";
    return false;
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { lastMsg = "capture impossible"; return false; }
  recW = fb->width; recH = fb->height;
  esp_camera_fb_return(fb);

  recIdx = (uint8_t*)ps_malloc(16UL * REC_MAX_FRAMES);
  if (!recIdx) { lastMsg = "PSRAM insuffisante"; return false; }

  for (uint16_t i = 1; i < 10000; i++) {
    snprintf(recPath, sizeof(recPath), "/rec_%04u.avi", i);
    if (!SD_MMC.exists(recPath)) break;
  }
  recFile = SD_MMC.open(recPath, FILE_WRITE);
  if (!recFile) {
    free(recIdx); recIdx = nullptr;
    lastMsg = String("creation impossible: ") + recPath;
    return false;
  }

  uint8_t hdr[AVI_HEADER_BYTES];
  buildAviHeader(hdr, recW, recH, REC_FPS, 0, 0, 0);
  recFile.write(hdr, AVI_HEADER_BYTES);

  recFrames = 0;
  recMovi   = 0;
  recMax    = 0;
  recFree   = sdFreeBytes();
  recNextMs = millis();
  recOn     = true;
  lastMsg   = "";
  Serial.printf("[REC] %s  %ux%u @ %d i/s\n", recPath, recW, recH, REC_FPS);
  return true;
}

static void stopRecording(const char* why) {
  if (!recOn) return;
  recOn = false;

  uint8_t ih[8];
  tag(ih, "idx1"); w32(ih + 4, 16UL * recFrames);
  recFile.write(ih, 8);
  recFile.write(recIdx, 16UL * recFrames);
  recFile.close();

  File patch = SD_MMC.open(recPath, "r+");
  if (patch) {
    uint8_t hdr[AVI_HEADER_BYTES];
    buildAviHeader(hdr, recW, recH, REC_FPS, recFrames, recMovi, recMax);
    patch.seek(0);
    patch.write(hdr, AVI_HEADER_BYTES);
    patch.close();
  }
  free(recIdx); recIdx = nullptr;

  lastMsg = String(why ? why : "arrete") + " - " + String(recFrames) + " images";
  Serial.printf("[REC] %s: %u images, %.1f Mo (%s)\n", recPath, (unsigned)recFrames,
                recMovi / 1048576.0, why ? why : "arrete");
}

/* Appelee en boucle : une image par periode, et on coupe avant que la
 * carte soit reellement pleine - un .avi tronque en pleine ecriture n'a
 * ni index ni entete correcte, donc ne se lit plus. */
static void recordTick() {
  if (!recOn) return;
  const uint32_t now = millis();
  if ((int32_t)(now - recNextMs) < 0) return;
  recNextMs = now + 1000UL / REC_FPS;

  if (recFrames >= REC_MAX_FRAMES) { stopRecording("duree maximale atteinte"); return; }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return;
  const uint32_t len = fb->len;
  const uint32_t pad = (len & 1) ? 1 : 0;
  const uint64_t need = 8 + len + pad + 16;          // chunk + entree d'index

  /* La place est suivie par soustraction : interroger la FAT a chaque
   * image bloquerait l'ecriture. Il reste la marge SD_MIN_FREE_MB. */
  if (recFree < need + (uint64_t)SD_MIN_FREE_MB * 1024ULL * 1024ULL) {
    esp_camera_fb_return(fb);
    stopRecording("carte SD presque pleine");
    return;
  }

  uint8_t ch[8];
  tag(ch, "00dc"); w32(ch + 4, len);
  const bool ok = recFile.write(ch, 8) == 8 && recFile.write(fb->buf, len) == len;
  esp_camera_fb_return(fb);
  if (!ok) { stopRecording("ecriture SD en echec"); return; }
  if (pad) { const uint8_t z = 0; recFile.write(&z, 1); }

  uint8_t* e = recIdx + 16UL * recFrames;
  tag(e, "00dc"); w32(e + 4, 0x10); w32(e + 8, 4 + recMovi); w32(e + 12, len);

  recMovi += 8 + len + pad;
  recFree -= need;
  if (len > recMax) recMax = len;
  recFrames++;
  if ((recFrames & 0x1F) == 0) recFile.flush();
}

/* ----------------------------- page ---------------------------------- */
static const char PAGE[] PROGMEM = R"HTML(<!doctype html>
<html lang="fr"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32 CAM</title>
<style>
 body{font:16px system-ui;margin:0;padding:16px;background:#111;color:#eee}
 button{font:600 17px system-ui;padding:14px;width:100%;margin:6px 0;border:0;
        border-radius:10px;background:#2d6cdf;color:#fff}
 button.rec{background:#c62828}
 img{width:100%;border-radius:10px;margin-top:10px;background:#000}
 #st{font-size:14px;color:#aaa;margin-top:10px;white-space:pre-line}
 a{color:#7fb0ff}
</style>
<h2>ESP32 CAM</h2>
<button onclick="shot()">Prendre une image</button>
<button id="rb" onclick="rec()">Demarrer l'enregistrement</button>
<img id="im" alt="">
<div id="st">...</div>
<div id="fl"></div>
<script>
const g=i=>document.getElementById(i);
let on=false;
function shot(){g('im').src='/shot?t='+Date.now();}
function rec(){fetch('/record?on='+(on?0:1)).then(st);}
function st(){
  fetch('/status').then(r=>r.json()).then(s=>{
    on=s.rec;
    g('rb').textContent=on?"Arreter l'enregistrement":"Demarrer l'enregistrement";
    g('rb').className=on?'rec':'';
    g('st').textContent=
      (s.sd?'carte SD ok, '+s.freemb+' Mo libres':'pas de carte SD')+
      (on?'\nenregistre '+s.file+' : '+s.frames+' images, '+s.mb+' Mo':'')+
      (s.msg?'\n'+s.msg:'');
    files();
  }).catch(()=>{g('st').textContent='carte injoignable';});
}
function files(){
  fetch('/files').then(r=>r.json()).then(l=>{
    g('fl').innerHTML=l.map(f=>'<p><a href="/download?f=/'+f.n+'">'+f.n+'</a> '+
      (f.s/1048576).toFixed(1)+' Mo <a href="#" onclick="del(\''+f.n+'\')">supprimer</a></p>').join('');
  });
}
function del(n){fetch('/delete?f=/'+n).then(st);}
setInterval(st,2000); st();
</script>
</html>)HTML";

/* ---------------------------- routes --------------------------------- */
static void cors() { server.sendHeader("Access-Control-Allow-Origin", "*"); }

static void handleRoot() { server.send_P(200, "text/html", PAGE); }

/* Une seule image, prise au moment du clic. Rien n'est diffuse en continu. */
static void handleShot() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { server.send(503, "text/plain", "capture impossible"); return; }
  cors();
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(fb->len);
  server.send(200, "image/jpeg", "");
  server.sendContent((const char*)fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

static void handleRecordRoute() {
  if (server.arg("on") == "1") {
    if (!startRecording()) { cors(); server.send(500, "text/plain", lastMsg); return; }
  } else {
    stopRecording("arrete depuis le telephone");
  }
  cors();
  server.send(200, "text/plain", recOn ? "enregistre" : "arrete");
}

static void handleStatus() {
  const uint64_t freeMb = (recOn ? recFree : sdFreeBytes()) / (1024ULL * 1024ULL);
  char j[256];
  snprintf(j, sizeof(j),
           "{\"ver\":\"" FW_VERSION "\",\"sd\":%s,\"rec\":%s,\"frames\":%u,"
           "\"mb\":%.1f,\"freemb\":%u,\"file\":\"%s\",\"msg\":\"%s\"}",
           sdReady ? "true" : "false", recOn ? "true" : "false",
           (unsigned)recFrames, recMovi / 1048576.0, (unsigned)freeMb,
           recOn ? recPath : "", lastMsg.c_str());
  cors();
  server.send(200, "application/json", j);
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
      if (n.endsWith(".avi")) {
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
    server.send(404, "text/plain", "introuvable");
    return;
  }
  File file = SD_MMC.open(f, FILE_READ);
  if (!file) { server.send(500, "text/plain", "ouverture impossible"); return; }

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
}

static void handleDelete() {
  String f = server.arg("f");
  if (recOn && f == recPath) { cors(); server.send(409, "text/plain", "en cours"); return; }
  if (sdReady && f.startsWith("/") && f.indexOf("..") < 0 && SD_MMC.exists(f)) SD_MMC.remove(f);
  cors();
  server.send(200, "text/plain", "ok");
}

/* ---------------------------- setup ---------------------------------- */
static void announceMdns() {
  static bool up = false;
  if (up) return;
  if (MDNS.begin(HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    up = true;
    Serial.printf("[MDNS] http://%s.local\n", HOSTNAME);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[BOOT] ESP32 CAM simple " FW_VERSION);

  if (!psramFound()) Serial.println("[BOOT] attention: pas de PSRAM");
  if (!initCamera()) { Serial.println("[BOOT] camera absente - arret"); while (true) delay(1000); }
  sdReady = initSD();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setHostname(HOSTNAME);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[WIFI] %s", WIFI_SSID);
  const uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) { delay(400); Serial.print('.'); }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WIFI] http://"); Serial.println(WiFi.localIP());
    announceMdns();
  } else {
    Serial.println("[WIFI] hotspot injoignable - nouvelles tentatives en arriere-plan");
  }

  server.on("/",         handleRoot);
  server.on("/shot",     handleShot);
  server.on("/record",   handleRecordRoute);
  server.on("/status",   handleStatus);
  server.on("/files",    handleFiles);
  server.on("/download", handleDownload);
  server.on("/delete",   handleDelete);
  server.onNotFound([]() { server.send(404, "text/plain", "404"); });
  server.begin();
  Serial.println("[HTTP] port 80");
}

void loop() {
  server.handleClient();
  recordTick();

  /* Le hotspot est souvent allume apres la carte, et Android le coupe
     quand plus personne n'est connecte : on reessaie sans abandonner. */
  static uint32_t lastTry = 0;
  static bool wasUp = false;
  if (WiFi.status() == WL_CONNECTED) {
    if (!wasUp) {
      wasUp = true;
      Serial.print("[WIFI] connecte, http://"); Serial.println(WiFi.localIP());
      announceMdns();
    }
  } else if (millis() - lastTry > 20000) {
    wasUp = false;
    lastTry = millis();
    WiFi.disconnect(false);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
}
