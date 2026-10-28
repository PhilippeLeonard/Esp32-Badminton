/* ESP32-WROVER CAM - version simple
 *
 * La carte rejoint le hotspot du telephone et sert une page a deux
 * boutons :
 *   - une image : /shot renvoie un JPEG pris a l'instant
 *   - un enregistrement : /record?on=1 ecrit /rec_NNNN.rgb sur la carte SD,
 *     en couleur et sans aucune compression (RGB565, les octets sortis du
 *     capteur tels quels)
 *
 * L'enregistrement s'arrete tout seul quand il ne reste plus assez de
 * place sur la carte, ou quand la duree maximale est atteinte.
 *
 * Le fichier est lisible meme si la carte perd le courant en pleine
 * ecriture : l'entete est complete et vidangee des la premiere image, et
 * le nombre d'images se deduit de la taille du fichier. Seule la derniere
 * image en cours d'ecriture est perdue. C'est ce qui manquait a la
 * version .avi : son index et sa taille n'etaient ecrits qu'a la
 * fermeture, donc une coupure laissait un fichier illisible.
 *
 * IDE : carte "ESP32 Wrover Module", PSRAM Enabled,
 *       Partition Scheme "Huge APP (3MB No OTA)".
 */

#include "esp_camera.h"
#include "img_converters.h"
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include "FS.h"
#include "SD_MMC.h"

#define FW_VERSION "2.1.0-raw"

/* ------------------------- reglages ---------------------------------- */
static const char* WIFI_SSID = "Philphone";   // hotspot 2.4 GHz du telephone
static const char* WIFI_PASS = "12345678";
static const char* HOSTNAME  = "esp32cam";    // http://esp32cam.local

/* La carte monte TOUJOURS son propre point d'acces, a 192.168.4.1, en
 * plus d'essayer de rejoindre le hotspot. Brancher le telephone dessus
 * sort le hotspot du chemin, et surtout la carte reste joignable quand
 * le hotspot est absent, en 5 GHz, ou simplement eteint. AP_ONLY coupe
 * la station et ne garde que le point d'acces. */
#define AP_ENABLED       1
#define AP_ONLY          0
static const char* AP_SSID = "ESP32-CAM";
static const char* AP_PASS = "camera123";     // au moins 8 caracteres
#define STA_RETRY_MS     60000           // intervalle entre deux tentatives
#define STA_MAX_TRY      5               // au-dela on arrete la station : chaque
                                         // tentative fait scanner la radio,
                                         // partagee avec l'AP et la camera

/* Debit de l'enregistrement = largeur * hauteur * 2 * REC_FPS. La carte
 * SD ne tourne qu'en 1 bit (voir SD_ONE_BIT_MODE plus bas) et le debit
 * mesure au demarrage est ce qui decide des reglages tenables :
 *   carte a 0,3 Mo/s : QQVGA 160x120 @ 5  -> 192 ko/s   (defaut prudent)
 *                      QVGA  320x240 @ 2  -> 307 ko/s
 *   carte a 1,0 Mo/s : QVGA  320x240 @ 5  -> 768 ko/s
 *                      QQVGA 160x120 @ 20 -> 768 ko/s
 * Laisser au moins 30 % de marge : le reste du temps processeur part
 * dans le wifi et la camera.                                            */
#define FRAME_SIZE       FRAMESIZE_QQVGA // 160x120
#define REC_FPS          5
#define PREVIEW_QUALITY  10              // qualite du JPEG de /shot seulement
#define XCLK_FREQ_HZ     10000000

#define REC_MAX_MINUTES  30              // garde-fou de duree
#define SD_MIN_FREE_MB   20              // marge sous laquelle on coupe
#define SD_BENCH         1               // mesure le debit de la carte au demarrage

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

/* GPIO 4 porte Y2 de la camera, et c'est aussi DATA1 du bus SD. Le mode
 * 4 bits est donc impossible sur cette carte : on reste en 1 bit, d'ou le
 * plafond de debit ci-dessus. */
#define SD_ONE_BIT_MODE  true
#define SD_MOUNT_POINT   "/sdcard"

WebServer server(80);

static bool     sdReady   = false;
static String   lastMsg   = "";
static uint8_t  staTries  = 0;        // tentatives de hotspot deja faites
static float    sdMbPerSec = 0.0f;      // mesure au demarrage, 0 = non mesure

/* ------------------------ enregistrement ------------------------------ *
 * Format du fichier, petit-boutiste, entete de 32 octets puis les images
 * a la suite, chacune de width*height*2 octets en RGB565 :
 *    0  "ESPC"    4 o   magie
 *    4  version   u16   1
 *    6  format    u16   0 = RGB565 petit-boutiste
 *    8  width     u16
 *   10  height    u16
 *   12  fps       u16
 *   14  (reserve) u16
 *   16  frames    u32   0 tant que l'enregistrement tourne, renseigne a
 *                       l'arret ; un lecteur doit de toute facon se fier
 *                       a la taille du fichier, cette valeur n'est qu'une
 *                       indication
 *   20  (reserve) 12 o
 * tools/rgb_tool.py lit exactement ca.                                   */
static const uint32_t REC_HEADER_BYTES = 32;

static File     recFile;
static bool     recOn       = false;
static uint32_t recFrames   = 0;
static uint32_t recDropped  = 0;
static uint32_t recNextMs   = 0;
static uint32_t recStartMs  = 0;
static uint32_t recFrameLen = 0;        // octets par image
static uint16_t recW = 0, recH = 0;
static uint64_t recFree     = 0;        // place restante estimee, en octets
static char     recPath[32] = {0};

static inline void w32(uint8_t* p, uint32_t v) {
  p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}
static inline void w16(uint8_t* p, uint16_t v) { p[0] = v; p[1] = v >> 8; }

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
  c.pixel_format = PIXFORMAT_RGB565;      // couleur, aucune compression
  c.frame_size   = FRAME_SIZE;
  c.jpeg_quality = PREVIEW_QUALITY;
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
    s->set_whitebal(s, 1);    // la balance des blancs reste automatique :
    s->set_awb_gain(s, 1);    // sans elle le gymnase tire au vert
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

/* Quelques mega-octets ecrits puis effaces : une carte lente se voit tout
 * de suite au demarrage, au lieu de donner un enregistrement qui saute.
 * Les blocs font exactement la taille d'une image, comme a l'usage : une
 * carte SD est bien plus rapide par gros blocs, mesurer par petits
 * paquets la sous-estime largement. */
static void benchSD(uint32_t blockLen) {
  if (!sdReady || !blockLen) return;
  uint8_t* blk = (uint8_t*)ps_malloc(blockLen);
  if (!blk) return;
  memset(blk, 0xA5, blockLen);

  File f = SD_MMC.open("/bench.tmp", FILE_WRITE);
  if (!f) { free(blk); return; }
  const uint32_t blocks = 4194304UL / blockLen + 1;   // ~4 Mo
  const uint32_t t0 = millis();
  uint32_t written = 0;
  for (uint32_t i = 0; i < blocks; i++) {
    if (f.write(blk, blockLen) != blockLen) break;
    written += blockLen;
  }
  f.flush();
  const uint32_t ms = millis() - t0;
  f.close();
  SD_MMC.remove("/bench.tmp");
  free(blk);

  if (ms) sdMbPerSec = (written / 1048576.0f) * 1000.0f / ms;
  Serial.printf("[SD] ecriture mesuree: %.2f Mo/s par blocs de %u ko\n",
                sdMbPerSec, (unsigned)(blockLen / 1024));
}

/* ------------------------ enregistrement ----------------------------- */
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
  recFrameLen = (uint32_t)recW * recH * 2;
  esp_camera_fb_return(fb);

  for (uint16_t i = 1; i < 10000; i++) {
    snprintf(recPath, sizeof(recPath), "/rec_%04u.rgb", i);
    if (!SD_MMC.exists(recPath)) break;
  }
  recFile = SD_MMC.open(recPath, FILE_WRITE);
  if (!recFile) { lastMsg = String("creation impossible: ") + recPath; return false; }

  uint8_t h[REC_HEADER_BYTES];
  memset(h, 0, sizeof(h));
  memcpy(h, "ESPC", 4);
  w16(h + 4, 1);                 // version
  w16(h + 6, 0);                 // RGB565
  w16(h + 8, recW);
  w16(h + 10, recH);
  w16(h + 12, REC_FPS);
  w32(h + 16, 0);                // renseigne a l'arret, jamais indispensable
  if (recFile.write(h, REC_HEADER_BYTES) != REC_HEADER_BYTES) {
    recFile.close();
    lastMsg = "ecriture de l'entete impossible";
    return false;
  }
  /* Vidange immediate : c'est l'omission de ce flush qui laissait des
   * fichiers entierement a zero quand la carte perdait le courant. */
  recFile.flush();

  recFrames  = 0;
  recDropped = 0;
  recFree    = sdFreeBytes();
  recStartMs = millis();
  recNextMs  = recStartMs;
  recOn      = true;
  lastMsg    = "";
  Serial.printf("[REC] %s  %ux%u RGB565 @ %d i/s  (%.2f Mo/s)\n",
                recPath, recW, recH, REC_FPS,
                recFrameLen * (float)REC_FPS / 1048576.0f);
  return true;
}

static void stopRecording(const char* why) {
  if (!recOn) return;
  recOn = false;
  recFile.flush();
  recFile.close();

  /* Confort uniquement : le fichier se lit deja sans ce chiffre. */
  File patch = SD_MMC.open(recPath, "r+");
  if (patch) {
    uint8_t n[4];
    w32(n, recFrames);
    patch.seek(16);
    patch.write(n, 4);
    patch.close();
  }

  const float secs = (millis() - recStartMs) / 1000.0f;
  lastMsg = String(why ? why : "arrete") + " - " + String(recFrames) + " images";
  if (recDropped) lastMsg += ", " + String(recDropped) + " sautees";
  Serial.printf("[REC] %s: %u images en %.1f s, %.1f Mo, %u sautees (%s)\n",
                recPath, (unsigned)recFrames, secs,
                (double)recFrames * recFrameLen / 1048576.0,
                (unsigned)recDropped, why ? why : "arrete");
}

/* Appelee en boucle : une image par periode, et on coupe avant que la
 * carte soit reellement pleine. */
static void recordTick() {
  if (!recOn) return;
  const uint32_t now = millis();
  if ((int32_t)(now - recNextMs) < 0) return;

  const uint32_t period = 1000UL / REC_FPS;
  /* Si la carte a pris du retard, on saute la ou les images en retard au
   * lieu d'accumuler : mieux vaut un trou franc qu'une boucle qui ne
   * rattrape jamais et finit par ne plus rendre la main au serveur. */
  if ((int32_t)(now - recNextMs) > (int32_t)period) {
    recDropped += (now - recNextMs) / period;
    recNextMs = now;
  }
  recNextMs += period;

  if (now - recStartMs > (uint32_t)REC_MAX_MINUTES * 60000UL) {
    stopRecording("duree maximale atteinte");
    return;
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { recDropped++; return; }
  if (fb->len != recFrameLen) {          // taille inattendue : on n'ecrit pas
    esp_camera_fb_return(fb);
    stopRecording("taille d'image inattendue");
    return;
  }

  /* La place est suivie par soustraction : interroger la FAT a chaque
   * image bloquerait l'ecriture. Il reste la marge SD_MIN_FREE_MB. */
  if (recFree < (uint64_t)recFrameLen + (uint64_t)SD_MIN_FREE_MB * 1024ULL * 1024ULL) {
    esp_camera_fb_return(fb);
    stopRecording("carte SD presque pleine");
    return;
  }

  const bool ok = recFile.write(fb->buf, recFrameLen) == recFrameLen;
  esp_camera_fb_return(fb);
  if (!ok) { stopRecording("ecriture SD en echec"); return; }

  recFrames++;
  recFree -= recFrameLen;
  /* Une vidange par seconde : la FAT suit la taille reelle, donc une
   * coupure ne coute qu'une seconde d'images, jamais le fichier. */
  if (recFrames % REC_FPS == 0) recFile.flush();
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
<p><a href="#" onclick="fetch('/wifi').then(st);return false">reessayer le hotspot</a></p>
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
      (s.sd?'carte SD ok, '+s.freemb+' Mo libres'+(s.cardmbs?', '+s.cardmbs+' Mo/s':''):'pas de carte SD')+
      '\n'+s.w+'x'+s.h+' couleur brute, '+s.fps+' i/s = '+s.needmbs+' Mo/s'+
      (s.ap?'\npoint d\'acces '+s.ap:'')+
      (s.ip?'  /  hotspot '+s.ip:'  /  hotspot non rejoint')+
      (on?'\nenregistre '+s.file+' : '+s.frames+' images, '+s.mb+' Mo, '+s.dropped+' sautees':'')+
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

/* Une seule image, prise au moment du clic, convertie en JPEG pour le
 * telephone : le brut RGB565 ne s'affiche pas dans un navigateur. Rien
 * n'est diffuse en continu. */
static void handleShot() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { server.send(503, "text/plain", "capture impossible"); return; }
  uint8_t* jpg = nullptr;
  size_t   len = 0;
  const bool ok = fmt2jpg(fb->buf, fb->len, fb->width, fb->height,
                          PIXFORMAT_RGB565, PREVIEW_QUALITY, &jpg, &len);
  esp_camera_fb_return(fb);
  if (!ok) { server.send(503, "text/plain", "conversion impossible"); return; }
  cors();
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(len);
  server.send(200, "image/jpeg", "");
  server.sendContent((const char*)jpg, len);
  free(jpg);
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
  const uint32_t flen   = recFrameLen ? recFrameLen : (uint32_t)320 * 240 * 2;
  const String sta = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : String("");
  char j[480];
  snprintf(j, sizeof(j),
           "{\"ver\":\"" FW_VERSION "\",\"sd\":%s,\"rec\":%s,\"frames\":%u,"
           "\"mb\":%.1f,\"freemb\":%u,\"dropped\":%u,\"file\":\"%s\","
           "\"w\":%u,\"h\":%u,\"fps\":%u,\"needmbs\":\"%.2f\",\"cardmbs\":\"%.2f\","
           "\"ap\":\"%s\",\"ip\":\"%s\",\"msg\":\"%s\"}",
           sdReady ? "true" : "false", recOn ? "true" : "false",
           (unsigned)recFrames, (double)recFrames * flen / 1048576.0,
           (unsigned)freeMb, (unsigned)recDropped, recOn ? recPath : "",
           (unsigned)(recW ? recW : 320), (unsigned)(recH ? recH : 240),
           (unsigned)REC_FPS, flen * (double)REC_FPS / 1048576.0,
           (double)sdMbPerSec,
#if AP_ENABLED || AP_ONLY
           WiFi.softAPIP().toString().c_str(),
#else
           "",
#endif
           sta.c_str(), lastMsg.c_str());
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
      if (n.endsWith(".rgb")) {
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
  Serial.printf("[HTTP] %s: %u/%u octets%s\n", f.c_str(), (unsigned)sent,
                (unsigned)total, sent == total ? "" : "  <-- TRONQUE");
}

/* Relance la station apres l'abandon : le telephone est sur l'AP, il
 * vient d'allumer son hotspot, et rien ne doit obliger a redemarrer. */
static void handleWifi() {
  staTries = 0;
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  cors();
  server.send(200, "text/plain", String("nouvelle tentative vers ") + WIFI_SSID);
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

/* Pourquoi l'association a echoue : le code du pilote, en clair. */
static const char* wifiStatusName(int st) {
  switch (st) {
    case WL_NO_SSID_AVAIL:  return "reseau introuvable (eteint, 5 GHz, ou SSID masque)";
    case WL_CONNECT_FAILED: return "refus (mot de passe, ou WPA3 seul)";
    case WL_CONNECTION_LOST: return "lien perdu";
    case WL_DISCONNECTED:   return "deconnecte";
    case WL_IDLE_STATUS:    return "toujours en cours";
    default:                return "inconnu";
  }
}

/* Ce que la carte voit vraiment autour d'elle. Affiche seulement quand
 * l'association a echoue : un hotspot absent de cette liste est en
 * 5 GHz, masque ou eteint, et aucun reglage du croquis n'y changera
 * rien. S'il y figure en WPA3 seul, l'ESP32 ne sait pas s'y associer. */
static void scanAndReport() {
  Serial.println("[WIFI] reseaux 2,4 GHz visibles :");
  const int n = WiFi.scanNetworks();
  if (n <= 0) { Serial.println("  aucun"); return; }
  for (int i = 0; i < n; i++) {
    const char* enc = "?";
    switch (WiFi.encryptionType(i)) {
      case WIFI_AUTH_OPEN:            enc = "ouvert"; break;
      case WIFI_AUTH_WEP:             enc = "WEP"; break;
      case WIFI_AUTH_WPA_PSK:         enc = "WPA"; break;
      case WIFI_AUTH_WPA2_PSK:        enc = "WPA2"; break;
      case WIFI_AUTH_WPA_WPA2_PSK:    enc = "WPA/WPA2"; break;
      case WIFI_AUTH_WPA3_PSK:        enc = "WPA3 (non supporte)"; break;
      case WIFI_AUTH_WPA2_WPA3_PSK:   enc = "WPA2/WPA3"; break;
      default: break;
    }
    Serial.printf("  %-24s  canal %2d  %4d dBm  %s%s\n",
                  WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i), enc,
                  WiFi.SSID(i) == WIFI_SSID ? "   <-- le votre" : "");
  }
  WiFi.scanDelete();
}

static void connectWifi() {
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
  /* Monte en premier : quoi qu'il arrive ensuite cote hotspot, la page
   * est servie. */
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("[AP] \"%s\" / \"%s\"  ->  http://%s\n",
                AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());
  announceMdns();          // http://esp32cam.local marche aussi sur l'AP
#endif

#if AP_ONLY
  return;
#else
  /* Pas de setAutoReconnect : le pilote relancerait ses propres tentatives
   * en parallele de celles de loop(), et chaque begin() tomberait alors
   * sur "sta is connecting, return error". */
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("[WIFI] %s", WIFI_SSID);
  const uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) { delay(400); Serial.print('.'); }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WIFI] http://"); Serial.println(WiFi.localIP());
    Serial.printf("[WIFI] canal %d, %d dBm\n", WiFi.channel(), WiFi.RSSI());
    announceMdns();
    return;
  }

  const int st = WiFi.status();
  /* Le pilote continue d'essayer apres notre delai : sans cet arret, la
   * tentative suivante se superpose et il repond "sta is connecting,
   * return error". Meme correctif que dans esp32_cam_recorder. */
  WiFi.disconnect(false);
  Serial.printf("[WIFI] echec: %s (code %d)\n", wifiStatusName(st), st);
  scanAndReport();
  Serial.printf("[WIFI] nouvelle tentative toutes les %u s, %d au total\n",
                (unsigned)(STA_RETRY_MS / 1000), STA_MAX_TRY);
#if AP_ENABLED
  Serial.printf("[WIFI] en attendant, le point d'acces \"%s\" reste disponible\n",
                AP_SSID);
#endif
#endif  // AP_ONLY
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[BOOT] ESP32 CAM simple " FW_VERSION);

  if (!psramFound()) Serial.println("[BOOT] attention: pas de PSRAM");
  sdReady = initSD();

  /* Le wifi passe AVANT la camera : s'associer fait scanner la radio, et
   * ce scan monopolise assez le bus pour que le pilote camera laisse
   * tomber des images - les cam_hal: EV-VSYNC-OVF du demarrage. Camera
   * pas encore demarree, plus d'images a perdre. */
  connectWifi();

  if (!initCamera()) { Serial.println("[BOOT] camera absente - arret"); while (true) delay(1000); }

#if SD_BENCH
  /* Une capture suffit a connaitre la taille d'une image, donc le debit
   * exige : si la carte est en dessous, autant le savoir avant le match. */
  {
    uint32_t flen = 0;
    uint16_t fw = 0, fh = 0;
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) { flen = fb->len; fw = fb->width; fh = fb->height; esp_camera_fb_return(fb); }
    benchSD(flen);
    if (flen) {
      const float need = (float)flen * REC_FPS / 1048576.0f;
      const float ok   = need * 1.3f;            // 30 % de marge
      Serial.printf("[SD] l'enregistrement demande %.2f Mo/s%s\n", need,
                    (sdMbPerSec > 0.0f && sdMbPerSec < ok)
                      ? "  <-- carte trop lente, baisser REC_FPS ou FRAME_SIZE" : "");
      if (sdMbPerSec > 0.0f) {
        Serial.printf("[SD] cette carte tient environ %.1f images/s a %ux%u\n",
                      sdMbPerSec * 1048576.0f / (flen * 1.3f),
                      (unsigned)fw, (unsigned)fh);
      }
    }
  }
#endif

  server.on("/",         handleRoot);
  server.on("/shot",     handleShot);
  server.on("/record",   handleRecordRoute);
  server.on("/status",   handleStatus);
  server.on("/files",    handleFiles);
  server.on("/download", handleDownload);
  server.on("/delete",   handleDelete);
  server.on("/wifi",     handleWifi);
  server.onNotFound([]() { server.send(404, "text/plain", "404"); });
  server.begin();
  Serial.println("[HTTP] port 80");
}

void loop() {
  server.handleClient();
  recordTick();

#if !AP_ONLY
  /* Le hotspot est souvent allume apres la carte, et Android le coupe
     quand plus personne n'est connecte : on reessaie. Mais chaque
     tentative fait scanner la radio, partagee avec l'AP et la camera -
     c'est ce qui produisait les cam_hal: EV-VSYNC-OVF en rafale. D'ou
     l'intervalle large et l'abandon apres quelques essais : le point
     d'acces, lui, doit rester net. */
  static uint32_t lastTry = 0;
  static bool     wasUp = false;
  if (millis() - lastTry > STA_RETRY_MS) {
    lastTry = millis();
    const wl_status_t st = WiFi.status();
    if (st == WL_CONNECTED) {
      staTries = 0;
      if (!wasUp) {
        wasUp = true;
        Serial.print("[WIFI] connecte, http://"); Serial.println(WiFi.localIP());
        announceMdns();
      }
    } else if (staTries < STA_MAX_TRY) {
      if (wasUp) { wasUp = false; Serial.println("[WIFI] lien perdu"); }
      staTries++;
      Serial.printf("[WIFI] tentative %u/%u vers \"%s\"\n",
                    staTries, STA_MAX_TRY, WIFI_SSID);
      WiFi.disconnect(false);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    } else if (staTries == STA_MAX_TRY) {
      staTries++;                        // une seule fois
      WiFi.disconnect(true);
      Serial.println("[WIFI] hotspot injoignable, station arretee"
                     " - utiliser le point d'acces, ou /wifi pour reessayer");
    }
  }
#endif
}
