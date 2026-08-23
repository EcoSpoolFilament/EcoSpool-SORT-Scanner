/* ============================================================================
   EcoSpool SORT Scanner  —  ESP32-S3 (LilyGO T-Display S3)
   NIR plastic classifier: 4 SWIR LEDs + G12180 InGaAs + OPA392 TIA + ADS1115

   Reflectance-normalized: every scan is DARK -> per-LED ON -> ratio vs a stored
   WHITE REFERENCE, so the classifier sees reflectance (0..1), not raw counts.
   Portrait UI, EcoSpool palette (plant green / dark blue / grey / black / white).

   BUTTONS (active-low):
     PASS (GPIO 17): tap = scan / mark CORRECT ; hold ~0.8s = capture reference
     FAIL (GPIO 18): tap = mark WRONG
   SERIAL: raw | ref | ref? | s [label]   (bring-up + Edge Impulse CSV)

   LIBRARIES: Adafruit ADS1X15, TFT_eSPI (Setup206_LilyGo_T_Display_S3.h).
   Board: ESP32S3 Dev Module, PSRAM on, USB CDC On Boot = Enabled.
   ============================================================================ */

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <TFT_eSPI.h>

#define FW_VERSION "1.2.1"

// 1 = run the UI on a bare T-Display S3 with NO sensor board (fake readings).
// 0 = real hardware. SET BACK TO 0 before using the actual board.
#define SIM_MODE 1

/* LED CHANNEL MAP (confirmed) — ascending wavelength; never reorder once you
   start collecting data.
     ch0 1200 nm -> GPIO 12   ch1 1450 nm -> GPIO 11
     ch2 1550 nm -> GPIO 13   ch3 1650 nm -> GPIO 10                          */
const int  LED_PIN[4] = { 12, 11, 13, 10 };
const int  LED_NM[4]  = { 1200, 1450, 1550, 1650 };

// ---- other pins ----
#define PIN_PWR_EN   15
#define PIN_BTN_PASS 17
#define PIN_BTN_FAIL 18
#define PIN_SDA      43
#define PIN_SCL      44
#define PIN_BATT     4
#define ADS_ADDR     0x48

// ---- config ----
#define AVG_SAMPLES   32
#define LED_SETTLE_MS 4
#define REF_STALE_MS  (10UL * 60UL * 1000UL)
#define MIN_REF_V     0.05f
#define LONGPRESS_MS  800

// ---- EcoSpool palette (green primary, blue/grey accents; red = alert only) ----
#define C_GREEN tft.color565(70, 155, 85)
#define C_BLUE  tft.color565(28, 52, 100)
#define C_GREY  tft.color565(150, 150, 150)
#define C_WARN  tft.color565(210, 130, 40)   // amber warning
#define C_FAIL  tft.color565(200, 60, 60)    // red alert

const char* CLASS_LABELS[] = { "PET", "HDPE", "PVC", "PP", "PS" };

// ============================================================================
Adafruit_ADS1115 ads;
TFT_eSPI tft = TFT_eSPI();

float    refSignal[4] = {0,0,0,0};
bool     haveRef      = false;
uint32_t refMillis    = 0;

enum AppState { AWAIT_SCAN, AWAIT_JUDGE };
AppState state = AWAIT_SCAN;

int    lastClass = -2;      // -2 none, -1 undecided
float  lastConf  = 0;
float  lastRefl[4] = {0,0,0,0};
bool   lastValid = false;
uint32_t nPass = 0, nFail = 0;

String   lastAction  = "ready";
uint16_t actionColor = C_GREY;
int      scanProgress = 0;   // 0..100 meter fill during a scan

// ---------------------------------------------------------------- LOW LEVEL --
void ledAllOff() { for (int i = 0; i < 4; i++) digitalWrite(LED_PIN[i], LOW); }

float readVoltsAvg(int n) {
  double acc = 0;
  for (int k = 0; k < n; k++) acc += ads.computeVolts(ads.readADC_SingleEnded(0));
  return (float)(acc / n);
}

void drawMeter(int pct);   // fwd decl (defined in DISPLAY section)

void measure(float sig[4], float rawOut[4], float* darkOut) {
  const int STEPS = 5;                 // dark + 4 LED channels
  scanProgress = 0; drawMeter(0);
#if SIM_MODE
  float dark = 0.19f + (random(-15, 15) / 10000.0f);
  scanProgress = 100 / STEPS; drawMeter(scanProgress); delay(130);
  for (int i = 0; i < 4; i++) {
    float on = 0.70f + 0.45f * i + (random(-40, 40) / 1000.0f);
    if (rawOut) rawOut[i] = on;
    sig[i] = on - dark;
    scanProgress = (i + 2) * 100 / STEPS; drawMeter(scanProgress); delay(130);
  }
  if (darkOut) *darkOut = dark;
#else
  ledAllOff();
  delay(LED_SETTLE_MS);
  float dark = readVoltsAvg(AVG_SAMPLES);
  scanProgress = 100 / STEPS; drawMeter(scanProgress);
  for (int i = 0; i < 4; i++) {
    digitalWrite(LED_PIN[i], HIGH);
    delay(LED_SETTLE_MS);
    float on = readVoltsAvg(AVG_SAMPLES);
    digitalWrite(LED_PIN[i], LOW);
    if (rawOut) rawOut[i] = on;
    sig[i] = on - dark;
    scanProgress = (i + 2) * 100 / STEPS; drawMeter(scanProgress);
    delay(1);
  }
  if (darkOut) *darkOut = dark;
#endif
}

// ------------------------------------------------------ REFERENCE / RATIO --
// (setAction is defined below; forward-referenced, fine on Arduino.)
void setAction(const char* msg, uint16_t color);

void takeReference() {
  scanProgress = 0;
  setAction("Measuring...", C_BLUE);
  float sig[4], raw[4], dark;
  measure(sig, raw, &dark);
  bool ok = true;
  for (int i = 0; i < 4; i++) { refSignal[i] = sig[i]; if (sig[i] < MIN_REF_V) ok = false; }
  haveRef = ok; refMillis = millis();
  if (ok) setAction("Reference set", C_GREEN);
  else    setAction("Ref too dark", C_FAIL);
}

bool reflectance(float refl[4]) {
  if (!haveRef) return false;
  float sig[4], raw[4], dark;
  measure(sig, raw, &dark);
  for (int i = 0; i < 4; i++) {
    if (refSignal[i] < MIN_REF_V) return false;
    refl[i] = sig[i] / refSignal[i];
  }
  return true;
}

bool refStale() { return haveRef && (millis() - refMillis > REF_STALE_MS); }

// ----------------------------------------------------------------- CLASSIFY --
int classify(const float refl[4], float* conf) {
#if SIM_MODE
  static int demo = -1;
  demo = (demo + 1) % 5;                          // cycles the 5 classes
  if (conf) *conf = 0.60f + (random(0, 40) / 100.0f);
  return demo;
#endif
  if (conf) *conf = 0.0f;
  return -1;
  /* ---- replace with the Edge Impulse call once exported ----
  #include <EcoSpool_SORT_inferencing.h>   // at top of file
  signal_t signal; float f[4] = { refl[0],refl[1],refl[2],refl[3] };
  numpy::signal_from_buffer(f, 4, &signal);
  ei_impulse_result_t r;
  if (run_classifier(&signal, &r, false) != EI_IMPULSE_OK) return -1;
  int best=-1; float bv=0;
  for (uint16_t i=0;i<EI_CLASSIFIER_LABEL_COUNT;i++)
    if (r.classification[i].value>bv){bv=r.classification[i].value;best=i;}
  if (conf)*conf=bv; return best;
  --------------------------------------------------------- */
}

// ------------------------------------------------------------------ DISPLAY --
float battVolts() { return analogReadMilliVolts(PIN_BATT) * 2.0f / 1000.0f; }

// simple 6-petal flower motif
void drawFlower(int cx, int cy, int r, uint16_t petal, uint16_t center) {
  for (int a = 0; a < 6; a++) {
    float ang = a * 1.0472f;                       // 60 deg steps
    int px = cx + (int)(cos(ang) * r);
    int py = cy + (int)(sin(ang) * r);
    tft.fillCircle(px, py, (r * 7) / 10, petal);
  }
  tft.fillCircle(cx, cy, (r * 6) / 10, center);
}

// horizontal progress meter — fills as the scan proceeds (0..100)
void drawMeter(int pct) {
  const int x = 15, y = 228, w = 140, h = 16;
  if (pct < 0) pct = 0; if (pct > 100) pct = 100;
  tft.fillRect(x - 2, y - 16, w + 4, h + 22, TFT_BLACK);   // clear caption + bar
  tft.setTextDatum(TL_DATUM); tft.setTextSize(1);
  tft.setTextColor(C_GREY, TFT_BLACK);
  tft.setCursor(x, y - 13); tft.print("scan");
  tft.drawRoundRect(x, y, w, h, 4, C_GREY);                // track
  int fw = (w - 4) * pct / 100;
  if (fw > 0) tft.fillRoundRect(x + 2, y + 2, fw, h - 4, 3, C_GREEN);  // fill
  tft.setTextDatum(MC_DATUM); tft.setTextColor(TFT_WHITE); // transparent (region cleared)
  char b[8]; snprintf(b, sizeof(b), "%d%%", pct);
  tft.drawString(b, x + w / 2, y + h / 2);
  tft.setTextDatum(TL_DATUM);
}

void drawScreen() {
  tft.fillScreen(TFT_BLACK);

  // ---- title ----
  tft.setTextDatum(TL_DATUM);
  tft.setTextSize(2); tft.setTextColor(C_GREEN, TFT_BLACK);
  tft.setCursor(8, 8);  tft.print("EcoSpool");
  tft.setTextSize(1); tft.setTextColor(C_GREY, TFT_BLACK);
  tft.setCursor(8, 28); tft.print("SORT " FW_VERSION);
  drawFlower(150, 20, 9, C_GREEN, TFT_WHITE);      // accent
  tft.drawFastHLine(8, 42, 154, C_GREEN);

  // ---- reference status + battery ----
  tft.setTextSize(1);
  tft.setCursor(8, 50);
  if (!haveRef)        { tft.setTextColor(C_WARN, TFT_BLACK); tft.print("REF: none"); }
  else if (refStale()) { tft.setTextColor(C_WARN, TFT_BLACK); tft.printf("REF %lus STALE", (millis()-refMillis)/1000); }
  else                 { tft.setTextColor(C_GREEN, TFT_BLACK); tft.printf("REF %lus ok", (millis()-refMillis)/1000); }
  tft.setTextColor(C_GREY, TFT_BLACK);
  tft.setCursor(120, 50); tft.printf("%.2fV", battVolts());

  // ---- action banner (centered) ----
  tft.setTextDatum(TC_DATUM);
  tft.setTextSize(2); tft.setTextColor(actionColor, TFT_BLACK);
  tft.drawString(lastAction, 85, 68);

  // ---- plastic type (hero) ----
  tft.setTextSize(4); tft.setTextColor(C_GREEN, TFT_BLACK);
  if (lastClass == -2)      tft.drawString("----", 85, 108);
  else if (lastClass == -1) tft.drawString("?", 85, 108);
  else                      tft.drawString(CLASS_LABELS[lastClass], 85, 108);

  // ---- confidence ----
  tft.setTextSize(2); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  if (lastClass >= 0) { char b[16]; snprintf(b, sizeof(b), "conf %.0f%%", lastConf*100.0f); tft.drawString(b, 85, 158); }
  else                tft.drawString("conf --", 85, 158);

  // ---- tally ----
  tft.setTextSize(1); tft.setTextColor(C_GREY, TFT_BLACK);
  { uint32_t tot = nPass + nFail; char b[28];
    snprintf(b, sizeof(b), "OK %lu   X %lu   (%lu%%)", nPass, nFail, tot ? (nPass*100/tot) : 0);
    tft.drawString(b, 85, 188); }

  // ---- scan progress meter ----
  drawMeter(scanProgress);
  tft.drawFastHLine(8, 262, 154, C_GREEN);

  // ---- prompt (centered, bottom) ----
  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  if (!haveRef)                 tft.drawString("Hold PASS on white ref", 85, 290);
  else if (state == AWAIT_SCAN) tft.drawString("PASS = scan", 85, 290);
  else                          tft.drawString("PASS=OK    FAIL=wrong", 85, 290);

  tft.setTextDatum(TL_DATUM);
}

void setAction(const char* msg, uint16_t color) {
  lastAction = msg; actionColor = color; drawScreen();
}

// ------------------------------------------------------------------ SERIAL --
void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n'); line.trim();
  if (line == "raw") {
    float sig[4], raw[4], dark; measure(sig, raw, &dark);
    Serial.printf("dark=%.4f V\n", dark);
    for (int i = 0; i < 4; i++)
      Serial.printf("  ch%d %4dnm raw=%.4f sig=%.4f\n", i, LED_NM[i], raw[i], sig[i]);
    Serial.println("  (target: dark ~0.19 V, brightest raw ~2.8 V)");
  } else if (line == "ref") {
    takeReference();
    Serial.println(haveRef ? "reference captured" : "REF FAILED (too dark)");
  } else if (line == "ref?") {
    if (!haveRef) { Serial.println("no reference"); return; }
    Serial.printf("ref age %lus:", (millis()-refMillis)/1000);
    for (int i = 0; i < 4; i++) Serial.printf(" %.4f", refSignal[i]);
    Serial.println();
  } else if (line.startsWith("s")) {
    String label = ""; int sp = line.indexOf(' ');
    if (sp > 0) label = line.substring(sp + 1);
    float refl[4];
    if (!reflectance(refl)) { Serial.println("ERR: no/bad reference - run 'ref'"); return; }
    for (int i = 0; i < 4; i++) Serial.printf(i ? ",%.5f" : "%.5f", refl[i]);
    if (label.length()) Serial.printf(",%s", label.c_str());
    Serial.println();
  } else if (line.length()) {
    Serial.println("cmds: raw | ref | ref? | s [label]");
  }
}

// ------------------------------------------------------------------ BUTTONS --
int readButton(int pin, bool meter = false) {   // 0 none, 1 tap, 2 hold
  if (digitalRead(pin) != LOW) return 0;
  // confirm a real press: LOW sustained ~15 ms (rejects contact bounce)
  uint32_t p0 = millis();
  while (millis() - p0 < 15) { if (digitalRead(pin) != LOW) return 0; delay(2); }

  uint32_t t0 = millis(), highStart = 0;
  int lastPct = -1; bool longp = false;
  while (true) {
    if (digitalRead(pin) == LOW) {                  // still held
      highStart = 0;
      uint32_t held = millis() - t0;
      if (meter) {
        int pct = ((int)(held * 100 / LONGPRESS_MS)) / 5 * 5;
        if (pct > 100) pct = 100;
        if (pct != lastPct) { drawMeter(pct); lastPct = pct; }
      }
      if (held > LONGPRESS_MS) longp = true;
    } else {                                         // maybe released
      if (highStart == 0) highStart = millis();
      else if (millis() - highStart > 40) break;     // 40 ms stable HIGH = real release
    }
    delay(3);
  }
  if (meter && !longp) { scanProgress = 0; drawMeter(0); }  // tap -> clear meter
  return longp ? 2 : 1;
}

void doScan() {
  if (!haveRef) { setAction("Need ref", C_WARN); return; }
  scanProgress = 0;
  setAction("Scanning...", C_BLUE);
  float refl[4];
  if (!reflectance(refl)) { setAction("Ref bad", C_WARN); return; }
  float conf; int cls = classify(refl, &conf);
  for (int i = 0; i < 4; i++) lastRefl[i] = refl[i];
  lastClass = cls; lastConf = conf; lastValid = true;
  state = AWAIT_JUDGE;
  setAction("Scanned", C_GREEN);
  Serial.print("SCAN ");
  for (int i = 0; i < 4; i++) Serial.printf(i ? ",%.4f" : "%.4f", refl[i]);
  Serial.printf("  -> %s (%.0f%%)\n", cls >= 0 ? CLASS_LABELS[cls] : "?", conf * 100.0f);
}

// --------------------------------------------------------------------- SETUP --
void setup() {
  pinMode(PIN_PWR_EN, OUTPUT); digitalWrite(PIN_PWR_EN, HIGH);
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) { pinMode(LED_PIN[i], OUTPUT); digitalWrite(LED_PIN[i], LOW); }
  pinMode(PIN_BTN_PASS, INPUT_PULLUP);
  pinMode(PIN_BTN_FAIL, INPUT_PULLUP);

  tft.init();
  tft.setRotation(0);            // portrait (90 CCW from landscape; use 2 if flipped)

#if !SIM_MODE
  Wire.begin(PIN_SDA, PIN_SCL);
  if (!ads.begin(ADS_ADDR, &Wire)) {
    tft.fillScreen(C_FAIL); tft.setTextColor(TFT_WHITE);
    tft.setCursor(6, 6); tft.print("ADS1115 not found");
    while (1) delay(1000);
  }
  ads.setGain(GAIN_ONE);
  ads.setDataRate(RATE_ADS1115_860SPS);
#endif

  drawScreen();
#if SIM_MODE
  setAction("SIM MODE", C_BLUE);
#endif
  Serial.println("EcoSpool SORT " FW_VERSION " ready. cmds: raw | ref | ref? | s [label]");
}

// ---------------------------------------------------------------------- LOOP --
void loop() {
  handleSerial();

  int p = readButton(PIN_BTN_PASS, !haveRef);   // hold-meter only until referenced
  if (p == 2) {
    if (!haveRef) takeReference();                // locks after first success
    else { scanProgress = 0; setAction("Ref locked", C_GREY); }
  } else if (p == 1) {
    if (state == AWAIT_SCAN) doScan();
    else { if (lastValid) { nPass++; Serial.println("JUDGE pass"); }
           state = AWAIT_SCAN; setAction("Correct", C_GREEN); }
  }

  if (readButton(PIN_BTN_FAIL) == 1) {
    if (state == AWAIT_JUDGE && lastValid) { nFail++; Serial.println("JUDGE fail"); }
    state = AWAIT_SCAN; setAction("Wrong", C_FAIL);
  }

  static uint32_t lastHdr = 0;
  if (millis() - lastHdr > 2000) { drawScreen(); lastHdr = millis(); }
}
