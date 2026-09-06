// EcoSpool SORT scanner - ESP32-S3 (LilyGO T-Display S3)
// 4 SWIR LEDs + G12180 InGaAs + OPA392 TIA + ADS1115
//
// Every scan does DARK -> each LED ON -> divide by a stored white reference,
// so classify() sees reflectance (0-1) instead of raw voltage. Raw voltage
// changes too much with ambient light and battery level to train on directly.
//
// PASS (17): tap = scan / mark correct, hold ~0.8s = take new reference
// FAIL (18): tap = mark wrong
// serial cmds: raw | ref | ref? | s [label]  -- for bring-up + CSV logging
//
// libs: Adafruit_ADS1X15, TFT_eSPI (Setup206_LilyGo_T_Display_S3.h)
// board settings: ESP32S3 Dev Module, PSRAM on, USB CDC On Boot = Enabled

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <TFT_eSPI.h>

#define FW_VERSION "1.2.1"

// set to 1 to test the UI on a bare board with no sensor attached (fake data)
#define SIM_MODE 0

// LED channels, low to high wavelength. don't reorder these once real data
// collection starts or the Edge Impulse model + this mapping go out of sync
const int LED_PIN[4] = { 12, 11, 13, 10 };
const int LED_NM[4]  = { 1200, 1450, 1550, 1650 };

#define PIN_PWR_EN   15
#define PIN_BTN_PASS 17
#define PIN_BTN_FAIL 18
#define PIN_SDA      43
#define PIN_SCL      44
#define PIN_BATT     4
#define ADS_ADDR     0x48

#define AVG_SAMPLES   32
#define LED_SETTLE_MS 4
#define REF_STALE_MS  (10UL * 60UL * 1000UL)
#define MIN_REF_V     0.05f   // below this the reference read is basically noise
#define LONGPRESS_MS  800

// green/blue/grey palette, red reserved for alerts only
#define C_GREEN tft.color565(70, 155, 85)
#define C_BLUE  tft.color565(28, 52, 100)
#define C_GREY  tft.color565(150, 150, 150)
#define C_WARN  tft.color565(210, 130, 40)
#define C_FAIL  tft.color565(200, 60, 60)

const char* CLASS_LABELS[] = { "PET", "HDPE", "PVC", "PP", "PS" };

Adafruit_ADS1115 ads;
TFT_eSPI tft = TFT_eSPI();

float    refSignal[4] = {0,0,0,0};
bool     haveRef      = false;
uint32_t refMillis    = 0;

enum AppState { AWAIT_SCAN, AWAIT_JUDGE };
AppState state = AWAIT_SCAN;

int    lastClass = -2;      // -2 = nothing scanned yet, -1 = classifier gave up
float  lastConf  = 0;
float  lastRefl[4] = {0,0,0,0};
bool   lastValid = false;
uint32_t nPass = 0, nFail = 0;

String   lastAction  = "ready";
uint16_t actionColor = C_GREY;
int      scanProgress = 0;

// ---- low level ----

void ledAllOff() { for (int i = 0; i < 4; i++) digitalWrite(LED_PIN[i], LOW); }

float readVoltsAvg(int n) {
  double acc = 0;
  for (int k = 0; k < n; k++) acc += ads.computeVolts(ads.readADC_SingleEnded(0));
  return (float)(acc / n);
}

void drawMeter(int pct);

void measure(float sig[4], float rawOut[4], float* darkOut) {
  const int STEPS = 5;
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
    // clamp - noisy reads can put "on" slightly below "dark" and a negative
    // reflectance value later just confuses the classifier for no reason
    sig[i] = max(0.0f, on - dark);
    scanProgress = (i + 2) * 100 / STEPS; drawMeter(scanProgress);
    delay(1);
  }
  if (darkOut) *darkOut = dark;
#endif
}

void setAction(const char* msg, uint16_t color);  // defined further down, used earlier

void takeReference() {
  scanProgress = 0;
  setAction("Measuring...", C_BLUE);
  float sig[4], raw[4], dark;
  measure(sig, raw, &dark);
  bool ok = true;
  for (int i = 0; i < 4; i++) { refSignal[i] = sig[i]; if (sig[i] < MIN_REF_V) ok = false; }
  haveRef = ok; refMillis = millis();
  setAction(ok ? "Reference set" : "Ref too dark", ok ? C_GREEN : C_FAIL);
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

// ---- classify ----
// stub for now - swap in the Edge Impulse call once there's a trained model.
// SIM_MODE just cycles through the 5 labels so the UI is testable without
// a real sensor attached.
int classify(const float refl[4], float* conf) {
#if SIM_MODE
  static int demo = -1;
  demo = (demo + 1) % 5;
  if (conf) *conf = 0.60f + (random(0, 40) / 100.0f);
  return demo;
#else
  if (conf) *conf = 0.0f;
  return -1;
  /* once EI model is exported:
  #include <EcoSpool_SORT_inferencing.h>
  signal_t signal; float f[4] = { refl[0],refl[1],refl[2],refl[3] };
  numpy::signal_from_buffer(f, 4, &signal);
  ei_impulse_result_t r;
  if (run_classifier(&signal, &r, false) != EI_IMPULSE_OK) return -1;
  int best=-1; float bv=0;
  for (uint16_t i=0;i<EI_CLASSIFIER_LABEL_COUNT;i++)
    if (r.classification[i].value>bv){bv=r.classification[i].value;best=i;}
  if (conf)*conf=bv; return best;
  */
#endif
}

// ---- display ----

float battVolts() { return analogReadMilliVolts(PIN_BATT) * 2.0f / 1000.0f; }

void drawFlower(int cx, int cy, int r, uint16_t petal, uint16_t center) {
  for (int a = 0; a < 6; a++) {
    float ang = a * 1.0472f;
    int px = cx + (int)(cos(ang) * r);
    int py = cy + (int)(sin(ang) * r);
    tft.fillCircle(px, py, (r * 7) / 10, petal);
  }
  tft.fillCircle(cx, cy, (r * 6) / 10, center);
}

void drawMeter(int pct) {
  const int x = 15, y = 228, w = 140, h = 16;
  if (pct < 0) pct = 0; if (pct > 100) pct = 100;
  tft.fillRect(x - 2, y - 16, w + 4, h + 22, TFT_BLACK);
  tft.setTextDatum(TL_DATUM); tft.setTextSize(1);
  tft.setTextColor(C_GREY, TFT_BLACK);
  tft.setCursor(x, y - 13); tft.print("scan");
  tft.drawRoundRect(x, y, w, h, 4, C_GREY);
  int fw = (w - 4) * pct / 100;
  if (fw > 0) tft.fillRoundRect(x + 2, y + 2, fw, h - 4, 3, C_GREEN);
  tft.setTextDatum(MC_DATUM); tft.setTextColor(TFT_WHITE);
  char b[8]; snprintf(b, sizeof(b), "%d%%", pct);
  tft.drawString(b, x + w / 2, y + h / 2);
  tft.setTextDatum(TL_DATUM);
}

void drawScreen() {
  tft.fillScreen(TFT_BLACK);

  tft.setTextDatum(TL_DATUM);
  tft.setTextSize(2); tft.setTextColor(C_GREEN, TFT_BLACK);
  tft.setCursor(8, 8);  tft.print("EcoSpool");
  tft.setTextSize(1); tft.setTextColor(C_GREY, TFT_BLACK);
  tft.setCursor(8, 28); tft.print("SORT " FW_VERSION);
  drawFlower(150, 20, 9, C_GREEN, TFT_WHITE);
  tft.drawFastHLine(8, 42, 154, C_GREEN);

  tft.setTextSize(1);
  tft.setCursor(8, 50);
  if (!haveRef)        { tft.setTextColor(C_WARN, TFT_BLACK); tft.print("REF: none"); }
  else if (refStale()) { tft.setTextColor(C_WARN, TFT_BLACK); tft.printf("REF %lus STALE", (millis()-refMillis)/1000); }
  else                 { tft.setTextColor(C_GREEN, TFT_BLACK); tft.printf("REF %lus ok", (millis()-refMillis)/1000); }
  tft.setTextColor(C_GREY, TFT_BLACK);
  tft.setCursor(120, 50); tft.printf("%.2fV", battVolts());

  tft.setTextDatum(TC_DATUM);
  tft.setTextSize(2); tft.setTextColor(actionColor, TFT_BLACK);
  tft.drawString(lastAction, 85, 68);

  tft.setTextSize(4); tft.setTextColor(C_GREEN, TFT_BLACK);
  if (lastClass == -2)      tft.drawString("----", 85, 108);
  else if (lastClass == -1) tft.drawString("?", 85, 108);
  else                      tft.drawString(CLASS_LABELS[lastClass], 85, 108);

  tft.setTextSize(2); tft.setTextColor(TFT_WHITE, TFT_BLACK);
  if (lastClass >= 0) { char b[16]; snprintf(b, sizeof(b), "conf %.0f%%", lastConf*100.0f); tft.drawString(b, 85, 158); }
  else                tft.drawString("conf --", 85, 158);

  tft.setTextSize(1); tft.setTextColor(C_GREY, TFT_BLACK);
  { uint32_t tot = nPass + nFail; char b[28];
    snprintf(b, sizeof(b), "OK %lu   X %lu   (%lu%%)", nPass, nFail, tot ? (nPass*100/tot) : 0);
    tft.drawString(b, 85, 188); }

  drawMeter(scanProgress);
  tft.drawFastHLine(8, 262, 154, C_GREEN);

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

// ---- serial ----
// raw = dump one dark+4LED cycle, ref = capture reference, ref? = show it,
// s [label] = one reflectance row for the CSV, with an optional class label
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
    label.replace(",", ";");   // commas in a label would break the CSV column count
    float refl[4];
    if (!reflectance(refl)) { Serial.println("ERR: no/bad reference - run 'ref'"); return; }
    for (int i = 0; i < 4; i++) Serial.printf(i ? ",%.5f" : "%.5f", refl[i]);
    if (label.length()) Serial.printf(",%s", label.c_str());
    Serial.println();
  } else if (line.length()) {
    Serial.println("cmds: raw | ref | ref? | s [label]");
  }
}

// ---- buttons ----
// returns 0 nothing, 1 tap, 2 hold. meter=true draws the hold progress bar
// (only used for PASS before a reference exists)
int readButton(int pin, bool meter = false) {
  if (digitalRead(pin) != LOW) return 0;
  uint32_t p0 = millis();
  while (millis() - p0 < 15) { if (digitalRead(pin) != LOW) return 0; delay(2); }  // debounce

  uint32_t t0 = millis(), highStart = 0;
  int lastPct = -1; bool longp = false;
  while (true) {
    if (digitalRead(pin) == LOW) {
      highStart = 0;
      uint32_t held = millis() - t0;
      if (meter) {
        int pct = ((int)(held * 100 / LONGPRESS_MS)) / 5 * 5;
        if (pct > 100) pct = 100;
        if (pct != lastPct) { drawMeter(pct); lastPct = pct; }
      }
      if (held > LONGPRESS_MS) longp = true;
    } else {
      if (highStart == 0) highStart = millis();
      else if (millis() - highStart > 40) break;
    }
    delay(3);
  }
  if (meter && !longp) { scanProgress = 0; drawMeter(0); }
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

void setup() {
  pinMode(PIN_PWR_EN, OUTPUT); digitalWrite(PIN_PWR_EN, HIGH);
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) { pinMode(LED_PIN[i], OUTPUT); digitalWrite(LED_PIN[i], LOW); }
  pinMode(PIN_BTN_PASS, INPUT_PULLUP);
  pinMode(PIN_BTN_FAIL, INPUT_PULLUP);

  tft.init();
  tft.setRotation(0);   // portrait - use 2 if the display comes up upside down

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

void loop() {
  handleSerial();

  int p = readButton(PIN_BTN_PASS, !haveRef);
  if (p == 2) {
    if (!haveRef) takeReference();
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
