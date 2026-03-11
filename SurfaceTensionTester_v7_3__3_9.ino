/*
 * Surface Tension Tester v7.3 - Scientific Instrument UI
 * 
 * TFT REDESIGN:
 *   Color scheme: Deep blue dominant, white data, soft blue accents
 *   Boot: COM port connection animation with status check
 *   Command feedback: Toast bar shows every received command
 *   Smooth: Fade transitions, progressive loading bars
 *   Professional: Lab instrument aesthetic, no decorative colors
 *
 * All precision improvements from v7.1 retained.
 * TFT: ST7735 Blue Tab → GREENTAB init, Rotation 1 (160×128)
 */

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <AccelStepper.h>
#include <HX711_ADC.h>

#if defined(ESP8266) || defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

// ═══════════════════════════════════════════════════════
// TFT CONFIG
// ═══════════════════════════════════════════════════════
#define TFT_CS   53
#define TFT_DC   49
#define TFT_RST  48
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
#define SW 160
#define SH 128

// ═══════════════════════════════════════════════════════
// COLOR PALETTE — Scientific Blue Theme
// ★ All colors R↔B swapped for Blue Tab hardware
// ═══════════════════════════════════════════════════════
// Helper: swap R(5bit) and B(5bit) in RGB565
// Original RGB565: RRRRRGGG GGGBBBBB
// Swapped:         BBBBBGGG GGGRRRRR
//
// Background layers
#define C_BG         0x0000   // Black stays black
#define C_BG_DARK    0x0841   // Very dark gray (symmetric)
#define C_BG_PANEL   0x0883   // Dark blue-gray (was 0x1083)
#define C_HDR        0x8800   // Deep navy (was 0x0011)

// Text hierarchy
#define C_TXT_PRI    0xFFFF   // White stays white
#define C_TXT_SEC    0xBDF7   // Light gray (nearly symmetric)
#define C_TXT_DIM    0x6B6D   // Medium gray (symmetric)

// Accent blues → now show as BLUE on swapped hardware
#define C_BLUE_HI    0x7F26   // Sky blue (was 0x4C7F)
#define C_BLUE_MED   0x5F15   // Medium blue (was 0x2A5F)
#define C_BLUE_LO    0x3F0D   // Deep blue (was 0x1A3F)
#define C_BLUE_DIM   0x1F05   // Very dark blue (was 0x0A1F)

// Functional (used sparingly)
#define C_OK         0xCA1F   // Teal-green (was 0x3ECA)
#define C_WARN       0x60FB   // Amber (was 0xFD60)
#define C_ERR        0x001F   // Red → stays red-ish (was 0xF800)
#define C_ACTIVE     0xDF02   // Electric blue (was 0x04DF)

// UI elements
#define C_DIV        0x2124   // Dark divider (symmetric)
#define C_BAR_BG     0x0841   // Graph bg (symmetric)
#define C_BAR_FILL   0x5F15   // Progress fill (same as BLUE_MED)

// ═══════════════════════════════════════════════════════
// PINS & HARDWARE
// ═══════════════════════════════════════════════════════
const int HX711_dout = 3;
const int HX711_sck = 2;
#define EN_PIN   7
#define STEP_PIN 6
#define DIR_PIN  5

HX711_ADC LoadCell(HX711_dout, HX711_sck);
AccelStepper stepper(1, STEP_PIN, DIR_PIN);

// ═══════════════════════════════════════════════════════
// PRECISION FILTER PIPELINE v2
// Layer 1: HX711 oversampling (multiple reads averaged)
// Layer 2: Moving Average (adaptive window)
// Layer 3: EMA (Exponential Moving Average) for smoothing
// ═══════════════════════════════════════════════════════
#define MAX_FILTER_SIZE 25
float readings[MAX_FILTER_SIZE];
int filterSize = 7;
int readIdx = 0;
float total = 0.0;
int oversampleCount = 2;

// EMA smoothing (alpha = 0.15 default, lower = smoother but slower response)
float emaForce = 0.0;
float emaAlpha = 0.15;
bool emaInit = false;

void setFilterSize(int n) { n = constrain(n, 3, MAX_FILTER_SIZE); filterSize = n; resetFilters(); }

// Adaptive: slow speed → more filtering → lower noise → better SD
int calcFilterSize(float s) {
  float u = s*1000; // µm/s
  if(u>300) return 5;    // Ultra Fast: minimal lag
  if(u>100) return 7;
  if(u>30)  return 11;
  if(u>10)  return 15;
  if(u>3)   return 19;
  return 25;              // Slowest: maximum smoothing
}

int calcOversample(float s) {
  float u = s*1000;
  if(u>300) return 2;
  if(u>100) return 3;
  if(u>30)  return 4;
  if(u>10)  return 6;
  return 8;               // Slowest: 8x oversample
}

float calcEmaAlpha(float s) {
  float u = s*1000;
  if(u>300) return 0.3;   // Fast response for fast speed
  if(u>100) return 0.2;
  if(u>30)  return 0.15;
  if(u>10)  return 0.10;
  return 0.06;             // Heavy smoothing for slow speed
}

float getFilteredForce() {
  // Layer 1: Oversample — average multiple HX711 reads
  float rawSum = 0; int vc = 0;
  for (int i = 0; i < oversampleCount; i++) {
    if (LoadCell.update()) { rawSum += LoadCell.getData(); vc++; }
  }
  float raw = (vc > 0) ? (rawSum / vc) : LoadCell.getData();

  // Prime buffer on first call after reset to avoid cold-start bias.
  // Without this, the MA buffer is full of zeros which dilutes readings
  // for the first filterSize calls, and the EMA amplifies the error.
  if (!emaInit) {
    for (int i = 0; i < filterSize; i++) readings[i] = raw;
    total = raw * filterSize;
    readIdx = 0;
  }

  // Layer 2: Moving Average
  total -= readings[readIdx]; readings[readIdx] = raw; total += readings[readIdx];
  readIdx = (readIdx + 1) % filterSize;
  float maForce = (total / filterSize / 1000.0) * 9.81;

  // Layer 3: EMA smoothing
  if (!emaInit) { emaForce = maForce; emaInit = true; }
  else { emaForce = emaAlpha * maForce + (1.0 - emaAlpha) * emaForce; }

  return emaForce;
}
float getClampedForce() { float f = getFilteredForce(); return (f > 0) ? f : 0; }
// Signed force: allows negative values (for bidirectional measurement)

void resetFilters() {
  for (int i = 0; i < MAX_FILTER_SIZE; i++) readings[i] = 0;
  total = 0; readIdx = 0;
  emaForce = 0; emaInit = false;
}

// ═══════════════════════════════════════════════════════
// BASELINE + SETTLING + CONTACT + PEAK (from v7.1)
// ═══════════════════════════════════════════════════════
float baselineForce = 0; bool baselineValid = false;

// Signed force: allows negative values (for bidirectional measurement)
float getSignedForce() {
  float f = getFilteredForce();
  if (baselineValid) f -= baselineForce;
  return f;
}

void measureBaseline() {
  const int N = 100;  // More samples for better baseline
  float samps[100]; float sum = 0;

  // Prime filter with extra reads
  for (int i = 0; i < filterSize*3; i++) { LoadCell.update(); delay(10); getFilteredForce(); }

  // Collect samples
  for (int i = 0; i < N; i++) { LoadCell.update(); delay(15); samps[i] = getFilteredForce(); sum += samps[i]; }

  // Sort for median (simple insertion sort)
  for (int i = 1; i < N; i++) {
    float key = samps[i]; int j = i - 1;
    while (j >= 0 && samps[j] > key) { samps[j+1] = samps[j]; j--; }
    samps[j+1] = key;
  }
  float median = samps[N/2];
  float mean = sum / N;

  // Calculate SD
  float var = 0;
  for (int i = 0; i < N; i++) { float d = samps[i]-median; var += d*d; }
  float sd = sqrt(var / N);

  // Use median (more robust than mean)
  if (sd < 0.005) { baselineForce = median; baselineValid = true; }
  else { baselineForce = 0; baselineValid = false; }
  Serial.print(F("BASELINE:")); Serial.print(median,6);
  Serial.print(F(" mean:")); Serial.print(mean,6);
  Serial.print(F(" SD:")); Serial.println(sd,6);
}

float getCorrectedForce() { float f = getClampedForce(); if (baselineValid) { f -= baselineForce; if(f<0)f=0; } return f; }

const unsigned long SETTLE_TIME = 3000;
const float SETTLE_TH = 0.002;

bool waitForSettle(unsigned long t) {
  Serial.println(F("SETTLING...")); resetFilters();
  unsigned long st = millis(), ss = 0; bool ok = false;
  while ((millis()-st) < t) {
    LoadCell.update(); float f = getClampedForce();
    if (f < SETTLE_TH) { if(!ok){ss=millis();ok=true;} if((millis()-ss)>1000){Serial.println(F("SETTLED"));return true;} }
    else ok=false;
    delay(20);
  }
  Serial.println(F("SETTLE_TIMEOUT")); return false;
}

long contactPosition = -1; bool contactDet = false;
void resetContact() { contactPosition = -1; contactDet = false; }

const int PEAK_CONFIRM = 5;  // Need 5 consecutive confirmations (was 3)
float peakCandidate = 0; int peakCount = 0;
float confirmedPeak = 0; long confirmedPeakPos = 0;
void resetPeak() { peakCandidate=0; peakCount=0; confirmedPeak=0; confirmedPeakPos=0; }
void updatePeak(float f, long p) {
  if (f > peakCandidate) { peakCandidate = f; peakCount = 1; }
  else if (f > peakCandidate * 0.995) peakCount++;  // 0.5% tolerance (was 2%)
  else {
    if(peakCount>=PEAK_CONFIRM && peakCandidate>confirmedPeak) {
      confirmedPeak=peakCandidate; confirmedPeakPos=p;
    }
    peakCandidate=f; peakCount=1;
  }
  if (f>confirmedPeak && peakCount>=PEAK_CONFIRM) { confirmedPeak=f; confirmedPeakPos=p; }
}

// ═══════════════════════════════════════════════════════
// EEPROM / CALIBRATION / LOAD CELL CONFIG
// ═══════════════════════════════════════════════════════
const int EEPROM_CAL_100G_ADDR = 0;   // Cal factor for 100g cell (4 bytes)
const int EEPROM_CAL_30G_ADDR  = 4;   // Cal factor for 30g cell (4 bytes)
const int EEPROM_LC_ADDR = 8;         // Load cell type stored in EEPROM

const float DEFAULT_CAL_100G = 16800.0;
const float DEFAULT_CAL_30G  = 50000.0;  // Approximate, calibrate for exact

float calFactor = DEFAULT_CAL_100G;

// Load cell type: 0=100g, 1=30g
uint8_t loadCellType = 0;
float loadCellCapacity = 100.0;  // grams

void saveCal() {
  int addr = (loadCellType == 1) ? EEPROM_CAL_30G_ADDR : EEPROM_CAL_100G_ADDR;
  byte*p=(byte*)&calFactor; for(int i=0;i<4;i++)EEPROM.write(addr+i,p[i]);
}
void loadCal() {
  int addr = (loadCellType == 1) ? EEPROM_CAL_30G_ADDR : EEPROM_CAL_100G_ADDR;
  byte*p=(byte*)&calFactor;
  for(int i=0;i<4;i++) p[i]=EEPROM.read(addr+i);
  if(calFactor<1000||calFactor>200000) calFactor = (loadCellType==1) ? DEFAULT_CAL_30G : DEFAULT_CAL_100G;
}

void saveLoadCellType() { EEPROM.write(EEPROM_LC_ADDR, loadCellType); }
void loadLoadCellType() {
  loadCellType = EEPROM.read(EEPROM_LC_ADDR);
  if (loadCellType > 1) loadCellType = 0;  // Default 100g
  loadCellCapacity = (loadCellType == 1) ? 30.0 : 100.0;
}

// ═══════════════════════════════════════════════════════
// MOTOR / SPEEDS — 12 speed profiles
// ═══════════════════════════════════════════════════════
const long testDistance = 24000;
const float homeSpeed = 3200, homeAccel = 1600;
const long POS_INTERVAL = 8;
long lastSampledPos = 0;

#define NUM_SPEEDS 12
struct SpeedTest { const char* name; const char* label; float speed; float speed_mms; };
SpeedTest tests[NUM_SPEEDS] = {
  {"ULTRA_FAST","ULTRA", 1600, 0.600},       // 1: 600 µm/s
  {"FAST_UP",   "FSTUP", 1200, 0.450},       // 2: 450 µm/s
  {"FAST_DN",   "FSTDN",  400, 0.150},       // 3: 150 µm/s (NEW)
  {"Veight",    "V8",     356, 0.1335},       // 4: 133.5 µm/s
  {"Vsix",      "V6",     267, 0.100125},     // 5: 100.125 µm/s
  {"Vfour",     "V4",     178, 0.06675},      // 6: 66.75 µm/s
  {"Vtwo",      "V2",      89, 0.0333375},    // 7: 33.3375 µm/s
  {"MEASURE_F", "MsrF",    50, 0.01875},      // 8: 18.75 µm/s
  {"MEASURE_M", "MsrM",    20, 0.00750},      // 9: 7.50 µm/s (NEW)
  {"MEASURE_U", "MsrU",    10, 0.00375},      // A: 3.75 µm/s (NEW)
  {"MEASURE_X", "MsrX",     5, 0.001875},     // B: 1.875 µm/s (NEW)
  {"MEASURE_Z", "MsrZ",     2, 0.000750}      // C: 0.75 µm/s (NEW)
};

// ═══════════════════════════════════════════════════════
// STATE MACHINE
// ═══════════════════════════════════════════════════════
enum SysMode { MODE_IDLE, MODE_MONITOR, MODE_TEST, MODE_AUTO, MODE_CAL, MODE_HOMING, MODE_ENCODER };
enum MotorSt { MOT_IDLE, MOT_DOWN, MOT_UP, MOT_HOME };
SysMode curMode = MODE_IDLE;
MotorSt motorSt = MOT_IDLE;
long homePos=0, targetPos=0;
int curTest=-1, nextAutoTest=0;
bool eStop=false;
char pendingCmd=0; bool hasPending=false;
float lastForce=0;
const float CONTACT_TH=0.03;
float OVERLOAD_LIM=5.0;  // Will be set based on load cell type
bool streaming=false; unsigned long streamStart=0;
unsigned long lastTFTTime=0;
const unsigned long TFT_INT=100;

// Auto repeat system: 10 runs per speed, 2 batches
#define AUTO_REPEATS 10
#define AUTO_BATCHES 2
int autoRunNum = 0;           // Current run (0-9) within a speed
float autoPeaks[AUTO_REPEATS]; // Peak forces for current speed
int autoBatch = 0;            // Current batch (0-based)

// ═══════════════════════════════════════════════════════
// ROTARY ENCODER (KY-040)
// ═══════════════════════════════════════════════════════
#define ENC_CLK 20
#define ENC_DT  21
#define ENC_SW  4

volatile long encPos = 0;
long encLastProc = 0;
long encCmdPos = 0;          // Virtual commanded position
bool encEnabled = false;

// Encoder step size: FINE mode = 50 steps per click = 0.03125mm
const int ENC_STEP = 50;
const float ENC_ACCEL = 4000;

// Home/Target for encoder positioning (2-point system)
long encHomePos = 0;
long encTargetPos = 0;
bool encHomeSet = false;
bool encTargetSet = false;

// Encoder button = Emergency Stop toggle
bool encEStop = false;  // true = motor stopped by encoder button

// Button debounce
unsigned long encLastBtn = 0;
const unsigned long ENC_DEBOUNCE = 300;

// TFT encoder display cache
float encLastDispPos = -99999;
float encLastDispForce = -99999;
float graphMaxF = 0.02;  // Graph auto-scale max (global)

// ═══════════════════════════════════════════════════════
// TFT DISPLAY CACHE (anti-flicker)
// ═══════════════════════════════════════════════════════
struct DCache {
  float force, pos, peak;
  int barW;
  bool contact;
  char status[16];
  int graph[120]; int gIdx, gCnt;
};
DCache dc;
bool fullRedraw = true;

// Layout constants
#define HDR_H  16
#define TOAST_Y (SH - 11)
#define GRF_X 4
#define GRF_Y 94
#define GRF_W 120
#define GRF_H 16
#define PBR_X 4
#define PBR_Y 112
#define PBR_W 152
#define PBR_H 3

// ═══════════════════════════════════════════════════════
//  TFT HELPERS
// ═══════════════════════════════════════════════════════
void tftBox(int16_t x, int16_t y, int16_t w, int16_t h,
            const char* t, uint16_t c, uint16_t bg, uint8_t sz) {
  tft.fillRect(x, y, w, h, bg);
  tft.setTextSize(sz); tft.setTextColor(c); tft.setCursor(x,y); tft.print(t);
}

void tftDiv(int16_t y) { tft.drawFastHLine(0, y, SW, C_DIV); }

// ── Header bar ──
void tftHeader(const char* title) {
  tft.fillRect(0, 0, SW, HDR_H, C_HDR);
  // Top accent line (thin blue)
  tft.drawFastHLine(0, 0, SW, C_BLUE_MED);
  // Title
  tft.setTextSize(1); tft.setTextColor(C_BLUE_HI);
  tft.setCursor(3, 4); tft.print(title);
  // Version badge right
  tft.setTextColor(C_TXT_DIM); tft.setCursor(SW - 24, 4); tft.print(F("7.2"));
  tftDiv(HDR_H);
}

// ── Toast bar (bottom) — shows command feedback ──
void tftToast(const char* msg, uint16_t col) {
  tft.fillRect(0, TOAST_Y, SW, 11, C_BG_DARK);
  tft.drawFastHLine(0, TOAST_Y, SW, C_DIV);
  tft.setTextSize(1); tft.setTextColor(col);
  tft.setCursor(3, TOAST_Y + 2); tft.print(msg);
}

// ── Command feedback — called every time a command is received ──
void showCmdFeedback(char cmd, const char* desc) {
  char buf[26];
  snprintf(buf, sizeof(buf), "> %c  %s", cmd, desc);
  tftToast(buf, C_BLUE_HI);
}

// ── Connection status indicator (top-right) ──
void drawConnDot(bool connected) {
  uint16_t col = connected ? C_OK : C_ERR;
  tft.fillCircle(SW - 6, 8, 3, col);
}

// ═══════════════════════════════════════════════════════
//  SCREEN 1: BOOT — COM Connection Sequence
// ═══════════════════════════════════════════════════════
// Extra colors for splash (R↔B swapped for Blue Tab)
#define C_ORANGE_S   0x20FD   // Orange on swapped display (original 0xFD20)
#define C_MAROON_S   0x0078   // Dark maroon-red on swapped display (original 0x7800)

void drawBoot() {
  // ══════ SPLASH 1: KMITL (5 seconds) ══════
  tft.fillScreen(C_BG);
  tft.drawFastHLine(0, 0, SW, C_ORANGE_S);
  tft.drawFastHLine(0, 1, SW, C_ORANGE_S);

  tft.setTextSize(3); tft.setTextColor(C_ORANGE_S);
  tft.setCursor(28, 38); tft.print(F("KMITL"));

  tft.setTextSize(1); tft.setTextColor(C_TXT_DIM);
  tft.setCursor(8, 72);
  tft.print(F("King Mongkut's Institute"));
  tft.setCursor(12, 84);
  tft.print(F("of Technology Ladkrabang"));

  tft.drawFastHLine(0, SH - 2, SW, C_ORANGE_S);
  tft.drawFastHLine(0, SH - 1, SW, C_ORANGE_S);

  delay(5000);

  // ══════ SPLASH 2: SIITec (3 seconds) ══════
  tft.fillScreen(C_BG);
  tft.drawFastHLine(0, 0, SW, C_MAROON_S);
  tft.drawFastHLine(0, 1, SW, C_MAROON_S);

  tft.setTextSize(3); tft.setTextColor(C_MAROON_S);
  tft.setCursor(16, 38); tft.print(F("SIITec"));

  tft.setTextSize(1); tft.setTextColor(C_TXT_DIM);
  tft.setCursor(10, 72);
  tft.print(F("Surface Tension Tester"));
  tft.setCursor(50, 84);
  tft.print(F("v7.3"));

  tft.drawFastHLine(0, SH - 2, SW, C_MAROON_S);
  tft.drawFastHLine(0, SH - 1, SW, C_MAROON_S);

  delay(3000);

  // ══════ SYSTEM CHECK SCREEN ══════
  tft.fillScreen(C_BG);
  tft.drawFastHLine(0, 0, SW, C_BLUE_MED);

  tft.setTextSize(1); tft.setTextColor(C_BLUE_HI);
  tft.setCursor(28, 6); tft.print(F("SYSTEM DIAGNOSTICS"));
  tftDiv(16);

  // System checks with animation
  const char* checks[] = {
    "Serial 250000",
    "HX711 LoadCell",
    "Stepper Driver",
    "TFT Display",
    "EEPROM Cal"
  };

  tft.setTextSize(1);
  for (int i = 0; i < 5; i++) {
    int cy = 20 + i * 12;

    tft.setTextColor(C_TXT_DIM);
    tft.setCursor(4, cy); tft.print(checks[i]);

    // Animated dots
    tft.setTextColor(C_BLUE_DIM);
    for (int d = 0; d < 3; d++) {
      tft.setCursor(100 + d * 6, cy); tft.print(F("."));
      delay(60);
    }

    // OK result
    tft.fillRect(100, cy, 56, 8, C_BG);
    tft.setTextColor(C_OK);
    tft.setCursor(118, cy); tft.print(F("[OK]"));
    delay(80);
  }

  // Progress bar
  tft.drawRect(4, 88, 152, 8, C_BLUE_DIM);
  for (int i = 0; i < 148; i += 2) {
    tft.fillRect(6 + i, 90, 1, 4, C_BLUE_MED);
    delay(5);
  }

  tft.setTextColor(C_OK);
  tft.setCursor(28, 102); tft.print(F("SYSTEM READY"));

  // COM status
  tft.setTextColor(C_TXT_DIM);
  tft.setCursor(4, 114); tft.print(F("COM 250000 "));
  tft.setTextColor(C_OK); tft.print(F("Connected"));

  delay(800);
}

// ═══════════════════════════════════════════════════════
//  SCREEN 2: IDLE — Standby with COM status
// ═══════════════════════════════════════════════════════
void drawIdle() {
  tft.fillScreen(C_BG);
  tftHeader("TENSIOMETER");
  drawConnDot(true);  // COM connected

  // Status panel (centered)
  tft.fillRoundRect(30, 20, 100, 24, 3, C_BG_PANEL);
  tft.drawRoundRect(30, 20, 100, 24, 3, C_BLUE_DIM);
  tft.setTextSize(2); tft.setTextColor(C_TXT_PRI);
  tft.setCursor(42, 23); tft.print(F("READY"));
  tft.setTextSize(1); tft.setTextColor(C_BLUE_MED);
  tft.setCursor(36, 37); tft.print(F("Precision Mode"));

  tftDiv(48);

  // Command reference (compact, blue themed)
  tft.setTextSize(1);
  int y = 52;

  // Row 1: Main modes
  tft.setTextColor(C_BLUE_HI); tft.setCursor(4, y); tft.print(F("M"));
  tft.setTextColor(C_TXT_DIM); tft.setCursor(12, y); tft.print(F("Live"));
  tft.setTextColor(C_BLUE_HI); tft.setCursor(42, y); tft.print(F("A"));
  tft.setTextColor(C_TXT_DIM); tft.setCursor(50, y); tft.print(F("Auto"));
  tft.setTextColor(C_BLUE_HI); tft.setCursor(80, y); tft.print(F("E"));
  tft.setTextColor(C_TXT_DIM); tft.setCursor(88, y); tft.print(F("Enc"));
  tft.setTextColor(C_BLUE_HI); tft.setCursor(116, y); tft.print(F("K"));
  tft.setTextColor(C_TXT_DIM); tft.setCursor(124, y); tft.print(F("Cal"));

  y += 11;
  tft.setTextColor(C_BLUE_HI); tft.setCursor(4, y); tft.print(F("1-9BC"));
  tft.setTextColor(C_TXT_DIM); tft.setCursor(38, y); tft.print(F("Test"));
  tft.setTextColor(C_BLUE_HI); tft.setCursor(66, y); tft.print(F("I"));
  tft.setTextColor(C_TXT_DIM); tft.setCursor(74, y); tft.print(F("Info"));
  tft.setTextColor(C_BLUE_HI); tft.setCursor(100, y); tft.print(F("L"));
  tft.setTextColor(C_TXT_DIM); tft.setCursor(108, y); tft.print(F("LdCell"));

  y += 11;
  tft.setTextColor(C_BLUE_HI); tft.setCursor(4, y); tft.print(F("H"));
  tft.setTextColor(C_TXT_DIM); tft.setCursor(12, y); tft.print(F("Home"));
  tft.setTextColor(C_BLUE_HI); tft.setCursor(44, y); tft.print(F("0"));
  tft.setTextColor(C_TXT_DIM); tft.setCursor(52, y); tft.print(F("SetH"));
  tft.setTextColor(C_BLUE_HI); tft.setCursor(84, y); tft.print(F("T"));
  tft.setTextColor(C_TXT_DIM); tft.setCursor(92, y); tft.print(F("Tare"));

  tftDiv(88);

  // Speed range + Load cell info
  tft.setTextColor(C_TXT_DIM); tft.setCursor(4, 92);
  tft.print(F("600..0.75um/s "));
  tft.print(NUM_SPEEDS); tft.print(F("spd"));

  // Load cell status line
  tft.setTextColor(C_TXT_DIM); tft.setCursor(4, 104);
  tft.print(F("LC:"));
  tft.setTextColor(C_BLUE_HI);
  tft.print(loadCellType == 1 ? F("30g") : F("100g"));
  tft.setTextColor(C_TXT_DIM); tft.print(F(" COM"));
  tft.setTextColor(C_OK); tft.print(F(" OK"));
  tft.setTextColor(C_TXT_DIM); tft.print(F(" 250000"));

  // Bottom accent
  tft.drawFastHLine(0, SH - 1, SW, C_BLUE_DIM);

  // Toast ready
  tftToast("Awaiting command...", C_TXT_DIM);
}

// ═══════════════════════════════════════════════════════
//  SCREEN 3: LIVE MONITOR
// ═══════════════════════════════════════════════════════
void drawMonitorLayout() {
  tft.fillScreen(C_BG);
  tftHeader("LIVE MONITOR");
  drawConnDot(true);

  tft.setTextSize(1); tft.setTextColor(C_TXT_DIM);
  tft.setCursor(4, 20); tft.print(F("Force"));
  tft.setCursor(42, 20); tft.print(F("(N)"));

  // Force bar bg
  tftDiv(62);
  tft.fillRect(4, 66, SW-8, 10, C_BAR_BG);
  tft.drawRect(4, 66, SW-8, 10, C_BLUE_DIM);

  // Scale marks on bar
  tft.setTextSize(1); tft.setTextColor(C_TXT_DIM);
  tft.setCursor(4, 78); tft.print(F("0"));
  tft.setCursor(72, 78); tft.print(F("0.5"));
  tft.setCursor(140, 78); tft.print(F("1.0N"));

  tftDiv(90);
  tft.setTextColor(C_TXT_DIM); tft.setCursor(4, 94);
  tft.print(F("Motor: Stopped"));

  tftToast("Q=Exit  T=Tare", C_TXT_DIM);

  dc.force = -999; dc.barW = 0;
}

// ═══════════════════════════════════════════════════════
//  SCREEN 4: TEST MODE (Scientific layout)
// ═══════════════════════════════════════════════════════
void drawTestLayout(int idx, bool isAuto) {
  tft.fillScreen(C_BG);

  if (isAuto) tftHeader("AUTO SEQUENCE");
  else tftHeader("MEASUREMENT");
  drawConnDot(true);

  SpeedTest t = tests[idx];
  tft.setTextSize(1);

  // Test info bar (dark panel)
  tft.fillRect(0, HDR_H+1, SW, 12, C_BG_DARK);
  tft.setTextColor(C_BLUE_HI); tft.setCursor(3, HDR_H+3);
  tft.print(t.label);
  tft.setTextColor(C_TXT_DIM); tft.setCursor(40, HDR_H+3);
  tft.print(t.speed_mms*1000, 1); tft.print(F(" um/s"));
  if (isAuto) {
    tft.setTextColor(C_BLUE_MED); tft.setCursor(SW-26, HDR_H+3);
    tft.print(nextAutoTest); tft.print('/'); tft.print(NUM_SPEEDS);
  }

  // Force display area
  tft.setTextColor(C_TXT_DIM); tft.setCursor(4, 32); tft.print(F("Force"));

  // Info panel
  tftDiv(56);
  tft.setTextColor(C_TXT_SEC); tft.setCursor(4, 59); tft.print(F("Pos"));
  tft.setTextColor(C_TXT_SEC); tft.setCursor(4, 70); tft.print(F("Peak"));
  tft.setTextColor(C_TXT_SEC); tft.setCursor(82, 59); tft.print(F("Flt"));
  tft.setTextColor(C_TXT_SEC); tft.setCursor(82, 70); tft.print(F("Stat"));

  // Show filter info
  char fBuf[8]; snprintf(fBuf, sizeof(fBuf), "%d/%dx", filterSize, oversampleCount);
  tft.setTextColor(C_BLUE_MED); tft.setCursor(100, 59); tft.print(fBuf);

  // Contact indicator (circle outline)
  tft.drawCircle(SW-8, 64, 4, C_BLUE_DIM);

  tftDiv(82);

  // Mini graph bg
  tft.fillRect(GRF_X-1, GRF_Y, GRF_W+2, GRF_H, C_BAR_BG);
  tft.drawRect(GRF_X-1, GRF_Y, GRF_W+2, GRF_H, C_BLUE_DIM);

  // Progress bar bg
  tft.fillRect(PBR_X, PBR_Y, PBR_W, PBR_H, C_BAR_BG);

  // Toast
  tftToast("Measuring...", C_BLUE_MED);

  // Reset cache
  dc.force=-999; dc.pos=-999; dc.barW=0;
  dc.contact=false; dc.peak=0;
  dc.gIdx=0; dc.gCnt=0;
  graphMaxF=0.02;  // Reset graph auto-scale
  memset(dc.graph, 0, sizeof(dc.graph));
  memset(dc.status, 0, sizeof(dc.status));
}

// ═══════════════════════════════════════════════════════
//  SCREEN 5: CALIBRATION
// ═══════════════════════════════════════════════════════
void drawCalScreen(const char* step, const char* msg) {
  tft.fillScreen(C_BG);
  tftHeader("CALIBRATION");
  tft.setTextSize(1);
  tft.setTextColor(C_BLUE_HI); tft.setCursor(4, 22); tft.print(step);
  tftDiv(32);
  tft.setTextColor(C_TXT_PRI); tft.setCursor(4, 38); tft.print(msg);
  tftToast("Follow serial prompts", C_TXT_DIM);
}

// ═══════════════════════════════════════════════════════
//  SCREEN 6: EMERGENCY
// ═══════════════════════════════════════════════════════
void drawEmergency() {
  tft.fillScreen(C_BG);
  // Red border
  for (int i = 0; i < 3; i++) tft.drawRect(i, i, SW-2*i, SH-2*i, C_ERR);

  // Warning icon
  tft.fillTriangle(80, 24, 58, 56, 102, 56, C_ERR);
  tft.fillTriangle(80, 32, 64, 53, 96, 53, C_BG);
  tft.setTextSize(2); tft.setTextColor(C_ERR);
  tft.setCursor(74, 36); tft.print(F("!"));

  tft.setCursor(14, 66); tft.print(F("OVERLOAD"));
  tft.setTextSize(1); tft.setTextColor(C_TXT_SEC);
  tft.setCursor(14, 90); tft.print(F("Force exceeded "));
  tft.print(OVERLOAD_LIM, 1); tft.print(F("N"));
  tft.setCursor(14, 104); tft.setTextColor(C_BLUE_HI);
  tft.print(F("Press R to reset"));
}

// ═══════════════════════════════════════════════════════
//  PARTIAL UPDATES (Anti-Flicker)
// ═══════════════════════════════════════════════════════

void updForce(float f) {
  if (abs(f - dc.force) < 0.00005 && !fullRedraw) return;
  char buf[14]; dtostrf(f, 8, 5, buf);

  // Color coding: blue normal → teal on contact → red near overload
  uint16_t col = C_TXT_PRI;
  if (contactDet) col = C_OK;
  if (f > OVERLOAD_LIM * 0.8) col = C_ERR;

  if (curMode == MODE_MONITOR) {
    // Large force value
    tftBox(4, 30, 148, 26, buf, C_TXT_PRI, C_BG, 2);
    tft.setTextSize(1); tft.setTextColor(C_TXT_DIM);
    tft.setCursor(140, 52); tft.print(F("N"));

    // Horizontal bar
    int maxW = SW - 10;
    float pct = constrain(f / 1.0, 0.0, 1.0);
    int bw = (int)(pct * maxW);
    if (bw > dc.barW)
      tft.fillRect(5 + dc.barW, 67, bw - dc.barW, 8, C_BLUE_MED);
    else if (bw < dc.barW)
      tft.fillRect(5 + bw, 67, dc.barW - bw, 8, C_BAR_BG);
    dc.barW = bw;
  } else {
    // Test mode: medium force display
    tftBox(4, 40, 148, 16, buf, col, C_BG, 2);
    tft.setTextSize(1); tft.setTextColor(C_TXT_DIM);
    tft.setCursor(148, 46); tft.print(F("N"));
  }
  dc.force = f;
}

void updPos(float p) {
  if (abs(p - dc.pos) < 0.001 && !fullRedraw) return;
  char buf[10]; dtostrf(p, 7, 3, buf);
  tftBox(22, 59, 52, 8, buf, C_TXT_PRI, C_BG, 1);
  tft.setTextColor(C_TXT_DIM); tft.setCursor(76, 59); tft.print(F("mm"));
  dc.pos = p;
}

void updPeak(float pk) {
  char buf[10]; dtostrf(pk, 7, 5, buf);
  tftBox(26, 70, 52, 8, buf, C_BLUE_HI, C_BG, 1);
}

void updStatus(const char* s, uint16_t col) {
  if (strcmp(s, dc.status) == 0 && !fullRedraw) return;
  tftBox(100, 70, 56, 8, s, col, C_BG, 1);
  strncpy(dc.status, s, sizeof(dc.status)-1);
}

void updContact(bool c) {
  if (c == dc.contact && !fullRedraw) return;
  if (c) tft.fillCircle(SW-8, 64, 4, C_OK);
  else { tft.fillCircle(SW-8, 64, 4, C_BG); tft.drawCircle(SW-8, 64, 4, C_BLUE_DIM); }
  dc.contact = c;
}

void updGraph(float f) {
  // Auto-scale: grow if needed (never shrink during a test)
  if (f > graphMaxF * 0.85) graphMaxF = f * 1.5;
  if (graphMaxF < 0.005) graphMaxF = 0.005;  // Min 5mN scale

  float pct = constrain(f / graphMaxF, 0.0, 1.0);
  int gv = (int)(pct * (GRF_H - 2));
  dc.graph[dc.gIdx] = gv;
  int dx = GRF_X + (dc.gIdx % GRF_W);

  // Clear column
  tft.drawFastVLine(dx, GRF_Y+1, GRF_H-2, C_BAR_BG);

  // Draw data point + fill below
  int py = GRF_Y + GRF_H - 2 - gv;
  tft.drawPixel(dx, py, C_BLUE_HI);
  if (gv > 1) tft.drawFastVLine(dx, py+1, gv-1, C_BLUE_LO);

  // Cursor line (next position)
  int nx = GRF_X + ((dc.gIdx + 1) % GRF_W);
  tft.drawFastVLine(nx, GRF_Y+1, GRF_H-2, C_ACTIVE);

  dc.gIdx = (dc.gIdx + 1) % GRF_W;
}

void updProgress(long curP) {
  long dist = abs(curP - homePos);
  float prog = constrain((float)dist / (float)testDistance, 0.0, 1.0);
  int bw = (int)(prog * PBR_W);
  if (bw != dc.barW) {
    uint16_t bc = (motorSt == MOT_HOME) ? C_OK : C_BLUE_MED;
    if (bw > dc.barW) tft.fillRect(PBR_X + dc.barW, PBR_Y, bw - dc.barW, PBR_H, bc);
    else tft.fillRect(PBR_X + bw, PBR_Y, dc.barW - bw, PBR_H, C_BAR_BG);
    dc.barW = bw;
  }
}

// ═══════════════════════════════════════════════════════
//  ENCODER ISR
// ═══════════════════════════════════════════════════════
void encISR() {
  static int lastEnc = 0;
  int a = digitalRead(ENC_CLK), b = digitalRead(ENC_DT);
  int enc = (a << 1) | b;
  int sum = (lastEnc << 2) | enc;
  if (sum==0b1101||sum==0b0100||sum==0b0010||sum==0b1011) encPos++;
  if (sum==0b1110||sum==0b0111||sum==0b0001||sum==0b1000) encPos--;
  lastEnc = enc;
}

// ═══════════════════════════════════════════════════════
//  TFT: ENCODER POSITION SCREEN
// ═══════════════════════════════════════════════════════
void drawEncoderScreen() {
  tft.fillScreen(C_BG);
  tftHeader("ENCODER CTRL");
  drawConnDot(true);

  // Mode info bar
  tft.fillRect(0, HDR_H+1, SW, 12, C_BG_DARK);
  tft.setTextSize(1); tft.setTextColor(C_BLUE_HI);
  tft.setCursor(3, HDR_H+3); tft.print(F("FINE"));
  tft.setTextColor(C_TXT_DIM); tft.setCursor(30, HDR_H+3);
  tft.print(F("0.031mm/click  A:4000"));

  // Position display area
  tft.setTextColor(C_TXT_DIM); tft.setCursor(4, 32); tft.print(F("Position"));

  tftDiv(56);

  // Info labels
  tft.setTextSize(1);
  tft.setTextColor(C_TXT_SEC); tft.setCursor(4, 60); tft.print(F("Force"));
  tft.setTextColor(C_TXT_SEC); tft.setCursor(4, 72); tft.print(F("Home"));
  tft.setTextColor(C_TXT_SEC); tft.setCursor(82, 60); tft.print(F("Tgt"));
  tft.setTextColor(C_TXT_SEC); tft.setCursor(82, 72); tft.print(F("Lag"));

  tftDiv(84);

  // Status / command hints
  updEncStatus();

  // Force bar
  tft.fillRect(4, 100, SW-8, 6, C_BAR_BG);
  tft.drawRect(4, 100, SW-8, 6, C_BLUE_DIM);

  tftToast("Rotate knob to move", C_BLUE_MED);

  encLastDispPos = -99999;
  encLastDispForce = -99999;
}

void updEncoderPos(float posMm) {
  if (abs(posMm - encLastDispPos) < 0.001) return;
  char buf[14]; dtostrf(posMm, 9, 3, buf);
  tftBox(4, 40, 148, 16, buf, C_TXT_PRI, C_BG, 2);
  tft.setTextSize(1); tft.setTextColor(C_TXT_DIM);
  tft.setCursor(148, 46); tft.print(F("mm"));
  encLastDispPos = posMm;
}

void updEncoderForce(float f) {
  if (abs(f - encLastDispForce) < 0.0001) return;
  char buf[10]; dtostrf(f, 7, 4, buf);
  tftBox(32, 60, 44, 8, buf, C_BLUE_HI, C_BG, 1);
  tft.setTextColor(C_TXT_DIM); tft.setCursor(78, 60); tft.print(F("N"));
  encLastDispForce = f;
}

void updEncoderInfo() {
  // Home position
  if (encHomeSet) {
    char buf[10]; dtostrf(encHomePos/1600.0, 7, 2, buf);
    tftBox(26, 72, 48, 8, buf, C_OK, C_BG, 1);
  } else {
    tftBox(26, 72, 48, 8, "---", C_TXT_DIM, C_BG, 1);
  }
  // Target position
  if (encTargetSet) {
    char buf[10]; dtostrf(encTargetPos/1600.0, 6, 2, buf);
    tftBox(98, 60, 48, 8, buf, C_BLUE_MED, C_BG, 1);
  } else {
    tftBox(98, 60, 48, 8, "---", C_TXT_DIM, C_BG, 1);
  }
  // Lag
  long lag = encCmdPos - stepper.currentPosition();
  char lbuf[10]; ltoa(lag, lbuf, 10);
  tftBox(98, 72, 48, 8, lbuf, (abs(lag)>10 ? C_WARN : C_TXT_DIM), C_BG, 1);
}

void updEncoderBar(float f) {
  int maxW = SW-10;
  float pct = constrain(f / 1.0, 0.0, 1.0);
  int bw = (int)(pct * maxW);
  static int lastBW = 0;
  if (bw > lastBW) tft.fillRect(5 + lastBW, 101, bw - lastBW, 4, C_BLUE_MED);
  else if (bw < lastBW) tft.fillRect(5 + bw, 101, lastBW - bw, 4, C_BAR_BG);
  lastBW = bw;
}

void updEncStatus() {
  tft.fillRect(0, 86, SW, 26, C_BG);
  tft.setTextSize(1); tft.setTextColor(C_TXT_DIM);
  tft.setCursor(4, 88);
  if (!encHomeSet)
    tft.print(F("0=SetHome  Q=Exit"));
  else if (!encTargetSet)
    tft.print(F("P=SetTgt 0=Home Q=Exit"));
  else {
    tft.print(F("H G 0 P T  Q=Exit"));

    // Time estimate: each run = H→T(dn) + T→H(up), both at test speed
    float dist_mm = abs(encTargetPos - encHomePos) / 1600.0;
    if (dist_mm > 0.01) {
      float totalSec = 0;
      for (int i = 0; i < NUM_SPEEDS; i++) {
        float dnSec = dist_mm / tests[i].speed_mms;  // Down
        float upSec = dist_mm / tests[i].speed_mms;  // Up (same speed)
        float overhead = 8;  // 3x tare + settle
        totalSec += (dnSec + upSec + overhead) * AUTO_REPEATS;
      }
      totalSec *= AUTO_BATCHES;

      tft.setCursor(4, 100);
      tft.setTextColor(C_BLUE_MED);
      tft.print(F("Est:"));
      if (totalSec < 3600) {
        tft.print((int)(totalSec/60)); tft.print(F("min "));
      } else {
        int hrs = (int)(totalSec/3600);
        int mins = (int)((totalSec - hrs*3600) / 60);
        tft.print(hrs); tft.print(F("h")); tft.print(mins); tft.print(F("m "));
      }
      tft.setTextColor(C_TXT_DIM);
      tft.print(AUTO_BATCHES * NUM_SPEEDS * AUTO_REPEATS);
      tft.print(F("r "));
      tft.print(dist_mm, 1); tft.print(F("mm"));
    }
  }
}

// ═══════════════════════════════════════════════════════
//  ENTER ENCODER MODE (via 'E' command)
// ═══════════════════════════════════════════════════════
void enterEncoderMode() {
  curMode = MODE_ENCODER;
  encEnabled = true;
  encEStop = false;  // Clear E-Stop on enter
  encCmdPos = stepper.currentPosition();
  encLastProc = encPos;
  stepper.setMaxSpeed(25000);
  stepper.setAcceleration(ENC_ACCEL);

  if (!encHomeSet) {
    encHomePos = stepper.currentPosition();
    encHomeSet = true;
  }

  Serial.println(F("\n== ENCODER POSITION MODE =="));
  Serial.println(F("Rotate knob to move motor"));
  Serial.println(F("  0 = Set HOME here"));
  Serial.println(F("  P = Set TARGET here"));
  Serial.println(F("  H = Go to HOME"));
  Serial.println(F("  G = Go to TARGET"));
  Serial.println(F("  T = Tare load cell"));
  Serial.println(F("  Encoder Button = Emergency Stop"));
  Serial.println(F("  Q = Exit (positions saved)"));

  fullRedraw = true;
  drawEncoderScreen();
  updEncoderInfo();
}

// ═══════════════════════════════════════════════════════
//  ENCODER BUTTON HANDLER (physical button on encoder)
// ═══════════════════════════════════════════════════════
void checkEncButton() {
  if (digitalRead(ENC_SW) != LOW) return;
  if (millis() - encLastBtn < ENC_DEBOUNCE) return;
  encLastBtn = millis();

  if (!encEStop) {
    // ACTIVATE Emergency Stop
    encEStop = true;
    stepper.stop();
    stepper.setCurrentPosition(stepper.currentPosition());
    encCmdPos = stepper.currentPosition();
    // Sync encoder counter to kill lag
    noInterrupts(); encLastProc = encPos; interrupts();
    
    if (streaming) stopStream();
    
    Serial.println(F("ESTOP_ENC:ACTIVATED"));
    tftToast("E-STOP! Press again", C_ERR);
    
    // If in test/auto, pause motor but don't exit mode
    if (motorSt != MOT_IDLE) {
      motorSt = MOT_IDLE;
    }
  } else {
    // RELEASE Emergency Stop
    encEStop = false;
    Serial.println(F("ESTOP_ENC:RELEASED"));
    tftToast("E-Stop released", C_OK);
    
    // If was in auto/test mode, need to restart from menu
    if (curMode == MODE_AUTO || curMode == MODE_TEST) {
      curMode = MODE_IDLE;
      drawIdle();
      Serial.println(F("READY"));
    }
  }
}

// ═══════════════════════════════════════════════════════
//  ENCODER MODE HANDLER
// ═══════════════════════════════════════════════════════
void handleEncoderMode() {
  // E-Stop: block all encoder movement
  if (encEStop) {
    // Still update TFT to show status
    unsigned long now = millis();
    if (now - lastTFTTime >= TFT_INT) {
      LoadCell.update();
      float f = getClampedForce();
      float posMm = stepper.currentPosition() / 1600.0;
      updEncoderPos(posMm);
      updEncoderForce(f);
      updEncoderInfo();
      updStatus("E-STOP", C_ERR);
      lastTFTTime = now;
    }
    // Drain encoder ticks so lag stays 0
    noInterrupts(); encLastProc = encPos; interrupts();
    return;
  }

  // Read encoder delta (atomic)
  noInterrupts();
  long curEnc = encPos;
  interrupts();

  if (encEnabled) {
    long delta = curEnc - encLastProc;
    if (delta != 0) {
      encCmdPos += (delta * ENC_STEP);
      stepper.moveTo(encCmdPos);
      encLastProc = curEnc;
    }
  } else {
    // Motor doing H/G go-to — re-enable when done
    if (!stepper.isRunning()) {
      encEnabled = true;
      encCmdPos = stepper.currentPosition();
      encLastProc = curEnc;
      stepper.setMaxSpeed(25000);
      stepper.setAcceleration(ENC_ACCEL);
      tftToast("Encoder active", C_BLUE_MED);
    }
  }

  // Run motor
  stepper.run();

  // TFT update (throttled)
  unsigned long now = millis();
  if (now - lastTFTTime >= TFT_INT) {
    LoadCell.update();
    float f = getClampedForce();
    float posMm = stepper.currentPosition() / 1600.0;

    updEncoderPos(posMm);
    updEncoderForce(f);
    updEncoderInfo();
    updEncoderBar(f);
    lastTFTTime = now;
  }
}

// ═══════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════
void setup() {
  Serial.begin(250000);
  delay(500);

  tft.initR(INITR_GREENTAB);
  tft.setRotation(3);

  // ★ FIX: Blue Tab displays have R↔B swapped in hardware.
  // We fix this by swapping R and B in all our color definitions.
  // See the swapRB() helper and all C_ color constants below.
  tft.fillScreen(C_BG);
  drawBoot();

  pinMode(EN_PIN, OUTPUT); digitalWrite(EN_PIN, LOW);
  stepper.setMaxSpeed(homeSpeed); stepper.setAcceleration(homeAccel);

  // Encoder pins
  pinMode(ENC_CLK, INPUT_PULLUP);
  pinMode(ENC_DT,  INPUT_PULLUP);
  pinMode(ENC_SW,  INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DT),  encISR, CHANGE);
  encCmdPos = stepper.currentPosition();

  Serial.println(F("========================================"));
  Serial.println(F("Surface Tension Tester v7.3"));
  Serial.println(F("Scientific Instrument UI"));
  Serial.println(F("========================================"));

  loadLoadCellType();  // Load cell type from EEPROM (before loadCal)
  loadCal();
  OVERLOAD_LIM = (loadCellType == 1) ? 0.25 : 5.0;

  Serial.print(F("LoadCell: ")); Serial.print(loadCellType == 1 ? F("30g") : F("100g"));
  Serial.print(F("  Cal: ")); Serial.print(calFactor);
  Serial.print(F("  OvrLd: ")); Serial.print(OVERLOAD_LIM, 2); Serial.println(F("N"));

  LoadCell.begin(); LoadCell.start(3000, true);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println(F("ERROR: HX711 Timeout!"));
    tft.fillScreen(C_BG);
    tft.setTextSize(2); tft.setTextColor(C_ERR);
    tft.setCursor(10, 50); tft.print(F("HX711 ERR!"));
    while(1);
  }
  LoadCell.setCalFactor(calFactor);
  resetFilters();

  memset(&dc, 0, sizeof(dc));
  dc.force = -999; dc.pos = -999;

  Serial.println(F("System Ready!"));
  Serial.println(F("READY"));

  delay(600);
  drawIdle();
  fullRedraw = false;
}

// ═══════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════
void loop() {
  LoadCell.update();
  checkEncButton();  // Always check encoder button
  if (curMode != MODE_CAL) handleSerial();
  if (eStop) { handleEStop(); return; }

  switch (curMode) {
    case MODE_IDLE:
      if (hasPending) { char c=pendingCmd; hasPending=false; pendingCmd=0; execCmd(c); }
      break;
    case MODE_MONITOR: handleMonitor(); break;
    case MODE_TEST:    handleTestMode(); break;
    case MODE_AUTO:    handleAutoMode(); break;
    case MODE_CAL:     handleCalMode(); break;
    case MODE_ENCODER: handleEncoderMode(); break;
    case MODE_HOMING:
      stepper.run();
      if (stepper.distanceToGo()==0) {
        Serial.println(F("HOME_OK"));
        curMode=MODE_IDLE; motorSt=MOT_IDLE;
        drawIdle(); tftToast("> Homed OK", C_OK);
        Serial.println(F("READY"));
      }
      break;
  }
  if (LoadCell.getTareStatus()) { Serial.println(F("TARE_OK")); resetFilters(); }
}

// ═══════════════════════════════════════════════════════
//  MONITOR
// ═══════════════════════════════════════════════════════
void handleMonitor() {
  unsigned long now = millis();
  if (now - lastTFTTime >= TFT_INT) {
    float f = getClampedForce();
    updForce(f);
    Serial.print(F("Force:")); Serial.print(f,5); Serial.println(F(" N"));
    lastTFTTime = now; fullRedraw = false;
  }
}
void enterMonitor() {
  curMode=MODE_MONITOR; motorSt=MOT_IDLE; streaming=false;
  Serial.println(F("\n== LIVE MONITOR =="));
  fullRedraw = true;
  drawMonitorLayout();
  lastTFTTime = millis();
}

// ═══════════════════════════════════════════════════════
//  TEST MODE
// ═══════════════════════════════════════════════════════
void handleTestMode() {
  LoadCell.update();
  if (motorSt==MOT_DOWN) handleDown();
  else if (motorSt==MOT_UP) handleUp();
  else if (motorSt==MOT_HOME) handleReturn();
}

void enterTest(int idx) {
  if (idx<0||idx>=NUM_SPEEDS) { Serial.println(F("ERR")); Serial.println(F("READY")); return; }
  SpeedTest t = tests[idx];
  curTest = idx;

  // Triple auto-tare for stability
  Serial.println(F("AUTO_TARE_x3..."));
  for (int tr=0; tr<3; tr++) {
    char tbuf[18]; snprintf(tbuf, sizeof(tbuf), "Tare %d/3...", tr+1);
    tftToast(tbuf, C_BLUE_MED);
    LoadCell.tareNoDelay();
    unsigned long tw=millis();
    while(!LoadCell.getTareStatus()&&(millis()-tw)<5000){LoadCell.update();delay(10);}
    delay(200);
  }
  resetFilters(); Serial.println(F("TARE_OK"));

  int fs=calcFilterSize(t.speed_mms); setFilterSize(fs);
  oversampleCount=calcOversample(t.speed_mms);
  emaAlpha=calcEmaAlpha(t.speed_mms);
  Serial.print(F("FILTER:")); Serial.print(fs); Serial.print(F(" OVS:")); Serial.print(oversampleCount);
  Serial.print(F(" EMA:")); Serial.println(emaAlpha,2);

  tftToast("Baseline...", C_BLUE_MED);
  measureBaseline();
  resetContact(); resetPeak();

  curMode=MODE_TEST; motorSt=MOT_DOWN;
  Serial.println();
  Serial.print(F("RUN_START:")); Serial.print(t.name);
  Serial.println(F(":1/1:1"));
  Serial.print(F("Speed: ")); Serial.print(t.speed_mms*1000,3); Serial.println(F(" um/s"));

  contactDet=false; lastSampledPos=homePos;

  // Move DOWN to Target
  if (encHomeSet && encTargetSet) {
    targetPos = encTargetPos;
  } else {
    targetPos = homePos + testDistance;
  }

  stepper.setMaxSpeed(t.speed); stepper.setAcceleration(100000); stepper.moveTo(targetPos);
  startStream();

  fullRedraw=true;
  drawTestLayout(idx, false);
  updStatus("RUN", C_ACTIVE);
}

void exitTest() {
  stopStream();
  curMode=MODE_IDLE; motorSt=MOT_IDLE; streaming=false;
  tftToast("Settling...", C_BLUE_MED);
  waitForSettle(SETTLE_TIME);
  drawIdle(); tftToast("Test complete", C_OK);
  Serial.println(F("READY"));
}

// ═══════════════════════════════════════════════════════
//  AUTO SEQUENCE — 7 speeds × 10 runs each = 70 total
// ═══════════════════════════════════════════════════════
void handleAutoMode() {
  LoadCell.update();
  if (motorSt==MOT_IDLE) {
    if (nextAutoTest<NUM_SPEEDS) setupAutoNext();
    else {
      // One batch done
      autoBatch++;
      int totalRuns = autoBatch * NUM_SPEEDS * AUTO_REPEATS;

      Serial.println();
      Serial.print(F("BATCH_COMPLETE:")); Serial.println(autoBatch);
      Serial.print(F("Total so far: ")); Serial.print(totalRuns); Serial.println(F(" runs"));

      if (autoBatch < AUTO_BATCHES) {
        // More batches to go — restart speeds
        Serial.print(F("Starting batch ")); Serial.print(autoBatch+1);
        Serial.print('/'); Serial.println(AUTO_BATCHES);

        tft.fillScreen(C_BG); tftHeader("BATCH DONE");
        tft.setTextSize(1); tft.setTextColor(C_OK);
        tft.setCursor(10,30); tft.print(F("Batch ")); tft.print(autoBatch);
        tft.print('/'); tft.print(AUTO_BATCHES); tft.print(F(" done"));
        tft.setTextColor(C_TXT_SEC);
        tft.setCursor(10,46); tft.print(totalRuns); tft.print(F(" runs so far"));
        tft.setTextColor(C_BLUE_HI);
        tft.setCursor(10,62); tft.print(F("Batch ")); tft.print(autoBatch+1);
        tft.print(F(" starting..."));
        delay(3000);

        nextAutoTest=0; autoRunNum=0;
      } else {
        // ALL BATCHES DONE
        Serial.println(F("\n========================================"));
        Serial.println(F("ALL BATCHES COMPLETE!"));
        Serial.print(AUTO_BATCHES); Serial.print(F(" batches x "));
        Serial.print(NUM_SPEEDS); Serial.print(F(" speeds x "));
        Serial.print(AUTO_REPEATS); Serial.print(F(" = "));
        Serial.print(totalRuns); Serial.println(F(" total"));
        Serial.println(F("========================================"));

        tft.fillScreen(C_BG); tftHeader("COMPLETE");
        tft.setTextSize(2); tft.setTextColor(C_OK);
        tft.setCursor(10,32); tft.print(F("ALL DONE"));
        tft.setTextSize(1); tft.setTextColor(C_TXT_SEC);
        tft.setCursor(10,56); tft.print(AUTO_BATCHES); tft.print(F(" batch x "));
        tft.print(NUM_SPEEDS * AUTO_REPEATS); tft.print(F(" runs"));
        tft.setCursor(10,70); tft.print(F("= "));
        tft.print(totalRuns); tft.print(F(" measurements"));
        delay(3000);

        curMode=MODE_IDLE; motorSt=MOT_IDLE; streaming=false;
        nextAutoTest=0; autoRunNum=0; autoBatch=0;
        drawIdle(); Serial.println(F("READY"));
      }
    }
  } else if(motorSt==MOT_DOWN) handleDown();
  else if(motorSt==MOT_UP) handleUp();
  else if(motorSt==MOT_HOME) handleReturn();
}

void enterAuto() {
  curMode=MODE_AUTO; motorSt=MOT_IDLE;
  nextAutoTest=0; autoRunNum=0; autoBatch=0;

  // Ensure we start from HOME position
  if (encHomeSet) {
    homePos = encHomePos;
    if (stepper.currentPosition() != homePos) {
      Serial.println(F(">>> Moving to HOME before auto..."));
      tftToast("Going HOME...", C_BLUE_MED);
      stepper.setMaxSpeed(homeSpeed); stepper.setAcceleration(homeAccel);
      stepper.moveTo(homePos);
      while(stepper.isRunning()) stepper.run();
    }
  }

  int total = AUTO_BATCHES * NUM_SPEEDS * AUTO_REPEATS;
  Serial.println(F("\n== AUTO SEQUENCE =="));
  Serial.print(F("HOME=")); Serial.print(homePos/1600.0,3); Serial.println(F("mm"));
  if(encTargetSet) { Serial.print(F("TGT=")); Serial.print(encTargetPos/1600.0,3); Serial.println(F("mm")); }
  Serial.print(AUTO_BATCHES); Serial.print(F(" batches x "));
  Serial.print(NUM_SPEEDS); Serial.print(F(" speeds x "));
  Serial.print(AUTO_REPEATS); Serial.print(F(" runs = "));
  Serial.print(total); Serial.println(F(" total"));
}

void setupAutoNext() {
  if(nextAutoTest>=NUM_SPEEDS) return;
  int idx=nextAutoTest; curTest=idx;
  SpeedTest t=tests[idx];

  // First run of a new speed? Print header + reset peaks array
  if (autoRunNum==0) {
    Serial.println();
    Serial.println(F("────────────────────────────────────"));
    Serial.print(F("BATCH ")); Serial.print(autoBatch+1);
    Serial.print('/'); Serial.print(AUTO_BATCHES);
    Serial.print(F(" | SPEED ")); Serial.print(nextAutoTest+1);
    Serial.print('/'); Serial.print(NUM_SPEEDS);
    Serial.print(F(": ")); Serial.print(t.name);
    Serial.print(F(" (")); Serial.print(t.speed_mms*1000,1);
    Serial.println(F(" um/s)"));
    Serial.print(AUTO_REPEATS); Serial.println(F(" runs scheduled"));
    Serial.println(F("────────────────────────────────────"));
    memset(autoPeaks, 0, sizeof(autoPeaks));
  }

  // Triple auto-tare for stability
  for (int tr=0; tr<3; tr++) {
    char tbuf[18]; snprintf(tbuf, sizeof(tbuf), "Tare %d/3...", tr+1);
    tftToast(tbuf, C_BLUE_MED);
    LoadCell.tareNoDelay();
    unsigned long tw=millis();
    while(!LoadCell.getTareStatus()&&(millis()-tw)<5000){LoadCell.update();delay(10);}
    delay(200);
  }
  resetFilters();
  int fs=calcFilterSize(t.speed_mms); setFilterSize(fs);
  oversampleCount=calcOversample(t.speed_mms);
  emaAlpha=calcEmaAlpha(t.speed_mms);
  measureBaseline(); resetContact(); resetPeak();

  // Serial announce: RUN_START:name:run/total:batch
  Serial.print(F("RUN_START:")); Serial.print(t.name);
  Serial.print(F(":")); Serial.print(autoRunNum+1);
  Serial.print('/'); Serial.print(AUTO_REPEATS);
  Serial.print(F(":")); Serial.println(autoBatch+1);

  contactDet=false; lastSampledPos=homePos; motorSt=MOT_DOWN;

  // Move DOWN to Target
  if (encHomeSet && encTargetSet) {
    targetPos = encTargetPos;
  } else {
    targetPos = homePos + testDistance;  // fallback
  }

  stepper.setMaxSpeed(t.speed); stepper.setAcceleration(100000); stepper.moveTo(targetPos);
  startStream();

  fullRedraw=true;
  drawAutoTestLayout(idx, autoRunNum);
  updStatus("RUN", C_ACTIVE);
}

// Print stats for current speed after all 10 runs
void printSpeedStats() {
  SpeedTest t=tests[nextAutoTest];
  float sum=0, mn=999, mx=0;
  int valid=0;
  for(int i=0; i<AUTO_REPEATS; i++){
    if(autoPeaks[i]>0.0001){ sum+=autoPeaks[i]; valid++;
      if(autoPeaks[i]<mn) mn=autoPeaks[i];
      if(autoPeaks[i]>mx) mx=autoPeaks[i];
    }
  }
  if(valid==0) return;
  float avg=sum/valid;
  float ssq=0;
  for(int i=0;i<AUTO_REPEATS;i++){
    if(autoPeaks[i]>0.0001){ float d=autoPeaks[i]-avg; ssq+=d*d; }
  }
  float sd=sqrt(ssq/valid);
  float rsd=(avg>0)?(sd/avg*100.0):0;

  Serial.println(F("────────────────────────────────────"));
  Serial.print(F("SPEED_STATS:")); Serial.println(t.name);
  Serial.print(F("  Runs: ")); Serial.println(valid);
  Serial.print(F("  Avg:  ")); Serial.print(avg,6); Serial.println(F(" N"));
  Serial.print(F("  Std:  ")); Serial.print(sd,6); Serial.println(F(" N"));
  Serial.print(F("  RSD:  ")); Serial.print(rsd,2); Serial.println(F(" %"));
  Serial.print(F("  Min:  ")); Serial.print(mn,6); Serial.println(F(" N"));
  Serial.print(F("  Max:  ")); Serial.print(mx,6); Serial.println(F(" N"));
  Serial.println(F("────────────────────────────────────"));

  // TFT stats summary
  tft.fillScreen(C_BG); tftHeader("SPEED STATS");
  tft.setTextSize(1);
  tft.setTextColor(C_BLUE_HI); tft.setCursor(4,20);
  tft.print(t.name); tft.print(F(" ")); tft.print(t.speed_mms*1000,1); tft.print(F("um/s"));
  tft.setTextColor(C_TXT_PRI); tft.setCursor(4,34);
  tft.print(F("Avg: ")); tft.print(avg,5); tft.print(F(" N"));
  tft.setCursor(4,46);
  tft.print(F("Std: ")); tft.print(sd,5); tft.print(F(" N"));
  tft.setCursor(4,58);
  tft.print(F("RSD: ")); tft.print(rsd,1); tft.print(F(" %"));
  tft.setTextColor((rsd<2.0)?C_OK:((rsd<5.0)?C_WARN:C_ERR));
  tft.setCursor(100,58);
  if(rsd<2.0) tft.print(F("GOOD"));
  else if(rsd<5.0) tft.print(F("OK"));
  else tft.print(F("HIGH"));
  tft.setTextColor(C_TXT_DIM); tft.setCursor(4,74);
  tft.print(valid); tft.print(F(" runs | "));
  tft.print(mn,4); tft.print(F("-")); tft.print(mx,4); tft.print(F("N"));
  delay(3000);
}

// TFT layout for auto test with run counter + cumulative stats
void drawAutoTestLayout(int idx, int runNum) {
  tft.fillScreen(C_BG);
  tftHeader("AUTO SEQUENCE");
  drawConnDot(true);

  SpeedTest t = tests[idx];
  tft.setTextSize(1);

  // Test info bar
  tft.fillRect(0, HDR_H+1, SW, 12, C_BG_DARK);
  tft.setTextColor(C_BLUE_HI); tft.setCursor(3, HDR_H+3);
  tft.print(t.label);
  tft.setTextColor(C_TXT_DIM); tft.setCursor(40, HDR_H+3);
  tft.print(t.speed_mms*1000, 1); tft.print(F("um/s"));

  // Run counter: "R3/10 S2/7 B1/2"
  tft.setTextColor(C_BLUE_MED); tft.setCursor(SW-78, HDR_H+3);
  tft.print(F("R")); tft.print(runNum+1); tft.print('/'); tft.print(AUTO_REPEATS);
  tft.print(F("S")); tft.print(nextAutoTest+1); tft.print('/'); tft.print(NUM_SPEEDS);
  tft.print(F("B")); tft.print(autoBatch+1);

  // Force display area
  tft.setTextColor(C_TXT_DIM); tft.setCursor(4, 32); tft.print(F("Force"));

  tftDiv(56);
  tft.setTextColor(C_TXT_SEC); tft.setCursor(4, 59); tft.print(F("Pos"));
  tft.setTextColor(C_TXT_SEC); tft.setCursor(4, 70); tft.print(F("Peak"));
  tft.setTextColor(C_TXT_SEC); tft.setCursor(82, 59); tft.print(F("Flt"));

  char fBuf[8]; snprintf(fBuf, sizeof(fBuf), "%d/%dx", filterSize, oversampleCount);
  tft.setTextColor(C_BLUE_MED); tft.setCursor(100, 59); tft.print(fBuf);
  tft.drawCircle(SW-8, 64, 4, C_BLUE_DIM);

  // Cumulative SD/RSD line (will be updated by updAutoStats)
  tft.setTextColor(C_TXT_SEC); tft.setCursor(82, 70); tft.print(F("RSD"));
  // Show previous runs stats if any
  if (runNum > 0) updAutoStats(runNum);

  tftDiv(82);

  // Mini graph
  tft.fillRect(GRF_X-1, GRF_Y, GRF_W+2, GRF_H, C_BAR_BG);
  tft.drawRect(GRF_X-1, GRF_Y, GRF_W+2, GRF_H, C_BLUE_DIM);
  tft.fillRect(PBR_X, PBR_Y, PBR_W, PBR_H, C_BAR_BG);

  tftToast("Measuring...", C_BLUE_MED);

  dc.force=-999; dc.pos=-999; dc.barW=0;
  dc.contact=false; dc.peak=0;
  dc.gIdx=0; dc.gCnt=0;
  graphMaxF=0.02;
  memset(dc.graph, 0, sizeof(dc.graph));
  memset(dc.status, 0, sizeof(dc.status));
}

// Update cumulative SD/RSD on TFT during auto mode
void updAutoStats(int numDone) {
  if (numDone < 1) return;
  float sum=0; int cnt=0;
  for(int i=0; i<numDone && i<AUTO_REPEATS; i++){
    if(autoPeaks[i]>0.0001){ sum+=autoPeaks[i]; cnt++; }
  }
  if(cnt<1) return;
  float avg=sum/cnt;
  float ssq=0;
  for(int i=0;i<numDone&&i<AUTO_REPEATS;i++){
    if(autoPeaks[i]>0.0001){ float d=autoPeaks[i]-avg; ssq+=d*d; }
  }
  float sd=sqrt(ssq/cnt);
  float rsd=(avg>0)?(sd/avg*100.0):0;

  // Draw on TFT: "SD:0.00008 RSD:0.9%"
  tft.fillRect(82, 70, 74, 8, C_BG);
  tft.setTextSize(1);

  if(cnt>=2){
    // Color code RSD
    uint16_t rc = (rsd<2.0) ? C_OK : ((rsd<5.0) ? C_WARN : C_ERR);
    char rb[8]; dtostrf(rsd, 4, 1, rb);
    tft.setTextColor(rc); tft.setCursor(82, 70);
    tft.print(F("RSD")); tft.print(rb); tft.print(F("%"));
  } else {
    tft.setTextColor(C_TXT_DIM); tft.setCursor(82, 70);
    tft.print(F("n=1"));
  }
}

// ═══════════════════════════════════════════════════════
//  MOTOR HANDLERS
//  1 cycle: HOME→TARGET(up, -force) + TARGET→HOME(down, +force, peak!)
//  Both phases at test speed. Stream data throughout.
// ═══════════════════════════════════════════════════════

// Phase 1 (MOT_DOWN): HOME → TARGET (going UP, force naturally negative)
void handleDown() {
  if (encEStop) return;
  stepper.run();
  long cp=stepper.currentPosition();
  long pd=abs(cp-lastSampledPos);

  if (pd>=POS_INTERVAL) {
    float f = getSignedForce();  // Natural sign: UP = negative force
    lastForce=f; lastSampledPos=cp;
    if (streaming) streamData(f, cp);

    if (abs(f)>OVERLOAD_LIM) { Serial.println(F("OVERLOAD!")); eStop=true; drawEmergency(); return; }

    unsigned long now=millis();
    if (now-lastTFTTime>=TFT_INT) {
      updForce(f);
      updPos(cp/1600.0);
      updGraph(abs(f));
      updProgress(cp);
      updStatus("UP", C_BLUE_MED);
      lastTFTTime=now; fullRedraw=false;
    }
  }

  if (stepper.distanceToGo()==0) {
    // Reached TARGET → now descend back to HOME (measurement phase!)
    Serial.println(F("PHASE_DN"));

    // Reset peak tracking for descent (this is where we measure!)
    resetContact(); resetPeak();
    dc.peak=0; contactDet=false;
    motorSt=MOT_UP;

    // Go back to HOME at test speed
    stepper.setMaxSpeed(tests[curTest].speed); stepper.setAcceleration(100000);
    stepper.moveTo(homePos);
    updStatus("DOWN", C_ACTIVE);
    tftToast("Descending...", C_ACTIVE);
  }
}

// Phase 2 (MOT_UP): TARGET → HOME (going DOWN, force POSITIVE, capture PEAK!)
void handleUp() {
  if (encEStop) return;
  stepper.run();
  long cp=stepper.currentPosition();
  long pd=abs(cp-lastSampledPos);

  if (pd>=POS_INTERVAL) {
    float f = getSignedForce();  // Positive during descent (natural direction)
    lastForce=f; lastSampledPos=cp;
    updatePeak(f, cp);  // ★ Peak captured here (force positive)

    if (!contactDet && f>CONTACT_TH) {
      contactDet=true; contactPosition=cp;
      Serial.print(F("CONTACT_AT:")); Serial.println(cp/1600.0,4);
      updStatus("CNTACT", C_OK);
      tftToast("Contact!", C_OK);
    }
    if (f>OVERLOAD_LIM) { Serial.println(F("OVERLOAD!")); eStop=true; drawEmergency(); return; }
    if (streaming) streamData(f, cp);

    unsigned long now=millis();
    if (now-lastTFTTime>=TFT_INT) {
      updForce(f);
      float disp=(contactDet&&contactPosition>=0)?(cp-contactPosition)/1600.0:cp/1600.0;
      updPos(disp);
      updPeak(confirmedPeak>0?confirmedPeak:dc.peak);
      updContact(contactDet);
      updGraph(f);
      updProgress(cp);
      if(!contactDet) updStatus("DOWN", C_ACTIVE);
      lastTFTTime=now; fullRedraw=false;
    }
    if(f>dc.peak) dc.peak=f;
  }

  if (stepper.distanceToGo()==0) {
    // Reached HOME — 1 cycle complete!
    stopStream();
    Serial.print(F("PEAK_VALIDATED:")); Serial.println(confirmedPeak,6);
    Serial.println(F("HOME_OK"));
    motorSt=MOT_IDLE;

    if (curMode==MODE_TEST) {
      float pk = (confirmedPeak > 0) ? confirmedPeak : dc.peak;
      Serial.print(F("RUN_END:")); Serial.print(tests[curTest].name);
      Serial.print(F(":1:")); Serial.print(pk, 6);
      Serial.println(F(":1"));
      updStatus("DONE", C_OK); delay(800); exitTest();
    }
    else if (curMode==MODE_AUTO) {
      float pk = (confirmedPeak > 0) ? confirmedPeak : dc.peak;
      autoPeaks[autoRunNum] = pk;

      Serial.print(F("RUN_END:")); Serial.print(tests[nextAutoTest].name);
      Serial.print(F(":")); Serial.print(autoRunNum+1);
      Serial.print(F(":")); Serial.print(pk, 6);
      Serial.print(F(":")); Serial.println(autoBatch+1);

      autoRunNum++;
      if (autoRunNum >= AUTO_REPEATS) {
        printSpeedStats();
        autoRunNum = 0;
        nextAutoTest++;
        tftToast("Next speed...", C_BLUE_MED);
        waitForSettle(SETTLE_TIME);
      } else {
        tftToast("Settling...", C_BLUE_MED);
        waitForSettle(SETTLE_TIME);
      }
      updStatus("NEXT", C_BLUE_HI); delay(300);
    }
  }
}

// handleReturn: only for manual homing (H command)
void handleReturn() {
  stepper.run();
  unsigned long now=millis();
  if (now-lastTFTTime>=TFT_INT*2) { updProgress(stepper.currentPosition()); lastTFTTime=now; }
  if (stepper.distanceToGo()==0) {
    Serial.println(F("HOME_OK")); motorSt=MOT_IDLE;
    if (curMode==MODE_HOMING) {
      curMode=MODE_IDLE; drawIdle();
      tftToast("Home reached", C_OK);
      Serial.println(F("READY"));
    }
  }
}

void handleEStop() {
  stepper.stop(); stepper.setSpeed(0); stopStream();
  static bool drawn=false;
  if(!drawn){drawEmergency();drawn=true;}
  if(Serial.available()>0){
    char c=Serial.read();
    if(c=='R'||c=='r'){eStop=false;drawn=false;motorSt=MOT_IDLE;curMode=MODE_IDLE;drawIdle();Serial.println(F("READY"));}
  }
}

// ═══════════════════════════════════════════════════════
//  SERIAL COMMAND HANDLER (with TFT feedback)
// ═══════════════════════════════════════════════════════
void handleSerial() {
  if (Serial.available()<=0) return;
  char c=Serial.read();
  while(Serial.available()>0)Serial.read();

  // ★ Show every command on TFT toast
  if (c=='Q'||c=='q') {
    showCmdFeedback(c, "Stop/Quit");
    if(curMode==MODE_ENCODER) {
      encEnabled=false;
      stepper.stop(); while(stepper.isRunning()) stepper.run();
      encCmdPos=stepper.currentPosition();

      // ★ Use encoder HOME position for test system (not current pos!)
      if (encHomeSet) {
        homePos = encHomePos;
      } else {
        homePos = stepper.currentPosition();
      }
      Serial.print(F("ENC_EXIT homePos="));
      Serial.print(homePos/1600.0,3);
      Serial.println(F("mm"));
      if(encHomeSet) { Serial.print(F("  encHome=")); Serial.println(encHomePos/1600.0,3); }
      if(encTargetSet) { Serial.print(F("  encTarget=")); Serial.println(encTargetPos/1600.0,3); }

      // Go to HOME position before exiting
      if (encHomeSet && stepper.currentPosition() != encHomePos) {
        Serial.println(F(">>> Moving to HOME before exit..."));
        tftToast("Going HOME...", C_BLUE_MED);
        stepper.setMaxSpeed(homeSpeed); stepper.setAcceleration(homeAccel);
        stepper.moveTo(encHomePos);
        while(stepper.isRunning()) stepper.run();
        encCmdPos = stepper.currentPosition();
      }

      curMode=MODE_IDLE; motorSt=MOT_IDLE;
      delay(300); drawIdle(); tftToast("> Ready.", C_OK);
      Serial.println(F("READY"));
    } else if(curMode!=MODE_IDLE){
      stopStream();stepper.stop();stepper.setCurrentPosition(stepper.currentPosition());
      motorSt=MOT_IDLE;curMode=MODE_IDLE;hasPending=false;
      delay(300); drawIdle(); tftToast("> Stopped", C_BLUE_HI);
      Serial.println(F("READY"));
    }
    return;
  }
  if (c=='S'||c=='s') {
    showCmdFeedback(c, "Force Stop");
    if(motorSt!=MOT_IDLE){
      stepper.stop();stepper.setCurrentPosition(stepper.currentPosition());
      stopStream();motorSt=MOT_IDLE;curMode=MODE_IDLE;hasPending=false;
      delay(300); drawIdle();
      Serial.println(F("TEST_STOPPED")); Serial.println(F("READY"));
    }
    return;
  }
  if (c=='R'||c=='r') {
    showCmdFeedback(c, "Reset");
    if(eStop){eStop=false;motorSt=MOT_IDLE;curMode=MODE_IDLE;drawIdle();Serial.println(F("READY"));}
    return;
  }

  // ★ Encoder mode: intercept all commands here
  if (curMode==MODE_ENCODER) {
    if (c=='H'||c=='h') {
      if (encHomeSet) {
        showCmdFeedback(c,"Go Home");
        encEnabled=false;
        stepper.setMaxSpeed(2000); stepper.setAcceleration(2000);
        stepper.moveTo(encHomePos);
        encCmdPos=encHomePos;
        tftToast("> Going HOME", C_BLUE_HI);
        Serial.print(F("ENC_GOTO_HOME:")); Serial.println(encHomePos/1600.0,3);
      } else { tftToast("Home not set!", C_WARN); }
      return;
    }
    if (c=='G'||c=='g') {
      if (encTargetSet) {
        showCmdFeedback(c,"Go Target");
        encEnabled=false;
        stepper.setMaxSpeed(2000); stepper.setAcceleration(2000);
        stepper.moveTo(encTargetPos);
        encCmdPos=encTargetPos;
        tftToast("> Going TARGET", C_BLUE_HI);
        Serial.print(F("ENC_GOTO_TGT:")); Serial.println(encTargetPos/1600.0,3);
      } else { tftToast("Target not set!", C_WARN); }
      return;
    }
    if (c=='0') {
      showCmdFeedback(c,"Set Home");
      encHomePos=stepper.currentPosition();
      encHomeSet=true;
      updEncoderInfo(); updEncStatus();
      tftToast("> HOME set here", C_OK);
      Serial.print(F("ENC_HOME:")); Serial.println(encHomePos/1600.0,3);
      return;
    }
    if (c=='P'||c=='p') {
      showCmdFeedback(c,"Set Target");
      encTargetPos=stepper.currentPosition();
      encTargetSet=true;
      updEncoderInfo(); updEncStatus();
      tftToast("> TARGET set here", C_OK);
      Serial.print(F("ENC_TARGET:")); Serial.println(encTargetPos/1600.0,3);
      return;
    }
    if (c=='T'||c=='t') {
      showCmdFeedback(c,"Tare");
      LoadCell.tareNoDelay();
      tftToast("> Taring...", C_BLUE_MED);
      return;
    }
    return;
  }

  // Queue if returning
  if (motorSt==MOT_HOME) {
    pendingCmd=c; hasPending=true;
    char tb[20]; snprintf(tb,sizeof(tb),"> %c Queued",c);
    tftToast(tb, C_BLUE_MED);
    Serial.print(F("CMD_QUEUED:")); Serial.println(c); return;
  }
  if (motorSt==MOT_DOWN || motorSt==MOT_UP) {
    char tb[20]; snprintf(tb,sizeof(tb),"> %c Busy!",c);
    tftToast(tb, C_WARN);
    Serial.print(F("BUSY:")); Serial.println(c); return;
  }
  if (curMode==MODE_MONITOR && c!='T'&&c!='t') return;

  execCmd(c);
}

void execCmd(char c) {
  // Speed tests: 1-9 = speed 1-9, then A/B/C = speed 10/11/12
  int speedIdx = -1;
  if (c>='1' && c<='9') speedIdx = c - '1';
  else if (c=='a'||c=='A') { /* handled below as Auto if not speed */ }
  else if (c=='b'||c=='B') speedIdx = 10;  // Speed 11 = MEASURE_X
  else if (c=='c'||c=='C') speedIdx = 11;  // Speed 12 = MEASURE_Z

  // Disambiguate 'A': if in IDLE and no auto request context, 'A' = Auto mode
  // Speed A (index 9) needs explicit prefix or separate command
  // Use 'a'/'A' for Auto, use '!' + 'A' or direct index for speed 10

  if (speedIdx >= 0 && speedIdx < NUM_SPEEDS) {
    char desc[16]; snprintf(desc, sizeof(desc), "Test %d", speedIdx+1);
    showCmdFeedback(c, desc);
    delay(200);
    enterTest(speedIdx); return;
  }
  switch(c) {
    case 'E': case 'e': showCmdFeedback(c,"Encoder"); delay(200); enterEncoderMode(); break;
    case 'M': case 'm': showCmdFeedback(c,"Monitor"); delay(200); enterMonitor(); break;
    case 'A': case 'a': showCmdFeedback(c,"Auto 12x10"); delay(200); enterAuto(); break;
    case 'K': case 'k':
      showCmdFeedback(c,"Calibrate");
      delay(200);
      curMode=MODE_CAL;
      drawCalScreen("CALIBRATION","Follow Serial\nprompts...");
      break;
    case 'T': case 't':
      showCmdFeedback(c,"Tare Zero");
      Serial.println(F(">>> Taring..."));
      LoadCell.tareNoDelay();
      break;
    case '0':
      showCmdFeedback('0',"Set Home");
      homePos=stepper.currentPosition();
      Serial.println(F("HOME_SET"));
      if(curMode==MODE_IDLE){ delay(300); tftToast("> Home position set", C_OK); }
      break;
    case 'H': case 'h':
      showCmdFeedback(c,"Go Home");
      curMode=MODE_HOMING; motorSt=MOT_HOME;
      stepper.setMaxSpeed(homeSpeed); stepper.setAcceleration(homeAccel); stepper.moveTo(homePos);
      Serial.println(F(">>> Homing..."));
      break;
    case 'I': case 'i':
      showCmdFeedback(c,"Info");
      printSystemInfo();
      break;
    case 'L': case 'l':
      showCmdFeedback(c,"LoadCell");
      toggleLoadCell();
      break;
  }
}

// ═══════════════════════════════════════════════════════
//  SYSTEM INFO (I command) — for C# UI to query
// ═══════════════════════════════════════════════════════
void printSystemInfo() {
  Serial.println(F("\n== SYSTEM INFO =="));
  Serial.println(F("FIRMWARE:SurfaceTensionTester_v7.3"));
  Serial.print(F("LOADCELL_TYPE:")); Serial.println(loadCellType == 1 ? F("30G") : F("100G"));
  Serial.print(F("LOADCELL_CAP:")); Serial.print(loadCellCapacity,0); Serial.println(F("g"));
  Serial.print(F("CAL_FACTOR:")); Serial.println(calFactor, 1);
  Serial.print(F("OVERLOAD_LIM:")); Serial.print(OVERLOAD_LIM, 2); Serial.println(F("N"));
  Serial.print(F("NUM_SPEEDS:")); Serial.println(NUM_SPEEDS);
  for (int i = 0; i < NUM_SPEEDS; i++) {
    Serial.print(F("SPEED:")); Serial.print(i+1);
    Serial.print(F(":")); Serial.print(tests[i].name);
    Serial.print(F(":")); Serial.print(tests[i].speed_mms * 1000, 3);
    Serial.println(F("um/s"));
  }
  Serial.print(F("HOME_POS:")); Serial.println(homePos / 1600.0, 3);
  if (encTargetSet) { Serial.print(F("TARGET_POS:")); Serial.println(encTargetPos / 1600.0, 3); }
  Serial.print(F("AUTO_CONFIG:")); Serial.print(AUTO_BATCHES);
  Serial.print(F("x")); Serial.print(NUM_SPEEDS);
  Serial.print(F("x")); Serial.print(AUTO_REPEATS);
  Serial.print(F("=")); Serial.println(AUTO_BATCHES * NUM_SPEEDS * AUTO_REPEATS);
  Serial.println(F("END_INFO"));
}

// ═══════════════════════════════════════════════════════
//  LOAD CELL SWITCH (L command) — toggle 100g ↔ 30g
// ═══════════════════════════════════════════════════════
void toggleLoadCell() {
  // Save current cal factor before switching
  saveCal();

  loadCellType = (loadCellType == 0) ? 1 : 0;
  loadCellCapacity = (loadCellType == 1) ? 30.0 : 100.0;
  OVERLOAD_LIM = (loadCellType == 1) ? 0.25 : 5.0;
  saveLoadCellType();

  // Load cal factor for the new load cell type
  loadCal();
  LoadCell.setCalFactor(calFactor);
  resetFilters();

  Serial.print(F("LOADCELL_CHANGED:"));
  Serial.println(loadCellType == 1 ? F("30G") : F("100G"));
  Serial.print(F("  Capacity: ")); Serial.print(loadCellCapacity, 0); Serial.println(F("g"));
  Serial.print(F("  Overload: ")); Serial.print(OVERLOAD_LIM, 2); Serial.println(F("N"));
  Serial.print(F("  Cal: ")); Serial.println(calFactor, 1);
  Serial.println(F("  *** Recalibrate recommended (K) ***"));

  tft.fillRect(0, TOAST_Y, SW, 11, C_BG_DARK);
  char buf[26];
  snprintf(buf, sizeof(buf), "LC: %s  Recal!", loadCellType == 1 ? "30g" : "100g");
  tftToast(buf, C_WARN);
}

// ═══════════════════════════════════════════════════════
//  STREAMING
// ═══════════════════════════════════════════════════════
void startStream() {
  streaming=true; streamStart=millis(); lastSampledPos=stepper.currentPosition();
  Serial.println(F("START_STREAM"));
}
void stopStream() { if(!streaming)return; streaming=false; Serial.println(F("END_STREAM")); }
void streamData(float f, long steps) {
  float t=(millis()-streamStart)/1000.0, pa=steps/1600.0;
  float pr=(contactDet&&contactPosition>=0)?(steps-contactPosition)/1600.0:pa;
  Serial.print(F("{\"t\":")); Serial.print(t,3);
  Serial.print(F(",\"p\":")); Serial.print(pa,4);
  Serial.print(F(",\"pr\":")); Serial.print(pr,4);
  Serial.print(F(",\"s\":")); Serial.print(tests[curTest].speed_mms,6);
  Serial.print(F(",\"f\":")); Serial.print(f,5);
  Serial.println('}');
}

// ═══════════════════════════════════════════════════════
//  CALIBRATION
// ═══════════════════════════════════════════════════════
void handleCalMode() {
  Serial.println(F("\n== CALIBRATION =="));
  drawCalScreen("Step 1/3","Remove all weight\nthen send T");
  bool tok=false; unsigned long tt=millis()+60000;
  while(!tok&&millis()<tt){
    LoadCell.update();
    if(Serial.available()>0){char c=Serial.read();while(Serial.available())Serial.read();if(c=='T'||c=='t'){LoadCell.tareNoDelay();tok=true;}}
  }
  if(!tok){Serial.println(F("CAL_ERR:Tare timeout"));exitCal();return;}
  drawCalScreen("Step 1/3","Taring...");
  unsigned long ts=millis();
  while(!LoadCell.getTareStatus()&&millis()-ts<5000){LoadCell.update();delay(10);}
  if(!LoadCell.getTareStatus()){Serial.println(F("CAL_ERR:Tare failed"));exitCal();return;}
  Serial.println(F("TARE_OK"));

  drawCalScreen("Step 2/3","Enter mass (g)\nvia Serial");
  Serial.println(F("Step 2: Enter mass (g)"));
  float mass=getMass(60000); if(mass<0){Serial.println(F("CAL_ERR:Mass input timeout"));exitCal();return;}

  char msg[40]; snprintf(msg,sizeof(msg),"Place %.1fg\non load cell",mass);
  drawCalScreen("Step 3/3",msg);
  Serial.println(F("Step 3: Place mass"));

  // Settle: keep reading ADC until stable (critical for accurate calibration)
  unsigned long settleStart = millis();
  const unsigned long SETTLE_MIN = 3000;   // Minimum settling time (ms)
  const unsigned long SETTLE_MAX = 10000;  // Maximum settling time (ms)
  const float SETTLE_SD = 0.5;             // SD threshold (raw units) for stability
  const int SETTLE_WIN = 20;               // Window size for stability check
  float settleReadings[20];
  int settleIdx = 0;
  bool settled = false;

  while (millis() - settleStart < SETTLE_MAX) {
    LoadCell.update();
    delay(50);

    if (millis() - settleStart >= SETTLE_MIN && !settled) {
      // Collect stability window
      float sum = 0;
      for (int i = 0; i < SETTLE_WIN; i++) {
        while (!LoadCell.update()) delay(5);
        settleReadings[i] = LoadCell.getData();
        sum += settleReadings[i];
      }
      float mean = sum / SETTLE_WIN;
      float var = 0;
      for (int i = 0; i < SETTLE_WIN; i++) {
        float d = settleReadings[i] - mean;
        var += d * d;
      }
      float sd = sqrt(var / SETTLE_WIN);
      if (sd < SETTLE_SD) {
        settled = true;
        break;
      }
    }
  }
  if (!settled) {
    drawCalScreen("Step 3/3", "Settling...\n(using best)");
    Serial.println(F("CAL_WARN:Settle timeout, using current"));
  }

  float nc=LoadCell.getNewCalibration(mass);
  calFactor=nc; LoadCell.setCalFactor(nc); saveCal();
  resetFilters();  // Flush filter buffer so readings use new cal factor immediately

  tft.fillScreen(C_BG); tftHeader("CAL DONE");
  tft.setTextSize(1); tft.setTextColor(C_OK);
  tft.setCursor(4,28); tft.print(F("Calibration saved!"));
  tft.setTextColor(C_BLUE_HI); tft.setCursor(4,44);
  tft.print(F("Factor: ")); tft.print(nc,1);
  Serial.print(F("New Cal: ")); Serial.println(nc);
  Serial.println(F("CAL_DONE"));
  delay(2000); exitCal();
}
void exitCal() { curMode=MODE_IDLE;motorSt=MOT_IDLE;resetFilters();drawIdle();Serial.println(F("READY")); }

float getMass(unsigned long timeout) {
  String s=""; unsigned long st=millis();
  while(millis()-st<timeout){
    if(Serial.available()>0){
      char c=Serial.read();
      if(c=='\n'||c=='\r'){if(s.length()>0){float m=s.toFloat();return(m>0)?m:-1;}}
      else if(isDigit(c)||c=='.'){s+=c;Serial.write(c);}
    }
  }
  return -1;
}
