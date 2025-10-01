#include "Filters.h"

#define TRIG_PIN 5
#define ECHO_PIN 2
const unsigned long PERIOD_MS = 100;

const bool COL_EMA     = true;
const bool COL_LP      = true;
const bool COL_MEDIAN  = true;
const bool COL_OUTLIER = true;
const bool COL_KALMAN  = true;

const float EMA_ALPHA = 0.25f;
const float LP_BETA   = 0.25f;
const int   MED_WIN   = 5;
const int   OUT_WIN   = 11;
const float OUT_THR   = 3.5f;
float KALMAN_Q = 1e-3f;
float KALMAN_R = 0.5f;

EmaFilter            g_ema(EMA_ALPHA);
LowpassFilter        g_lp(LP_BETA);
MedianFilter<21>     g_median;
OutlierReplacer<21>  g_outlier;
Kalman1D             g_kf(KALMAN_Q, KALMAN_R);

unsigned long last_ms = 0;

static float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (dur == 0) return -1.0f;
  return (dur * 0.0343f) * 0.5f;
}

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  g_median.setWindow(MED_WIN);
  g_outlier.setWindow(OUT_WIN);
  g_outlier.setThreshold(OUT_THR);
  g_kf.setNoise(KALMAN_Q, KALMAN_R);
  delay(400);
  Serial.print("t_ms,raw_cm");
  if (COL_EMA)     Serial.print(",ema_cm");
  if (COL_LP)      Serial.print(",lp_cm");
  if (COL_MEDIAN)  Serial.print(",median_cm");
  if (COL_OUTLIER) Serial.print(",outlier_cm");
  if (COL_KALMAN)  Serial.print(",kalman_cm");
  Serial.println();
}

void loop() {
  const unsigned long now = millis();
  if (now - last_ms < PERIOD_MS) return;
  last_ms = now;
  const float raw = readDistanceCm();
  const bool valid = flt_is_valid(raw);
  g_median.push(raw, valid);
  g_outlier.push(raw, valid);
  float ema_val = NAN, lp_val = NAN, med_val = NAN, out_val = NAN, kf_val = NAN;
  if (COL_EMA)     ema_val = g_ema.update(raw, valid);
  if (COL_LP)      lp_val  = g_lp.update(raw, valid);
  if (COL_MEDIAN)  med_val = g_median.median();
  if (COL_OUTLIER) out_val = g_outlier.replace(raw, valid);
  if (COL_KALMAN)  kf_val  = g_kf.update(valid ? raw : NAN, valid);
  Serial.print(now); Serial.print(",");
  if (isfinite(raw)) Serial.print(raw, 3);
  if (COL_EMA)     { Serial.print(","); if (isfinite(ema_val)) Serial.print(ema_val, 3); }
  if (COL_LP)      { Serial.print(","); if (isfinite(lp_val))  Serial.print(lp_val, 3); }
  if (COL_MEDIAN)  { Serial.print(","); if (isfinite(med_val)) Serial.print(med_val, 3); }
  if (COL_OUTLIER) { Serial.print(","); if (isfinite(out_val)) Serial.print(out_val, 3); }
  if (COL_KALMAN)  { Serial.print(","); if (isfinite(kf_val))  Serial.print(kf_val, 3); }
  Serial.println();
}
