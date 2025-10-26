#include <MPU9250_WE.h>
#include <Wire.h>
#define MPU9250_ADDR 0x68

/* ----------------------- Filters ----------------------- */
struct FilterOutputs {
  float ema;     // exponential moving average
  float median;  // median of last 3 samples
  float lp;      // simple moving-average (boxcar) low-pass
  float outlier; // outlier-clamped sample
};

struct FilterBank {
  // Tunables
  float emaAlpha = 0.2f;      // EMA smoothing factor
  static const int LP_N = 5;  // moving-average window

  // State
  float ema = 0.f;
  float lpBuf[LP_N];
  int   lpIdx = 0;
  int   lpCount = 0;
  float m3a = 0.f, m3b = 0.f, m3c = 0.f;
  bool  inited = false;

  float lastGood = 0.f;
  float estMean  = 0.f;
  float estVar   = 1e-6f;
  float beta     = 0.01f;     // update rate for mean/var
  float zThresh  = 3.5f;      // outlier threshold

  // Optional API to match your previous code style
  void configure(float fs) {
    // You can adapt internal parameters based on fs if desired.
    // For now, keep defaults; just having this method fixes the compile error.
    (void)fs;
  }

  void init(float x0) {
    ema = x0;
    for (int i=0;i<LP_N;i++) lpBuf[i] = x0;
    lpIdx = 0; lpCount = LP_N;
    m3a = m3b = m3c = x0;
    lastGood = x0;
    estMean  = x0;
    estVar   = 1e-6f;
    inited   = true;
  }

  static float median3(float a, float b, float c) {
    if (a > b) { float t=a; a=b; b=t; }
    if (b > c) { float t=b; b=c; c=t; }
    if (a > b) { float t=a; a=b; b=t; }
    return b;
  }

  FilterOutputs update(float x) {
    if (!inited) init(x);

    ema = ema + emaAlpha * (x - ema);

    lpBuf[lpIdx] = x;
    lpIdx = (lpIdx + 1) % LP_N;
    if (lpCount < LP_N) lpCount++;
    float sum = 0.f;
    for (int i=0;i<lpCount;i++) sum += lpBuf[i];
    float lp = sum / (float)lpCount;

    m3a = m3b; m3b = m3c; m3c = x;
    float med = median3(m3a, m3b, m3c);

    float diff = x - estMean;
    estMean += beta * diff;
    estVar  = (1.0f - beta) * (estVar + beta * diff * diff);
    float sigma = sqrtf(fmaxf(estVar, 1e-9f));
    float z = fabsf(x - estMean) / (sigma < 1e-6f ? 1e-6f : sigma);
    float xo = (z > zThresh) ? lastGood : x;
    lastGood = xo;

    return { ema, med, lp, xo };
  }
};

/* ----------------------- Madgwick (IMU-only) ----------------------- */
struct MadgwickIMU {
  float q0=1, q1=0, q2=0, q3=0;
  float beta = 0.1f;
  float dt   = 0.01f;    // seconds

  void setSamplePeriod(float dt_s) { dt = dt_s; }
  void setBeta(float b) { beta = b; }

  void update(float gx, float gy, float gz, float ax, float ay, float az) {
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 1e-6f) return;
    ax /= norm; ay /= norm; az /= norm;

    float _2q0 = 2.0f*q0, _2q1 = 2.0f*q1, _2q2 = 2.0f*q2, _2q3 = 2.0f*q3;
    float _4q0 = 4.0f*q0, _4q1 = 4.0f*q1, _4q2 = 4.0f*q2;
    float _8q1 = 8.0f*q1, _8q2 = 8.0f*q2;
    float q0q0 = q0*q0, q1q1 = q1*q1, q2q2 = q2*q2, q3q3 = q3*q3;

    float s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
    float s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
    float s2 = 4.0f*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
    float s3 = 4.0f*q1q1*q3 - _2q1*ax + 4.0f*q2q2*q3 - _2q2*ay;
    norm = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    if (norm > 1e-9f) { s0/=norm; s1/=norm; s2/=norm; s3/=norm; }

    float qDot0 = 0.5f*(-q1*gx - q2*gy - q3*gz) - beta*s0;
    float qDot1 = 0.5f*( q0*gx + q2*gz - q3*gy) - beta*s1;
    float qDot2 = 0.5f*( q0*gy - q1*gz + q3*gx) - beta*s2;
    float qDot3 = 0.5f*( q0*gz + q1*gy - q2*gx) - beta*s3;

    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm > 1e-9f) { q0/=norm; q1/=norm; q2/=norm; q3/=norm; }
  }
};

MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

FilterBank accXFilt, accYFilt, accZFilt;
FilterBank gyrXFilt, gyrYFilt, gyrZFilt;

MadgwickIMU ahrs;

static inline float deg2rad(float d) { return d * (PI / 180.0f); }
static inline float g2ms2 (float g)  { return g * 9.80665f; }  // g -> m/s^2

void printCsvHeader() {
  Serial.println(
    "time_ms,"
    "ax,ay,az,"           // accel (m/s^2)
    "gx,gy,gz,"           // gyro  (rad/s)
    "ax_ema,ay_ema,az_ema,"
    "gx_ema,gy_ema,gz_ema,"
    "ax_median,ay_median,az_median,"
    "gx_median,gy_median,gz_median,"
    "ax_lp,ay_lp,az_lp,"
    "gx_lp,gy_lp,gz_lp,"
    "ax_outlier,ay_outlier,az_outlier,"
    "gx_outlier,gy_outlier,gz_outlier,"
    "q0,q1,q2,q3"
  );
}

/* ----------------------- setup() ----------------------- */
void setup() {
  Serial.begin(115200);
  Wire.begin();
  if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }

  Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
  delay(1000);
  myMPU9250.autoOffsets();
  Serial.println("Done!");

  myMPU9250.setSampleRateDivider(5);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);

  const float fs = 100.0f;               // logical rate for filters/AHRS
  accXFilt.configure(fs); accYFilt.configure(fs); accZFilt.configure(fs);
  gyrXFilt.configure(fs); gyrYFilt.configure(fs); gyrZFilt.configure(fs);
  ahrs.setSamplePeriod(1.0f / fs);
  ahrs.setBeta(0.1f);

  printCsvHeader();
}

/* ----------------------- loop() ----------------------- */
void loop() {
  static uint32_t t_prev_ms = millis();

  xyzFloat accRaw = myMPU9250.getAccRawValues();
  xyzFloat accCorrRaw = myMPU9250.getCorrectedAccRawValues();
  xyzFloat gValue = myMPU9250.getGValues();
  float resultantG = myMPU9250.getResultantG(gValue);

  

  // === Filters + Madgwick + CSV ===
  float ax = g2ms2(gValue.x);
  float ay = g2ms2(gValue.y);
  float az = g2ms2(gValue.z);

  xyzFloat dps = myMPU9250.getGyrValues();
  float gx = dps.x;
  float gy = dps.y;
  float gz = dps.z;

  uint32_t t_now_ms = millis();
  float dt_s = (t_now_ms - t_prev_ms) * 1e-3f;
  t_prev_ms = t_now_ms;
  if (dt_s <= 0.0f) dt_s = 0.01f;
  ahrs.setSamplePeriod(dt_s);

  FilterOutputs fax = accXFilt.update(ax);
  FilterOutputs fay = accYFilt.update(ay);
  FilterOutputs faz = accZFilt.update(az);
  FilterOutputs fgx = gyrXFilt.update(gx);
  FilterOutputs fgy = gyrYFilt.update(gy);
  FilterOutputs fgz = gyrZFilt.update(gz);

  ahrs.update(gx, gy, gz, gValue.x, gValue.y, gValue.z);
  float q0 = ahrs.q0, q1 = ahrs.q1, q2 = ahrs.q2, q3 = ahrs.q3;

  Serial.print(t_now_ms); Serial.print(',');

  Serial.print(ax, 6);  Serial.print(',');
  Serial.print(ay, 6);  Serial.print(',');
  Serial.print(az, 6);  Serial.print(',');

  Serial.print(gx, 6);  Serial.print(',');
  Serial.print(gy, 6);  Serial.print(',');
  Serial.print(gz, 6);  Serial.print(',');

  Serial.print(fax.ema, 6); Serial.print(',');
  Serial.print(fay.ema, 6); Serial.print(',');
  Serial.print(faz.ema, 6); Serial.print(',');

  Serial.print(fgx.ema, 6); Serial.print(',');
  Serial.print(fgy.ema, 6); Serial.print(',');
  Serial.print(fgz.ema, 6); Serial.print(',');

  Serial.print(fax.median, 6); Serial.print(',');
  Serial.print(fay.median, 6); Serial.print(',');
  Serial.print(faz.median, 6); Serial.print(',');

  Serial.print(fgx.median, 6); Serial.print(',');
  Serial.print(fgy.median, 6); Serial.print(',');
  Serial.print(fgz.median, 6); Serial.print(',');

  Serial.print(fax.lp, 6); Serial.print(',');
  Serial.print(fay.lp, 6); Serial.print(',');
  Serial.print(faz.lp, 6); Serial.print(',');

  Serial.print(fgx.lp, 6); Serial.print(',');
  Serial.print(fgy.lp, 6); Serial.print(',');
  Serial.print(fgz.lp, 6); Serial.print(',');

  Serial.print(fax.outlier, 6); Serial.print(',');
  Serial.print(fay.outlier, 6); Serial.print(',');
  Serial.print(faz.outlier, 6); Serial.print(',');

  Serial.print(fgx.outlier, 6); Serial.print(',');
  Serial.print(fgy.outlier, 6); Serial.print(',');
  Serial.print(fgz.outlier, 6); Serial.print(',');

  Serial.print(q0, 6);  Serial.print(',');
  Serial.print(q1, 6);  Serial.print(',');
  Serial.print(q2, 6);  Serial.println(q3, 6);

  delay(1000);
}