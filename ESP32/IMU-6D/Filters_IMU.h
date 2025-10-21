#ifndef _FILTERS_IMU_H_
#define _FILTERS_IMU_H_

#include <Arduino.h>

struct FilterOutputs {
  float ema;
  float median;
  float lp;
  float outlier;
};

class FilterBank {
public:
  void configure(float fs_hz, float ema_alpha_in = 0.2f, float lp_cut_hz = 5.0f, uint8_t win = 5, float hampel_k = 3.0f) {
    fs = fs_hz;
    ema_alpha = ema_alpha_in;
    winK = (win % 2 == 0) ? win + 1 : win;
    hampel_thresh = hampel_k;
    float dt = 1.0f / (fs > 1 ? fs : 100.0f);
    float rc = 1.0f / (2.0f * PI * lp_cut_hz);
    lp_alpha = dt / (rc + dt);
    reset();
  }
  void reset() { ema_state = NAN; lp_state = NAN; bufCount = 0; bufIndex = 0; }
  FilterOutputs update(float x) {
    if (isnan(ema_state)) ema_state = x; else ema_state += ema_alpha * (x - ema_state);
    if (isnan(lp_state)) lp_state = x; else lp_state += lp_alpha * (x - lp_state);
    pushToBuffer(x);
    float med = medianOfBuffer();
    float mad = madOfBuffer(med);
    float z = (mad > 1e-9f) ? 0.6745f * (x - med) / mad : 0.0f;
    float out = (fabsf(z) > hampel_thresh) ? med : x;
    FilterOutputs o; o.ema = ema_state; o.median = med; o.lp = lp_state; o.outlier = out; return o;
  }
private:
  float fs = 100.0f, ema_alpha = 0.2f, lp_alpha = 0.1f; uint8_t winK = 5; float hampel_thresh = 3.0f;
  float ema_state = NAN, lp_state = NAN, buf[9]; uint8_t bufCount = 0, bufIndex = 0;
  void pushToBuffer(float v) { if (winK > 9) winK = 9; if (bufCount < winK) buf[bufCount++] = v; else { buf[bufIndex] = v; bufIndex = (bufIndex + 1) % winK; } }
  float medianOfBuffer() { uint8_t n = bufCount; if (!n) return NAN; float temp[9]; for (uint8_t i=0;i<n;i++) temp[i]=buf[i]; for (uint8_t i=1;i<n;i++){float key=temp[i];int j=i-1;while(j>=0&&temp[j]>key){temp[j+1]=temp[j];j--;}temp[j+1]=key;} return temp[n/2]; }
  float madOfBuffer(float med){uint8_t n=bufCount;if(!n)return 0;float dev[9];for(uint8_t i=0;i<n;i++)dev[i]=fabsf(buf[i]-med);for(uint8_t i=1;i<n;i++){float key=dev[i];int j=i-1;while(j>=0&&dev[j]>key){dev[j+1]=dev[j];j--;}dev[j+1]=key;}return dev[n/2]+1e-9f;}
};

#endif // _FILTERS_IMU_H_