#ifndef _FILTERS_H_
#define _FILTERS_H_
#include <Arduino.h>

static inline bool flt_is_valid(float v) {
  return isfinite(v) && v >= 0.0f;
}

template<typename T, int N>
class RingBuffer {
public:
  RingBuffer(): _count(0), _head(0) {}
  void clear() { _count = 0; _head = 0; }
  void push(const T& v) {
    _buf[_head] = v;
    _head = (_head + 1) % N;
    if (_count < N) _count++;
  }
  int size() const { return _count; }
  int capacity() const { return N; }
  bool empty() const { return _count == 0; }
  int copy_recent(T* dst, int take) const {
    if (_count == 0) return 0;
    if (take > _count) take = _count;
    int idx = 0;
    int p = _head - 1; if (p < 0) p = N - 1;
    while (idx < take) {
      dst[idx++] = _buf[p];
      p--; if (p < 0) p = N - 1;
    }
    return take;
  }
private:
  T _buf[N];
  int _count;
  int _head;
};

namespace detail {
  template<int N>
  static float median_sorted_copy(const float* src, int n) {
    if (n <= 0) return NAN;
    float tmp[N];
    for (int i = 0; i < n; ++i) tmp[i] = src[i];
    for (int i = 1; i < n; ++i) {
      float key = tmp[i];
      int j = i - 1;
      while (j >= 0 && tmp[j] > key) { tmp[j+1] = tmp[j]; j--; }
      tmp[j+1] = key;
    }
    if (n & 1) return tmp[n/2];
    return 0.5f * (tmp[n/2 - 1] + tmp[n/2]);
  }
}

class EmaFilter {
public:
  explicit EmaFilter(float alpha=0.25f): _alpha(alpha) { reset(); }
  void setAlpha(float a) { _alpha = a; }
  void reset() { _y = NAN; }
  float update(float x, bool valid=true) {
    if (!isfinite(_y)) {
      if (valid && flt_is_valid(x)) _y = x;
      return _y;
    }
    float in = (valid && flt_is_valid(x)) ? x : _y;
    _y = _y + _alpha * (in - _y);
    return _y;
  }
  float value() const { return _y; }
private:
  float _alpha;
  float _y;
};

class LowpassFilter {
public:
  explicit LowpassFilter(float beta=0.25f): _beta(beta) { reset(); }
  void setBeta(float b) { _beta = b; }
  void reset() { _y = NAN; }
  float update(float x, bool valid=true) {
    if (!isfinite(_y)) {
      if (valid && flt_is_valid(x)) _y = x;
      return _y;
    }
    float in = (valid && flt_is_valid(x)) ? x : _y;
    _y = _y + _beta * (in - _y);
    return _y;
  }
  float value() const { return _y; }
private:
  float _beta;
  float _y;
};

template<int MAX_WIN=21>
class MedianFilter {
public:
  MedianFilter(): _win(5) {}
  void reset() { _ring.clear(); }
  void setWindow(int w) { _win = constrain_odd_window(w); }
  void push(float x, bool valid=true) {
    if (valid && flt_is_valid(x)) _ring.push(x);
  }
  float median() {
    if (_ring.empty()) return NAN;
    int take = min(_ring.size(), _win);
    float buf[MAX_WIN];
    take = _ring.copy_recent(buf, take);
    return detail::median_sorted_copy<MAX_WIN>(buf, take);
  }
private:
  static int constrain_odd_window(int w) {
    if (w < 1) w = 1;
    if (w > MAX_WIN) w = MAX_WIN;
    if ((w & 1) == 0) ++w;
    return w;
  }
  RingBuffer<float, MAX_WIN> _ring;
  int _win;
};

template<int MAX_WIN=21>
class OutlierReplacer {
public:
  OutlierReplacer(): _win(11), _thr(3.5f) {}
  void reset() { _ring.clear(); }
  void setWindow(int w) { _win = constrain_odd_window(w); }
  void setThreshold(float t) { _thr = t; }
  void push(float x, bool valid=true) {
    if (valid && flt_is_valid(x)) _ring.push(x);
  }
  float replace(float raw, bool rawValid=true) {
    if (_ring.empty()) return rawValid && flt_is_valid(raw) ? raw : NAN;
    int take = min(_ring.size(), _win);
    float buf[MAX_WIN];
    take = _ring.copy_recent(buf, take);
    float med = detail::median_sorted_copy<MAX_WIN>(buf, take);
    if (!isfinite(med)) return raw;
    for (int i=0;i<take;++i) buf[i] = fabsf(buf[i] - med);
    float mad = detail::median_sorted_copy<MAX_WIN>(buf, take);
    float mad_scaled = 1.4826f * mad;
    if (!(mad_scaled > 1e-6f)) {
      return (rawValid && flt_is_valid(raw)) ? raw : med;
    }
    float candidate = (rawValid && flt_is_valid(raw)) ? raw : med;
    float dev = fabsf(candidate - med);
    if (dev > _thr * mad_scaled) return med;
    return candidate;
  }
private:
  static int constrain_odd_window(int w) {
    if (w < 1) w = 1;
    if (w > MAX_WIN) w = MAX_WIN;
    if ((w & 1) == 0) ++w;
    return w;
  }
  RingBuffer<float, MAX_WIN> _ring;
  int _win;
  float _thr;
};

class Kalman1D {
public:
  Kalman1D(float Q=1e-3f, float R=0.5f): _Q(Q), _R(R) { reset(); }
  void setNoise(float Q, float R) { _Q = Q; _R = R; }
  void reset() { _x = NAN; _P = 1.0f; }
  float update(float z, bool valid=true) {
    if (!isfinite(_x)) {
      if (valid && flt_is_valid(z)) { _x = z; _P = 1.0f; }
      return _x;
    }
    float x_minus = _x;
    float P_minus = _P + _Q;
    if (valid && flt_is_valid(z)) {
      float K = P_minus / (P_minus + _R);
      _x = x_minus + K * (z - x_minus);
      _P = (1.0f - K) * P_minus;
    } else {
      _x = x_minus;
      _P = P_minus;
    }
    return _x;
  }
  float value() const { return _x; }
  float variance() const { return _P; }
private:
  float _Q, _R;
  float _x, _P;
};

#endif // _FILTERS_H_