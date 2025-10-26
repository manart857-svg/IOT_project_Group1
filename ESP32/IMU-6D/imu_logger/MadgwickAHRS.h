#ifndef _MADGWICK_AHRS_H_
#define _MADGWICK_AHRS_H_

#include <Arduino.h>

class MadgwickAHRS {
public:
  void setSamplePeriod(float sp) { samplePeriod = sp; }
  void setBeta(float b) { beta = b; }
  void update(float gx, float gy, float gz, float ax, float ay, float az) {
    float q1 = q0, q2 = q1_, q3 = q2_, q4 = q3_;
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm < 1e-9f) { integrateGyro(gx, gy, gz); return; }
    ax /= norm; ay /= norm; az /= norm;
    float _2q1 = 2.0f*q1, _2q2 = 2.0f*q2, _2q3 = 2.0f*q3, _2q4 = 2.0f*q4, _4q1 = 4.0f*q1, _4q2 = 4.0f*q2, _4q3 = 4.0f*q3;
    float q1q1 = q1*q1, q2q2 = q2*q2, q3q3 = q3*q3, q4q4 = q4*q4;
    float s1 = _4q1*q3q3 + _2q3*ax + _4q1*q2q2 - _2q2*ay;
    float s2 = _4q2*q4q4 - _2q4*ax + 4.0f*q1q1*q2 - _2q1*ay - _4q2 + 8.0f*q2q2*q2 + 8.0f*q2*q3q3 + _4q2*az;
    float s3 = 4.0f*q1q1*q3 + _2q1*ax + _4q3*q4q4 - _2q4*ay - _4q3 + 8.0f*q3*q2q2 + 8.0f*q3*q3q3 + _4q3*az;
    float s4 = 4.0f*q2*q2*q4 - _2q2*ax + 4.0f*q3*q3*q4 - _2q3*ay;
    norm = sqrtf(s1*s1 + s2*s2 + s3*s3 + s4*s4); if (norm > 1e-9f){s1/=norm;s2/=norm;s3/=norm;s4/=norm;}
    float qDot1 = 0.5f*(-q2*gx - q3*gy - q4*gz) - beta*s1;
    float qDot2 = 0.5f*( q1*gx + q3*gz - q4*gy) - beta*s2;
    float qDot3 = 0.5f*( q1*gy - q2*gz + q4*gx) - beta*s3;
    float qDot4 = 0.5f*( q1*gz + q2*gy - q3*gx) - beta*s4;
    q1 += qDot1*samplePeriod; q2 += qDot2*samplePeriod; q3 += qDot3*samplePeriod; q4 += qDot4*samplePeriod;
    norm = sqrtf(q1*q1 + q2*q2 + q3*q3 + q4*q4);
    if (norm < 1e-9f) { q1=1;q2=q3=q4=0; } else { q1/=norm; q2/=norm; q3/=norm; q4/=norm; }
    q0=q1; q1_=q2; q2_=q3; q3_=q4;
  }
  void getQuaternion(float &oq0,float &oq1,float &oq2,float &oq3)const{oq0=q0;oq1=q1_;oq2=q2_;oq3=q3_;}
private:
  float samplePeriod=1.0f/100.0f,beta=0.1f,q0=1.0f,q1_=0.0f,q2_=0.0f,q3_=0.0f;
  void integrateGyro(float gx,float gy,float gz){float q1=q0,q2=q1_,q3=q2_,q4=q3_;float qDot1=0.5f*(-q2*gx - q3*gy - q4*gz);float qDot2=0.5f*( q1*gx + q3*gz - q4*gy);float qDot3=0.5f*( q1*gy - q2*gz + q4*gx);float qDot4=0.5f*( q1*gz + q2*gy - q3*gx);q1+=qDot1*samplePeriod;q2+=qDot2*samplePeriod;q3+=qDot3*samplePeriod;q4+=qDot4*samplePeriod;float norm=sqrtf(q1*q1+q2*q2+q3*q3+q4*q4);if(norm<1e-9f){q0=1;q1_=q2_=q3_=0;return;}q0=q1/norm;q1_=q2/norm;q2_=q3/norm;q3_=q4/norm;}
};

#endif // _MADGWICK_AHRS_H_