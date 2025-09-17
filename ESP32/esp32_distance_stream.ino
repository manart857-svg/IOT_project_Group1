#define TRIG_PIN 4
#define ECHO_PIN 2


// Measurement period (ms)
const unsigned long PERIOD_MS = 100; // 10 Hz


void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  // Print CSV header once
  delay(800);
  Serial.println("t_ms,duration_us,dist_cm");
  delay(50);
  Serial.println("t_ms,duration_us,dist_cm");
}


unsigned long last_ms = 0;


void loop() {
  unsigned long now = millis();
  if (now - last_ms >= PERIOD_MS) {
      last_ms = now;


  // Trigger 10us pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);


  // Measure echo (timeout ~30ms)
  unsigned long dur = pulseIn(ECHO_PIN, HIGH, 30000UL);


  float dist_cm = -1.0f;
  if (dur > 0) {
  // Sound speed ~ 343 m/s => 0.0343 cm/us
      dist_cm = (dur * 0.0343f) / 2.0f;
  }


  // CSV line
  Serial.print(now);
  Serial.print(",");
  Serial.print(dur);
  Serial.print(",");
  Serial.println(dist_cm, 3);
  }
}