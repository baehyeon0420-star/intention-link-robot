/*
  MyoWare 2.0 (EMG 센서) -> ESP32 -> 시리얼 출력

  intention-link-robot의 emg_pipeline/serial_reader.py (collect.py 포맷과 동일)이
  기대하는 형식으로 출력한다: "t_ms,raw\n"

  배선 (MyoWare 2.0 -> ESP32 DevKitC V4):
    +  (VCC) -> 3.3V
    -  (GND) -> GND
    SIG      -> GPIO34 (ADC1, 입력 전용 핀. WiFi 쓸 때도 ADC2와 안 겹쳐서 안전)

  전극(패드) 부착:
    +/- 전극 2개 : 전완근(손 쥘 때 쓰는 아래팔 안쪽 근육) 위에 근육 결 방향으로 나란히
    REF 전극 1개 : 손목뼈/팔꿈치처럼 근육이 없는 전기적 중립 부위

  시리얼: 115200bps. main.py --port 로 이 포트를 지정하면 됨.
*/

const int EMG_PIN = 34;      // MyoWare SIG 연결 핀
const unsigned long SAMPLE_INTERVAL_MS = 2;  // 약 500Hz

unsigned long last_sample_ms = 0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);   // ESP32 ADC 기본 12비트 (0~4095)
  // MyoWare 신호가 안정화될 시간을 좀 준다.
  delay(500);
}

void loop() {
  unsigned long now = millis();
  if (now - last_sample_ms < SAMPLE_INTERVAL_MS) {
    return;
  }
  last_sample_ms = now;

  int raw = analogRead(EMG_PIN);

  Serial.print(now);
  Serial.print(",");
  Serial.println(raw);
}
