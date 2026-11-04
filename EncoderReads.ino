
const int LA = 10;
const int LB = 11;
const int RA = 12;
const int RB = 13;

volatile long leftCount = 0;
volatile long rightCount = 0;

const float COUNTS_PER_REV = 1092.0;
const unsigned long INTERVAL = 250;

void ARDUINO_ISR_ATTR leftEncoder() {
  if (digitalRead(LA) == digitalRead(LB))
    leftCount++;
  else
    leftCount--;
}

void ARDUINO_ISR_ATTR rightEncoder() {
  if (digitalRead(RA) == digitalRead(RB))
    rightCount++;
  else
    rightCount--;
}

void setup() {
  Serial.begin(115200);

  pinMode(LA, INPUT_PULLUP);
  pinMode(LB, INPUT_PULLUP);
  pinMode(RA, INPUT_PULLUP);
  pinMode(RB, INPUT_PULLUP);

  attachInterrupt(LA, leftEncoder, CHANGE);
  attachInterrupt(RA, rightEncoder, CHANGE);

  Serial.println("Dual Encoder Test Started");
}

void loop() {
  static unsigned long lastTime = millis();
  static long previousLeft = 0;
  static long previousRight = 0;

  unsigned long now = millis();

  if (now - lastTime >= INTERVAL) {
    noInterrupts();
    long left = leftCount;
    long right = rightCount;
    interrupts();

    float dt = (now - lastTime) / 60000.0;

    float leftRPM =
      (left - previousLeft) / COUNTS_PER_REV / dt;

    float rightRPM =
      (right - previousRight) / COUNTS_PER_REV / dt;

    Serial.print("Left: ");
    Serial.print(left);
    Serial.print(" counts, ");
    Serial.print(leftRPM);
    Serial.print(" RPM | Right: ");
    Serial.print(right);
    Serial.print(" counts, ");
    Serial.print(rightRPM);
    Serial.println(" RPM");

    previousLeft = left;
    previousRight = right;
    lastTime = now;
  }
}
