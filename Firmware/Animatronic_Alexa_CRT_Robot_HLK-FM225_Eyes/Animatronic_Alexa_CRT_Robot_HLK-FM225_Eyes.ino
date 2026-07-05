#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// How long to wait between processing tracking frames.
const int32_t SAMPLE_DELAY_MS = 200;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN 140  // Minimum pulse length count (out of 4096)
#define SERVOMAX 520  // Maximum pulse length count (out of 4096)

int box_center_x;
int box_center_y;
int prev_center_x = 1;

unsigned long currentBlinkMillis = 0;
unsigned long previousBlinkMillis = 0;  
const long blinkInterval = random(1200, 5000); 
int blinkNumber = random(1, 2);

unsigned long currentSleepMillis = 0;
unsigned long previousSleepMillis = 0;
const long sleepInterval = 20000; 

unsigned long loop_number = 0;

enum States {
  DORMANT,   // eyelids are closed, eyes are not moving
  AWAKE      // eyelids are open, eyes are tracking faces
};

States State;

uint8_t servonum = 0;

int xval;
int yval;
int trimval;
int current_xval;
int prev_xval = 512;

int lexpulse;
int rexpulse;

int leypulse;
int reypulse;

int uplidpulse;
int lolidpulse;
int altuplidpulse;
int altlolidpulse;

int sensorValue = 0;
int outputValue = 0;
int switchval = 0;
int loopNumber = 0;

int ledPin = 3;
int sensorPin = A6;
unsigned long awakeTime = 30000;

// HLK-FM225 Parsing Variables
uint8_t packetBuffer[64];
int bufferIndex = 0;
unsigned long lastSensorReadTime = 0;

// Extracted face box details
int face_left = 0;
int face_top = 0;
int face_right = 0;
int face_bottom = 0;
bool face_detected = false;

void setup() {
  Wire.begin();
  pinMode(2, INPUT);
  
  Serial.begin(9600);     // USB Debugging
  Serial1.begin(115200);  // HLK-FM225 Default Baud Rate (Pins 18/19 on Mega)
  
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  State = DORMANT;
  
  trimval = 500;   // Higher number = wider eyes
  trimval = map(trimval, 320, 580, -40, 40);
  uplidpulse = map(yval, 0, 1023, 400, 280);
  uplidpulse -= (trimval - 40);
  uplidpulse = constrain(uplidpulse, 280, 400);
  altuplidpulse = 680 - uplidpulse;

  lolidpulse = map(yval, 0, 1023, 410, 280);
  lolidpulse += (trimval / 2);
  lolidpulse = constrain(lolidpulse, 280, 400);
  altlolidpulse = 680 - lolidpulse;
  
  // Power up sequence to test eyes
  closeEyes();   
  delay(3000);
  wakeup();
  delay(1000);
  driftoff();
  delay(1000);

  Serial.println("Setup complete. Awaiting FM225 frames...");
}

void loop() {
  int sensorValue = analogRead(sensorPin); // Reads Echo LEDs
  float voltage = sensorValue * (5 / 1023.0);

  // Parse incoming serial data from the FM225 in the background
  readFM225();

  if (voltage > 1.5 && State == DORMANT) {  
    State = DORMANT;
  } else if (voltage <= 1.5 && State == DORMANT) {  
    wakeup();
  } else if (voltage > 1.5 && State == AWAKE) {  
    if(millis() - awakeTime > 0) {
        awake();
        awakeTime = millis();
    } else {
      driftoff();
    }
  } else if (voltage <= 1.5 && State == AWAKE) {   
    awake();
  }   
  delay(10); // Smoother background loop processing
}

void readFM225() {
  while (Serial1.available() > 0) {
    uint8_t c = Serial1.read();
    
    // Look for Header: assuming standard 0xEF 0xAA or matching sequence layout
    if (bufferIndex == 0 && c != 0xEF) continue; 
    
    packetBuffer[bufferIndex++] = c;
    
    // Once packet length byte or baseline frame limit reached (e.g. 24-32 bytes structural info frame)
    if (bufferIndex >= 24) { 
      // Basic verification of the frame's integrity or tracking state
      uint8_t statusFlag = packetBuffer[3]; 
      
      if (statusFlag == 0x00) { // 0x00 typically denotes a valid face profile target
        face_detected = true;
        // High/low byte reconstruction for boundary limits
        face_left   = (packetBuffer[5] << 8) | packetBuffer[6];
        face_top    = (packetBuffer[7] << 8) | packetBuffer[8];
        face_right  = (packetBuffer[9] << 8) | packetBuffer[10];
        face_bottom = (packetBuffer[11] << 8) | packetBuffer[12];
      } else {
        face_detected = false;
      }
      bufferIndex = 0; // Reset parse sequence
    }
  }
}

void blink() {  
  trimval = 550;   
  trimval = map(trimval, 320, 580, -40, 40);
  uplidpulse = map(yval, 0, 1023, 400, 280);
  uplidpulse -= (trimval - 40);
  uplidpulse = constrain(uplidpulse, 280, 400);
  altuplidpulse = 680 - uplidpulse;

  lolidpulse = map(yval, 0, 1023, 410, 280);
  lolidpulse += (trimval / 2);
  lolidpulse = constrain(lolidpulse, 280, 400);
  altlolidpulse = 680 - lolidpulse;
  
  // Closes eyelids
  pwm.setPWM(2, 0, 500);
  pwm.setPWM(3, 0, 240);
  pwm.setPWM(4, 0, 240);
  pwm.setPWM(5, 0, 500);

  delay(80);

  // Opens eyelids to trimval value  
  pwm.setPWM(2, 0, uplidpulse);
  pwm.setPWM(3, 0, lolidpulse);
  pwm.setPWM(4, 0, altuplidpulse);
  pwm.setPWM(5, 0, altlolidpulse);
}

void awake() {
  if (State != AWAKE) {
    Serial.println("AWAKE");
  }

  // Restrict parsing evaluation timing to target Sample Delay Interval
  if (millis() - lastSensorReadTime < SAMPLE_DELAY_MS) {
    return;
  }
  lastSensorReadTime = millis();

  if (!face_detected) {
    return;
  }

  // Turn sensor coordinate space into precise layout targets
  box_center_x = (face_left + (face_right - face_left) / 2);  
  box_center_y = (face_bottom + (face_top - face_bottom) / 2);  
  
  prev_center_x = box_center_x;

  // SERVO POSITIONING
  xval = ((box_center_x * -4) + 1023) * 1.2;   
  yval = ((box_center_y * -4) + 1023) * 1.2;
  
  lexpulse = map(xval, 0, 1023, 220, 440);
  rexpulse = lexpulse;
  leypulse = map(yval, 0, 1023, 250, 500);
  reypulse = map(yval, 0, 1023, 400, 280);

  trimval = 550;   
  trimval = map(trimval, 320, 580, -40, 40);
  uplidpulse = map(yval, 0, 1023, 400, 280);
  uplidpulse -= (trimval - 40);
  uplidpulse = constrain(uplidpulse, 280, 400);
  altuplidpulse = 680 - uplidpulse;

  lolidpulse = map(yval, 0, 1023, 410, 280);
  lolidpulse += (trimval / 2);
  lolidpulse = constrain(lolidpulse, 280, 400);
  altlolidpulse = 680 - lolidpulse;
  
  pwm.setPWM(0, 0, lexpulse);
  pwm.setPWM(1, 0, leypulse);

  /* PERIODIC BLINKING */
  unsigned long currentBlinkMillis = millis(); 
  if (currentBlinkMillis - previousBlinkMillis >= blinkInterval) { 
    previousBlinkMillis = currentBlinkMillis;   
    blink();
  }
  
  State = AWAKE;
}

void wakeup() {   
  Serial.println("WAKEUP");  
  xval = 500;   
  yval = 500;
  
  lexpulse = map(xval, 0, 1023, 220, 440);
  rexpulse = lexpulse;
  leypulse = map(yval, 0, 1023, 250, 500);
  reypulse = map(yval, 0, 1023, 400, 280);

  trimval = 650;   
  trimval = map(trimval, 320, 580, -40, 40);
  uplidpulse = map(yval, 0, 1023, 400, 280);
  uplidpulse -= (trimval - 40);
  uplidpulse = constrain(uplidpulse, 280, 400);
  altuplidpulse = 680 - uplidpulse;

  lolidpulse = map(yval, 0, 1023, 410, 280);
  lolidpulse += (trimval / 2);
  lolidpulse = constrain(lolidpulse, 280, 400);
  altlolidpulse = 680 - lolidpulse;
  pwm.setPWM(0, 0, lexpulse);
  pwm.setPWM(1, 0, leypulse);

  pwm.setPWM(2, 0, uplidpulse);  // opens eyelids
  pwm.setPWM(3, 0, lolidpulse);
  pwm.setPWM(4, 0, altuplidpulse);
  pwm.setPWM(5, 0, altlolidpulse);

  delay(100);

  blink();
  delay(500);
  blink();
  delay(500);

  pwm.setPWM(0, 0, 450); // eyes glance right
  delay(800);
  pwm.setPWM(0, 0, 220); // eyes glance left
  delay(1000);
  pwm.setPWM(0, 0, 330); // eyes look forward
  delay(1000);

  blink();
  delay(200);
  blink();

  State = AWAKE;
}

void driftoff() {   
  pwm.setPWM(0, 0, 330);  // centers eyes on x-axis
  for (int i = 1; i <= 50; i++) { 
    const double a = i / 50.0;
    pwm.setPWM(2, 0, uplidpulse + (400 - uplidpulse) * (a)); 
    pwm.setPWM(3, 0, lolidpulse + (240 - lolidpulse) * (a));
    pwm.setPWM(4, 0, altuplidpulse + (240 - altuplidpulse) * (a));
    pwm.setPWM(5, 0, altlolidpulse + (400 - altlolidpulse) * (a));
    pwm.setPWM(1, 0, 400 + (i)); // eyes roll up
    delay(40);
  }
  pwm.setPWM(2, 0, 460); // closes eyelids completely
  pwm.setPWM(3, 0, 240);
  pwm.setPWM(4, 0, 240);
  pwm.setPWM(5, 0, 460);
  delay(1000);
  State = DORMANT;
}

void closeEyes() {
  Serial.println("closeEyes start");
  xval = 500;   
  yval = 500;
  
  lexpulse = map(xval, 0, 1023, 220, 440);
  rexpulse = lexpulse;
  leypulse = map(yval, 0, 1023, 250, 500);
  reypulse = map(yval, 0, 1023, 400, 280);

  trimval = 650;   
  trimval = map(trimval, 320, 580, -40, 40);
  uplidpulse = map(yval, 0, 1023, 400, 280);
  uplidpulse -= (trimval - 40);
  uplidpulse = constrain(uplidpulse, 280, 400);
  altuplidpulse = 680 - uplidpulse;

  lolidpulse = map(yval, 0, 1023, 410, 280);
  lolidpulse += (trimval / 2);
  lolidpulse = constrain(lolidpulse, 280, 400);
  altlolidpulse = 680 - lolidpulse;
  pwm.setPWM(0, 0, lexpulse);
  pwm.setPWM(1, 0, leypulse);
  pwm.setPWM(2, 0, 460);
  pwm.setPWM(3, 0, 240);
  pwm.setPWM(4, 0, 240);
  pwm.setPWM(5, 0, 460);  

  delay(100);
  Serial.println("closeEyes finish");
}

void setServoPulse(uint8_t n, double pulse) {
  double pulselength = 1000000;  
  pulselength /= 60;      
  pulselength /= 4096;  
  pulse *= 1000000;  
  pulse /= pulselength;
}