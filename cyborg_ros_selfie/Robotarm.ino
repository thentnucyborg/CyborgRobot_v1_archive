#include<Wire.h>

// Deklarering av variabler
const int MPU_addr = 0x68;                        // I2C addresse til akselerometer MPU-6050 (BUS adresse)
int16_t accelReadM1X, accelRead1Y, accelRead1Z, Tmp, GyX, GyY, GyZ; // Akselerometer i skulderledd
int16_t accelReadM2X;                                   // Akselerometer i albueledd


// Setting av arduino pinner
const int EN1 = 13;   // Enable til motorer
const int EN2 = 6;
const int G1 = 3;     // Gates til transistorer på H-bridge (brukt til retningsstyring av motorene)
const int G2 = 2;     
const int G3 = 5;
const int G4 = 4;
char c = 'o';
int t1, t0;

void setup() {
 
  Serial.begin(9600);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(G1, OUTPUT);
  pinMode(G2, OUTPUT);
  pinMode(G3, OUTPUT);
  pinMode(G4, OUTPUT);


  // Initialisering av I/O
  digitalWrite(EN1, LOW);
  digitalWrite(EN2, LOW);
  digitalWrite(G1, LOW);
  digitalWrite(G2, LOW);
  digitalWrite(G3, LOW);
  digitalWrite(G4, LOW);

  // Initialisering av bus til accelReadelerometer
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  // Opprette kontakt med python program
  while (!(c == 't')){
    c = Serial.read();  // Leser inn kommando fra python program om å knytte kontakt
    delay(50);
  }
  
  delay(200);
  Serial.println('k');  // Sender klarsignal
}

void loop() {

  while (Serial.available() && Serial.read());  // Tøm buffer
  while (!Serial.available()) {
    accelRead();                        // Print accelReadelerometer data
    delay(50);
  }
  
  c = Serial.read();
  Serial.println(c);              // Kommando sendt fra python program om å ta selfie

  if (c == 'a') {
    M1_Clockwise();                // Kjører motor1 opp til selfie posisjon
    while (AcZ < 12000) {
      accelRead();
      delay(100);
      
    }
    
    M1_break();
    M2_CounterClockwise();                // Kjører motor2 opp til selfie posisjon
    
    while(val<360){
      accelRead();
      delay(100);
    }
    
    M2_break();
    Serial.print('b');          // Kommando sendes til python program om at selfie kan bli tatt
  }
  
  else if (c == 'h') {          // Bilde er blitt tatt, og robotarm kan sendes ned i hvileposisjon
    t0 = millis();
    M2_Clockwise();              // Kjører motor2 til initialposisjon
    while(val >310){
      accelRead();
      t1=millis();
      if (t1-t0>4000){
        break 
        }                       // Gå videre om motor2 ikke har nådd initialposisjon etter 4 s
    }
    delay(100);
  }
  
    M2_break();
    M1_CounterClockwise();              // Kjører motor1 til initialposisjon
    
    while (AcZ > -13400){
      accelRead();
      delay(100);
    }
    M2_break();
  }
}


void M1_Clockwise() {
  digitalWrite(G1, LOW);
  digitalWrite(G2, HIGH);
  digitalWrite(EN1, HIGH);
}

void M1_CounterClockwise() {
  digitalWrite(G1, HIGH);
  digitalWrite(G2, LOW);
  digitalWrite(EN1, HIGH);
}
void M2_Clockwise() {

  digitalWrite(G3, LOW);
  digitalWrite(G4, HIGH);
  digitalWrite(EN2, HIGH);
}
void M2_CounterClockwise() {

  digitalWrite(G3, HIGH);
  digitalWrite(G4, LOW);
  digitalWrite(EN2, HIGH);
}

void M2_break() {

  digitalWrite(G3, LOW);
  digitalWrite(G4, LOW);

}
void M1_break() {
  digitalWrite(G1, LOW);
  digitalWrite(G2, LOW);


}

void M1M2_OFF() {
  digitalWrite(G1, LOW);
  digitalWrite(G2, LOW);
  digitalWrite(G3, LOW);
  digitalWrite(G4, LOW);
  digitalWrite(EN1, LOW);
  digitalWrite(EN2, LOW);
}

void accelRead() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (accelReadEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (accelReadEL_XOUT_H) & 0x3C (accelReadEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (accelReadEL_YOUT_H) & 0x3E (accelReadEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (accelReadEL_ZOUT_H) & 0x40 (accelReadEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  val = analogRead(A3);


}

