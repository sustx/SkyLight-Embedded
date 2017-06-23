// SkyLightEmbedded | William Tyback 
// ----------------
// Enables PCB control over Bluetooth
//
// ver 0: Every task has been outsourced to its specific function
//
//  setFilm(): Used to set film ON/OFF (Working)
//  setTemperature(coolT, warmT): Set the temparture of LED's (Working)
//  setBrightness(): Controls the intensity of the LED's only +/- (Could be better)
//  getTemperature(): Returns temperature in °F (Working)
//  getLightIntensity(): Returns LightInstensity as a voltage (Working)
//
//  Working on implementing time controlled functions to only get data from
//  sensors when needed

#define DB 1

// Analog Inputs
int tempSensor = 0;
int lightSensor = 1;

// LED PWM Outputs
int coolLed = 10;
int warmLed = 9;
//int redLed = 9;
//int greenLed = 10;
//int blueLed = 3;

// Smart Film
int smartFilm = 8;

// Bluetooth Serial Input |   film  |  coolT   |    warmT |     +/-   |
byte rx_byte[4]  =        {B00000000, B00000000, B00000000, B00000000};

// Functions
void setFilm(byte state);
void setTemperature(byte coolTemp, byte warmTemp);
void setBrightness(byte brightness);
float getTemperature();
float getLightIntensity();

// Initial PCB setup:
// Set I/O Pins && Serial communication rate
void setup() {
  Serial.begin(9600);  
  pinMode(coolLed, OUTPUT);
  pinMode(warmLed, OUTPUT);
  pinMode(smartFilm, OUTPUT);
  pinMode(tempSensor, INPUT);
  pinMode(lightSensor, INPUT);
  
  //pinMode(redLed, OUTPUT);
  //pinMode(greenLed, OUTPUT);
  //pinMode(blueLed, OUTPUT);  
  //pinMode(LED_BUILTIN, OUTPUT);
}

// Main Operation (loop() runs forever)
void loop() {
  // Acquire Serial Data (4 bytes)
  if (Serial.available()) {
    Serial.readBytesUntil('\n', rx_byte, 4);
  }

  if (DB) {
    Serial.print("rx_byte[0]: "); Serial.println(rx_byte[0], BIN);
    Serial.print("rx_byte[1]: "); Serial.println(rx_byte[1], BIN);
    Serial.print("rx_byte[2]: "); Serial.println(rx_byte[2], BIN);
    Serial.print("rx_byte[3]: "); Serial.println(rx_byte[3], BIN);
  }
  
  setFilm(rx_byte[0] & B00000001);
  setTemperature(rx_byte[1],rx_byte[2]);
  setBrightness(rx_byte[3]);
  getTemperature();
  getLightIntensity();

  delay(3000);
}

// Smart Film control
void setFilm(byte state){
  digitalWrite(smartFilm, state);
}

// LED Temperature control
void setTemperature(byte coolTemp, byte warmTemp){
  analogWrite(coolLed, coolTemp); analogWrite(warmLed, warmTemp);
}

// LED Brightness control
// + : Increases brightness by 64
// - : Decreases brightness by 64
void setBrightness(byte brightness){
  if (brightness == '+') {
    rx_byte[1] = (rx_byte[1] + '@') % 255;
    rx_byte[2] = (rx_byte[2] + '@') % 255;
    Serial.println(rx_byte[1]); 
  }

  if (brightness == '-') {
    rx_byte[1] = (rx_byte[1] - '@') % 255;
    rx_byte[2] = (rx_byte[2] - '@') % 255; 
    Serial.println(rx_byte[1]);
  }

  setTemperature(rx_byte[1], rx_byte[2]);
  rx_byte[3] = B00000000;
}

float getTemperature(){
  int reading = analogRead(tempSensor);

  // Normalize reading (Vs)
  float voltage = reading * 5;
  voltage /= 1024.0;
  
  // 10 mv per degree wit 500 mV offset
  float temperatureC = (voltage - 0.5) * 100 ;
  float temperatureF = (temperatureC * 9.0 / 5.0) + 32.0;

  if (DB){
    Serial.print("Temp reading: "); Serial.print(temperatureF); Serial.println(" °F");
  }

  return temperatureF;
}

float getLightIntensity(){
  int lightReading = analogRead(lightSensor);

  // Normalize 
  float light = lightReading * 5;
  light /= 1024;

  if (DB){
    Serial.print("Light reading: "); Serial.print(light); Serial.println("v");
  }

  return light;
}



