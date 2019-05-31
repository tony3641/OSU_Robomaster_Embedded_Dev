#include <Adafruit_SSD1306.h>
#include <splash.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <eeprom.h>

#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 32 
#define OLED_RESET     4 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

float buffer=60.0f;

int val[3];
int sum;
float current;
float power;
int address = 0;
int limit_count;
int exceed_count;
String outputAmp;
String outputPow;
String output;
int serial_input;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(13,OUTPUT);
  for(int i=0;i<3;i++){
    val[i]=0;
  }
  sum=0;
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  //display.display();
  //delay(2000); 
  display.clearDisplay();
}

void loop() {
  // put your main code here, to run repeatedly:
	exceed_count = 0;
	if (Serial.available() > 0) {
		serial_input = Serial.read();
		if (serial_input > 0) {
			limit_count = 0;
			buffer = 60;
		}
  }
  write_eeprom();
  for (int i = 0; i < 20; i++){
	  sum = 0;
	  measure();
	  current = current_calc(sum);
	  power = power_calc(current);
	  if (abs(power) > 50) {
		  exceed_count++;
	  }
	  if (abs(power) > 80) {
		  limit_count++;
		  buffer -= abs(power) * 0.001f;
	  }
	  delay(1);
  }
  signal_to_stm32(exceed_count == 20);

  outputAmp=current;
  outputAmp+='A';
  outputPow=abs(power);
  outputPow+='W';
  output=outputAmp+" "+outputPow;
  
  print_to_serial();
  print_to_lcd();
}

void measure(void) {
	for (int i = 0; i < 3; i++) {//filter
		if (i == 2) {
			val[2] = analogRead(A0);
		}
		else {
			val[i] = val[i + 1];
		}
		sum += val[i];
	}
}

float current_calc(int val) {
	return (sum - 1536) / 17.067f*0.82f/3.0f;
}

float power_calc(float current) {
	return  current * 24.0f;
}

void write_eeprom() {
	EEPROM.write(address, (uint8_t)(abs(power)));
	address++;
	if (address == EEPROM.length()) {
		address = 0;
	}
}

void print_to_serial() {
	Serial.print(outputAmp);
	Serial.print("   ");
	Serial.print(outputPow);
	Serial.print("   ");
	Serial.print(buffer);
	Serial.println("   ");
}

void print_to_lcd() {
	display.clearDisplay();
	display.setTextSize(2);
	display.setTextColor(WHITE);
	display.setCursor(0, 0);
	display.println(outputAmp);
	display.setCursor(0, 16);
	display.println(outputPow);
	display.display();
	display.clearDisplay();
}

void signal_to_stm32(bool state) {
	if (state)	digitalWrite(13, HIGH);
	else {
		digitalWrite(13, LOW);
	}
}