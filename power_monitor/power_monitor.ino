#include <Adafruit_SSD1306.h>
#include <splash.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <eeprom.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


int val[3];
int sum;
float current;
float power;
int address = 0;
int exceed_time = 0;
String outputAmp;
String outputPow;
String output;

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
    for(;;); // Don't proceed, loop forever
  }
  //display.display();
  //delay(2000); 
  display.clearDisplay();
}

void loop() {
  // put your main code here, to run repeatedly:
  sum=0;
  
  measure();
  current = current_calc(sum);
  power = power_calc(current);
  
  write_eeprom();

  outputAmp=current;
  outputAmp+='A';
  outputPow=abs(power);
  outputPow+='W';
  output=outputAmp+" "+outputPow;
  
  print_to_serial();
  print_to_lcd();
  signal_to_stm32(power);
  delay(50);
}

void measure(void) {
	for (int i = 0; i < 3; i++) {//滑块滤波
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
	Serial.println(exceed_time);
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

void signal_to_stm32(float power) {
	if (abs(power) > 60) {
		digitalWrite(13, HIGH);
		exceed_time++;
	}
	else {
		digitalWrite(13, LOW);
	}
}