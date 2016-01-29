#include <PID_v1.h>
#include <LiquidCrystal.h>
#include <SPI.h>

// LCD(D2-D7)
#define LCDrsPin 2
#define LCDenablePin 3
#define LCDd4Pin 4
#define LCDd5Pin 5
#define LCDd6Pin 6
#define LCDd7Pin 7
// button(D8)
#define StartButton 8
// Beep(D9)
#define TonePin 9
// Tempratier(D10,D12-D13)
#define TemperatureSlavePin 10
#define TemperatureMisoPin 12
#define TemperatureSckPin 13
// PowerControl(A0,A1)
#define Heat1Pin 14
#define Cooler1Pin 15
#define delayWait 100
#define oneSec (1000 / delayWait)
#define VERSION "P4"

enum ReflowState { WARMUP, PREHEAT, PEAK, COOLDOWN, FINISH };
typedef struct tcdatom {
  ReflowState state_;
  float temp_;
  int wait_;
} tcd_t;

tcd_t temperature_control_data[] = {
  { WARMUP, 80, 60 },
  { PREHEAT, 120, 60 },
  { PEAK, 230, 8 },
  { COOLDOWN, 25, 150 },
  { FINISH,   0,  0 }
};

enum MainSeq { M_INIT, M_START, M_TARGET, M_KEEP, M_LOOPEND, M_FINISH }; 

double Kp = 107.9;
double Ki = 0.65;
double Kd = 18.5;
int WindowSize = 2000;
double Setpoint, Input, Output;
LiquidCrystal lcd(LCDrsPin,LCDenablePin,LCDd4Pin,LCDd5Pin,LCDd6Pin,LCDd7Pin);
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

tcd_t stage;         // now parameter.
MainSeq state;       // main program mode
byte tableCounter;   // data table counter
int blinkTimer;      // blink timer
boolean blinkFlag;   // blink ON/OFF flag
int tickCount;       // ticker
float tempstart = 0; // temperature
int tempptr = 0;     // pointer of temperature
float temprate = 0;  // rate of temperature
unsigned long windowStartTime;
float temperature = 0;
byte heaton = 0;
byte coolon = 0;

void setup() {
  // degug Initialize(SerialMonitor)
  Serial.begin(9600);
  // LCD initialize
  lcd.begin(20, 4);
  // button initialize
  pinMode(StartButton, INPUT_PULLUP);
  // PowerControl initialize
  pinMode(Heat1Pin, OUTPUT);
  pinMode(Cooler1Pin, OUTPUT);
  // Temprature initialize
  pinMode(TemperatureSlavePin, OUTPUT);
  digitalWrite(TemperatureSlavePin, HIGH);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  SPI.setDataMode(SPI_MODE0);
  // memory initialize
  state = M_INIT;
  tickCount = 0;
  tempptr = 0;
  heaton = 0;
  coolon = 0;
  myPID.SetOutputLimits(0, WindowSize);
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  tempratureRead();
  
  
  switch (state) {
    case M_INIT: // initialize
      lcd.clear();
      tableCounter = 0;
      state = M_START;
      break;
    case M_START: // start switch wait
      if (digitalRead(StartButton) == LOW) {
        tone(TonePin,600,800);  // StartSound
        lcd.clear();
        setTempratureData();
        state = M_TARGET;
        Serial.print("M_START -> M_TARGET");
        Serial.println();  
      }
      break;
    case M_TARGET: // target Temperature
      if (temperature > stage.temp_) {
        state  = M_KEEP;
        Serial.print("M_TARGET -> M_KEEP");
        Serial.println();  
      }
      break;
    case M_KEEP: // keep time
      if (--stage.wait_ <= 0) {
        state = M_LOOPEND;
        Serial.print("M_KEEP -> M_LOOPEND");
        Serial.println();  
      }
      break;
    case M_LOOPEND: // Loop or Finish?
      if (stage.state_ != FINISH) tableCounter++;
      setTempratureData();
      if (stage.state_ != FINISH) {
        Serial.print("M_LOOPEND -> M_TARGET");
        Serial.println();  
        state = M_TARGET;
      } else {
        tone(TonePin,600,1500);  // FinishSound
        Serial.print("M_LOOPEND -> M_FINISH");
        Serial.println();  
        state = M_FINISH;
      }
      break;
    case M_FINISH: // finish switch wait
      if (digitalRead(StartButton) == LOW) {
        state = M_INIT;
      }
      break;
  }
  heatControl();
  lcdDisplay();
  delay(delayWait);
}

void setTempratureData() {
  stage = temperature_control_data[tableCounter];
  stage.wait_  = stage.wait_ * oneSec;
  Setpoint = stage.temp_;
  windowStartTime = millis();
}

void tempratureRead() {
  unsigned int thermocouple;
  unsigned int internal;
  float disp;
  // read tem
  digitalWrite(TemperatureSlavePin, LOW);
  thermocouple = (unsigned int)SPI.transfer(0x00) << 8;
  thermocouple |= (unsigned int)SPI.transfer(0x00);
  internal = (unsigned int)SPI.transfer(0x00) << 8;
  internal |= (unsigned int)SPI.transfer(0x00);
  digitalWrite(TemperatureSlavePin, HIGH);
  if ((thermocouple & 0x0001) != 0) {
    Serial.print("ERROR: ");
    if ((internal & 0x0004) !=0) {
      Serial.print("Short to Vcc, ");
    }
    if ((internal & 0x0002) !=0) {
      Serial.print("Short to GND, ");
    }
    if ((internal & 0x0001) !=0) {
      Serial.print("Open Circuit, ");
    }    
    Serial.println();
  } else {
    if ((thermocouple & 0x8000) == 0) {
      temperature = (thermocouple >> 2) * 0.25;
    } else {
      temperature = (0x3fff - (thermocouple >> 2) + 1)  * -0.25;
    }
    //温度勾配を求める
    if (tempptr == 0) {
      tempstart = temperature;
    }
    tempptr++;
    if (tempptr >= oneSec) {
      temprate = temperature - tempstart;
      tempptr = 0;
    }

    tickCount++;
    //デバッグ出力
    if ((tickCount % oneSec) == 0) {
      Serial.print(tickCount / oneSec);
      Serial.print(",");
      Serial.print(Input);
      Serial.print(",");
      Serial.print(Setpoint);
      Serial.print(",");
      Serial.print(Output);
      Serial.print(",");
      Serial.print(heaton);
      Serial.println();     
    }
  }
}

void heatControl() {
  Input = temperature;
  myPID.Compute();
  if (millis() - windowStartTime > WindowSize){
    windowStartTime += WindowSize;
  }
  if (Output > millis() - windowStartTime) {
    heaton = 1;
  } else {
    heaton = 0;
  }
  if (stage.state_ == COOLDOWN) {
    coolon = 1;
  } else {
    coolon = 0;
  }
  if (heaton == 0) {
    digitalWrite(Heat1Pin, LOW);
  } else {
    digitalWrite(Heat1Pin, HIGH);
  }  
  if (coolon == 0) {
     digitalWrite(Cooler1Pin, LOW);
  } else {
     digitalWrite(Cooler1Pin, HIGH);
  }
}

void lcdDisplay() {
  lcd.setCursor(3, 0);
  lcd.print("STATUS:");
  switch (state) {
    case 0: // initialize
    case 1: // start switch wait
      lcd.print("-------");
      lcd.setCursor(1, 1);
      if (blinkFlag == true) {
        lcd.print("press START button");
      } else {
        lcd.print("                  ");
      }
      lcd.setCursor(3, 3);
      lcd.print("S.SIENCE + ");
      lcd.print(VERSION);
      break;
    case 2: // target Temperature
    case 3: // keep time
    case 4: // Loop or Finish?
    case 5: // finish switch wait
      if (state != 5) {
        if (blinkFlag == true) {
          switch ((int)(tickCount / oneSec) % 3) {
            case 1:
            lcd.print(temprate);
              break;
            default:
              lcd.print("RUNNING");
          }
        } else {
          lcd.print("       ");
        }
      } else {
        lcd.print("FINISH!");
      }
      lcd.setCursor(0, 1);
      if (heaton == 0) {
        lcd.print("HEAT:OFF  ");
      } else {
        lcd.print("HEAT:ON   ");
      }
      if (coolon == 0) {
        lcd.print("COOL:OFF  ");
      } else {
        lcd.print("COOL:ON   ");
      }
      lcd.setCursor(5, 3);
      lcd.print("WAIT:");
      if (state == 3) {
        lcd.print(stage.wait_ / oneSec);
        lcd.print(".");
        lcd.print(stage.wait_ % oneSec * 10 / oneSec);
        lcd.print("sec");
      } else {
        lcd.print("---.-  ");
      }
      lcd.print("  ");
      break;
  }
  lcd.setCursor(2, 2);
  if (temperature < 100.0) lcd.print(" ");
  if (temperature < 10.0) lcd.print(" ");
  lcd.print(temperature);
  lcd.print(" / ");
  lcd.print(Setpoint);
  lcd.print("  ");
  // blink control
  if (++blinkTimer >= oneSec) {
    blinkTimer = 0;
    if (blinkFlag == false) {
      blinkFlag = true;
    } else {
      blinkFlag = false;
    }
  }
}

