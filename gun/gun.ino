//#define LCD_ENABLED
#define DFPLAYER_ENABLED

#include <Arduino.h>
#include <Chrono.h>
#include <IRremote.h>
#include <ArduinoUniqueID.h>

#ifdef LCD_ENABLED
#include <LiquidCrystal_I2C.h>
#endif

#ifdef DFPLAYER_ENABLED
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#endif

#include <AceButton.h>
using namespace ace_button;


// pin 2 - used for IR remote timer ??? TODO: check this
#define txPinIR 3            //IR carrier output
#define triggerPin 5
#define reloadChamberPin 6
#ifdef DFPLAYER_ENABLED
#define sndPlayerTx 10
#define sndPlayerRx 11
#endif
#define reloadMagPin 12


#define Duty_Cycle 40 //in percent (10->50), usually 33 or 50
//TIP for true 50% use a value of 56, because of rounding errors
//TIP for true 40% use a value of 48, because of rounding errors
//TIP for true 33% use a value of 40, because of rounding errors
#define Carrier_Frequency 36000 //usually one of 38000, 40000, 36000, 56000, 33000, 30000
#define IR_FREQ_KHZ 36
#define PERIOD (1000000+Carrier_Frequency/2)/Carrier_Frequency
#define HIGHTIME PERIOD*Duty_Cycle/100
#define LOWTIME PERIOD - HIGHTIME
IRsend irsend;

#ifdef LCD_ENABLED
LiquidCrystal_I2C lcd(0x27, 16, 2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
#endif

#ifdef DFPLAYER_ENABLED
SoftwareSerial mySoftwareSerial(sndPlayerTx, sndPlayerRx); // TX, RX
DFRobotDFPlayerMini myDFPlayer;
#define SND_SHOT 3
#define SND_DRY_FIRE 5
#define SND_SYSTEM_STARTED 7
#define SND_VOLUME 10  //Set volume value. From 0 to 30
#endif

Chrono shootingTimer; 
bool shooting = false;
bool wasTrigger = false;
bool loaded = true;
bool loading = false;
bool loadingMag = false;
int ammo = 8;
uint16_t gunId = 0;
uint16_t shotId = 0;


void irSend() {
  unsigned long now = micros();
  uint32_t data = ((uint32_t)gunId << 16) | shotId;
  Serial.print("IR data: ");
  Serial.println(data, HEX);
  for (int i=0; i<5; i++) {
    irsend.sendSony(data, 32);
    delay(30);
  }
  unsigned long now2 = micros();
  Serial.print("duration ");
  Serial.println(now2-now);
}

#ifdef LCD_ENABLED
#define lcdPrint(__x, __y, __text) \
  lcd.setCursor(__x, __y); \
  lcd.print(__text); 
#else
#define lcdPrint(__x, __y, __text)
#endif

void sndPlay(int idx) {
#ifdef DFPLAYER_ENABLED
  myDFPlayer.play(idx);
#endif
}

void triggerEv(AceButton* /* button */, uint8_t eventType, uint8_t /* buttonState */) {
  if (shootingTimer.isRunning()) {
    return;
  }
  switch (eventType) {
    case AceButton::kEventPressed:
      if (loaded) {
        sndPlay(SND_SHOT);
        ammo--;
        shotId++;
        loaded = false;
        wasTrigger = true;
        Serial.print("shot, ammo ");
        Serial.print(ammo);
        Serial.println("/8");
        lcdPrint(13, 0, ammo);
        if (ammo > 0) {
          lcdPrint(0, 1, "empty chamber      ");
        } else {
          lcdPrint(0, 1, "empty magazine");
        }
        shootingTimer.restart();
      } else {
        sndPlay(SND_DRY_FIRE);
        Serial.println("empty chamber, reload");
      }
      break;
  }
}

void reloadChamberEv(AceButton* /* button */, uint8_t eventType, uint8_t /* buttonState */) {
  switch (eventType) {
    case AceButton::kEventPressed:
      sndPlay(2);
      if (loaded && ammo > 0) {
        ammo--;
        Serial.print("chamber reloaded, ammo ");
        Serial.print(ammo);
        Serial.println("/8");
        lcdPrint(13, 0, ammo);
      }
      if (ammo > 0) {
        loaded = true;
        lcdPrint(0, 1, "chamber loaded  ");
      } else {
        loaded = false;
        Serial.println("empty magazine");
        lcdPrint(0, 1, "empty magazine");
      }
      break;
  }
}

void reloadMagEv(AceButton* /* button */, uint8_t eventType, uint8_t /* buttonState */) {
  switch (eventType) {
    case AceButton::kEventPressed:
      if (ammo < 8) {
        sndPlay(9);
        ammo++;
        Serial.print("mag loaded ");
        Serial.print(ammo);
        Serial.println("/8");
        lcdPrint(13, 0, ammo);
        lcdPrint(0, 1, "magazine loaded");
      } else {
        Serial.println("mag full 8/8");
      }
      break;
  }
}

ButtonConfig triggerBtnCfg;
AceButton triggerBtn(&triggerBtnCfg);

ButtonConfig reloadChamberBtnCfg;
AceButton reloadChamberBtn(&reloadChamberBtnCfg);

ButtonConfig reloadMagBtnCfg;
AceButton reloadMagBtn(&reloadMagBtnCfg);


void lcdInit() {
#ifdef LCD_ENABLED
  Serial.println("Gun booting: LCD");
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("LaserTag amo 8/8");
  lcd.setCursor(0,1);
  lcd.print("Godfryd (c) 2019");
#endif
}

void soundInit() {
#ifdef DFPLAYER_ENABLED
  Serial.println("Gun booting: software serial & DF Player");
  mySoftwareSerial.begin(9600); // for DF Plyaer communication
  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println("Unable to begin DF Player:");
    Serial.println("1.Please recheck the connection!");
    Serial.println("2.Please insert the SD card!");
    while(true){
      delay(0); // Code to compatible with ESP8266 watch dog.
    }
  }
  myDFPlayer.volume(SND_VOLUME);
  myDFPlayer.play(SND_SYSTEM_STARTED);  //Play the first mp3
#endif

}

void setup() {
  Serial.begin(9600); //debug info
  while(!Serial);

  Serial.print("Gun firmware: ");
  Serial.println(__DATE__ " " __TIME__);
  UniqueIDdump(Serial);
  
  gunId = (unsigned long)(UniqueID[0] ^ UniqueID[2] ^ UniqueID[4] ^ UniqueID[6] ^ UniqueID[8]) << 8 |
          (unsigned long)(UniqueID[1] ^ UniqueID[3] ^ UniqueID[5] ^ UniqueID[7]);
  Serial.print("Gun ID: ");
  Serial.println(gunId, HEX);
    
  Serial.println("Gun booting: timers");
  shootingTimer.stop();
 
  Serial.println("Gun booting: pins & interrupts");
  //pinMode(txPinIR, OUTPUT);

  pinMode(triggerPin, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(triggerPin), shot, RISING);
  triggerBtn.init(triggerPin);
  triggerBtn.setEventHandler(triggerEv);

  pinMode(reloadChamberPin, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(reloadChamberPin), reload, RISING);
  reloadChamberBtn.init(reloadChamberPin);
  reloadChamberBtn.setEventHandler(reloadChamberEv);

  pinMode(reloadMagPin, INPUT_PULLUP);
  //attachPCINT(digitalPinToPCINT(reloadMagPin), reloadMag, RISING);
  reloadMagBtn.init(reloadMagPin);
  reloadMagBtn.setEventHandler(reloadMagEv);

  lcdInit();

  soundInit();

  Serial.println("Gun Ready");
  delay(1000);
}

void loop() {
  if (wasTrigger) {
    if (shootingTimer.hasPassed(500)) { // returns true if it passed 1000 ms since it was started
      shootingTimer.stop();
      wasTrigger = false;
      Serial.println("passed");
    } else {
      //mark(500);      
      irSend();
    }
  }

  triggerBtn.check();
  reloadChamberBtn.check();
  reloadMagBtn.check();
}

void mark(unsigned long mLen) {
  if (mLen==0) return;
  unsigned long now = micros();
  while ((micros() - now) < mLen) {
    digitalWrite(txPinIR, HIGH);
    delayMicroseconds(HIGHTIME-6);
    digitalWrite(txPinIR, LOW);
    delayMicroseconds(LOWTIME-7);
  }
}

void space(unsigned long sLen) {
  if (sLen==0) return;
  while (sLen>16383) {
    delayMicroseconds(16383);
    sLen -= 16383;
  }
  delayMicroseconds(sLen);
}
