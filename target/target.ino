#include <IRremote.h>
#include <Chrono.h>
#include <ArduinoUniqueID.h>

const int interruptPin = 3; // orange wire
unsigned long hitCount = 0;
Chrono deadTimer; 
const int hitLedPin = 2;  // yellow wire
bool hit = false;

IRrecv irrecv(interruptPin); // Receive on pin 11
decode_results results;

void gotHit() {
  if (!deadTimer.isRunning()) {
    hit = true;
  }
}

void setup() {
  Serial.begin(9600); //debug info
  while(!Serial);

  Serial.print("Target firmware: ");
  Serial.println(__DATE__ " " __TIME__);
  UniqueIDdump(Serial);
  Serial.println("Target booting: pins");
  
  pinMode(hitLedPin,OUTPUT);
  digitalWrite(hitLedPin, LOW);
  
  deadTimer.stop();
//  pinMode(interruptPin, INPUT_PULLUP);
//  attachInterrupt(digitalPinToInterrupt(interruptPin), gotHit, RISING);

  Serial.println("Target booting: IR enable");
  irrecv.enableIRIn();
 
  Serial.println("Target Ready");
}

void dump(decode_results *results) {
  // Dumps out the decode_results structure.
  // Call this after IRrecv::decode()
  int count = results->rawlen;
  if (results->decode_type == SONY) {
    Serial.print("Decoded SONY: ");
  } else {
    Serial.print("Decoded ");
    Serial.print(results->decode_type);
    Serial.print(": ");
  }
  Serial.print(results->value, HEX);
  Serial.print(" (");
  Serial.print(results->bits, DEC);
  Serial.print(" bits, raw len:");
  Serial.print(count, DEC);
  Serial.println(")");
}

uint32_t receivedCode = -1;
uint32_t hitCode = -1;
int sameCnt = 0;

void loop() {
  if (deadTimer.isRunning() && deadTimer.hasPassed(1000) ) { // returns true if it passed 1000 ms since it was started
    deadTimer.stop();
    digitalWrite(hitLedPin, LOW);   
    Serial.print("Hit from: ");
    Serial.print(receivedCode, HEX);
    Serial.print(", confirmed times: ");
    Serial.println(sameCnt);
  }  

  if (irrecv.decode(&results)) {
    Serial.print("Received: ");
    Serial.print(results.value, HEX);
    Serial.print(", prev: ");
    Serial.println(receivedCode, HEX);
    if (results.value == receivedCode) {
      sameCnt++;
    } else {
      sameCnt = 0;
    }
    receivedCode = results.value;
    //dump(&results);
    irrecv.resume(); // Continue receiving
    if (sameCnt > 2 && receivedCode != hitCode) {
      hit = true;
    }
  }

  if (!deadTimer.isRunning() && hit) {
    hit = false;
    hitCode = receivedCode;
    hitCount++;
    deadTimer.restart();
    Serial.print("Hit #");
    Serial.println(hitCount);
    digitalWrite(hitLedPin,HIGH);
  }
}
