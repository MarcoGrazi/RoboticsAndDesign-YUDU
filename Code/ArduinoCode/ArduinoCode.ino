#include <SharpIR.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <LedControl.h>
#include <Adafruit_NeoPixel.h>


class Servo {
  private:
  int cp;
  int ip;
  
  const int max_pulse = 600;
  const int min_pulse = 150;
  int max_p;
  int min_p;
  int dir;
  int ch;
  long int prev_move = 0;
  Adafruit_PWMServoDriver pwm;

  public:
  Servo(int initial_position, int channel, int max_pos, int min_pos,
   int direction){
    ip = initial_position;
    cp = ip;
    ch = channel; 
    max_p = max_pos;
    min_p = min_pos;
    dir = direction;
  }

  void Init(Adafruit_PWMServoDriver p){
    int pulse;
    pwm = p;
    if(dir==1){
      pulse = map(ip, 0, 180, min_pulse, max_pulse);
    }
    else{
      pulse = map(ip, 0, 180, max_pulse, min_pulse);
    }
    pwm.setPWM(ch,0,pulse);
    //Serial.print("Servo Initialized ");
    //Serial.println(ch);
  }

  int Move(int target, float speed) {
    int speed_delay = int((1/speed)*1000);
    long int time_passed = millis() - prev_move;
    bool timing = time_passed>=speed_delay;
    if(target==300 || target==cp){
      target=cp;
      return 1;
    }
    else if(target!=cp && target<=max_p && target>=min_p && timing){
      prev_move = millis();
      int delta = (target-cp)/abs(target-cp); 
      cp=cp+delta;
      int pulse;
      if(dir==1){
        pulse = map(cp, 0, 180, min_pulse, max_pulse);
      }
      else{
        pulse = map(cp, 0, 180, max_pulse, min_pulse);
      }
      pwm.setPWM(ch,0,pulse);
      return 0;
    }
    else{
      return 0; 
    }
  }

//Function that return the current position of the Servo
  int getPos(){ 
    return cp;
  }
};

//SharpIR sensor( SharpIR::GP2Y0A21YK0F, A0 );
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//Definition of each Servo, we had to set the maximum and minimum position for safety purpouse, 
//and the pin numebr of the connection with the Shield
//LEGEND: "l" = left, "r" = right, "s" = shoulder, "h" = head, "b" = belly, "x" = X-axis, "y" = Y-axis and "z" = Z-axis 
Servo lsx(0, 0, 60, 0, -1); 
Servo lsy(0, 1, 180, 0, -1);
Servo rsx(0, 15, 60, 0, 1);
Servo rsy(0, 14, 180, 0, 1);
Servo hx(90, 7, 105, 75, -1);
Servo hy(108, 6, 115, 95, 1);
Servo hz(90, 9, 180, 0, -1);
Servo b(180, 4, 180, 160, 1);

//Variable for the angle and speed of each Servo, used for communication
//LEGEND: "a" = angle and "v" = speed
int rsxa, rsxv, rsya, rsyv, lsxa, lsxv, lsya, lsyv, hxa, hxv, hya, hyv, hza, hzv, ba, bv;

//Definition of the pin connection for the LED Matrixes
const int DIN_PIN = 10;
const int CS_PIN = 11;
const int CLK_PIN = 12;
const int numDevice = 2;
LedControl lc = LedControl(DIN_PIN, CLK_PIN, CS_PIN, numDevice);

//Matrix that control the images that we send to the matrix using uint64_t encoding
//The letters after each element represent the associated message in serial communication
const uint64_t IMAGES[] = {
 //IDLE STATE
  0x003c78787e7e3c00, //a
  0x003c72727e7e3c00,//b
  0x003c66667e7e3c00,//c
  0x003c4e4e7e7e3c00,//d
  0x003c1e1e7e7e3c00,//e
 //HAPPY TO SEE
  0x0000f8ff7e3c0000,//left, f
  0x00001fff7e3c0000,//right, 
 //CALM
  0x003c7effc3810000,//g
  //SAD
  0x003c7e7c78702000,//left, h
  0x003c7e3e1e0e0400,//right, 
 //DISAPPOINTED
  0x00003c7effff0000,//i
  //SATISFIED
  0x00000000423c0000,//l
  0x0000ffff7e3c0000,//m
  //ANGRY
  0x007cfe7f07010000, //left, n
  0x003e7ffee0800000, //right, 
  // BYE BYE
  0x000000667e3c0000,//o
  //HIGH
  0x80bfa1ada5bd81ff,//p
  0xff80bea2aaba82fe,//q
  0xff81bda5b585fd01,//r
  0x7f415d55457d01ff//s
};
const int IMAGES_LEN = sizeof(IMAGES)/8;

//Definition of PIN comunication for the NEOPixels stripe and lenght of the stripe
const int PIN = 7;
#define NUMPIXELS 15
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
int level=0;
int blinking=0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin( 9600 );
  Serial.setTimeout(5000);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(60);
  lsx.Init(pwm);
  lsy.Init(pwm);
  rsx.Init(pwm);
  rsy.Init(pwm);
  hx.Init(pwm);
  hy.Init(pwm);
  hz.Init(pwm);
  b.Init(pwm);

  for(int j =0; j< numDevice; j++) {
    lc.shutdown(j,false);
    lc.setIntensity(j,10); // Brightness od LED Matrixes
    lc.clearDisplay(j);
  }
  pixels.begin();
  pixels.clear();
}

//Function that checks the incoming message having "L" as prefix, from the Raspberry Pi to decide what to control between the LED matrices and the LED strip,
// and then which expression 
int CheckFace(String eyes, int* level, int*blinking){
  if (eyes[3] == 'f') {
    if (eyes[4] == 'a') {
      fIdle1();
    } else if (eyes[4] == 'b') {
      fIdle2();
    } else if (eyes[4] == 'c') {
      fIdle3();
    } else if (eyes[4] == 'd') {
      fIdle4();
    }
    else if(eyes[4] == 'e'){
      fIdle5();
    }
    else if(eyes[4] == 'f'){
      fHappy();
    }
    else if(eyes[4] == 'g'){
      fCalm();
    }
    else if(eyes[4] == 'h'){
      fSad();
    }
    else if(eyes[4] == 'i'){
      fDisappointed();
    }
    else if(eyes[4] == 'l'){
      fSatisfied1();
    }
    else if(eyes[4] == 'm'){
      fSatisfied2();
    }
    else if(eyes[4] == 'n'){
      fAngry();
    }
    else if(eyes[4] == 'o'){
      fBye();
    }
    else if(eyes[4] == 'p'){
      fHigh0();
    }
    else if(eyes[4] == 'q'){
      fHigh1();
    }
    else if(eyes[4] == 'r'){
      fHigh2();
    }
    else if(eyes[4] == 's'){
      fHigh3();
    }
  }
  else if (eyes[3] == 'l')
  {
    if(eyes[4]=='1'){
      *level=1;
      lotusL1(0);
      if(eyes.length()==6 && eyes[5]=='b'){
        *blinking=1;
      }
      else if(eyes.length()==6 && eyes[5]=='s'){
        lotusL1(4);
        *blinking=0;
        *level=0;
      }
      else{
        *blinking=0;
      }
    }
    else if(eyes[4]=='2'){
      *level=2;
      lotusL2(0);
      if(eyes.length()==6 && eyes[5]=='b'){
        *blinking=1;
      }
      else if(eyes.length()==6 && eyes[5]=='s'){
        lotusL2(4);
        *blinking=0;
        *level=0;
      }
      else{
        *blinking=0;
      }
    }
    else if(eyes[4]=='3'){
      *level=3;
      lotusL3(0);
      if(eyes.length()==6 && eyes[5]=='b'){
        *blinking=1;
      }
      else if(eyes.length()==6 && eyes[5]=='s'){
        lotusL3(4);
        *blinking=0;
        *level=0;
      }
      else{
        *blinking=0;
      }
    }
    else if(eyes[4]=='4'){
      lotusL4();
    }
  }
  Serial.println("Done");
}

//Function that allows the movement of the servos using a while loop that continues to move the motors until the target position,
//and controls the blinking of the LEDs in the lotus flower during the movement
void freeMov () {
  int lotusstate=1;
  long int prevblink = millis();
  while(lsx.Move(lsxa, lsxv)==0 | lsy.Move(lsya, lsyv)==0 | rsx.Move(rsxa, rsxv)==0 |
    rsy.Move(rsya, rsyv)==0 | hx.Move(hxa, hxv)==0 | hy.Move(hya, hyv)==0 | hz.Move(hza, hzv)==0 |
     b.Move(ba, bv)==0)
    {
      if(blinking==1 && millis()-prevblink>150){
        prevblink = millis();
        if(level==1){
          lotusL1(lotusstate);
          lotusstate++;
          if(lotusstate==4){
            lotusstate=1;
          }
        }
        else if(level==2){
          lotusL2(lotusstate);
          lotusstate++;
          if(lotusstate==4){
            lotusstate=1;
          }
        }
        else if(level==3){
          lotusL3(lotusstate);
          lotusstate++;
          if(lotusstate==4){
            lotusstate=1;
          }
        }
      }
    }
}

//Function that allows displaying the desired image on the matrix and select the matrix that we want

void displayImage(uint64_t image, int matIndex) {
for (int i = 0; i < 8; i++) {
    byte row = (image >> i * 8) & 0xFF;
    for (int j = 0; j < 8; j++) {
      lc.setLed(matIndex, i, j, bitRead(row, j));
    }
  }
}

//Series of functions that defines the eyes expressions
void fIdle1 () {
  displayImage(IMAGES[0], 0);
  displayImage(IMAGES[0], 1);
 }
 
void fIdle2() {
  displayImage(IMAGES[1], 0);
  displayImage(IMAGES[1], 1);
}

void fIdle3() {
  displayImage(IMAGES[2], 0);
  displayImage(IMAGES[2], 1);
}

void fIdle4() {
  displayImage(IMAGES[3], 0);
  displayImage(IMAGES[3], 1);
}
void fIdle5() {
  displayImage(IMAGES[4], 0);
  displayImage(IMAGES[4], 1);
}

void fHappy() {
  displayImage(IMAGES[5], 1);
  displayImage(IMAGES[6], 0);
}
void fCalm() {
  displayImage(IMAGES[7], 1);
  displayImage(IMAGES[7], 0);
}

void fSad() {
  displayImage(IMAGES[8], 1);
  displayImage(IMAGES[9], 0);
}

void fDisappointed() {
  displayImage(IMAGES[10], 0);
  displayImage(IMAGES[10], 1);
}

void fSatisfied1() {
  displayImage(IMAGES[11], 0);
  displayImage(IMAGES[11], 1);
}

void fSatisfied2() {
  displayImage(IMAGES[12], 0);
  displayImage(IMAGES[12], 1);
}

void fAngry() {
  displayImage(IMAGES[13], 1);
  displayImage(IMAGES[14], 0);
}

void fBye() {
  displayImage(IMAGES[15], 0);
  displayImage(IMAGES[15], 1);
}

void fHigh0() {
  displayImage(IMAGES[16], 0);
  displayImage(IMAGES[16], 1);
}

void fHigh1() {
  displayImage(IMAGES[17], 0);
  displayImage(IMAGES[17], 1);
}

void fHigh2() {
  displayImage(IMAGES[18], 0);
  displayImage(IMAGES[18], 1);
}

void fHigh3() {
  displayImage(IMAGES[19], 0);
  displayImage(IMAGES[19], 1);
}

//Series of functions that defines the color and animation of the lotus flower during the whole intercation
//For each LED we have to use the "setPixelColor" function to select the LED and the corrisponding color in the RGB coding 
void lotusL4() {
   pixels.setPixelColor(0, pixels.Color(1, 170, 50));
   pixels.setPixelColor(1, pixels.Color(1, 170, 50));
   pixels.setPixelColor(2, pixels.Color(1, 170, 50));
   pixels.setPixelColor(3, pixels.Color(1, 170, 50));
   pixels.setPixelColor(4, pixels.Color(1, 170, 50));
   pixels.setPixelColor(5, pixels.Color(1, 170, 50));
   pixels.setPixelColor(6, pixels.Color(1, 170, 50));
   pixels.setPixelColor(7, pixels.Color(1, 170, 50));
   pixels.setPixelColor(8, pixels.Color(1, 170, 50));
   pixels.setPixelColor(9, pixels.Color(1, 170, 50));
   pixels.setPixelColor(10, pixels.Color(1, 170, 50));
   pixels.setPixelColor(11, pixels.Color(1, 170, 50));
   pixels.setPixelColor(12, pixels.Color(1, 170, 50));
   pixels.setPixelColor(13, pixels.Color(1, 170, 50));
   pixels.setPixelColor(14, pixels.Color(1, 170, 50));
   pixels.show();
   pixels.clear();
}

//we uese the variable state to define the blinking animation using the messages
void lotusL3(int state) {
  //LOTUS1
   pixels.setPixelColor(2, pixels.Color(148, 30, 79));
   pixels.setPixelColor(12, pixels.Color(148, 30, 79));
   pixels.setPixelColor(1, pixels.Color(213, 48, 116));
   pixels.setPixelColor(13, pixels.Color(213, 48, 116));
   pixels.setPixelColor(0, pixels.Color(226, 111, 159));
   pixels.setPixelColor(14, pixels.Color(226, 111, 159));
  //LOTUS2
   pixels.setPixelColor(3, pixels.Color(161, 33, 159));
   pixels.setPixelColor(9, pixels.Color(161, 33, 159));
   pixels.setPixelColor(4, pixels.Color(213, 48, 210));
   pixels.setPixelColor(10, pixels.Color(213, 48, 210));
   pixels.setPixelColor(5, pixels.Color(226, 111, 224));
   pixels.setPixelColor(11, pixels.Color(226, 111, 224));
   pixels.show();
  if (state == 0) {
  //LOTUS3
   pixels.setPixelColor(6, pixels.Color(99, 30, 148));
   pixels.setPixelColor(7, pixels.Color(146, 52, 213));
   pixels.setPixelColor(8, pixels.Color(182, 120, 227));
   pixels.show();
   pixels.clear();
  }
  else if (state == 1) {
  pixels.setPixelColor(6, pixels.Color(99, 30, 148));
  pixels.show();
  pixels.clear();
  } else if (state == 2) {
    pixels.setPixelColor(7, pixels.Color(146, 52, 213));
    pixels.show();
    pixels.clear();
  } else if (state == 3) {
    pixels.setPixelColor(8, pixels.Color(182, 120, 227));
    pixels.show();
    pixels.clear();
  }
  else if (state == 4){
    pixels.setPixelColor(6, pixels.Color(0, 0, 0));
    pixels.setPixelColor(7, pixels.Color(0, 0, 0));
    pixels.setPixelColor(8, pixels.Color(0, 0, 0));
    pixels.show();
    pixels.clear();
  }
 
}

void lotusL2(int state) {
  // LOTUS 1
  pixels.setPixelColor(2, pixels.Color(148, 30, 79));
  pixels.setPixelColor(12, pixels.Color(148, 30, 79));
  pixels.setPixelColor(1, pixels.Color(213, 48, 116));
  pixels.setPixelColor(13, pixels.Color(213, 48, 116));
  pixels.setPixelColor(0, pixels.Color(226, 111, 159));
  pixels.setPixelColor(14, pixels.Color(226, 111, 159));
  pixels.show();
  if (state == 0) {
    //LOTUS2
    pixels.setPixelColor(3, pixels.Color(161, 33, 159));
    pixels.setPixelColor(9, pixels.Color(161, 33, 159));
    pixels.setPixelColor(4, pixels.Color(213, 48, 210));
    pixels.setPixelColor(10, pixels.Color(213, 48, 210));
    pixels.setPixelColor(5, pixels.Color(226, 111, 224));
    pixels.setPixelColor(11, pixels.Color(226, 111, 224));
    pixels.show();
    pixels.clear();
   } else if (state == 1 ) {
    pixels.setPixelColor(3, pixels.Color(161, 33, 159));
    pixels.setPixelColor(9, pixels.Color(161, 33, 159));
    pixels.show();
    pixels.clear();
   } else if (state == 2) {
    pixels.setPixelColor(4, pixels.Color(213, 48, 210));
    pixels.setPixelColor(10, pixels.Color(213, 48, 210));
    pixels.show();
    pixels.clear();
   } else if (state == 3) {
    pixels.setPixelColor(5, pixels.Color(226, 111, 224));
    pixels.setPixelColor(11, pixels.Color(226, 111, 224));
    pixels.show();
    pixels.clear();
   }
  else if (state == 4){
    pixels.setPixelColor(3, pixels.Color(0, 0, 0));
    pixels.setPixelColor(9, pixels.Color(0, 0, 0));
    pixels.setPixelColor(4, pixels.Color(0, 0, 0));
    pixels.setPixelColor(10, pixels.Color(0, 0, 0));
    pixels.setPixelColor(5, pixels.Color(0, 0, 0));
    pixels.setPixelColor(11, pixels.Color(0, 0, 0));
    pixels.show();
    pixels.clear();
  }
}

void lotusL1(int state) {
  if (state == 0) {
    pixels.setPixelColor(2, pixels.Color(148, 30, 79));
    pixels.setPixelColor(12, pixels.Color(148, 30, 79));
    pixels.setPixelColor(1, pixels.Color(213, 48, 116));
    pixels.setPixelColor(13, pixels.Color(213, 48, 116));
    pixels.setPixelColor(0, pixels.Color(226, 111, 159));
    pixels.setPixelColor(14, pixels.Color(226, 111, 159));
    pixels.show();
    pixels.clear();
  } else if (state == 1) {
    pixels.setPixelColor(2, pixels.Color(148, 30, 79));
    pixels.setPixelColor(12, pixels.Color(148, 30, 79));
    pixels.show();
    pixels.clear();
  } else if (state == 2) {
    pixels.setPixelColor(1, pixels.Color(213, 48, 116));
    pixels.setPixelColor(13, pixels.Color(213, 48, 116));
    pixels.show();
    pixels.clear();
  } else if (state == 3)  {
    pixels.setPixelColor(0, pixels.Color(226, 111, 159));
    pixels.setPixelColor(14, pixels.Color(226, 111, 159));
    pixels.show();
    pixels.clear();
  }
  else if (state == 4){
    pixels.setPixelColor(2, pixels.Color(0, 0, 0));
    pixels.setPixelColor(12, pixels.Color(0, 0, 0));
    pixels.setPixelColor(1, pixels.Color(0, 0, 0));
    pixels.setPixelColor(13, pixels.Color(0, 0, 0));
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.setPixelColor(14, pixels.Color(0, 0, 0));
    pixels.show();
    pixels.clear();
  }
}

//In the main loop we have to set the comunication between the arduino and the Raspberry Pi
void loop() {
  if(Serial.available()){
    String msg = Serial.readStringUntil('\n');
    if(msg!=NULL){
      if(msg[0]=='M'){
        sscanf(msg.c_str(), "M: %d:%d, %d:%d, %d:%d, %d:%d, %d:%d, %d:%d, %d:%d, %d:%d", 
        &lsxa, &lsxv, &lsya, &lsyv, &rsxa, &rsxv, &rsya, &rsyv, &hxa, &hxv, &hya, &hyv, &hza, &hzv,  &ba, &bv);
        freeMov();
        Serial.println("Done");
      }
      if(msg[0]=='L'){
        CheckFace(msg, &level, &blinking);
      }
      Serial.flush();
    }
  }
}



