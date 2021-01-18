#define LIGHT_FL A0
#define LIGHT_FR 5
#define LIGHT_RL 13
#define LIGHT_RR A5

static boolean blinking = false;
static char dir = 0;


setupLights() {
  pinMode(LIGHT_FL,OUTPUT);
  pinMode(LIGHT_FR,OUTPUT);
}


void lightsOnFull() {
  digitalWrite(LIGHT_FL, HIGH);
  digitalWrite(LIGHT_FR, HIGH);
}

void lightsOff() {
  digitalWrite(LIGHT_FL, LOW);
  digitalWrite(LIGHT_FR, LOW);
}

void lightsOnVar() {
  
}

void setBlinkDirection(char drctn) {
  dir = drctn; 
}


void blinker() {
  if(dir) {
    digitalWrite(LIGHT_FR, !blinking);
    blinking = !blinking;
  } else {
    digitalWrite(LIGHT_FL, !blinking);
    blinking = !blinking;
  }
  
  
  
  
}

