#include <Timers.h> 

#define toneC    1911
#define toneC1    1804
#define toneD    1703
#define toneEb    1607
#define toneE    1517
#define toneF    1432
#define toneF1    1352
#define toneG    1276
#define toneAb    1204
#define toneA    1136
#define toneBb    1073
#define toneB    1012
#define tonec       955
#define tonec1      902
#define toned       851
#define toneeb      803
#define tonee       758
#define tonef       716
#define tonef1      676
#define toneg       638
#define toneab      602
#define tonea       568
#define tonebb      536
#define toneb       506
 
#define tonep       0 

#define SPEAKER_PIN A1
#define TIMER_1 1
#define TUNE_TIME 1
#define ALARM_TIME 6000

long vel = 20000;

char static const tuneVal = 17;
char static tuneCount = 0;

int melod[] = {tonec, toneG, toneE, toneA, toneB, toneBb, toneA, toneG, tonee, toneg, tonea, tonef, toneg, tonee, tonec, toned, toneB};
int ritmo[] = {18, 18, 18, 12, 12, 6, 12, 8, 8, 8, 12, 6, 12, 12, 6, 6, 6};


void tocar(int tom, long tempo_value);
unsigned char TestTimerExpired(unsigned char timer);

void setupPiezo() {
  pinMode(SPEAKER_PIN,OUTPUT);
}

void setTune() {
  TMRArdReturn_t TMRArd_InitTimer(TIMER_1, TUNE_TIME);
  tuneCount = 0;
}

int playTune() {
//  for (int i=0; i<17; i++) {
  if(TestTimerExpired(TIMER_1)) {
    int tom = melod[tuneCount];
    int tempo = ritmo[tuneCount];

    long tvalue = tempo * vel;

    tocar(tom, tvalue);

    tuneCount++;
    TMRArdReturn_t TMRArd_InitTimer(TIMER_1, TUNE_TIME);

  }
  
  if(tuneCount == tuneVal) {
    tuneCount = 0;
    return 1;
  } else {
    return 0;
  }
}

void honk() {
  for (int i=0; i<2; i++) {
    int tom = melod[i];
    int tempo = ritmo[i];

    long tvalue = tempo * vel;

    tocar(tom, tvalue);

  }
}

void setAlarm() {
  TMRArdReturn_t TMRArd_InitTimer(TIMER_1, ALARM_TIME);
  honk();
}

void playAlarm() {
  if (TestTimerExpired(TIMER_1)) {
    honk();
    TMRArdReturn_t TMRArd_InitTimer(TIMER_1, ALARM_TIME);
  }
}


void tocar(int tom, long tempo_value) {
  long tempo_gasto = 0;
  while (tempo_gasto < tempo_value) {
    digitalWrite(SPEAKER_PIN, HIGH);
    delayMicroseconds(tom / 2);
 
    digitalWrite(SPEAKER_PIN, LOW);
    delayMicroseconds(tom/2);  
    tempo_gasto += tom;
  }
}

unsigned char TestTimerExpired(unsigned char timer) {
  return (unsigned char)(TMRArd_IsTimerExpired(timer));
}
