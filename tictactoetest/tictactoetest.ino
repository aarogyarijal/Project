
#include<avr/wdt.h>

/****** ONLY CHANGE THIS SETTING *****/

#define RED_PIN     0
#define GREEN_PIN   1
#define MAX_LED     9

#define ON_THRESHOLD 40
/*************************************/


/*      MULTIPLEXER VARIABLES  */

#define RED_COLOR   0
#define GREEN_COLOR 1

volatile bool gameOver = false;

uint8_t winPattern[8][3] = {
                            {0,1,2},
                            {3,4,5},
                            {6,7,8},
                            {0,3,6},
                            {1,4,7},
                            {2,5,8},
                            {0,4,8},
                            {2,4,6}
                            };

typedef struct _LEDInfo{
  bool      state;
  uint8_t   color;
}LEDInfo;

volatile LEDInfo led[MAX_LED];
volatile uint8_t multiplexer = 0;
volatile uint8_t rgbGroundPins[MAX_LED] = {9,10,11,12,13,14,15,16,17}; //only three pins for now

/*  MULTIPLEXER VARIABLES   */

/* CAPACITIVE SENSING VARIABLES */


#define PRESS(led) (led >= ON_THRESHOLD)
volatile int turnCount = 0;

uint8_t capacitivePins[MAX_LED] = {18,19,2,3,4,5,6,7,8};
int touchValue[9] = {-1};
int lastTouchValue[9] = {-1}; 

/* CAPACITIVE SENSING VARIABLES */

void turnLED(uint8_t, uint8_t);
int capSense(uint8_t);
void initializeGame();
uint8_t winColor = 255;
uint8_t pattern = 0;

void setup() {
  // put your setup code here, to run once:
  initializeGame();
  //Serial.begin(9600);

   for(int i=0; i<MAX_LED; ++i){
    touchValue[i] = capSense(capacitivePins[i]);
    lastTouchValue[i] = capSense(capacitivePins[i]);
   }
}

volatile long counter = 0;
int td = 0;
void loop() {
  //read touch values from each of the sensor

  for(int i=0; i<MAX_LED; ++i){

    if(gameOver){
      TIMSK2 = (1<<TOIE2);

      if(counter>=600){
        counter = 0;
        initializeGame();
      }

      if(winColor == RED_COLOR && counter%30==0){
        td++;
        for(int v=0; v<3;v++){
          if(td&1){
            led[winPattern[pattern][v]].state = false;
            led[winPattern[pattern][v]].color = 255;
          }
          else{
            led[winPattern[pattern][v]].state = true;
            led[winPattern[pattern][v]].color = RED_COLOR;
          }  
        }
        
      
      }

      if(winColor == GREEN_COLOR && counter%30==0){
        td++;
        for(int v=0; v<3;v++){
          if(td&1){
            led[winPattern[pattern][v]].state = false;
            led[winPattern[pattern][v]].color = 255;
          }
          else{
            led[winPattern[pattern][v]].state = true;
            led[winPattern[pattern][v]].color = GREEN_COLOR;
          }  
        }
      }
    }

     

    lastTouchValue[i] = touchValue[i];
    touchValue[i] = capSense(capacitivePins[i]);    
    
    //if(PRESS(touchValue[i]) && led[i].state == false){
    if(touchValue[i]-lastTouchValue[i] > 10 && led[i].state == false){
      led[i].color = turnCount&1;
      ++turnCount;
      led[i].state = true; 

      if(turnCount>=9)
        gameOver = true;
        
      if(led[i].color == RED_COLOR){
        int i,j;
        for(i=0; i<8; ++i){
          for(j=0; j<3; ++j){
            if(led[winPattern[i][j]].color == RED_COLOR && led[winPattern[i][j]].state == true){
              continue;
            }else break;
          }
          if(j==3){ //red win  
            /*for(int k=0; k<9; ++k){
              led[k].state = true;
              led[k].color = RED_COLOR;
            }*/
            gameOver = true;
            winColor = RED_COLOR;
            pattern = i;
            break;
          }
        }
      }else if(led[i].color == GREEN_COLOR){
         int i,j;
        for(i=0; i<8; ++i){
          for(j=0; j<3; ++j){
            if(led[winPattern[i][j]].color == GREEN_COLOR && led[winPattern[i][j]].state == true){
              continue;
            }else break;
          }
          if(j==3){ //green win  
            /* for(int k=0; k<9; ++k){
              led[k].state = true;
              led[k].color = GREEN_COLOR;
            }*/
            gameOver = true;
            winColor = GREEN_COLOR;
            pattern = i;
            break;
          }
        }
      }

      
    }
  }


}



ISR(TIMER1_COMPA_vect){

  if(multiplexer==0)
    digitalWrite(rgbGroundPins[MAX_LED-1], HIGH);
  else
    digitalWrite(rgbGroundPins[multiplexer-1], HIGH);
  //for(int i=0; i<MAX_LED; ++i)
    //if(i!=multiplexer)
    //  digitalWrite(rgbGroundPins[i], HIGH);
      
  if(led[multiplexer].state == true)
    digitalWrite(rgbGroundPins[multiplexer], LOW);
  
  if(led[multiplexer].color == RED_COLOR){
    digitalWrite(RED_PIN, HIGH);
    digitalWrite(GREEN_PIN, LOW);
  }
  
  if(led[multiplexer].color == GREEN_COLOR){
    digitalWrite(RED_PIN, LOW);
    digitalWrite(GREEN_PIN, HIGH);
  }
 
  
  ++multiplexer;
  if(multiplexer >= MAX_LED)
    multiplexer = 0;
}

ISR(TIMER2_OVF_vect){
  ++counter;
}

void initializeGame(){
  //multiplexer initializer
  noInterrupts();           // disable all interrupts
  
  td = 0;
  winColor = 255;
  turnCount = 0;
  gameOver = false;
  counter = 0;

  
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 2;              // compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12)|(1<<CS11)|(1<<CS10);    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt 
  
  TCCR2B |= (1<<CS22);
  TIMSK2 &= ~(1<<TOIE2);
  
  for(int i=0; i<MAX_LED; ++i){
    pinMode(rgbGroundPins[i], OUTPUT);
    digitalWrite(rgbGroundPins[i], HIGH);
  }

  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);

  //capacitive initializer
  for(int i=0; i<MAX_LED; ++i){
    led[i].state = false;
    led[i].color = 255; //no any color
  }
  interrupts();             // enable all interrupts
}

int capSense(uint8_t pinNo){
  int ticks = 0;
  volatile uint8_t *ddr;
  volatile uint8_t *port;
  volatile uint8_t *pin;
  uint8_t bitMask;
  
  if(pinNo>=0 && pinNo<=7){
    ddr = &DDRD;
    port = &PORTD;
    pin = &PIND;
    bitMask = (1<<pinNo);
  }
  if(pinNo>=8 && pinNo<=13){
    ddr = &DDRB;
    port = &PORTB;
    pin = &PINB;
    bitMask = (1<<(pinNo - 8));
  }
  if(pinNo>=14 && pinNo<=19){
    ddr = &DDRC;
    port = &PORTC;
    pin = &PINC;
    bitMask = (1<<(pinNo - 14));
  }

  
  *port &= ~bitMask;
  *ddr |= bitMask;
  
  *ddr &= ~bitMask;
  
  while(ticks<=20000 && !(*pin&bitMask))
    ++ticks;

 *port &=~bitMask;
 *ddr |= bitMask;
 
 if(ticks>=20000)
  return -1;

 return ticks;
}

