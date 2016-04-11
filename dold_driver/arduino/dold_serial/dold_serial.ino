/* 
 * Dold Button Reader
 * 
 * author: Guillaume Walck 2016
 */

#define USE_USBCON
#define NUM_PINS 7
#define NUM_BUTTONS  4

// in this order p17, p27, p37, p47, p57, p67, p77
const int dold_pins[NUM_PINS] = {2,3,4,5,6,7,8};

byte reading[NUM_PINS];     //raw pin reading
bool state[NUM_PINS];       //debounced pin state
byte button_state[NUM_BUTTONS]; //computed button state

long last_pub_time=0;
long pub_delay=10;
bool state_changed = false;
byte num_buttons = 0;

void setup()
{
  Serial.begin(9600);                  // The actual speed selection is arbitrary due to
                                       // virtual COM port over USB.
  //init pins
  for(int i=0; i<NUM_PINS; i++) 
  {
    //Set pins as inputs
    pinMode(dold_pins[i], INPUT);
 
    //disable the pullup resistor on the pins
    digitalWrite(dold_pins[i], LOW);
  
    //Inputs are normally low
    state[i] = digitalRead(dold_pins[i]);
  } 
  state_changed = true;
  last_pub_time = millis();
}

// encode state to match dold_msgs
unsigned char calc_state(bool half, bool full)
{
  if (full)
  {
     return 3;
  }
  else
  { 
    if (half)
    {
      return 2;
    }
    else
    {
      return 1;
    }
  }
}

void loop()
{
  for(int i=0; i<NUM_PINS; i++) 
  {
    // store the pin bit in a byte (to accumulate 8 samples)
    bool val = digitalRead(dold_pins[i]);
    if(val == true)
      reading[i] = reading[i] | B00000001;
    
    // did the state change ?
    if (reading[i]== B01111111 or reading[i] == B10000000){
       state[i] = reading[i] & B00000001;
       state_changed = true;
       
    }
    //shift to store
    reading[i] = reading[i] << 1;
  }  
  
  //if the state of the pins changed, recompute the button values and publish
  if ( state_changed ) {
    //B1
    bool BU11 = state[1] and not state[3];
    bool BU12 = state[1] and state[3] and not state[2];
    button_state[0] = calc_state(BU11, BU12);
   
    //B2
    bool BU21 = state[2] and not state[3];
    bool BU22 = state[2] and state[3] and not state[1];
    button_state[1] = calc_state(BU21, BU22);
    
    //B3
    bool BU31 = state[4] and not state[6];
    bool BU32 = state[4] and state[6] and not state[5];
    button_state[2] = calc_state(BU31, BU32);
    
    //B4
    bool BU41 = state[5] and not state[6];
    bool BU42 = state[5] and state[6] and not state[4];
    button_state[3] = calc_state(BU41, BU42);
    
    //power CURRENTLY UNUSED
    //bool power = state[0];

    num_buttons = NUM_BUTTONS;
    state_changed = false;
  }   

  // publish at a slower rate than the main loop
  if ((millis() - last_pub_time)  > pub_delay) {
    //always send the header and the number of buttons
    Serial.print("Dold:");
    //num_buttons +=1;
    Serial.write(&num_buttons, 1);
    //Serial.write(&(reading[1]), 1);
    // if buttons changed, send them too.
    if (num_buttons > 0) //1
      Serial.write(button_state, NUM_BUTTONS);
    last_pub_time = millis();
    num_buttons = 0;
  }

  delay(10);
}
