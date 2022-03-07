#include <FreqCount.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "M25LC256.h"
#include "wavtables.h"
#include "notes.h"

//#########################################################################
// PIN DESIGNATIONS AND STATIC VALUES

// Pulse outputs
#define PERIODPULSEOUT 1      // Pin providing pulse output undivided tempo
#define DIVPERIODPULSEOUT 0   // Pin providing pulse output at delay rate

// Tap input
#define TAPIN 2               // Pin for switch to denote "tap"

//Analog inputs
#define LFORATE A8            // Pin 22 - pot divider of 3V3 to give LFO rate value
#define LFODEPTH A6           // Pin 21 - pot divider of 3V3 to give LFO depth value
#define LFOSHAPE A5           // Pin 20 - pot divider of 3V3 to give LFO shape value
#define DELDIV A4             // Pin 18 - pot divider of 3V3 to give LFO depth value

// Display output pins and settings
#define SCREEN_WIDTH 128      // OLED display width, in pixels
#define SCREEN_HEIGHT 64      // OLED display height, in pixels
#define OLED_MOSI   4         // MOSI pin for external display
#define OLED_CLK   3          // CLK pin for external display 
#define OLED_DC    6          // DC pin for external display 
#define OLED_CS    7          // CS pin for external display
#define OLED_RESET 5          // RESET pin for external display

// PT2399 
#define FIN 9                 // Frequency measuring input pin - connects to pin5 of the PT2399

// Digital pots
#define CS_POT1 10            // Slave select for digipot 1
#define CS_POT2 14            // Slave select for digipot 2
#define SI 11                 // MOSI pin for digipot 1 & digipot 2
#define SO 12                 // MISO pin for digipot 1 & digipot 2
#define SCLK 13               // SCLK pin for digipot 1 & digipot 2

//#########################################################################
// Display and memory defintions

// Create a display object set to the current connections and size
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

// Create a memory class object for storing the delay table
M25LC256 memory;

//#########################################################################
// Global variables
volatile int startupdelayms = 280;  // Startup no-tap delay in milliseconds
volatile int tapmillis=280;         // Tap millisecond period
volatile int divtapmillis=280;      // Tap divided millisecond period
volatile int lastmillis=0;          // Last measured millisecond timer count
int tempo;                          // container for ms tempo to be stored as bpm
//float beatsinbar=4.0;             // Define how many beats in a bar. Could be changed by a switch or pot or something?
float ticksperbar=4.0;              // Define how many taps/ticks make 1 bar
int tapincntperbar;                 // Stores a 1-5 value which identifies what the input clock rate is
int lasttapincntperbar;             // Stores whatever the last value in the above was for comparison
int delaydivpot;                    // Stores the position of the delay division selection pot
int lastdelaydivpot;                // Stores the position of the LAST selected delay div selection pot so we can ensure a recalculation on change

const int debouncelimit=100;        // Minimum number of loops to have passed since last button press to debounce switch
int debouncecounter=0;              // Counter for debounce loop count

bool mindelayflag=false;            // Flag to identify when we are at the minimum possible delay time for the settings
bool maxdelayflag=false;            // Flag to identify when we are at the maximum possible delay time for the settings

int modtapm=0;                      // 

// multiplier variables that define what the divider selected does :
// Assumes 1/4 note taps
// 𝅝, 𝅗𝅥., 𝅗𝅥, 𝅘𝅥., 𝅘𝅥, ♪., ♪, 𝅘𝅥𝅯., 𝅘𝅥𝅯
// whole, dotted half, half, dotted quarter, quarter, dotted eighth, eighth, dotted sixteenth, sixteenth
float tapmultiplier[]={4.0,3.0,2.0,1.0,0.75,0.5,0.375,0.25};
float tapdivider[]={0.25,0.333,0.5,1.0,1.5,2,3,4};

int downbeatcounter=0;              // Counter for identifying where the downbeat of the tempo is
int modfactor;                      // Value that records how much we need to account for uneven divisions of milliseconds causing sync drift between divided pulse and standard pulse output
int modfactorarray[12];             // Array to hold the values to modify pulse output counts by when performing adjustments
int modarrayend=0;                  // Value identifying how many of the modfactorarray values are to be used for adjusting present output settings
int divbeatcounter=0;               // Counter for identifying which of the divided output pulses we're on
  

const int digpotmax = 255;          // Stores the max step value in the two digital pots
const int digpotmin = 0;            // Stores the min step value in the two digital pots
int r2max = 12;                     // Holds the max number of r2 positions that will be measured per step at startup. This will be changed to the digpotmax value in script after setup so needs to not be const

const int apotmax = 1023;           // Stores the max step value in the two digital pots
const int apotmin = 0;              // Stores the min step value in the two digital pots

// Figure out FULL delaytable arraysize = ((digpotmax+1)*r2max)+((digpotmax+1)-r2max) - so, with 256 step value pots, measuring 10 r2 values you get : ((255+1)*10)+((255+1)-10) = 2560 + 246 = 2806
// 10 = 2806
// 11 = 3061
// 12 = 3315
const int delaytablelen = 3316;

// Struct that contains 3 values for the delay table
typedef struct _delay_measurement_{
  int r1;
  int r2;
  int ms;
}delaymeasurement;

// Create the delaytable array of structs, to the size defined in delaytablelen
delaymeasurement delaytable[delaytablelen];

int delaytablesstoredvaluelen=0;    // The length of the array that is stored in memory (after sorting and deduping)
uint16_t memaddress = 0x0000;       // The first address of memory

// Variables to manage output of a pulse
elapsedMillis pulsecounter;         // always incrementing millisecond counter
elapsedMillis divpulsecounter;      // always incrementing millisecond counter
const int pulsewidthmillis = 5;     // width of output pulse in milliseconds

int tablearraypos=0;                // Variable to store the position of the current resistor values in the delay table array

// Variables to handle the switch to do PT2399 analysis on startup
int startupbuttoncounter=millis();  // Timer for how long button has been held on startup
int startupholdtime = 4000;         // Value that stores how long you need to hold the button for to enter setup at startup

//#########################################################################
//LFO VARIABLES START
unsigned long OSC;
unsigned long rateval;
unsigned char Out1;
unsigned int lforateval;
unsigned int lfodepthval;
unsigned int lfoshapeval;
const int ratemin=0;
const int ratemax=100;
float depthval;
int modval;
//LFO VARIABLES END
//#########################################################################

int totalcompares=0;                //

// Digital pot address value byte   //
byte digipotaddress = 0x13;         //

// Analog input read smoothing variables
const int maxsmoothreadings=50;
int lforatepotInputReadings[maxsmoothreadings];
int lforatepotInputNumberOfReadings=1;
int lforatepotInputPointer = 0;

int lfodepthpotInputReadings[maxsmoothreadings];
int lfodepthpotInputNumberOfReadings=1;
int lfodepthpotInputPointer = 0;

int lfoshapepotInputReadings[maxsmoothreadings];
int lfoshapepotInputNumberOfReadings=1;
int lfoshapepotInputPointer = 0;

int delaydivpotInputReadings[maxsmoothreadings];
int delaydivpotInputNumberOfReadings=1;
int delaydivpotInputPointer = 0;


void setup() {
  // Start serial comms
  Serial.begin(9600);
   
  // Start counting frequency with period of 10000
  FreqCount.begin(10000);
  
   Serial.println("setupbegins"); // Present for debugging purposes

  // VIDEO output setup
  // by default, we'll generate the high voltage from the 3.3v line internally
  display.begin(SSD1306_SWITCHCAPVCC);
  
  // The buffer is intialized with an Adafruit splashscreen so this will display the splashscreen.
  display.display();                  // Show image buffer on the display hardware.
  display.clearDisplay();             // Clear the display buffer
  Serial.println("DISPLAYON"); // Present for debugging purposes
  bool startupanalyse=false;          // value to identify if we are going to go to analysis of the 2399 or not
  
  Serial.println("set pin modes"); // Present for debugging purposes

  // Set up pins
  // DIGITAL POTENTIOMETERS
  pinMode(CS_POT1, OUTPUT);
  pinMode(CS_POT2, OUTPUT);

  // OUTPUT PULSE(S)
  pinMode(PERIODPULSEOUT, OUTPUT);
  pinMode(DIVPERIODPULSEOUT, OUTPUT);
  
  SPI.begin();                          // Begin SPI comms for pots, display and memory

  // DIGITAL INPUT
  pinMode(TAPIN, INPUT);                // Tap button (No need for pulldown as external is supplied)
  
  // MEMORY INITIALISATION
  Serial.println("meminit"); // Present for debugging purposes
  memory.init();
  Serial.println("meminit ends"); // Present for debugging purposes

    
  // Identify if tap button is held at startup
  if(digitalRead(TAPIN)==HIGH){
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    // set startupanalyse to true as button was pressed at start
    startupanalyse=true;
    display.println("Entering startup");
    display.display();
    // Loop continually checking that button is still pressed. If released, set startupanalyse to false
    // Stop looping when the startupbuttoncounter millisecond count hits the startupholdtime value
    while((startupanalyse==true) && (startupbuttoncounter<startupholdtime)){
      if(digitalRead(TAPIN)==LOW){
        startupanalyse=false;
      }else{
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Entering startup");
        display.println(startupholdtime-startupbuttoncounter);
        display.display();
      }
      startupbuttoncounter=millis();
    }
  }

  // Check if button was held for whole time by validating it is still "true" after the startupbuttoncounter reaches the hold time
  if(startupanalyse==true){
    pt2399_analyse();                 // If it was, fire off the PT2399 analysis to identify the relevant timings and write results to the memory
  }

  wipedelayarray();                   // Wipe array from local memory
  readmem();                          // Read array from storage

  // Set up falling edge interrupt - tapin has an external pulldown resistor - and so taps are counted on button release/pulse end
  attachInterrupt(digitalPinToInterrupt(TAPIN), tap, RISING); 

  // Show completion message
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Startup complete");
  display.display();

  // Identify the modification values required
  getmodifiers(delaydivpot);

  // Now we've set the modfactor and modarrayend values we can set up the modifier array
  setmodifiers();
  
  tablearraypos=get_closest_delay_pos(divtapmillis);

  pulsecounter=0;
  divpulsecounter=0;  
}


// Main loop
void loop() {
  // Check the pulse output and set as appropriate
  pulseoutput();
  
  // Read the values from the analog inputs for LFO settings
  readinputs();
  
  // Tempomaths - Calculate the current targetdelay based on the divisor selected
  tempomath();
  
  // Run the LFO to identify what the modification value should be
  get_mod_values();
  
  // Set the modified delay time based on LFO  
  set_mod_delay(tablearraypos, modval);
  
  // Put some info onto a screen  
  displaydelays(divtapmillis, tablearraypos);
  
  // Increment debounce counter
  debouncecounter++;
  
  // Do this forever and ever
}


// Set output pulse timer modification requirements
// Because the divided pulse output will drift from the pulse output in scenarios where the output is based on x/3 we need to know what the 
// length of modification that is required to fix the drift, and every how-many divided pulses does that modification need to be applied
// modfactor var will tell us how many milliseconds of offset we need to account for in the number of pulses defined in modarrayend
void getmodifiers(int ddp){
  switch (ddp){
    case 0 : // 4 : x*4 : 0 - 1 pulse every 4 / 0.75/3
    modtapm = tapmillis;
    divtapmillis = tapmillis * 4;
    modfactor = 0;
    modarrayend=4;
    break;
    case 1 : // 3 : x*3 : 1 - 1 pulse every 3 / 1/3
    modtapm = tapmillis;
    divtapmillis = tapmillis * 3;
    modfactor = 0;
    modarrayend=4;
    break;
    case 2 : // 2 : x*2 : 2 - 1 pulse every 2 / 1.5/3
    modtapm = tapmillis;
    divtapmillis = tapmillis * 2;
    modfactor = 0;
    modarrayend=4;
    break;
    case 3 : // 1 : x*1 : 3 - 1 pulse every 1 / 3/3
    modtapm = tapmillis;
    divtapmillis = tapmillis;
    modfactor = 0;
    modarrayend=4;
    break;
    case 4 : // 0.75 : (x*3) / 4 : 4 - 4 pulses every 3 /  4/3
    modtapm = tapmillis*3;
    divtapmillis = modtapm/4;    
    modfactor = modtapm%divtapmillis;
    modarrayend=4;
    break;
    case 5 : // 0.5 : x/2 : 5 - 2 pulses every 1 / 6/3
    modtapm = tapmillis*3;
    divtapmillis = modtapm/6;
    modfactor = tapmillis%divtapmillis;
    modarrayend=6;
    break;
    case 6 : // 0.375 : (x*3) / 8 : 6 - 3 pulses every 1 / 8/3
    modtapm = tapmillis*3;
    divtapmillis = modtapm/8;
    modfactor = modtapm%divtapmillis;
    modarrayend=8;
    break;
    case 7 : // 0.25 : x/4 : 7 - 4 pulses every 1 / 12/3
    modtapm = tapmillis*3;   //2043
    divtapmillis = tapmillis/12; // 56
    modfactor = tapmillis%divtapmillis; // 9
    modarrayend=12;
    break;
  }
}

// Function to set up the modifier arrays for the div outputs of the current tempo
// This is needed because certain divisions of pulses will result in the divided period not being a straight division of the standard pulse output
// And so we need to nudge some values in the divided pulse up by a millisecond to even them out over the spread
void setmodifiers(void){
  int modifiedcount=0;                                      // Renitialise the modified pulse count
  
  // initialize the array to all zeroes
  for(int z=0; z<12; z++){
    modfactorarray[z]=0;
  }

  // Check if the current division even requires a modification factor - if not leave the array initialised
  if(modfactor>0){
    int seperator=modarrayend/modfactor;                    // A value to identify which components of the array need adjusted
    
    modfactorarray[0]++;                                    // Always increment the first value
    modifiedcount++;                                        // Increment the counter of how many values have been modified
    for(int x=1; x<modarrayend; x++){
      if(((x%seperator)==0)&&(modifiedcount<modfactor)){    // Check if current position is one that should be modified
        modfactorarray[x]++;                                // Increment if so
        modifiedcount++;
      }
    }
  }
}

void tempomath(){
  float beatlengthms = divtapmillis;
  tempo=round(60000.0/beatlengthms);
}

void get_mod_values(){

  rateval=lforateval;       // get rate value from rate pot input

  // Use the lfo depth pot input to give a float multiplier which can 
  // be used to modify the value returned by the wave table lookup
  // This multiplication will give us depth control
  if(lfodepthval<100){
    depthval = 1.0/lfodepthval;
  }else{
    depthval = 0.0;
  }

  // add offset for non-zero freq at lowest setting
  rateval += 1;
  // increment the OSC based on the rate value supplied
  OSC += (rateval*65536);  

  // update which value we will be returning from the wave table by bit shifting the larger value
  int x = OSC >> 24;

  switch(lfoshapeval){
    case 1 : 
      modval = SinTable[x];
      break;
    case 2 : 
      modval = TriangleTable[x];
      break;
    case 3 : 
      modval = SquareTable[x];
      break;
    case 4 : 
      modval = SawTable[x];
      break;
    case 5 : 
      modval = RampTable[x];
      break;
    case 6 : 
      modval = NoiseTable[x];
      break;
    default : 
      modval = SinTable[x];
      break;
  }

  if(depthval>0){ 
    modval = modval*depthval;      
  }else{
    modval = 0;
  }
}

void set_mod_delay(int ap, int mv){
  int pos = ap+mv;
  if(pos>=delaytablesstoredvaluelen){
    pos = delaytablesstoredvaluelen-1;
  }else if(pos<0){
    pos=0;
  }
  // Set the digital potentiometers to the values in the table
  digitalPotWrite(CS_POT1, delaytable[pos].r1);
  digitalPotWrite(CS_POT2, delaytable[pos].r2);
}

// Reads the analog input values and get smoothed values. Recalculate modifiers for pulse division if the div input has changed
void readinputs(){
  int ratetotal=0;
  int depthtotal=0;
  int shapetotal=0;
  int tapcnttotal=0;
  int divtotal=0;
  
  ///######################################################################
  // read and smooth lfo rate pot
  lforatepotInputReadings[lforatepotInputPointer]=analogRead(LFORATE);          // Adds current analog rate input value to array
  lforatepotInputPointer++;                                                     // Increments array pointer position
  for(int i=0;i<lforatepotInputNumberOfReadings;i++){
    ratetotal = ratetotal+lforatepotInputReadings[i];                           // Adds together all readings taken so far
  }
  lforateval = ratetotal/lforatepotInputNumberOfReadings;                       // Gets an average of all the readings taken so far
  if(lforatepotInputNumberOfReadings<maxsmoothreadings){                        // Identifies if we've got to the maximum number of readings
    lforatepotInputNumberOfReadings++;                                          // Increments number of readings counter if not
  }
  if(lforatepotInputPointer>=maxsmoothreadings){                                // Identifies if we've filled the array
    lforatepotInputPointer=0;                                                   // If we have, start putting values into the array from position 0 again
  }
  lforateval = map(lforateval, apotmin, apotmax, ratemin, ratemax);             // Map the average reading into the allowable range
  
  ///######################################################################
  // read and smooth lfo depth pot
  lfodepthpotInputReadings[lfodepthpotInputPointer]=analogRead(LFODEPTH);       // Adds current analog rate input value to array
  lfodepthpotInputPointer++;                                                    // Increments array pointer position
  for(int i=0;i<lfodepthpotInputNumberOfReadings;i++){
    depthtotal = depthtotal+lfodepthpotInputReadings[i];                        // Adds together all readings taken so far
  }
  lfodepthval = depthtotal/lfodepthpotInputNumberOfReadings;                    // Gets an average of all the readings taken so far
  if(lfodepthpotInputNumberOfReadings<maxsmoothreadings){                       // Identifies if we've got to the maximum number of readings
    lfodepthpotInputNumberOfReadings++;                                         // Increments number of readings counter if not
  }
  if(lfodepthpotInputPointer>=maxsmoothreadings){                               // Identifies if we've filled the array
    lfodepthpotInputPointer=0;                                                  // If we have, start putting values into the array from position 0 again
  }
  lfodepthval = map(lfodepthval, apotmin, apotmax, 1, 105);                     // Map the average reading into the allowable range
  
  ///######################################################################
  // read and smooth lfo shop pot
  lfoshapepotInputReadings[lfoshapepotInputPointer]=analogRead(LFOSHAPE);       // Adds current analog rate input value to array
  lfoshapepotInputPointer++;                                                    // Increments array pointer position
  for(int i=0;i<lfoshapepotInputNumberOfReadings;i++){
    shapetotal = shapetotal+lfoshapepotInputReadings[i];                        // Adds together all readings taken so far
  }
  lfoshapeval = shapetotal/lfoshapepotInputNumberOfReadings;                    // Gets an average of all the readings taken so far
  if(lfoshapepotInputNumberOfReadings<maxsmoothreadings){                       // Identifies if we've got to the maximum number of readings
    lfoshapepotInputNumberOfReadings++;                                         // Increments number of readings counter if not
  }
  if(lfoshapepotInputPointer>=maxsmoothreadings){                               // Identifies if we've filled the array
    lfoshapepotInputPointer=0;                                                  // If we have, start putting values into the array from position 0 again
  } 
  lfoshapeval = map(lfoshapeval, apotmin, apotmax, 0, 6);                       // Map the average reading into the allowable range
  
  ///######################################################################
  // read and smooth Delaydivpot
  lastdelaydivpot = delaydivpot;
  delaydivpotInputReadings[delaydivpotInputPointer]=analogRead(DELDIV);       // Adds current analog rate input value to array
  delaydivpotInputPointer++;                                                    // Increments array pointer position
  for(int i=0;i<delaydivpotInputNumberOfReadings;i++){
    divtotal = divtotal+delaydivpotInputReadings[i];                            // Adds together all readings taken so far
  }
  delaydivpot = divtotal/delaydivpotInputNumberOfReadings;                      // Gets an average of all the readings taken so far
  if(delaydivpotInputNumberOfReadings<maxsmoothreadings){                       // Identifies if we've got to the maximum number of readings
    delaydivpotInputNumberOfReadings++;                                         // Increments number of readings counter if not
  }
  if(delaydivpotInputPointer>=maxsmoothreadings){                               // Identifies if we've filled the array
    delaydivpotInputPointer=0;                                                  // If we have, start putting values into the array from position 0 again
  }
  delaydivpot = map(delaydivpot, apotmin, apotmax, 0, 7);                       // Map the average reading into the allowable range
  
  if(delaydivpot!=lastdelaydivpot){                                             // Check if the delay division pot position has changed since last time
    getmodifiers(delaydivpot);                                                  // Perform required calculations for delay sync modifiers
    setmodifiers();                                                             // Set up array values for delay sync modifiers
    
    tablearraypos=get_closest_delay_pos(divtapmillis);                          // set global variable to the current array position of the current resistor settings
    pulsecounter=0;                                                             // Reset pulse counters as division has changed
    divpulsecounter=pulsecounter;
  }
  
}

// Function for outputting a pulse on a specific pin
void pulseoutput(){
  int pulselen=0;
  int targetmillis=0;
  // Identify if pulse counter needs to increase, and by how much
  // If the counter has gone beyond the end of the period, reset it

    pulselen = pulsewidthmillis;
    targetmillis = tapmillis;
  
  if(pulsecounter>targetmillis){
    pulsecounter=pulsecounter-targetmillis;
  }
  
  if(divpulsecounter>(divtapmillis+modfactorarray[divbeatcounter])){
    divpulsecounter=divpulsecounter-(divtapmillis+modfactorarray[divbeatcounter]) ;
  }
 
  // If the both pulsecounters are less than the width of pulse in millis, both outputs should be low (a low output makes the clock output high in hardware)
  if((pulsecounter<pulselen)&&(divpulsecounter<pulselen)){
    // If div pulse is currently high and switching low this is a div clock tick so should increment the divbeatcounter
    if(digitalRead(DIVPERIODPULSEOUT)==HIGH){
      // If the tick keeps it lower than the arrayend then increment
      if((divbeatcounter+1)<modarrayend){
        divbeatcounter++;
      // Otherwise reset
      }else{
        divbeatcounter=0;
      }
    }
    // Set both outputs low
    digitalWrite(PERIODPULSEOUT, LOW);
    digitalWrite(DIVPERIODPULSEOUT, LOW);
  // If the main pulse counter is higher than the pulse len but divpulse counter is lower, then only divpulsecounter should be set low
  }else if((pulsecounter>=pulselen)&&(divpulsecounter<pulselen)){
    // If div pulse is currently high and switching low this is a div clock tick so should increment the divbeatcounter
    if(digitalRead(DIVPERIODPULSEOUT)==HIGH){
      // If the tick keeps it lower than the arrayend then increment
      if((divbeatcounter+1)<modarrayend){
        divbeatcounter++;
      // Otherwise reset
      }else{
        divbeatcounter=0;
      }
    }
    digitalWrite(PERIODPULSEOUT, HIGH);
    digitalWrite(DIVPERIODPULSEOUT, LOW);
  }else if((pulsecounter<pulselen)&&(divpulsecounter>=pulselen)){  
    digitalWrite(PERIODPULSEOUT, LOW);
    digitalWrite(DIVPERIODPULSEOUT, HIGH);
  }else{
    digitalWrite(PERIODPULSEOUT, HIGH);
    digitalWrite(DIVPERIODPULSEOUT, HIGH);
  }

}

// Function for performing analysis of the PT2399 outputed delay times for
// each combination of potentiometer settings
// This takes a LONG time so should only be done on boot when requested
void pt2399_analyse(void){
  
  // Cycle through ALL possible combinations of resistance and measure F at each (256*256)
  // Build a massive array of objects containing delay(ms) alongside R1 & R2 settings for each
  // Then sort that array in order of delay
  
  double fcount; // local variable for the value of frequency read from PT2399
  int currentdelayms; // local var for most recently measured delay in milliseconds

  // Write both pots to zero and wait a few ms for the PT2399 to catch up
  digitalPotWrite(CS_POT1, 0);
  digitalPotWrite(CS_POT2, 0);
  delay(150);

  // Clear the display buffer
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  // output that analysis is begun
  display.println("STARTING ANALYSIS");
  display.display();

  // Now loop through all 256 possible positions of R1
  for(int r1 = 0; r1 <= digpotmax; r1++){
    
    // On the final loop through, measure all 256 positions of R2 to get full possible range
    if(r1==digpotmax){
      r2max=digpotmax+1;
    }

    // Loop through r2 positions up to a maximum position set by r2max.
    // with r1=100k, and r2=10k, then 10 steps on r2 should be 1 step on r1.
    // so - no point measuring values where we don't need to and can step r1 up instead
    for(int r2 = 0; r2 < r2max; r2++){
        do{
          // Set both pots values
          digitalPotWrite(CS_POT1, r1);
          digitalPotWrite(CS_POT2, r2);
          // wait a few usecs to allow to stabilise
          delayMicroseconds(500);
          // wait for a frequency count to be available
          while(!FreqCount.available()){
            delayMicroseconds(500);
          }
          // read in the freqcount value
          fcount=FreqCount.read()*100;
        // If it's over 28000000 (clock max speed ~25MHz) then it's an eroneous reading so do it again
        }while(fcount>28000000);
        // Calculate delay time in ms from frequency of clock
        // Equations from Electric Druid - https://electricdruid.net/useful-design-equations-for-the-pt2399/
        currentdelayms = (683.21 / (fcount/1000000.0)) + 0.08;
        // Write values of resistor settings and the delay measured for those settings to the delaytable objects
        delaytable[tablearraypos].r1=r1;
        delaytable[tablearraypos].r2=r2;
        delaytable[tablearraypos].ms=currentdelayms;

        // output to screen the readings taken
        displaytablebuild(r1, r2, currentdelayms, fcount);

        // increment array pointer for delaytable so next one goes in the next slot
        tablearraypos++;
      
    }
  }

  // Now sort the array
  // Get the delay table length based on the size of the array of structs divided by the size of each struct in the array
  size_t delaytable_len = sizeof(delaytable) / sizeof(delaymeasurement);
  
  // sort array using qsort functions and "struct_cmp_by_ms" to do the comparisons. This returns based on the ms struct component of the relevant array value
  qsort(delaytable, delaytable_len, sizeof(delaymeasurement), struct_cmp_by_ms);
  
  // deduplicate array and get new length back
  int newarraylen = dedupe_delay_table_array();

  // Now return the deduped array to memory
  // First memory position holds array length
  memory.writeword(memaddress,newarraylen);

  // Then write 16 bit values for each of the 3 vars in each array struct to sequential memory locations
  for(int p=0;p<newarraylen;p++){
    memaddress=memaddress+2;
    memory.writeword(memaddress,delaytable[p].r1);
    memaddress=memaddress+2;
    memory.writeword(memaddress,delaytable[p].r2);
    memaddress=memaddress+2;
    memory.writeword(memaddress,delaytable[p].ms);
  }
}

// Function to find the nearest delay time to that passed in. 
// Sets the target ms to that ms value
// Returns the array location of the set options
int get_closest_delay_pos(int ms){
  
  // Find first value in delay array that is higher than ms
  int setval = 0;
  bool setflag = false;
  // loop through array
  for(int i=0; i<delaytablesstoredvaluelen; i++){
    // if we've not yet found a higher value and set a flag
    // AND the current value is higher then
    // set the flag and value to the current increment of pointer
    if((setflag==false)&&(delaytable[i].ms>ms)){
      setval=i;
      setflag=true;
    }
  }

  // If none of the values in the array were higher than the passed in ms value
  // set to the final entry in the array
  if(!setflag){
    setval=delaytablesstoredvaluelen-1;
  }

  if(setval==0){
    mindelayflag=true;
    maxdelayflag=false;
  }else if(setval==delaytablesstoredvaluelen-1){
    mindelayflag=false;  
    maxdelayflag=true;
  }else{
    mindelayflag=false;
    maxdelayflag=false;
  }
  
 
  //divtapmillis=delaytable[setval].ms;

  // return the position of hte array pointer
  return setval;
}

// Function to clear all values in the delaytable array
void wipedelayarray(){
  for(int i=0; i<delaytablelen; i++){
    delaytable[i].r1=0;
    delaytable[i].r2=0;
    delaytable[i].ms=0;
  }
}

void readmem(){
  // First two bits contain array length
  delaytablesstoredvaluelen = memory.readword(0);
  // Start address for other content is 2
  
  int arradd = 2;
  for(int q=0; q<delaytablesstoredvaluelen; q++){
    delaytable[q].r1=memory.readword(arradd);
    arradd = arradd+2;
    delaytable[q].r2=memory.readword(arradd);
    arradd = arradd+2;
    delaytable[q].ms=memory.readword(arradd);
    arradd = arradd+2;
  }
  // rest of array set to zeroes
  for(int r=delaytablesstoredvaluelen; r<delaytablelen; r++){
    delaytable[r].r1=0;
    delaytable[r].r2=0;
    delaytable[r].ms=0;
  }
  
}

int dedupe_delay_table_array(){

  int arrlen = sizeof(delaytable) / sizeof(delaymeasurement);
  
  //Find duplicate elements in array
  for(int i=0; i<arrlen; i++)
  {
      for(int j=i+1; j<arrlen; j++)
      {
          // If any duplicate found
          if(delaytable[i].ms == delaytable[j].ms)
          {
              // Delete the current duplicate element 
              for(int k=j; k<arrlen; k++)
              {
                  delaytable[k] = delaytable[k + 1];
              }

              // Decrement size after removing duplicate element 
              arrlen--;

              // If shifting of elements occur then don't increment j 
              j--;
          }
      }
  }

  return arrlen;

}

int struct_cmp_by_ms(const void *a, const void *b){
    totalcompares++;
    delaymeasurement *ia = (delaymeasurement *)a;
    delaymeasurement *ib = (delaymeasurement *)b;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.println("QSORTING");
    display.println(totalcompares);
    display.display();

    if (ia->ms < ib->ms){
        return -1;
    }
    else if (ia->ms > ib->ms){
        return +1;

    }
    else{
        return 0;
    }
}

//MCP4xxxx write
void digitalPotWrite(int currentpot, int value){
  //Serial.println("Write CS low");
  digitalWrite(currentpot, LOW);
  //Serial.println("Transfer address"); // Present for debugging purposes
  SPI.transfer(digipotaddress);
  //Serial.println("Transfer value"); // Present for debugging purposes
  SPI.transfer(value);
  //Serial.println("Write CS high"); // Present for debugging purposes
  digitalWrite(currentpot, HIGH);
}

//DISPLAYWRITE
void displaydelays(int targetd, int arrpos) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  
  display.println("--------------------");
  if((mindelayflag==true)||(maxdelayflag==true)){
    display.print("DELAY - ");
    if(mindelayflag==true){
      display.println("MIN");
    }else{
      display.println("MAX");
    }
  }else{
    display.println("DELAY");
  }

  display.print(targetd);
  display.print("ms");
  display.print(" / ");
  display.print(tempo);
  display.println("bpm");

  display.print("TM:");
  display.print(tapmillis);
  
  display.print(" DTM:");
  display.println(divtapmillis+modfactorarray[divbeatcounter]);

  display.print("MODTAP:");
  display.println(modtapm);
  

  //LFO STUFF
  //display.println("--------------------");
  display.print("DDP:" );
  display.println(delaydivpot);
  display.println(debouncecounter);
  display.print("Shape : ");
switch(lfoshapeval){
    case 1 : 
      display.println("Sin");
      break;
    case 2 : 
      display.println("Tri");
      break;
    case 3 : 
      display.println("Sqr");
      break;
    case 4 : 
      display.println("Saw");
      break;
    case 5 : 
      display.println("Rmp");
      break;
    case 6 : 
      display.println("Nse");
      break;
    default : 
      display.println("Sin");
      break;
  }
/*
  
  display.print("MO:");
  display.print(modfactor);
  display.print("  VAL:");
  display.print(modfactorarray[divbeatcounter]);
  display.print("  DBC:");
  display.println(divbeatcounter);
  

  display.print("T:");
  display.print(tapmillis);
  display.print("|");
  display.print("D:");
  display.print(divtapmillis);
  display.print("|");
  display.print("V:");
  display.print(tapmultiplier[delaydivpot]);
  */
 



  /*
  switch(delaydivpot){
    case 0 :
      display.clearDisplay();
      display.drawBitmap(0,0,wholenote, NOTE_WIDTH, NOTE_HEIGHT, 1);
      display.display();
    case 1 :
      display.clearDisplay();
      display.drawBitmap(0,0,wholenote, NOTE_WIDTH, NOTE_HEIGHT, 1);
      display.display();
      //display.println("𝅝"); // 1
    case 2 :
    display.clearDisplay();
      display.drawBitmap(0,0,dottedhalfnote, NOTE_WIDTH, NOTE_HEIGHT, 1);
      display.display();
      //display.println("𝅗𝅥"); // 1/2
    case 3 :
      display.drawBitmap(0,0,halfnote, NOTE_WIDTH, NOTE_HEIGHT, 1);
      display.display();
      //display.println("𝅗𝅥."); // 1/3
    case 4 :
      display.drawBitmap(0,0,dottedquarternote, NOTE_WIDTH, NOTE_HEIGHT, 1);
      display.display();
      //display.println("𝅘𝅥"); // 1/4
    case 5 :
      display.drawBitmap(0,0,quarternote, NOTE_WIDTH, NOTE_HEIGHT, 1);
      display.display();
      //display.println("𝅘𝅥."); // 1/6
    case 6 :
      display.drawBitmap(0,0,dottedeighthnote, NOTE_WIDTH, NOTE_HEIGHT, 1);
      display.display();
      // display.println("♪"); // 1/8
    case 7 :
       display.drawBitmap(0,0,eighthnote, NOTE_WIDTH, NOTE_HEIGHT, 1);
      display.display();
      //display.println("♪."); // 1/12
    case 8 :
       display.drawBitmap(0,0,dottedsixteenthnote, NOTE_WIDTH, NOTE_HEIGHT, 1);
      display.display();
      //display.println("𝅘𝅥𝅯"); // 1/16
    case 9 :
       display.drawBitmap(0,0,sixteenthnote, NOTE_WIDTH, NOTE_HEIGHT, 1);
      display.display();
      //display.println("𝅘𝅥𝅯."); // 1/24
  }*/
  display.display();
}

void tap() {
  // Interrupt on tap Begin
  // Check if enough loops since last tap - otherwise ignore
  if(debouncecounter>debouncelimit){
    
    // Reset debounce counter
    debouncecounter=0;
    
    // Set delay time to current millis – last delay time;
    tapmillis = millis()-lastmillis;
    lastmillis = millis();
    
    // reset output pulse
    pulsecounter=0;
    divpulsecounter=0;
    
    // reset downbeat
    downbeatcounter=0;
    
    getmodifiers(delaydivpot);
    
    setmodifiers();
    
    // set global variable to the current array position of the current resistor settings
    tablearraypos=get_closest_delay_pos(divtapmillis);
    }

}

void displaytablebuild(int r_1, int r_2, int ms, int f) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print("R1 : ");
  display.println(r_1);
  display.print("R2 : ");
  display.println(r_2);
  display.print("MS : ");
  display.println(ms);
  display.print("F : ");
  display.println(f);
  display.display();
}
