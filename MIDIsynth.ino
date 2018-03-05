

#include <SPI.h>
#include "vs1003.h"

// It takes about 100 microseconds to read an analog input,
// so the maximum reading rate is about 10,000 times a second.
// The analog pins also have pullup resistors, which work 
// identically to pullup resistors on the digital pins. 
// They are enabled by issuing a command such as:
// digitalWrite(A0, INPUT_PULLUP);  // set pullup on analog pin 0 


// Hardware connections and definitions
#define VS_DREQ   2 // Data Request Pin: Player asks for more data
#define VS_XCS    6 // Control Chip Select Pin (for accessing SPI Control/Status registers)
#define VS_XDCS   7 // Data Chip Select / BSYNC Pin
#define VS_RESET  8 // Reset Pin: It is active low

// Piezo defines
#define NUM_PIEZOS 6
#define SNARE_THRESHOLD 60    // anything < TRIGGER_THRESHOLD is treated as 0
#define LTOM_THRESHOLD  60
#define RTOM_THRESHOLD  60
#define LCYM_THRESHOLD  60
#define RCYM_THRESHOLD  60
#define KICK_THRESHOLD  60

// MIDI note defines for each trigger
#define KICK_NOTE  36 // Cassa
#define SNARE_NOTE 38 // Rullante
#define LCYM_NOTE  42 // Charlie Chiuso
#define LTOM_NOTE  43 // Tom
#define RTOM_NOTE  45 // Floor Tom
#define RCYM_NOTE  51 // Piatto

// MIDI defines
#define DRUM_CHAN 9
#define MAX_MIDI_VELOCITY 127

// Program defines 
#define SIGNAL_BUFFER_SIZE 100
#define PEAK_BUFFER_SIZE   30

// ALL TIME MEASURED IN MILLISECONDS
#define MAX_TIME_BETWEEN_PEAKS 20
#define MIN_TIME_BETWEEN_NOTES 50


// Global Variables

// Ring buffers to store analog signal and peaks
unsigned short slotMap[NUM_PIEZOS];     // map that holds the mux slots of the piezos
unsigned short noteMap[NUM_PIEZOS];     // map that holds the respective note to each piezo
unsigned short thresholdMap[NUM_PIEZOS];//map that holds the respective threshold to each piezo

short          currentSignalIndex[NUM_PIEZOS];
short          currentPeakIndex[NUM_PIEZOS];
unsigned short signalBuffer[NUM_PIEZOS*SIGNAL_BUFFER_SIZE];
unsigned short peakBuffer[NUM_PIEZOS*PEAK_BUFFER_SIZE];

unsigned short noteReadyVelocity[NUM_PIEZOS];
boolean        noteReady[NUM_PIEZOS];
boolean        isLastPeakZeroed[NUM_PIEZOS];

unsigned long lastPeakTime[NUM_PIEZOS];
unsigned long lastNoteTime[NUM_PIEZOS];

byte chan;


// Function Prototypes

void LoadUserCode();
void WriteVS10xxRegister(unsigned char addressbyte, unsigned short data);
void sendMIDI(byte data);
void talkMIDI(byte cmd, byte data1, byte data2);
void noteOn(byte channel, byte note, byte attack_velocity);
void noteOff(byte channel, byte note, byte release_velocity);


// Function Bodies

// Write to VS10xx register
//   SCI Data transfers are always 16bit. 
//   When a new SCI operation comes in DREQ goes low.
//   We then have to wait for DREQ to go high again.
//   XCS should be low for the full duration of operation.
void 
WriteVS10xxRegister(unsigned char addressbyte, unsigned short data) {
  while(!digitalRead(VS_DREQ)) ;// Wait for DREQ to go high indicating IC is available
  digitalWrite(VS_XCS, LOW);    // Select board (xCS)
  //SCI consists of instruction byte, address byte, and 16-bit data word.
  SPI.transfer(0x02);           //Write instruction
  SPI.transfer(addressbyte);
  SPI.transfer(data >> 8);
  SPI.transfer(data & 0xff);
  while(!digitalRead(VS_DREQ)) ;// Wait for DREQ to go high indicating command is complete
  digitalWrite(VS_XCS, HIGH);   // Deselect board
}


// Load a code into the VS10xx RAM
#if 0
void 
LoadUserCode() {
  int code_size = sizeof(atab)/sizeof(*atab);
  for(int i=0; i<CODE_SIZE; i++) {
    WriteVS10xxRegister(atab[i], dtab[i]);
  }
}
#endif


#if 1
void 
LoadUserCode() {
  int i = 0;
  while (i<PLUGIN_SIZE) {
    unsigned short addr, n, val;
    addr = pgm_read_word_near(plugin + i++);
    n = pgm_read_word_near(plugin + i++);
    if (n & 0x8000U) { // RLE run, replicate n samples
      n &= 0x7FFF;
      val = pgm_read_word_near(plugin + i++);
      while (n--) {
        WriteVS10xxRegister(addr, val);
      }
    } else {           // Copy run, copy n samples
      while (n--) {
        val = pgm_read_word_near(plugin + i++);
        WriteVS10xxRegister(addr, val);
      }
    }
  }
}
#endif


// Send a MIDI byte trough SPI
//   Each MIDI byte MUST be prfixed with a Zero byte.
void 
sendMIDI(byte data) {
  SPI.transfer(0);
  SPI.transfer(data);
}


// Plays a MIDI note.
//   Doesn't check to see that cmd is greater than 127,
//   or that data values are less than 127
void 
talkMIDI(byte cmd, byte data1, byte data2) {
  // Wait for chip to be ready (Unlikely to be an issue with real time MIDI)
  while (!digitalRead(VS_DREQ)) ;
  digitalWrite(VS_XDCS, LOW);
  sendMIDI(cmd);
  // Some commands only have one data byte. 
  // All cmds less than 0xBn have 2 data bytes 
  // (sort of: http://253.ccarh.org/handout/midiprotocol/)
  if( (cmd & 0xF0) <= 0xB0 || (cmd & 0xF0) >= 0xE0) {
    sendMIDI(data1);
    sendMIDI(data2);
  } else {
    sendMIDI(data1);
  }
  digitalWrite(VS_XDCS, HIGH);
}


//Send a MIDI note-on message.  Like pressing a piano key
//channel ranges from 0-15
void 
noteOn(byte channel, byte note, byte attack_velocity) {
  talkMIDI( (0x90 | channel), note, attack_velocity);
}


//Send a MIDI note-off message.  Like releasing a piano key
void 
noteOff(byte channel, byte note, byte release_velocity) {
  talkMIDI( (0x80 | channel), note, release_velocity);
}


// First parameter is the event type (0x0B = control change).
// Second parameter is the event type, combined with the channel.
// Third parameter is the control number number (0-119).
// Fourth parameter is the control value (0-127).
void 
controlChange(byte channel, byte control, byte value) {
  talkMIDI( (0xB0 | channel), control, value);
}


void 
programChange(byte channel, byte program) {
  talkMIDI( (0xC0 | channel), program, 0);
}


void 
noteFire(byte channel, unsigned short note, unsigned short velocity) {
  if(velocity > MAX_MIDI_VELOCITY)
    velocity = MAX_MIDI_VELOCITY;
  noteOn(channel, note, velocity);
  noteOff(channel, note, velocity);
}


void 
recordNewPeak(short slot, short newPeak) {
//Serial.print("recordNewPeak ");
  isLastPeakZeroed[slot] = (newPeak == 0);
  unsigned long currentTime = millis();
  lastPeakTime[slot] = currentTime;
  //new peak recorded (newPeak)
  peakBuffer[slot*PEAK_BUFFER_SIZE+currentPeakIndex[slot]] = newPeak;
  // 1 of 3 cases can happen:
  //   1) note ready - if new peak >= previous peak
  //   2) note fire  - if new peak < previous peak and previous peak was a note ready
  //   3) no note    - if new peak < previous peak and previous peak was NOT note ready
  
  //get previous peak
  short prevPeakIndex = currentPeakIndex[slot]-1;
  if(prevPeakIndex < 0) prevPeakIndex = PEAK_BUFFER_SIZE-1;        
  unsigned short prevPeak = peakBuffer[slot*PEAK_BUFFER_SIZE+prevPeakIndex];
   
  if(newPeak > prevPeak && (currentTime-lastNoteTime[slot])>MIN_TIME_BETWEEN_NOTES) {
    noteReady[slot] = true;
    if(newPeak > noteReadyVelocity[slot])
      noteReadyVelocity[slot] = newPeak;
//Serial.print("newPeak= ");
//Serial.println(newPeak);
  }
  else if(newPeak < prevPeak && noteReady[slot]) {
    noteFire(DRUM_CHAN, noteMap[slot], noteReadyVelocity[slot]);
//Serial.print("NoteFire c=");
//Serial.print(DRUM_CHAN);
//Serial.print("\t n=");
//Serial.print(noteMap[slot]);
//Serial.print("\t v=");
//Serial.println(noteReadyVelocity[slot]);
    noteReady[slot] = false;
    noteReadyVelocity[slot] = 0;
    lastNoteTime[slot] = currentTime;
  }
  currentPeakIndex[slot]++;
  if(currentPeakIndex[slot] == PEAK_BUFFER_SIZE) currentPeakIndex[slot] = 0;  
}



void 
setup() {
//Serial.begin(115200);

  pinMode(VS_DREQ,  INPUT);
  pinMode(VS_XCS,   OUTPUT);
  pinMode(VS_XDCS,  OUTPUT);
  pinMode(VS_RESET, OUTPUT);
  
  // All AVR based boards have an SS pin that is useful when they act as a slave 
  // controlled by an external master. 
  // Since the library supports only master mode, this pin should be set always 
  // has OUTPUT otherwise the SPI interface could be put automatically into slave 
  // mode by hardware, rendering the library inoperative. 
#if defined(ARDUINO_ARCH_AVR)
  pinMode(10,       OUTPUT); //Pin 10 must be set as an output for the SPI communication to work
#endif

  digitalWrite(VS_XCS,  HIGH); //Deselect Control
  digitalWrite(VS_XDCS, HIGH); //Deselect Data

  //Initialize VS1003 chip 
  digitalWrite(VS_RESET, LOW); //Put VS1003 into hardware reset

  //Setup SPI for VS1003
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);

  //From page 12 of datasheet, max SCI reads are CLKI/7. Input clock is 12.288MHz. 
  //Internal clock multiplier is 1.0x after power up. 
  //Therefore, max SPI speed is 1.75MHz. We will use 1MHz to be safe.
  #if (F_CPU==8000000L)
    SPI.setClockDivider(SPI_CLOCK_DIV8); //Set SPI bus speed to 1MHz (8MHz / 8 = 1MHz)
  #else 
    SPI.setClockDivider(SPI_CLOCK_DIV16); //Set SPI bus speed to 1MHz (16MHz / 16 = 1MHz)
  #endif
  
  SPI.transfer(0xFF);                   //Throw a dummy byte at the bus
  delayMicroseconds(1);
  digitalWrite(VS_RESET, HIGH);         //Bring up VS1003

  LoadUserCode();                 // Load the Real-Time MIDI plugin into VS1003 RAM  
  WriteVS10xxRegister(0x0A, 0x30);// Start the code by writing 0x30 to SCI AIADDR (0x0A).
  
  controlChange(DRUM_CHAN, 0x07, 120);// Set channel volume near to max (127)
  programChange(DRUM_CHAN, DRUM_CHAN);// Set instrument number
  
  //Play notes just to show we are running...
  for(int note=35 ; note<82; note++) {
    //noteOn channel 1 (0x90), note value, velocity
    noteOn(DRUM_CHAN, note, 127);
    noteOff(DRUM_CHAN, note, 127);
    delay(150);
  }

  thresholdMap[0] = KICK_THRESHOLD;
  thresholdMap[1] = RTOM_THRESHOLD;
  thresholdMap[2] = RCYM_THRESHOLD;
  thresholdMap[3] = LCYM_THRESHOLD;
  thresholdMap[4] = SNARE_THRESHOLD;
  thresholdMap[5] = LTOM_THRESHOLD;

  slotMap[0] = A0;// ADC 0
  slotMap[1] = A1;// ADC 1
  slotMap[2] = A2;// ADC 2
  slotMap[3] = A3;// ADC 3
  slotMap[4] = A6;// ADC 6
  slotMap[5] = A7;// ADC 7

  noteMap[0] = KICK_NOTE;
  noteMap[1] = RTOM_NOTE;
  noteMap[2] = RCYM_NOTE;
  noteMap[3] = LCYM_NOTE;
  noteMap[4] = SNARE_NOTE;
  noteMap[5] = LTOM_NOTE;  
/*
  for(int i=0 ; i<NUM_PIEZOS; i++) {
    delay(500);
    noteOn(DRUM_CHAN, noteMap[i], 127);
    noteOff(DRUM_CHAN, noteMap[i], 127);
  }
*/  
  for(short i=0; i<NUM_PIEZOS; ++i) {
    currentSignalIndex[i] = 0;
    currentPeakIndex[i]   = 0;
    noteReady[i]          = false;
    noteReadyVelocity[i]  = 0;
    isLastPeakZeroed[i]   = true;
    lastPeakTime[i]       = millis();
    lastNoteTime[i]       = lastPeakTime[i];    
    //slotMap[i]            = START_ADC_SLOT + i;
  }
  memset(signalBuffer, 0, NUM_PIEZOS*SIGNAL_BUFFER_SIZE*sizeof(unsigned short));
  memset(peakBuffer,   0, NUM_PIEZOS*PEAK_BUFFER_SIZE*sizeof(unsigned short));
}


void 
loop() {
  unsigned long currentTime = millis();
  // The vibration frequency of a drum should be lower than 500 Hz
  // so the Nyquist minimum sampling frequency must be > 1000Hz
  for(short i=0; i<NUM_PIEZOS; ++i) {
    // Get a new signal. It takes about 100 microseconds to read an analog input
    unsigned short newSignal = analogRead(slotMap[i]);
    newSignal = analogRead(slotMap[i]);// Double read for allowing ADC stabilizing...
    signalBuffer[i*SIGNAL_BUFFER_SIZE+currentSignalIndex[i]] = newSignal;
    if(newSignal < thresholdMap[i]) {
      if(!isLastPeakZeroed[i] && (currentTime-lastPeakTime[i]) > MAX_TIME_BETWEEN_PEAKS) {
        recordNewPeak(i, 0);
      }
      else {
        // Get previous signal from the circular buffer
        short prevSignalIndex = currentSignalIndex[i]-1;
        if(prevSignalIndex < 0) prevSignalIndex = SIGNAL_BUFFER_SIZE-1;        
        unsigned short prevSignal = signalBuffer[i*SIGNAL_BUFFER_SIZE+prevSignalIndex];
        unsigned short newPeak = 0;
        // Find the wave peak if previous signal was not 0 by going
        // through previous signal values until another 0 is reached
        while(prevSignal >= thresholdMap[i]) {
          if(signalBuffer[i*SIGNAL_BUFFER_SIZE+prevSignalIndex] > newPeak) {
            newPeak = signalBuffer[i*SIGNAL_BUFFER_SIZE+prevSignalIndex];        
          }
          // Decrement previous signal index, and get previous signal
          prevSignalIndex--;
          if(prevSignalIndex < 0) prevSignalIndex = SIGNAL_BUFFER_SIZE-1;
          prevSignal = signalBuffer[i*SIGNAL_BUFFER_SIZE+prevSignalIndex];
        }
        if(newPeak > 0) {
          recordNewPeak(i, newPeak);
        }
      }
    }// if(newSignal < thresholdMap[i])
    currentSignalIndex[i]++;
    if(currentSignalIndex[i] == SIGNAL_BUFFER_SIZE) currentSignalIndex[i] = 0;
  }// for(short i=0; i<NUM_PIEZOS; ++i)
}

