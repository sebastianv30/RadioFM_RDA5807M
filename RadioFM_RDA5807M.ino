//#include <Arduino.h>
#include <Wire.h>
#include <radio.h>

// all possible radio chips included.
#include <RDA5807M.h>
#include <SI4703.h>
#include <SI4705.h>
#include <SI4721.h>
#include <TEA5767.h>
#include <LiquidCrystal_I2C.h>

#include <RDSParser.h>

#define lcdColums 20
#define lcdRows 4
LiquidCrystal_I2C lcd(0x27, lcdColums, lcdRows);

/// The radio object has to be defined by using the class corresponding to the used chip.
/// by uncommenting the right radio object definition.

/// Create the radio instance that fits the current chip:
 RDA5807M radio;  ///< Create an instance of a RDA5807 chip radio
// SI4703   radio;  ///< Create an instance of a SI4703 chip radio.
// SI4705 radio;    ///< Create an instance of a SI4705 chip radio.
// SI4721 radio; ///< Create an instance of a SI4705 chip radio.
// TEA5767  radio;  ///< Create an instance of a TEA5767 chip radio.

/// get a RDS parser
RDSParser rds;


/// State of Keyboard input for this radio implementation.
enum RADIO_STATE {
  STATE_PARSECOMMAND, ///< waiting for a new command character.
  STATE_PARSEINT, ///< waiting for digits for the parameter.
  STATE_EXEC ///< executing the command.
};

RADIO_STATE kbState; ///< The state of parsing input characters.
char kbCommand;
int16_t kbValue;


uint16_t g_block1;
bool lowLevelDebug = false;

float frequency = 88;


//char cmd=' ';
// - - - - - - - - - - - - - - - - - - - - - - - - - -



// use a function in between the radio chip and the RDS parser
// to catch the block1 value (used for sender identification)
void RDS_process(uint16_t block1, uint16_t block2, uint16_t block3, uint16_t block4)
{
  // Serial.printf("RDS: 0x%04x 0x%04x 0x%04x 0x%04x\n", block1, block2, block3, block4);
  g_block1 = block1;
  rds.processData(block1, block2, block3, block4);
}

/// Update the Time
void DisplayTime(uint8_t hour, uint8_t minute)
{
  Serial.print("Time: ");
  if (hour < 10)
    Serial.print('0');
  Serial.print(hour);
  Serial.print(':');
  if (minute < 10)
    Serial.print('0');
  Serial.println(minute);
} // DisplayTime()


/// Update the ServiceName text on the LCD display.
void DisplayServiceName(char *name)
{
  bool found = false;

  for (uint8_t n = 0; n < 8; n++)
    if (name[n] != ' ')
      found = true;

  if (found) {
    Serial.print("Sender:<");
    Serial.print(name);
    Serial.println('>');
  }
} // DisplayServiceName()


/// Update the ServiceName text on the LCD display.
void DisplayText(char *txt)
{
  Serial.print("Text: <");
  Serial.print(txt);
  Serial.println('>');
} // DisplayText()


/// Execute a command identified by a character and an optional number.
/// See the "?" command for available commands.
/// \param cmd The command character.
/// \param value An optional parameter for the command.
void runSerialCommand(char cmd, int16_t value)
{
  unsigned long startSeek; // after 300 msec must be tuned. after 500 msec must have RDS.
  RADIO_FREQ fSave, fLast;
  RADIO_FREQ f = radio.getMinFrequency();
  RADIO_FREQ fMax = radio.getMaxFrequency();
  char sFreq[12];
  RADIO_INFO ri;

  if ((cmd == '\n') || (cmd == '\r')) {
    return;
  }

  Serial.print("do:");
  Serial.println(cmd);

  if (cmd == '?') {
    Serial.println();
    Serial.println("? Help");
    Serial.println("+ increase volume");
    Serial.println("- decrease volume");
    Serial.println("1 start scan version 1");
    Serial.println("2 start scan version 2");
    Serial.println(". scan up   : scan up to next sender");
    Serial.println(", scan down ; scan down to next sender");
    Serial.println("i station status");
    Serial.println("s mono/stereo mode");
    Serial.println("b bass boost");
    Serial.println("m mute/unmute");
    Serial.println("u soft mute/unmute");
    Serial.println("x debug...");
    Serial.println("* toggle i2c debug output");

    // ----- control the volume and audio output -----

  } else if (cmd == '+') {
    // increase volume
    int v = radio.getVolume();
    if (v < 15)
      radio.setVolume(++v);
  } else if (cmd == '-') {
    // decrease volume
    int v = radio.getVolume();
    if (v > 0)
      radio.setVolume(--v);

  } else if (cmd == 'm') {
    // toggle mute mode
    radio.setMute(!radio.getMute());

  } else if (cmd == 'u') {
    // toggle soft mute mode
    radio.setSoftMute(!radio.getSoftMute());

  } else if (cmd == 's') {
    // toggle stereo mode
    radio.setMono(!radio.getMono());

  } else if (cmd == 'b') {
    // toggle bass boost
    radio.setBassBoost(!radio.getBassBoost());


  } else if (cmd == '1') {
    // ----- control the frequency -----
    Serial.println("Scanning all available frequencies... (1)");
    fSave = radio.getFrequency();

    // start Simple Scan: all channels
    while (f <= fMax) {
      radio.setFrequency(f);
      delay(80);

      radio.getRadioInfo(&ri);
      if (ri.tuned) {
        radio.formatFrequency(sFreq, sizeof(sFreq));
        Serial.print(sFreq);
        Serial.print(' ');

        Serial.print(ri.rssi);
        Serial.print(' ');
        Serial.print(ri.snr);
        Serial.print(' ');
        Serial.print(ri.stereo ? 'S' : '-');
        Serial.print(ri.rds ? 'R' : '-');
        Serial.println();
      } // if

      // tune up by 1 step
      f += radio.getFrequencyStep();
    } // while
    radio.setFrequency(fSave);
    Serial.println();

  } else if (cmd == '2') {
    Serial.println("Seeking all frequencies... (2)");
    fSave = radio.getFrequency();

    // start Scan
    radio.setFrequency(f);

    while (f <= fMax) {
      radio.seekUp(true);
      delay(100); //
      startSeek = millis();

      // wait for seek complete
      do {
        radio.getRadioInfo(&ri);
      } while ((!ri.tuned) && (startSeek + 600 > millis()));

      // check frequency
      f = radio.getFrequency();
      if (f < fLast) {
        break;
      }
      fLast = f;

      if ((ri.tuned) && (ri.rssi > 42) && (ri.snr > 12)) {
        radio.checkRDS();

        // print frequency.
        radio.formatFrequency(sFreq, sizeof(sFreq));
        Serial.print(sFreq);
        Serial.print(' ');

        do {
          radio.checkRDS();
          // Serial.print(g_block1); Serial.print(' ');
        } while ((!g_block1) && (startSeek + 600 > millis()));

        // fetch final status for printing
        radio.getRadioInfo(&ri);
        Serial.print(ri.rssi);
        Serial.print(' ');
        Serial.print(ri.snr);
        Serial.print(' ');
        Serial.print(ri.stereo ? 'S' : '-');
        Serial.print(ri.rds ? 'R' : '-');

        if (g_block1) {
          Serial.print(' ');
          Serial.print('[');
          Serial.print(g_block1, HEX);
          Serial.print(']');
        } // if
        Serial.println();
      } // if
    } // while
    radio.setFrequency(fSave);
    Serial.println();


  } else if (cmd == 'f') {
    radio.setFrequency(value);
  }

  else if (cmd == '.') {
    radio.seekUp(false);
  } else if (cmd == ':') {
    radio.seekUp(true);
  } else if (cmd == ',') {
    radio.seekDown(false);
  } else if (cmd == ';') {
    radio.seekDown(true);
  }


  else if (cmd == '!') {
    // not in help
    RADIO_FREQ f = radio.getFrequency();
    if (value == 0) {
      radio.term();
    } else if (value == 1) {
      radio.init();
      radio.setBandFrequency(RADIO_BAND_FM, f);
    }

  } else if (cmd == 'i') {
    // info
    char s[12];
    radio.formatFrequency(s, sizeof(s));
    Serial.print("Station:");
    Serial.println(s);
    Serial.print("Radio:");
    radio.debugRadioInfo();
    Serial.print("Audio:");
    radio.debugAudioInfo();

  }  else if (cmd == 'x') {
    radio.debugStatus(); // print chip specific data.

  } else if (cmd == '*') {
    lowLevelDebug = !lowLevelDebug;
    radio._wireDebug(lowLevelDebug);
  }
} // runSerialCommand()

void datos(){
    // info
    
    char s[12];
    radio.formatFrequency(s, sizeof(s));
    lcd.clear();
    lcd.setCursor(0, 0); 
    lcd.print("  ARDUINO FM RADIO");
    lcd.setCursor(0, 1);
    lcd.print("  Modulo: RDA5807M");
    lcd.setCursor(0, 2); 
    lcd.print("Station:");
    lcd.print(s);
    //lcd.setCursor(0, 3); 
    //lcd.print("******************");
    
  }

//void interrupcion1(){
//     cmd='.';
//     radio.seekUp(false);
//     }
//  void interrupcion2(){
//     cmd=',';
//     radio.seekDown(false);
//     }



/// Setup a FM only radio configuration with I/O for commands and debugging on the Serial port.
void setup()
{
  // open the Serial port
  lcd.init(); // initialize LCD
  lcd.backlight();  // turn on LCD backlight
  Serial.begin(115200);
  Serial.print("Radio...");
  delay(500);
  //attachInterrupt(digitalPinToInterrupt(2),interrupcion1,RISING);
  //attachInterrupt(digitalPinToInterrupt(3),interrupcion2,RISING);


  lcd.begin(16, 2);
  lcd.clear();

#ifdef ESP8266
  // For ESP8266 boards (like NodeMCU) the I2C GPIO pins in use
  // need to be specified.
  Wire.begin(D2, D1); // a common GPIO pin setting for I2C
#endif

  // Enable information to the Serial port
  radio.debugEnable(true);
  radio._wireDebug(lowLevelDebug);

  // Initialize the Radio
  radio.init();

  radio.setBandFrequency(RADIO_BAND_FM, 8930);

  // delay(100);

  radio.setMono(false);
  radio.setMute(false);
  radio.setVolume(10);

  Serial.write('>');

  // setup the information chain for RDS data.
  radio.attachReceiveRDS(RDS_process);
  rds.attachServicenNameCallback(DisplayServiceName);
  rds.attachTextCallback(DisplayText);
  rds.attachTimeCallback(DisplayTime);

  runSerialCommand('?', 0);
  kbState = STATE_PARSECOMMAND;
} // Setup


/// Constantly check for serial input commands and trigger command execution.
void loop()
{
  if (Serial.available() > 0) {
    // read the next char from input.
    char c = Serial.peek();

    if ((kbState == STATE_PARSECOMMAND) && (c < 0x20)) {
      // ignore unprintable chars
      Serial.read();
      datos();
      

    } else if (kbState == STATE_PARSECOMMAND) {
      // read a command.
      kbCommand = Serial.read();
      kbState = STATE_PARSEINT;

    } else if (kbState == STATE_PARSEINT) {
      if ((c >= '0') && (c <= '9')) {
        // build up the value.
        c = Serial.read();
        kbValue = (kbValue * 10) + (c - '0');
      } else {
        // not a value -> execute
        runSerialCommand(kbCommand, kbValue);
        kbCommand = ' ';
        kbState = STATE_PARSECOMMAND;
        kbValue = 0;
        
      } // if
    } // if
  } // if
  
  // check for RDS data
  radio.checkRDS();


} // loop

// End.
