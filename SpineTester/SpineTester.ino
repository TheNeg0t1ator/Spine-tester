
/*******************************************************************************
********************************************************************************
**                                                                            **
**                           Arrow Spine Measuring Tool                       **
**                                                                            **
********************************************************************************
********************************************************************************
**                                                                            **
**                                 Version 3.1                                **
**                                                                            **
**      0.1 -   Initial development version                                   **
**      0.2 -   EEPROM implemented to store setup configuration               **
**      0.3 -   Converted to use Arduino Nano Every with Grove IC2 LCD        **
**              Display and moved buttons onto digital pins                   **
**      1.0 -   First version with all basic features in place                **
**      1.1 -   Sleep/low power mode implemented                              **
**      2.0 -   Extended to handle two load cells; debugging simplified       **
**      2.1 -   Averaging of spine readings; convert button to Bounce2        **
**              class to handle debouncing to simplify long/short presses.    **
**      2.2 -   Implemented constant reading of HX711 to improve              **
**              responsiveness by reducing time blocked reading HX711         **
**      2.3 -   Centre of Gravity / FOC calculations                          **
**      3.0 -   Added 23" mode, by reading SWITCH23INCHMODE (PIN9 by def)     **
**              and apply ATASPINEFACTORMEASUREDAT23 factor to spine if       **
**              on.                                                           **
**      3.1 -   Changed the debug statements from Debug->Line to a define     **
**              allowing the disabling of serial debug.  This can save a      ** 
**              lot of memory.  Debug statements could make some arduino      **
**              models rundels out of memory.                                 **
**                                                                            **
**                                                                            **
**                                                                            **
********************************************************************************
********************************************************************************
**                                                                            **                                                                           
**      Code for measuring and calculating arrow spine.                       **                                                                       
**                                                                            ** 
**      The code runs on an Arduino interfacing to HC711 DACs connected       **
**      to load cells.  The arrow to be measured will be supported at a       **
**      spacing of 28".  Each support will have a load cell under it to       **
**      measure the weight on it when it is depressed at the centre by        **
**      1/2".                                                                 **
**                                                                            **                                                                           
**      Arrow spine is measured according to the deflection caused by         **
**      hanging a 1.94lb weight on an arrow supported at 28" spacing,         **
**      according to the ASTM standards.                                      ** 
**                                                                            **     
**      A 500 arrow is used as the model.  A 500 arrow will deflect 1/2"      **
**      with a 1.94lb weight hanging from it.  Therefore, if a 500 arrow      **
**      is manually deflected 1/2" a total reaction of 880g will be           **
**      caused at the supports; 440g at each.  Thus a load cell under         **
**      one of the supports will read 440g.  Therefore any arrow spine        **
**      can be inferred by depressing it 1/2" and factoring 500 according     **
**      to the fraction of 440 read from the load cells.                      **        
**                                                                            **
**      The tester can also be built with supports at 23" rather than with    **
**      supports at 28".  A 23" spine tester trades precision for being       **
**      able to test a greater range of arrow sizes and for a more            **
**      convenient size. A switch or jumper on pin 9 allows the hardware      **
**      configuration (23" vs 28") to be selected.  Readings are converted    **
**      between the two configurations by applying a factor of 1.804 to the   ** 
**      spine readings.  Thanks to Jean-Noel Kulichenski for the changes.     **   
**                                                                            **
********************************************************************************
********************************************************************************
**                                                                            **
**      Arduino Pin Assignments:                                              **
**                                                                            **
**      A4 = SDA on IC2 display connector                                     **
**      A5 = SCL on IC2 display connector                                     **
**      D0,1 = serial                                                         **
**      D2 = OK button                                                        **
**      D3 = Spine button                                                     **
**      D4 = Calibrate button                                                 **
**      D5 = HX711 SCK                                                        **
**      D6 = HX711 DOUT                                                       **
**      D7 = HX711 SCK                                                        **
**      D8 = HX711 DOUT                                                       **
**      D9 = 23" or 28" centres switch                                        **
**      D13 = Onboard LED                                                     **
**                                                                            **
********************************************************************************
*******************************************************************************/

#include <Wire.h>
#include <rgb_lcd.h>
#include "HX711.h"
#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <Bounce2.h>


//
// Debug controls. 
// 
// Debug information is written to the serial output.
// This is very useful but can cause problems.  Most routines have debug information.
// This does incur a processing overhead.  This may or may not be an issue depending
// upon the processor.  Also the debug strings can use up a lot of memory which can 
// be a problem on some Arduinos depending upon how much memory the Arduino has and 
// depending upon which memory the IDE decides to place the debug strings.  Set the 
// FUNC define to either __PRETTY_FUNCTION__ (which is the clearest debug but also 
// the most memory hungry) or to __FUNCTION__ (which doesn't differentiate as well 
// between class members with the same name but which uses less memory).
// If processing power or memory is a big problem then debug can be turned on and off 
// globally by defining or undefining the constant DEBUGON 
//
#define DEBUGON
#define FUNC                            __PRETTY_FUNCTION__
//define FUNC                           __FUNCTION__



// display messages
#define ARROWSCALETITLE                 "Spine Tester"
#define ARROWSCALEVERSION               "SW Version 3.1.0"
#define ARROWSCALEDELAY                 2000
//define BUILDERMESSAGELINE1            "Cie des Archers" // owner - displayed if defined
//define BUILDERMESSAGELINE2            "Quevenois"       // owner - displayed if defined
//define BUILDERMESSAGEDELAY            2000
#define READYMESSAGE                    "Position arrow; press ok"
#define REMOVEWEIGHT                    "Remove weight; press ok"
#define ZEROINGSCALE                    "Zeroing the scale..."
#define ZEROFACTOR                      "Zero factor"
#define PLACE1KGWEIGHT                  "Position 1kg weight; press ok"
#define CALIBRATINGSCALE                "Calibrating the arrow scale..."
#define CALIBRATINGGAUGE                "Calibrating gauge %d"
#define CALIBRATION                     "Calibration"
#define CHECKINGSCALE                   "Checking scale..."
#define KILOGRAMREADING                 "1kg reading"
#define GRAMS                           "g"
#define CALIBRATIONCOMPLETE             "Calibration complete"
#define ARROWSPINE                      "Arrow spine:"
#define ASTMAT28SHORTSTRING             "ASTM@28"
#define AMOAT26SHORTSTRING              "AMO@26"          
#define AMOLBSAT26HORTSTRING            "lbs@26"          
#define GRAMSAT28HORTSTRING             "grams@28"          
#define ASTMAT28STRING                  "ASTM spine measured @28\""
#define AMOAT26STRING                   "AMO spine measured @26\""
#define AMOLBSAT26STRING                "AMO poundage measured @26\""          
#define GRAMSAT28STRING                 "Grams force measured @28\""          
#define ASTMAT28UNITS                   ""
#define AMOAT26UNITS                    "" 
#define AMOLBSAT26UNITS                 "lb"          
#define GRAMSUNITS                      "g" 
#define AVERAGESPINESTRING              "Avg spine %s%s max %s%s @ %i%c"
#define DEGREESYMBOL                    223
#define WEIGHT                          "Weight:"
#define FOC                             "F of C:"
#define FOCUNITS                        "%" 
#define GRAINSUNITS                     "gn"
#define ARROWLENGTH                     "Arrow length:"
#define MOUNTEDIN23                     "Mounted at 23\""
#define MOUNTEDIN28                     "Mounted at 28\""
#define MOUTEDSIZEDELAY                 2000




// pin assignments


#define SERIAL0                         0       // pin for serial connection
#define SERIAL1                         1       // pin for serial connection
#define BUTTON_OK_PIN                   2       // OK button input
#define BUTTON_SPINE_PIN                3       // SPINE button input
#define BUTTON_CALIBRATE_PIN            4       // CALIBRATE button input
#define LOADCELL1_SCK_PIN               5       // CLK for HX711 for load cell
#define LOADCELL1_DOUT_PIN              6       // data for HX711 for load cell
#define LOADCELL2_SCK_PIN               7       // CLK for HX711 for load cell
#define LOADCELL2_DOUT_PIN              8       // data for HX711 for load cell
#define SWITCH23INCHMODE                9       // Switch button if in 23" mode (apply the ATASPINEFACTORMEASUREDAT23 factor on SPINE reading)
#define SDAIC2CONNECTOR                 A4      // SDA on IC2 display connector
#define SCLIC2CONNECTOR                 A5      // SCL on IC2 display connector



// LCD dimensions
#define LCDROWS                         2
#define LCDCOLUMNS                      16



// delay until the tester is put into sleep in milliseconds
#define SLEEPTIMEOUT                    300000



// delay until the press ok message is displayed in milliseconds
#define DISPLAYDELAY                    5000



// interval that we have to meet before the spine change is initiated
#define SPINECHANGEPRESSDELAY           5000



// delay until EEPROM data is written out
#define EEPROMWRITEDELAY                60000



//return values for ReadButtons()
#define BUTTON_NONE                     0 
#define BUTTON_1_OK                     1
#define BUTTON_3_CALIBRATE_UP           2
#define BUTTON_2_SPINE_UP               3
#define BUTTON_3LONG_CALIBRATE_BIGUP    4
#define BUTTON_2LONG_SPINE_UP           5




// HX711 controlling load cell definitions
#define DEFAULTCALIBRATION              943     // vaguely correct for 2 kg
#define CELLS                           2       // there are two load cells
#define SIGNIFICANTREADING              10      // reading different enough
#define SUPPORTSPACING28                711.2   // dist between load cells (28")
#define SUPPORTSPACING23                584.2   // dist between load cells (23")



// arrow data definitions
#define MAXSPINE                        9999    // max spine for display
#define MAXWEIGHT                       9999    // max weight for display
#define DEFAULTSPINEMETHOD              ASTMAT28 // default is 28" ASTM spine
#define DEFAULTARROWLENGTH              (28*16) // default arrow length (28") 



// spine methodology constants
enum SpineMethodologies 
{
 ASTMAT28,                              // ASTM spine - thou depression caused 
                                        // by 1.94lb weight with arrow supported 
                                        // at 28" centres
 AMOAT26,                               // ATA spine - thou depression caused 
                                        // by 2lb weight with arrow supported at 
                                        // 26" centres 
 AMOLBSAT26,                            // bow poundage measured according to 
                                        // deflection caused by 2lb weight to 
                                        // arrow supported at 26" centres          
 GRAMSAT28,                             // grams force on one support pillar 
                                        // caused by 1/2" deflection supported 
                                        // at 28" centres          
 MAXSPINEMETHOD                            
};



// spine string style flags
enum SpineStringFlags 
{
  LONGSPINESTRING,
  SHORTSPINESTRING,
  SPINEUNITS
};



// conversion factors
#define ATASPINEFACTORMEASUREDAT23      1.804224542 // convert 23" measure to 28
#define AMOMULTIPLIER                   0.825       // convert 26" AMO to 28
#define ASTMFACTOR                      440000.0    // convert force to ASTM
#define SIXTEENTHSTOMM                  (25.4/16)   // convert 1/16" to mm
#define GRAMTOGRAINS                    15.4324     // convert grams to grains



/*******************************************************************************
********************************************************************************
**                                                                            **
**                             Utility Routines                               **
**                                                                            **
********************************************************************************
*******************************************************************************/



/*******************************************************************************
*                                                                              *
*                                  trim                                        *
*                                                                              *
*       String whitespace trimming routine; C doesn't have one for char *      *
*       strings for some reason.                                               *
*                                                                              *
*******************************************************************************/

char * trim (char *s) 
{
    char * ptr;

    // return if NULL string
    if (!s) return NULL;    

    // return if empty string
    if (!*s) return s;   
       
    for (ptr = s + strlen (s) - 1; (ptr >= s) && isspace(*ptr); --ptr);
    ptr[1] = '\0';
    for (ptr = s; (*ptr != '\0') && isspace(*ptr); ptr++);
    
    return (ptr);
    
} /* trim */



/*******************************************************************************
*                                                                              *
*                                   sqr                                        *
*                                                                              *
*       Simple floating point square function; simpler than the general        *
*       purpose c power function and improves readability of code.             *
*                                                                              *
*******************************************************************************/

//float sqr (float f) // not currently needed
//{
//  
//  return (f*f);
//    
//} /* sqr */



/*******************************************************************************
********************************************************************************
**                                                                            **
**                             Serial Debug                                   **
**                                                                            **
**      Class to write debug info to the serial port (USB).                   **
**      Originally written to use String class but converted to a more        **
**      simple char * as the String class causes memory fragmentation.        **
**      Floats don't work with format strings in Arduino libraries so         **
**      there is a special routine for floats.                                **
**                                                                            **
********************************************************************************
*******************************************************************************/

#define MAXDEBUGSTRLEN                  200
#define SERIALBAUDRATE                  115200 // max for serial monitor

class SerialDebug 
{
  public:
    SerialDebug (const char * DebugString);
    void Line (const char * ActivityString, const char * Format = NULL, ...);
    void Line (const char * ActivityString, float FloatData1, 
                                             float FloatData2 = 0.0, 
                                             float FloatData3 = 0.0);
}; /* SerialDebug */

SerialDebug * Debug; 


#ifdef DEBUGON
  #define DEBUG(...) Debug->Line (__VA_ARGS__)
#else
  #define DEBUG(...)
#endif


/*******************************************************************************
*                                                                              *
*                  SerialDebug::Line (char *, char *, varargs)                 *
*                                                                              *
*******************************************************************************/

void SerialDebug::Line (const char * ActivityString, const char * Format, ...)
{
  char IntDebugStr [MAXDEBUGSTRLEN];
  va_list args;

  // if no format then just write out the activity string
  if ((Format == NULL) || (Format [0] == 0))
  {
    Serial.print (millis()); Serial.print (" ");
    Serial.println (ActivityString);
    return;
  }

  // create the debug string
  va_start (args, Format);
  vsnprintf (IntDebugStr, MAXDEBUGSTRLEN, Format, args);
  IntDebugStr [sizeof (IntDebugStr) - 1] = 0;
  va_end (args);
  
  // write out the debug string
  va_start (args, Format);
  Serial.print (millis()); Serial.print (" ");
  Serial.print (ActivityString); Serial.print (": ");
  Serial.println (IntDebugStr);  
  va_end (args);
  
} /* SerialDebug::Line (char *, char *, ...) */



/*******************************************************************************
*                                                                              *
*                SerialDebug::Line (char *, float, float, float)               *
*                                                                              *
*******************************************************************************/

void SerialDebug::Line (const char * ActivityString, float FloatData1, 
                                            float FloatData2, float FloatData3)
{
  char IntDebugStr [MAXDEBUGSTRLEN];
  char IntDebugStr1 [MAXDEBUGSTRLEN];
  char IntDebugStr2 [MAXDEBUGSTRLEN];
  char IntDebugStr3 [MAXDEBUGSTRLEN];

  // create the debug string - sprintf of float fails - dtostrf workaround
  dtostrf (FloatData1, 11, 5, IntDebugStr1);
  dtostrf (FloatData2, 11, 5, IntDebugStr2);
  dtostrf (FloatData3, 11, 5, IntDebugStr3);
  snprintf (IntDebugStr, MAXDEBUGSTRLEN, "%s: %s %s %s", ActivityString, 
               trim (IntDebugStr1), trim (IntDebugStr2), trim (IntDebugStr3));
  
  // write out the debug string
  Serial.print (millis()); Serial.print (" ");
  Serial.println(IntDebugStr);  

} /* SerialDebug::Line (char *, float, float, float) */



/*******************************************************************************
*                                                                              *
*                          SerialDebug::SerialDebug                            *
*                                                                              *
*       SerialDebug debug class constructor- initialise USB serial and         *
*       write a start line.                                                    *
*                                                                              *
*******************************************************************************/

SerialDebug::SerialDebug (const char * DebugString)
{
  // initialize serial communication at 115200 bits per second which is 
  // max for communication with IDE serial monitor
  Serial.begin(SERIALBAUDRATE);

  Line ("");
  Serial.print (millis()); Serial.print (" ");
  Line (DebugString);
  
} /* SerialDebug::SerialDebug */

                          

/*******************************************************************************
********************************************************************************
**                                                                            **
**                             Onboard LED Control                            **
**                                                                            **
**      Routines to turn the onboard LED (on pin 13) on and off.  Useful      **              
**      for debug but after the board is cased up, not so much...             **
**                                                                            **
********************************************************************************
*******************************************************************************/

class OnboardLEDControl 
{
  public:
    OnboardLEDControl (void);
    void On (void);
    void Off (void);
    void Blink (int Flashes = 1);

}; /* OnboardLEDControl */

OnboardLEDControl * OnboardLED; 



/*******************************************************************************
*                                                                              *
*                     OnboardLEDControl::OnboardLEDControl                     *
*                                                                              *
*       Initialise the onboard LED (which really means initialising the        *                                                                       
*       pin that it's connected to).                                           *
*                                                                              *
*******************************************************************************/

OnboardLEDControl::OnboardLEDControl (void)
{
  // initialize digital pin LED_BUILTIN as an output
  pinMode (LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);   
  DEBUG (FUNC);
  
} /* OnboardLEDControl::OnboardLEDControl */



/*******************************************************************************
*                                                                              *
*                           OnboardLEDControl::On                              *
*                                                                              *
*       Turn on the onboard LED.                                               *
*                                                                              *
*******************************************************************************/

void OnboardLEDControl::On (void)
{
  // turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_BUILTIN, HIGH);   
  DEBUG (FUNC);
  
} /* OnboardLEDControl::On */



/*******************************************************************************
*                                                                              *
*                           OnboardLEDControl::Off                             *
*                                                                              *
*       Turn off the onboard LED.                                              *
*                                                                              *
*******************************************************************************/

void OnboardLEDControl::Off (void)
{
  // turn the LED off by making the voltage LOW
  digitalWrite (LED_BUILTIN, LOW);    
  //DEBUG (FUNC);
  
} /* OnboardLEDControl::Off */



/*******************************************************************************
*                                                                              *
*                        OnboardLEDControl::Blink                              *
*                                                                              *
*       Flash the onboard LED the specified number of times.                   *
*                                                                              *
*******************************************************************************/

void OnboardLEDControl::Blink (int Flashes)
{
  for (int x = 0; x < Flashes; x++)
  {
    On ();
    delay (100);
    Off ();
    delay (100);
  }
  
} /* OnboardLEDControl::Blink */



/*******************************************************************************
********************************************************************************
**                                                                            **
**                             LCD Control                                    **
**                                                                            **
**      Class to write strings to the LCD screen.                             **
**                                                                            **
**      The LCD object does not seem to work unless it is declared as a       **
**      global variable - if created dynamically with new then the library    **
**      behaves erratically, for example losing track of how many lines it    **
**      has - this isn't how I would have implemented the LCD ideally...      **
**                                                                            **
********************************************************************************
*******************************************************************************/

rgb_lcd LCDLib;

class LCDInterface 
{
    // rgb_lcd * LCDLib;
  public:
    LCDInterface (void);
    void Clear (void);
    void Clear (int Line);
    void Line (const char * LCDString, ...);
    void Line (int Line, const char * LCDString);
    void Line (int Line, int IntData, const char * LCDString="");
    void Line (int Line, long LongData, const char * LCDString="");
    void Line  (int Line, float FloatData, const char * LCDString="");
    void Off (void);
    void On (void);
    
}; /* LCDInterface */



/*******************************************************************************
*                                                                              *
*                        LCDInterface::LCDInterface                            *
*                                                                              *
*       Initialise the LCD and clear it.                                       *                                                                              *
*                                                                              *                                                                              *
*******************************************************************************/

LCDInterface::LCDInterface (void)
{
  // set up the LCD's number of columns and rows; clear it
  LCDLib.begin (LCDCOLUMNS, LCDROWS);
  LCDLib.setRGB (255, 255, 255);
  LCDLib.clear ();

  DEBUG (FUNC, "%i columns, %i rows", LCDCOLUMNS, LCDROWS);
  
} /* LCDInterface::LCDInterface */



/*******************************************************************************
*                                                                              *
*                           LCDInterface::Clear                                *
*                                                                              *
*       Clear the specified line of the LCD.                                   *
*                                                                              *
*******************************************************************************/

void LCDInterface::Clear (void)
{
  // clear the LCD
  LCDLib.clear ();
  
  DEBUG (FUNC);

} /* LCDInterface::Clear */



/*******************************************************************************
*                                                                              *
*                             LCDInterface::Off                                *
*                                                                              *
*       Turn off the LCD.                                                      *
*                                                                              *
*******************************************************************************/

void LCDInterface::Off (void)
{
  // turn off the LCD
  LCDLib.noDisplay ();
  
  DEBUG (FUNC);

} /* LCDInterface::Off */



/*******************************************************************************
*                                                                              *
*                             LCDInterface::On                                 *
*                                                                              *
*       Turn on the LCD.                                                       *
*                                                                              *
*******************************************************************************/

void LCDInterface::On (void)
{
  // turn on the LCD
  LCDLib.display ();
  
  DEBUG (FUNC);

} /* LCDInterface::On */



/*******************************************************************************
*                                                                              *
*                        LCDInterface::Clear (int)                             *
*                                                                              *
*       Clear the specified line of the LCD.                                   *
*                                                                              *
*******************************************************************************/

void LCDInterface::Clear (int Line)
{
  // don't try to write off the bottom of the LCD
  if (Line >= LCDROWS) return;
  
  // write LEDCOLUMNS spaces to the line
  LCDLib.setCursor (0, Line);
  for (int x = 0; x < LCDCOLUMNS; x++) LCDLib.print (' ');

  DEBUG (FUNC, "line %i", Line);

} /* LCDInterface::Clear */



/*******************************************************************************
*                                                                              *
*                          LCDInterface::Line (char *)                         *
*                                                                              *
*       Write a message to the LCD; use printf style arguments to build the    *
*       line.  Wrap the message across multiple lines if necessary.            *
*                                                                              *
*******************************************************************************/

void LCDInterface::Line (const char * LCDStringParam, ...)
{
  char LCDString [LCDROWS*(LCDCOLUMNS+1)];
  char LCDWorkString [LCDROWS][LCDCOLUMNS+1];
  int line;
  int length;
  va_list args;
  char * StartPosition;

  DEBUG (FUNC, "\"%s\"", LCDStringParam);
  
  // clear the LCD; saves blanking bits of the LCD selectively later
  LCDLib.clear ();

  // return if there's nothing worth writing
  if ((LCDStringParam == NULL) || (LCDStringParam [0]== '\0')) return;

  // apply formatting (there may be none; if there isn't it'll be ok)
  va_start (args, LCDStringParam);
  vsnprintf (LCDString, sizeof (LCDString), LCDStringParam, args);
  LCDString [sizeof (LCDString) - 1] = 0;
  va_end (args);    

  DEBUG (FUNC, "\"%s\"", LCDString);

  // clear the work array
  StartPosition = (char *) LCDString;
  for (line = 0; line < LCDROWS; line++) LCDWorkString [line][0] = '\0';

  // work through the lines of the work string array
  for (line = 0; line < LCDROWS; line++) 
  {
    // step over white space till we find a character or a null
    while ((*StartPosition != '\0') && (isspace (*StartPosition))) 
      StartPosition++;

    // if there are less than LCDCOLUMNS left, copy to the current line and stop
    if (strlen (StartPosition) <= LCDCOLUMNS)
    {
      strcpy (LCDWorkString [line], StartPosition);
      break;
    }

    // find a good end of line; start off at current position + LCDCOLUMNS and 
    // look for a space, copy that many characters or LCDCOLUMNS worth if we 
    // can't find one
    for (length = LCDCOLUMNS; length > 0; length--)
      if (isspace (StartPosition [length])) break;
    if (length <= 0) length = LCDCOLUMNS;

    // copy the characters and null terminate
    strncpy (LCDWorkString [line], StartPosition, length);
    LCDWorkString [line][length] = '\0';

    StartPosition += length;
  }
  
  // write the display lines, starting at column zero
  for (line = 0; line < LCDROWS; line++) 
  {
    LCDLib.setCursor (0, line);
    LCDLib.print (LCDWorkString [line]);
    DEBUG (FUNC, "line %i, \"%s\"", line, LCDWorkString [line]);
  }

} /* LCDInterface::Line */



/*******************************************************************************
*                                                                              *
*                         LCDInterface::Line (int, char *)                     *
*                                                                              *
*       Write a message to the specified line of the LCD.                      *
*                                                                              *
*******************************************************************************/

void LCDInterface::Line (int Line, const char * LCDString)
{
  int CharsWritten; 
  
  // don't try to write off the bottom of the LCD
  if (Line >= LCDROWS) return;

  // write the display line, starting at column zero
  LCDLib.setCursor (0, Line);
  CharsWritten = LCDLib.print (LCDString); 

  // blank off the end by writing spaces to the end of line
  for (int x = CharsWritten; x < LCDCOLUMNS; x++) LCDLib.print (' ');
  
  DEBUG (FUNC, "line %i, \"%s\", chars written %i", Line, LCDString, CharsWritten);

} /* LCDInterface::Line */



/*******************************************************************************
*                                                                              *
*                   LCDInterface::Line (int, int, char *)                      *
*                                                                              *
*       Write a message to the specified line of the LCD.  The message is      *                   
*       an integer followed by a string (typically units etc).                 *
*                                                                              *
*******************************************************************************/

void LCDInterface::Line (int Line, int IntData, const char * LCDString)
{
  int CharsWritten; 
  
  // don't try to write off the bottom of the LCD
  if (Line >= LCDROWS) return;

  // write the display line, starting at column zero
  LCDLib.setCursor (0, Line);
  CharsWritten = LCDLib.print (IntData); 
  CharsWritten += LCDLib.print (LCDString); 

  // blank off the end by writing spaces to the end of line
  for (int x = CharsWritten; x < LCDCOLUMNS; x++) LCDLib.print (' ');
  
  DEBUG (FUNC, "line %i, %i \"%s\"", Line, IntData, LCDString);

} /* LCDInterface::Line (int, int, char *) */



/*******************************************************************************
*                                                                              *
*                   LCDInterface::Line (int, int, char *)                      *
*                                                                              *
*       Write a message to the specified line of the LCD.  The message is      *                   
*       a long integer followed by a string (typically units etc).             *
*                                                                              *
*******************************************************************************/

void  LCDInterface::Line (int Line, long LongData, const char * LCDString)
{
  int CharsWritten; 
  
  // don't try to write off the bottom of the LCD
  if (Line >= LCDROWS) return;

  // write the display line, starting at column zero
  LCDLib.setCursor (0, Line);
  CharsWritten = LCDLib.print (LongData); 
  CharsWritten += LCDLib.print (LCDString); 

  // blank off the end by writing spaces to the end of line
  for (int x = CharsWritten; x < LCDCOLUMNS; x++) LCDLib.print (' ');
  
  DEBUG (FUNC, "line %i, \"%s\", %l", Line, LCDString, LongData);

} /* WriteLCDLineLong */



/*******************************************************************************
*                                                                              *
*                   LCDInterface::Line (int, float, char *)                    *
*                                                                              *
*       Write a message to the specified line of the LCD.  The message is      *                   
*       a float followed by a string (typically units etc).                    *
*                                                                              *
*******************************************************************************/

void LCDInterface::Line  (int Line, float FloatData, const char * LCDString)
{
  char LCDStringInt [LCDCOLUMNS+1]; 
  char LCDFloatString [LCDCOLUMNS+1]; 
  int CharsWritten; 
    
  // don't try to write off the bottom of the LCD
  if (Line >= LCDROWS) return;

  // build the display line; make sure that it's null terminated
  dtostrf(FloatData, 8, 4, LCDFloatString); //sprintf of float fails -workaround
  trim (LCDFloatString);

  // write the display line, starting at column zero
  LCDLib.setCursor (0, Line);
  CharsWritten = LCDLib.print (LCDFloatString); 
  CharsWritten += LCDLib.print (LCDString); 

  // blank off the end by writing spaces to the end of line
  for (int x = strlen(LCDString); x < LCDCOLUMNS; x++) LCDLib.print (' ');
  
  DEBUG (FUNC, "%s, \"%s\"", LCDFloatString, LCDString);

} /* LCDInterface::Line (int, float, char *) */



/*******************************************************************************
********************************************************************************
**                                                                            **
**                          Complex LCD Messages                              **
**                                                                            **
**      Screen message processing that are more complicated than a simple     **
**      line of text.                                                         **
**      The routines also implement a delay mechanism that holds a message    **
**      on the screen to give the user a chance to read it, but at the same   **
**      time not delay the usage of the scale, even if the user instructions  **
**      haven't been displayed yet.                                           **
**      This should have inherited the LCDInterface and LCDLib classes but    **
**      I couldn't make it work and couldn't figure out why...                **
**                                                                            **
********************************************************************************
*******************************************************************************/

class DisplayClass : public LCDInterface
{
    unsigned long DisplayDelay;         // delay until prompt displayed 
    float ConvertForceToSpine (float Force, int SpineMethodology,
                                                               bool Mode23Inch);
  public:
    DisplayClass (void);
    void LazyUpdate (bool Force = false);
    void DelayTheReadyMessage (void);
    void WriteSpineOnScreen (float Force, int SpineMethodology, bool Mode23Inch);
    void WriteWeightOnScreen (float Force, float CoG, int ArrowLength16ths, 
                                                               bool Mode23Inch);
    void WriteAverageSpineOnScreen (float AverageForce, int SpineMethodology, 
                                float MaxForce, int Direction, bool Mode23Inch);
};

DisplayClass * LCD ;


/*******************************************************************************
*                                                                              *
*                          DisplayClass::DisplayClass                          *
*                                                                              *
*       Class initialisation.                                                  *
*                                                                              *
*******************************************************************************/

DisplayClass::DisplayClass (void)
{

  DisplayDelay = 0;
  
} /* DisplayClass::DisplayClass */



/*******************************************************************************
*                                                                              *
*                                LazyUpdate                                    *
*                                                                              *
*       Routine to write the spine measuring instructions.  It is not          *
*       written immediately; writing is delayed to allow user to read the      *
*       previous message.                                                      *
*                                                                              *
*******************************************************************************/

void DisplayClass::LazyUpdate (bool Force)
{ 
  // if the force flag is set then the ready message is written immediately
  if (Force == true)
  {
    LCD->Line (READYMESSAGE);
    DisplayDelay = 0;
    DEBUG (FUNC, "force %i, delay %u", (int)Force, DisplayDelay);
    return;    
  }

  // if there isn't a message waiting to be written then do nothing
  if (DisplayDelay == 0)
    return;

  // if there is a message waiting then write the message if the delay
  // time has elapsed
  if (millis() > DisplayDelay)
  {
    LCD->Line (READYMESSAGE);
    DisplayDelay = 0;
    DEBUG (FUNC, "force %i, delay %u", (int)Force, DisplayDelay);
  }

} /* DisplayClass::LazyUpdate */



/*******************************************************************************
*                                                                              *
*                    DisplayClass::DelayTheReadyMessage                        *
*                                                                              *
*       Routine to set the delay before Ready writes the user instructions.    *
*                                                                              *
*******************************************************************************/

void DisplayClass::DelayTheReadyMessage (void)
{
  // work out the time at which the ready message will be displayed; remember it
  DisplayDelay = millis() + DISPLAYDELAY;

  DEBUG (FUNC, "delay %u", DisplayDelay);

} /* DisplayClass::DelayTheReadyMessage */



/*******************************************************************************
*                                                                              *
*                    DisplayClass::ConvertForceToSpine                         *
*                                                                              *
*       Routine to convert grams force to the appropriate spine reading.       *
*       There's a max spine value but this is really just enforced for         * 
*       display convenience.                                                   *
*                                                                              *
*******************************************************************************/

float DisplayClass::ConvertForceToSpine (float Force, int SpineMethodology, 
                                                               bool Mode23Inch)
{
  float ASTMSpine;
  float Spine;

  DEBUG (FUNC, Force, (float) SpineMethodology);

  // convert the grams force to an ASTM spine as most spines start from this
  if (Force <= 0)
    ASTMSpine = MAXSPINE;
  else
    //Apply or not the 23" factor
    if (Mode23Inch)
      ASTMSpine = (float) (ASTMFACTOR / Force * ATASPINEFACTORMEASUREDAT23);
    else
      ASTMSpine = (float) (ASTMFACTOR / Force);
      
  // switch according to what kind of spine is to be displayed
  switch (SpineMethodology)
  {
    // AMO spine - a simple multiplier converts from ASTM
    case AMOAT26:
      if (ASTMSpine >= MAXSPINE)
        Spine =  MAXSPINE;
      else if (ASTMSpine <= 0)
        Spine =  0;
      else
        Spine = constrain (ASTMSpine * AMOMULTIPLIER, 0, MAXSPINE);
      break;
                      
    // the bow poundage is a conversion to AMO as above then 26/AMO to give lbs
    case AMOLBSAT26:
      if (ASTMSpine >= MAXSPINE)
        Spine =  MAXSPINE;
      else if (ASTMSpine <= 0)
        Spine =  0;
      else
      {
        Spine = constrain ((float)ASTMSpine * AMOMULTIPLIER, 0, MAXSPINE);
        Spine = 26.0 / (Spine / 1000.0);
      }
      break;

    // grams force is just a range check
    case GRAMSAT28:
      Spine = constrain (Force, 0, MAXSPINE);
      break;
                      
    // ASTM spine - this needs no further conversion, just range check
    case ASTMAT28:
    default:
      Spine = constrain (ASTMSpine, 0, MAXSPINE);
      break;
  }

  DEBUG (FUNC, ASTMSpine, Spine, Mode23Inch);

  return (Spine);

} /* DisplayClass::ConvertForceToSpine */



/*******************************************************************************
*                                                                              *
*                      DisplayClass::WriteSpineOnScreen                        *
*                                                                              *
*       Take the supplied force on the scale and convert that value to the     *
*       appropriate spine type.  Write "Spine:" to the top line of the LCD     *
*       and the calculated arrow spine on line 2 of the screen along with      *
*       the spine reading methodology description.  This screen is laid out    *
*       such that when it's updated things don't jink about on the screen.     *
*       This routine is the place that knows about spine methodology           *
*       conversions; the rest of the program all works in grams as a           *
*       standard to save multiple conversions and loss of accuracy.            *
*                                                                              *
*******************************************************************************/

void DisplayClass::WriteSpineOnScreen (float Force, int SpineMethodology, bool Mode23Inch)
{
  char SpineString [100];
  char WorkString [100];
  int SpineLen;
  int SpineMethLen;
  float Spine;

  DEBUG (FUNC, Force, (float) SpineMethodology);

  // convert the force to a spine
  Spine = ConvertForceToSpine (Force, SpineMethodology, Mode23Inch);

  // write out the first line - just a title
  LCD->Line (0, ARROWSPINE);
  
  // limit the spine to valid values; if it's too big or too small then 
  // set to a blank, otherwise convert to a number
  if ((Spine >= MAXSPINE) || (Spine <= 0))
    strcpy (SpineString, " ");
  else
    strcpy (SpineString, itoa (Spine, WorkString, 10));
  
  // add the units
  strcat (SpineString, SpineMethodologyString (SpineMethodology, SPINEUNITS));

  // add the spine methodology to the right of the line
  SpineLen = strlen (SpineString);
  SpineMethLen = strlen (SpineMethodologyString (SpineMethodology, 
                                                         SHORTSPINESTRING));
  
  for (int x = 0; x < (LCDCOLUMNS - SpineLen - SpineMethLen - 2); x++) 
    strcat (SpineString, " ");
  strcat (SpineString, "(");
  strcat (SpineString, SpineMethodologyString (SpineMethodology, 
                                                         SHORTSPINESTRING));
  strcat (SpineString, ")");
  
  LCD->Line (1, SpineString);

  DisplayDelay = 0;

} /* DisplayClass::WriteSpineOnScreen */



/*******************************************************************************
*                                                                              *
*                      DisplayClass::WriteWeightOnScreen                       *
*                                                                              *
*       Take the supplied force on the scale and display it as weight.         *
*       Take the centre of gravity position (in mm from the back end), and     * 
*       the length of the arrow (in sixteenths of an inch), calculate the      *
*       centre of gravity position as a percentage of the arrow length         *
*       forward of the centre of the arrow (the front of centre position)      *
*       and display it.                                                        *
*                                                                              *
*******************************************************************************/

void DisplayClass::WriteWeightOnScreen (float Force, float CoG, 
                                        int ArrowLength16ths, bool Mode23Inch)
{
  float HalfSupportSpacing = (SUPPORTSPACING28/2);
  float SupportSpacing = SUPPORTSPACING28;
  char DisplayString [100];
  char WorkString [100];
  int Len;
  int ArrowLengthmm;
  float FoC;

  DEBUG (FUNC, Force, CoG);

  if (Mode23Inch)
  {
    HalfSupportSpacing = (SUPPORTSPACING23/2);
    SupportSpacing = (SUPPORTSPACING23);  
  }
  
  // limit the force to valid values; if it's too big or too small then 
  // set to a blank, otherwise convert to a number string
  if ((Force >= MAXWEIGHT) || (Force <= 0))
    strcpy (WorkString, " ");
  else
    itoa (round (Force * GRAMTOGRAINS), WorkString, 10);
  
  // add the units
  strcat (WorkString, GRAINSUNITS);

  // construct the full line
  strcpy (DisplayString, WEIGHT);
  Len = strlen (DisplayString) + strlen (WorkString);
  for (int x = Len; x < LCDCOLUMNS; x++) 
    strcat (DisplayString, " ");
  strcat (DisplayString, WorkString);

  // write out the first line containing the weight
  LCD->Line (0, DisplayString);

  // swap the centre of gravity round if it's less than half of
  // the arrow length - the front of centre is always in the front
  // half of the arrow
  if (CoG < HalfSupportSpacing)
    CoG = (HalfSupportSpacing - CoG) + HalfSupportSpacing;

  // if there's a reasonable centre of gravity then display it
  if ((CoG < 10) || (CoG > (SupportSpacing - 10)))
    LCD->Clear (1);
  else  
  {
    // convert arrow lengths in 1/16" to mm
    ArrowLengthmm = ArrowLength16ths * SIXTEENTHSTOMM;

    // calculate centre of gravity front of centre position (as a %)
    FoC = (CoG - (ArrowLengthmm / 2)) / ArrowLengthmm * 100;
    
    // convert to a number string
    itoa (round (FoC), WorkString, 10);
    
    // add the units
    strcat (WorkString, FOCUNITS);
  
    // construct the full line
    strcpy (DisplayString, FOC);
    Len = strlen (DisplayString) + strlen (WorkString);
    for (int x = Len; x < LCDCOLUMNS; x++) 
      strcat (DisplayString, " ");
    strcat (DisplayString, WorkString);
  
    // write out the second line containing the centre of gravity
    LCD->Line (1, DisplayString);
  }

} /* DisplayClass::WriteWeightOnScreen */



/*******************************************************************************
*                                                                              *
*                  DisplayClass::WriteAverageSpineOnScreen                     *
*                                                                              *
*       Take the supplied force on the scale and convert that value to the     *
*       appropriate spine type.  Write that, the error percent and the         *
*       direction to the display.  Don't bother trying to position the         *
*       elements on the screen like WriteSpineOnScreen does as this message    *
*       display is not constantly updated so there's no advantage to it.       *
*                                                                              *
*******************************************************************************/

void DisplayClass::WriteAverageSpineOnScreen (float AverageForce, 
                      int SpineMethodology, float MaxForce, int Direction, bool Mode23Inch)
{
  char SpineString [100];
  char MaxSpineString [100];
  char WorkString [100];
  int SpineLen;
  int SpineMethLen;
  float Spine;
  float MaxSpine;

  DEBUG (FUNC, AverageForce, MaxForce, (float) SpineMethodology);
  
  // convert the forces to spines
  Spine = ConvertForceToSpine (AverageForce, SpineMethodology, Mode23Inch);
  MaxSpine = ConvertForceToSpine (MaxForce, SpineMethodology, Mode23Inch);

  // limit the spines to valid values; if too big or too small then 
  // set to a blank, otherwise convert to a number
  if ((Spine >= MAXSPINE) || (Spine <= 0))
    strcpy (SpineString, " ");
  else
    strcpy (SpineString, itoa (round (Spine), WorkString, 10));
  if ((MaxSpine >= MAXSPINE) || (MaxSpine <= 0))
    strcpy (MaxSpineString, " ");
  else
    strcpy (MaxSpineString, itoa (round (MaxSpine), WorkString, 10));

  LCD->Line (AVERAGESPINESTRING, SpineString, 
                                    SpineMethodologyString (SpineMethodology, 
                                    SPINEUNITS), MaxSpineString, 
                                    SpineMethodologyString (SpineMethodology, 
                                    SPINEUNITS), Direction, DEGREESYMBOL);
  DisplayDelay = 0;

} /* DisplayClass::WriteAverageSpineOnScreen */



/*******************************************************************************
********************************************************************************
**                                                                            **
**                             Button Control                                 **
**                                                                            **
**      Routines to poll the pins to which the buttons are connected, and     **
**      debounce them.  Debouncing is done by Bounce2 library but to be       **                                                         
**      honest I'm not convinced that it's getting to do much work as the     **
**      polling happens so infrequently as the HX711 hogs so much of the      **
**      processing time.                                                      **
**                                                                            **
********************************************************************************
*******************************************************************************/

#define DEBOUNCEDELAY 5              // 5 ms debounce interval
#define LONGPRESSTIME 1000           // press this long for alternate function

class ButtonControl 
{
    Bounce OkButton = Bounce();
    Bounce SpineButton = Bounce();
    Bounce CalibrateButton = Bounce();
    bool Mode23 = false;
  public:
    ButtonControl (void);
    int Poll (void);
    bool Get23InchMode(void);
    bool WaitFor (int WaitForButton); 
    bool Init23InchModeFlag(void);

}; /* ButtonControl */

ButtonControl * Buttons; 



/*******************************************************************************
*                                                                              *
*                       ButtonControl::ButtonControl                           *
*                                                                              *
*       Initialise the analog pin that the buttons are connected to.           *
*                                                                              *
*******************************************************************************/

ButtonControl::ButtonControl (void)
{
  // set up pins for input buttons, turn on pull up resistor
  OkButton.attach (BUTTON_OK_PIN, INPUT_PULLUP); 
  OkButton.interval (DEBOUNCEDELAY);
  CalibrateButton.attach (BUTTON_CALIBRATE_PIN, INPUT_PULLUP); 
  CalibrateButton.interval(DEBOUNCEDELAY);
  SpineButton.attach (BUTTON_SPINE_PIN, INPUT_PULLUP);
  SpineButton.interval (DEBOUNCEDELAY);
  
  DEBUG (FUNC);

} /* ButtonControl::ButtonControl */

/*******************************************************************************
*                                                                              *
*                       ButtonControl::Init23InchModeFlag                      *
*                                                                              *
*       Read the switch on SWITCH23INCHMODE PIN and set the bool Mode23 prop.  *
*                                                                              *
*******************************************************************************/

boolean ButtonControl::Init23InchModeFlag (void)
{
  //Bounce is declared within this method cause he has to be destroyed when get out the scope. 
  Bounce Switch23mode = Bounce();
  Switch23mode.attach (SWITCH23INCHMODE, INPUT_PULLUP);
  Switch23mode.interval (DEBOUNCEDELAY);
  
  //the state of the Switch that indicate if we are mounted in 23" or 28" mode
  Switch23mode.update();
  if (Switch23mode.read() == HIGH)
    Mode23 = true;
  else
    Mode23 = false;

  DEBUG (FUNC, Mode23);
  
  return Mode23;
  
} /* ButtonControl::ButtonControl */

/*******************************************************************************
*                                                                              *
*                       ButtonControl::Get23InchMode                           *
*                                                                              *
*       Return true if the switch is ON on the SWITCH23INCHMODE PIN.           *
*                                                                              *
*******************************************************************************/

boolean ButtonControl::Get23InchMode (void)
{
  DEBUG (FUNC, Mode23);

  //the state of the Switch that indicate if we are mounted in 23" or 28" mode
  return Mode23;
  
} /* ButtonControl::ButtonControl */


/*******************************************************************************
*                                                                              *
*                           ButtonControl::Poll                                *
*                                                                              *
*******************************************************************************/

int ButtonControl::Poll (void) 
{
  
  int Button = BUTTON_NONE;  

  // poll the ok button - keep this responsive by reacting as soon as it
  // is pressed - the ok button doesn't have an alternate function
  OkButton.update ();
  if (OkButton.changed () && (OkButton.read () == LOW)) 
  {
    DEBUG (FUNC, "button %i", BUTTON_1_OK);
    return (BUTTON_1_OK);
  }  

  // poll the spine button - this can have a short or a long press so
  // we return appropriate flag according to how long it was pressed
  SpineButton.update ();
  if (SpineButton.changed ()) 
  {
    if ((SpineButton.read () == HIGH))
    {
      if (SpineButton.previousDuration() > LONGPRESSTIME) 
        Button = BUTTON_2LONG_SPINE_UP;
      else
        Button = BUTTON_2_SPINE_UP;
    }
    DEBUG (FUNC, "button %i, press time %u", Button, 
                                            SpineButton.previousDuration());
    return (Button);
  }  

  // poll the calibrate button - this can have a short or a long press so
  // we return appropriate flag according to how long it was pressed
  CalibrateButton.update ();
  if (CalibrateButton.changed ()) 
  {
    if ((CalibrateButton.read () == HIGH))
    {
      if (CalibrateButton.previousDuration() > LONGPRESSTIME) 
        Button = BUTTON_3LONG_CALIBRATE_BIGUP;
      else
        Button = BUTTON_3_CALIBRATE_UP;
    }
    DEBUG (FUNC, "button %i, press time %u", Button, 
                                        CalibrateButton.previousDuration());
    return (Button);
  }  

  DEBUG (FUNC, "button %i", Button);
  return (Button);

} /* ButtonControl::Poll */



/*******************************************************************************
*                                                                              *
*                           ButtonControl::WaitFor                             *
*                                                                              *
*       Wait for the specified button.  Return true if it was pressed or       * 
*       false if anything else was pressed.                                    *
*                                                                              *
*******************************************************************************/

bool ButtonControl::WaitFor (int WaitForButton)
{
  int Button = BUTTON_NONE;  

  DEBUG (FUNC, "wait flag %i", WaitForButton);

  // wait for button to be pressed
  do
  {
    // check if we need to sleep
    CheckForSleep (false);
    
    Button = Poll ();
  }
  while (Button == BUTTON_NONE);

  DEBUG (FUNC, "wait for %i, ok = %i", WaitForButton, 
                                             (int)(Button == WaitForButton));
                                             
  // return flag to say if the right button was pressed
  return (Button == WaitForButton);
  
} /* ButtonControl::WaitFor */



/*******************************************************************************
********************************************************************************
**                                                                            **
**                             EEPROM Control                                 **
**                                                                            **
**      Routines to store and retrieve non-volatile data in EEPROM.  So as    **     
**      to not wear the EEPROM out the data is only written out if it has     **    
**      remained stable for a minute or so.  This saves constantly writing    **
**      when, for example, cycling through options.                           **
**      The routines check that the data is for the correct program and the   **
**      correct version.                                                      **
**                                                                            **
********************************************************************************
*******************************************************************************/

#define MAXTITLEVERSION  20

typedef struct {

  char ProgName [MAXTITLEVERSION];
  char ProgVersion [MAXTITLEVERSION];
  int SpineMethodology = DEFAULTSPINEMETHOD;     // 28" is the standard 
  float ScaleCalibration [CELLS];
  int ArrowLength16ths;
  
} EEPROMStruct;

class EEPROMControl 
{
    EEPROMStruct EEPROMData;            // data to read and write to/form EEPROM
    unsigned long WriteDelay;           // when to write the data out to EEPROM
  public:
    int SpineMethodology;               // type of spine being calculated
    float ScaleCalibration [CELLS];     // the calibration of the scale
    int ArrowLength16ths;               // arrow length in 1/16ths of an inch
    EEPROMControl (void);
    void LazyUpdate (void);

}; /* EEPROMControl */

EEPROMControl * NVM; 



/*******************************************************************************
*                                                                              *
*                       EEPROMControl::EEPROMControl                           *
*                                                                              *
*       Either initialise the EEPROM, or if it has valid data in it read       *
*       it out.                                                                *
*                                                                              *
*******************************************************************************/

EEPROMControl::EEPROMControl (void)
{
  unsigned char * p;
  int x;

  WriteDelay = 0;
  
  // read the current data from EEPROM
  EEPROM.get (0, EEPROMData);

  // see if there is anything there and if so that the version is ok too
  if ((strncmp (EEPROMData.ProgName, ARROWSCALETITLE, 
                                      sizeof (EEPROMData.ProgName)) == 0) &&
                         (strncmp (EEPROMData.ProgVersion, ARROWSCALEVERSION, 
                                      sizeof (EEPROMData.ProgVersion)) == 0))
  {
    // set the externally visible data from EEPROM
    SpineMethodology = EEPROMData.SpineMethodology;
    for (int Cell = 0; Cell < CELLS; Cell++)
      ScaleCalibration [Cell] = EEPROMData.ScaleCalibration [Cell];                      
    ArrowLength16ths = EEPROMData.ArrowLength16ths;
  }
  else
  {
    // set the title and version
    strncpy (EEPROMData.ProgName, ARROWSCALETITLE, sizeof(EEPROMData.ProgName));
    strncpy (EEPROMData.ProgVersion, ARROWSCALEVERSION, 
                                            sizeof (EEPROMData.ProgVersion));
    
    // make sure that the spine method is in range
    EEPROMData.SpineMethodology = SpineMethodology = DEFAULTSPINEMETHOD;

    // set the calibration to the default
    for (int Cell = 0; Cell < CELLS; Cell++)
      EEPROMData.ScaleCalibration [Cell] = ScaleCalibration [Cell] 
                                                       = DEFAULTCALIBRATION;

    // initialise arrow to good initial value - this is in eigths of an inch
    EEPROMData.ArrowLength16ths = ArrowLength16ths = DEFAULTARROWLENGTH;

    // write out the structure to EEPROM
    for (x = 0, p = (unsigned char *) &EEPROMData; x < sizeof (EEPROMData); 
                                                                     x++, p++)
      EEPROM.update (x, *p);
  }

  DEBUG (FUNC, "spine methodology %i", SpineMethodology);
  DEBUG (FUNC, ScaleCalibration [0], ScaleCalibration [1]);

} /* EEPROMControl::EEPROMControl */



/*******************************************************************************
*                                                                              *
*                       EEPROMControl::LazyUpdate                              *
*                                                                              *
*       This routine writes constants out to EEPROM.  EEPROM has limited       *                                                                
*       life WRT writes so it watches for changes but does not write them      *
*       out until after a delay; there's no point changing a value if it's     *
*       going to change again soon.                                            *
*                                                                              *
*******************************************************************************/

void EEPROMControl::LazyUpdate (void)
{
  unsigned char * p;
  int x;
  bool Change = false; 

  // see if any of the values have changed then update the internal values
  if (SpineMethodology != EEPROMData.SpineMethodology) 
  {
    EEPROMData.SpineMethodology = SpineMethodology;
    Change = true;                      
  }
  for (int Cell = 0; Cell < CELLS; Cell++)
  {
    if (ScaleCalibration [Cell] != EEPROMData.ScaleCalibration [Cell])
    {
      EEPROMData.ScaleCalibration  [Cell]= ScaleCalibration [Cell];
      Change = true;                      
    }
  }
  if (ArrowLength16ths != EEPROMData.ArrowLength16ths) 
  {
    EEPROMData.ArrowLength16ths = ArrowLength16ths;
    Change = true;                      
  }

  // if something changed then note when to update EEPROM and quit
  if (Change)
  {
    WriteDelay = millis() + EEPROMWRITEDELAY; 
    return;                      
  }

  // if we have a delay time set up then evaluate it to see if it has elapsed
  if (WriteDelay != 0)
  {
    if (millis() > WriteDelay)
    {
      // write out the structure to EEPROM
      for (x = 0, p = (unsigned char *) &EEPROMData; x < sizeof (EEPROMData); 
                                                                    x++, p++)
        EEPROM.update (x, *p);
      WriteDelay = 0;    
      DEBUG (FUNC, "spine methodology %i", SpineMethodology);
      DEBUG (FUNC, ScaleCalibration [0], ScaleCalibration [1]);
    }
  }
  
} /* EEPROMControl::LazyUpdate */



/*******************************************************************************
********************************************************************************
**                                                                            **
**                        Arrow Scale and Spine Class                         **
**                                                                            **
**      Routines to weigh and calculate arrow spines.                         **
**                                                                            **
********************************************************************************
*******************************************************************************/

#define CELLREADS               12      // readings to average when reading
#define CELLREADSCALIBRATE      36      // readings to take when calibrating

// number of load cells and the pins they connect to
static int CellDOutPin [] = {LOADCELL1_DOUT_PIN, LOADCELL2_DOUT_PIN};
static int CellSckPin [] = {LOADCELL1_SCK_PIN, LOADCELL2_SCK_PIN};

class ArrowSpineClass 
{
    HX711 ArrowScale[CELLS];
    float Reading [CELLREADS][CELLS];
    int LastRead;
    void ZeroReadings (void);
  public:
    ArrowSpineClass (void);             // HX711 initialisations
    void ZeroArrowScale (void);
    void PowerDownArrowScale (void);
    void PowerUpArrowScale (void);
    bool AutoCalibrateArrowScale (void);
    void Poll (void);
    float ReadArrowScale (void);
    float ReadArrowCOG (bool Mode23Inch);
    int GetArrowLength (void);
    
}; /* ArrowSpineClass */

ArrowSpineClass * ArrowSpine;



/*******************************************************************************
*                                                                              *
*                         ArrowSpineClass::ZeroReadings                        *
*                                                                              *
*       Zero the cell reading history.  Initialise counters for using it.      *
*                                                                              *
*******************************************************************************/

void ArrowSpineClass::ZeroReadings (void)
{
  // zero the cell reading history
  for (int Cell = 0; Cell < CELLS; Cell++)
    for (int Read = 0; Read < CELLREADS; Read++)
      Reading [Read][Cell] = 0;
  LastRead = CELLS-1;
    
} /* ArrowSpineClass::ZeroReadings */



/*******************************************************************************
*                                                                              *
*                      ArrowSpineClass::ArrowSpineClass                        *
*                                                                              *
*******************************************************************************/

ArrowSpineClass::ArrowSpineClass (void)
{
  DEBUG (FUNC);

  // set up all load cells
  for (int Cell = 0; Cell < CELLS; Cell++)
  {
    // initialise the pins connecting the HX711
    ArrowScale [Cell].begin (CellDOutPin [Cell], CellSckPin [Cell]);
    // set the calibration factor to make the load cell read in grams
    /*JNK*/
    ArrowScale [Cell].set_scale (NVM->ScaleCalibration [Cell]); 
    //ArrowScale [Cell].set_scale(900);
  }

  // zero history array
  ZeroReadings ();
  
  DEBUG (FUNC, NVM->ScaleCalibration [0], NVM->ScaleCalibration [1]);
  
} /* ArrowSpineClass */



/*******************************************************************************
*                                                                              *
*                                 ZeroArrowScale                               *
*                                                                              *
*******************************************************************************/

void ArrowSpineClass::ZeroArrowScale (void)
{
  DEBUG (FUNC);
  
  // zero the scales
  for (int Cell = 0; Cell < CELLS; Cell++)
    ArrowScale [Cell].tare (CELLREADS);
  
  // zero history array (clearing out any readings taken with old tare value)
  ZeroReadings ();

  DEBUG (FUNC);

} /* ArrowSpineClass::ZeroArrowScale */



/*******************************************************************************
*                                                                              *
*                               PowerDownArrowScale                            *
*                                                                              *
*******************************************************************************/

void ArrowSpineClass::PowerDownArrowScale (void)
{
  DEBUG (FUNC);
  
  // power down the scales - ie the HX711s
  for (int Cell = 0; Cell < CELLS; Cell++)
    ArrowScale [Cell].power_down ();
  
} /* ArrowSpineClass::PowerDownArrowScale */



/*******************************************************************************
*                                                                              *
*                                 PowerUpArrowScale                            *
*                                                                              *
*******************************************************************************/

void ArrowSpineClass::PowerUpArrowScale (void)
{
  DEBUG (FUNC);
  
  // power up the scales - ie the HX711s
  for (int Cell = 0; Cell < CELLS; Cell++)
    ArrowScale [Cell].power_up ();
  
} /* ArrowSpineClass::PowerUpArrowScale */



/*******************************************************************************
*                                                                              *
*                                 ReadArrowScale                               *
*                                                                              *
*       Gather up the HX711 force history of readings and average them.        *                                 
*       This is better than using the library's averaging code as that         *
*       code blocks while taking all the readings and kills operator           *
*       responsiveness.  It doesn't matter if older readings are zero as       *
*       the readings array will fill up over time and converge to a good       *
*       reading.                                                               *
*                                                                              *
*******************************************************************************/

float ArrowSpineClass::ReadArrowScale (void)
{
  float Total = 0;

  // average the cell reading history
  for (int Cell = 0; Cell < CELLS; Cell++)
    for (int Read = 0; Read < CELLREADS; Read++)
      Total += Reading [Read][Cell];
  Total = Total / (float) (CELLREADS);

  DEBUG (FUNC, Total);

  // return force on scale in grams
  return (Total);  
  
} /* ArrowSpineClass::ReadArrowScale */



/*******************************************************************************
*                                                                              *
*                                  ReadArrowCOG                                *
*                                                                              *
*       Gather up the HX711 force history of readings and average them,        *                                 
*       keeping the force averages for the two gauges separate.  Use the       *
*       forces on the two gauges to calculate the position of the centre       *
*       of gravity in millimetres in front of the first gauge.                 *
*                                                                              *
*******************************************************************************/

float ArrowSpineClass::ReadArrowCOG (bool Mode23Inch)
{
  float HalfSupportSpacing = (SUPPORTSPACING28/2);
  float Total [CELLS];
  float CoG;

  for (int Cell = 0; Cell < CELLS; Cell++) Total [Cell] = 0;

  // figure out support spacing
  if (Mode23Inch)
    HalfSupportSpacing = (SUPPORTSPACING23/2); 
  
  // average the cell reading history
  for (int Cell = 0; Cell < CELLS; Cell++)
    for (int Read = 0; Read < CELLREADS; Read++)
      Total [Cell] += Reading [Read][Cell];
  for (int Cell = 0; Cell < CELLS; Cell++)
    Total [Cell] = Total [Cell] / (float) (CELLREADS);

  // calculate the centre of gravity distance from support 1
  if ((Total[1] < 1) || (Total[0] < 1))
    CoG = 0;
  else
    CoG = (((Total[1] - Total[0]) * HalfSupportSpacing) / (Total[1] + Total[0]))
                                  + HalfSupportSpacing;

  DEBUG (FUNC, CoG, HalfSupportSpacing);

  // return position of COG in mm from the first support
  return (CoG);  
  
} /* ArrowSpineClass::ReadArrowCOG */



/*******************************************************************************
*                                                                              *
*                                     Poll                                     *
*                                                                              *
*       Do the periodic read of the HX711s and store the results in the        * 
*       results array.  The scaling has been set already during                *
*       intialisation or calibration so that the stored value is in grams.     *
*                                                                              *
*******************************************************************************/

void ArrowSpineClass::Poll (void)
{
  DEBUG (FUNC, "Position %i", LastRead);

  // increment the reading array position pointer
  LastRead++;
  if (LastRead >= CELLREADS) LastRead = 0;
  
  // read the scales - result will be in grams - limit reading to 0 or above
  for (int Cell = 0; Cell < CELLS; Cell++)
  {
    if (ArrowScale [Cell].wait_ready_timeout(1000)) 
    {
      Reading [LastRead][Cell] = ArrowScale [Cell].get_units (1);
    } 
    else 
    {
      DEBUG(FUNC, "HX711 not found.");
    }

    if (Reading [LastRead][Cell] < 0) Reading [LastRead][Cell] = 0;
  }

  DEBUG (FUNC, Reading [LastRead][0], Reading [LastRead][1]);

} /* ArrowSpineClass::Poll */



/*******************************************************************************
*                                                                              *
*                             AutoCalibrateArrowScale                          *
*                                                                              *
*       Calibrate the scale by reading the scale raw with a 1kg weight on      *
*       it then calculating the necessary multiplier to make the result        *     
*       1000 (grams).                                                          *                                                                       *
*                                                                              *
*******************************************************************************/

bool ArrowSpineClass::AutoCalibrateArrowScale (void)
{
  float Calibration;
  float ScaleReading;
  long ZeroFactor;
  int Weight;
  float Spine;
  
  DEBUG (FUNC);
  
  // clear the display
  LCD->Clear ();

  // zero history array
  ZeroReadings ();

  // calibrate all the cells
  for (int Cell = 0; Cell < CELLS; Cell++)
  {
    LCD->Line (CALIBRATINGGAUGE, Cell+1);
    delay (2000);

    // tell user to remove weight from scale and press ok; if something else
    // is pressed then abort the calibration
    LCD->Line (REMOVEWEIGHT);
    if (!Buttons->WaitFor (BUTTON_1_OK))
    {
      DEBUG (FUNC);
      return (false);
    }
  
    // zero the scale
    DEBUG (ZEROINGSCALE);
    LCD->Line (ZEROINGSCALE);
    ArrowScale [Cell].set_scale (1);
    ArrowScale [Cell].tare (CELLREADSCALIBRATE);
    ZeroFactor = ArrowScale [Cell].read_average (); 
    DEBUG (FUNC, "factor %l", ZeroFactor);
    LCD->Line (0, ZEROFACTOR); 
    LCD->Line (1, ZeroFactor);
    delay (2000);
  
    // tell user to place a 1kg weight on from scale and press ok; if 
    // something else is pressed then abort the calibration
    LCD->Line (PLACE1KGWEIGHT);
    if (!Buttons->WaitFor (BUTTON_1_OK))
    {
      DEBUG (FUNC);
      return (false);
    }
  
    // read initial scale value, calculate the scale factor that 
    // will transform that reading to 1000g (ie 1 kg)
    LCD->Line (CALIBRATINGSCALE);
    ScaleReading = ArrowScale [Cell].get_units(CELLREADSCALIBRATE);
    Calibration = ScaleReading / 1000;
    DEBUG (FUNC, Calibration, ScaleReading);
    LCD->Line (0, CALIBRATION);
    LCD->Line (1, Calibration);
    ArrowScale [Cell].set_scale (Calibration);
    NVM->ScaleCalibration [Cell] = Calibration;
    delay (2000);
    
    // check the result
    LCD->Line (CHECKINGSCALE);
    ScaleReading = ArrowScale [Cell].get_units(CELLREADSCALIBRATE);
    DEBUG (FUNC, ScaleReading);  
    LCD->Line (0, KILOGRAMREADING);
    LCD->Line (1, ScaleReading, GRAMS);  
    delay (2000);
  
    // tell user to remove weight on from scale and press ok; if 
    // something else is pressed then abort the calibration, but 
    // at this point it's finished anyway
    LCD->Line (REMOVEWEIGHT);
    if (!Buttons->WaitFor (BUTTON_1_OK))
    {
      DEBUG (FUNC);
      return (false);
    }
  }
  
  LCD->Line (CALIBRATIONCOMPLETE);
  DEBUG (FUNC);
  return (true);
  
} /* ArrowSpineClass::AutoCalibrateArrowScale */



/*******************************************************************************
*                                                                              *
*                                GetArrowLength                                *
*                                                                              *
*       Display the courrent arrow length in use (stored in EEPROM) and        *
*       allow the user to change the value and confirm.  The user is only      *
*       asked once on the first call; if the value needs to be changed         *
*       later in a session then the device must be turned off and on.          *
*       The arrow length is stored in 1/16ths of an inch although the rest     *
*       of the internals of the device are metric and converted only upon      *
*       use.  This stops creep of the value due to repeated conversions in     *
*       the UI.                                                                *
*                                                                              *
*******************************************************************************/

int ArrowSpineClass::GetArrowLength (void)
{
  static bool Asked = false;
  bool Done = false;
  char DisplayString [100];
  int Button = BUTTON_NONE;       // last button pressed
  
  
  DEBUG (FUNC, NVM->ArrowLength16ths);

  if (!Asked)
  {
    // display the top line asking for arrow length
    LCD->Line (0, ARROWLENGTH);
    
    do 
    {
      // display the current arrow length - stored in 1/16FFths inch
      itoa (round (NVM->ArrowLength16ths / 16), DisplayString, 10);
      switch (NVM->ArrowLength16ths % 16)
      {
        case (1):
          strcat (DisplayString, " 1/16");
          break;
          
        case (2):
          strcat (DisplayString, " 1/8");
          break;
          
        case (3):
          strcat (DisplayString, " 3/16");
          break;
          
        case (4):
          strcat (DisplayString, " 1/4");
          break;
          
        case (5):
          strcat (DisplayString, " 5/16");
          break;
          
        case (6):
          strcat (DisplayString, " 3/8");
          break;
          
        case (7):
          strcat (DisplayString, " 7/16");
          break;
          
        case (8):
          strcat (DisplayString, " 1/2");
          break;
          
        case (9):
          strcat (DisplayString, " 9/16");
          break;
          
        case (10):
          strcat (DisplayString, " 5/8");
          break;
          
        case (11):
          strcat (DisplayString, " 11/16");
          break;
          
        case (12):
          strcat (DisplayString, " 3/4");
          break;
          
        case (13):
          strcat (DisplayString, " 13/16");
          break;
          
        case (14):
          strcat (DisplayString, " 7/8");
          break;
          
        case (15):
          strcat (DisplayString, " 15/16");
          break;
      }
      strcat (DisplayString, "\"");;
      LCD->Line (1, DisplayString);

      // look to see if any of the buttons have been pressed - 
      // if so increase or decrease the arrow length until correct
      Button = Buttons->Poll ();
    
      switch (Button)
      {
        // we are happy with the current length
        case BUTTON_1_OK:
          Done = true;
          break;
        
        // increase the arrow length
        case BUTTON_3_CALIBRATE_UP:
          NVM->ArrowLength16ths += 1;        
          break;
       
        // increase the arrow length by a big margin (inch)
        case BUTTON_3LONG_CALIBRATE_BIGUP:
          NVM->ArrowLength16ths += 16;        
          break;
    
        // decrease the arrow length
        case BUTTON_2_SPINE_UP:
          NVM->ArrowLength16ths -= 1;        
          break;
          
        // decrease the arrow length by a big margin (inch)
        case BUTTON_2LONG_SPINE_UP:
          NVM->ArrowLength16ths -= 16;        
          break;
       
        case BUTTON_NONE:
        default:
           break;
      }

      // keep arrow length in range
      if (NVM->ArrowLength16ths < 1) NVM->ArrowLength16ths = 1;
    
    } while (!Done);

    // don't ask again
    Asked = true;
    
    // clear the display
    LCD->Clear ();
  }

  DEBUG (FUNC, NVM->ArrowLength16ths);
  return (NVM->ArrowLength16ths);
  
} /* ArrowSpineClass::GetArrowLength */



/*******************************************************************************
********************************************************************************
**                                                                            **
**               Arrow Scale Methodology Description Routines                 **
**                                                                            **
**      Routines to cycle round and describe arrow spine measuring            **
**      methods.                                                              **
**                                                                            **
********************************************************************************
*******************************************************************************/

/*******************************************************************************
*                                                                              *
*                            NextSpineMethodology                              *
*                                                                              *
*       Cycle around the spine reading methodologies implemented in the        *
*       spine tool S/W.                                                        *
*                                                                              *
*******************************************************************************/

int NextSpineMethodology (int CurrentMethodology)
{
  int NextMethodology;

  // cycle around the spine methodologies; return the next in sequence
  NextMethodology = CurrentMethodology + 1;
  if ((NextMethodology < 0) || (NextMethodology >= MAXSPINEMETHOD)) 
    NextMethodology = 0;

  DEBUG (FUNC, "current methodology %i, next methodology %i", 
                               CurrentMethodology, NextMethodology);
  
  return (NextMethodology);
  
} /* NextSpineMethodology */



/*******************************************************************************
*                                                                              *
*                            SpineMethodologyString                            *
*                                                                              *
*       Return strings describing the different spine testing methodologies    *
*       as a long description, a short description, or the units used.         *
*                                                                              *
*******************************************************************************/

const char * SpineMethodologyString (int Methodology, int StringType)
{
  switch (Methodology)
  {
    case ASTMAT28:
      if (StringType == SHORTSPINESTRING) return (ASTMAT28SHORTSTRING);
      if (StringType == LONGSPINESTRING) return (ASTMAT28STRING);
      return (ASTMAT28UNITS);
      
    case AMOAT26:
      if (StringType == SHORTSPINESTRING) return (AMOAT26SHORTSTRING);
      if (StringType == LONGSPINESTRING) return (AMOAT26STRING);
      return (AMOAT26UNITS);
      
    case AMOLBSAT26:
      if (StringType == SHORTSPINESTRING) return (AMOLBSAT26HORTSTRING);
      if (StringType == LONGSPINESTRING) return (AMOLBSAT26STRING);
      return (AMOAT26UNITS);  

    case GRAMSAT28:
      if (StringType == SHORTSPINESTRING) return (GRAMSAT28HORTSTRING);
      if (StringType == LONGSPINESTRING) return (GRAMSAT28STRING);
      return (GRAMSUNITS);  
  }
  return ("");
  
} /* SpineMethodologyString */



/*******************************************************************************
********************************************************************************
**                                                                            **
**                             Spine Averaging                                **
**                                                                            **
**      Class to average spine readings from six readings taken at 60         **
**      degree spacings.                                                      **
**      This doesn't need to be a class but historically it was a more        **
**      complex process that was simplified.                                  **
**                                                                            **
********************************************************************************
*******************************************************************************/

class SpineAveragingClass 
{
  public:
    int MisalignmentDirection;          // direction of maximum force
    float AverageForce;                 // average spine force
    float MaxForce;                     // maximum force
    float ForceErrorPercent;            // percentage incerase of max
    SpineAveragingClass (void);
    void Calculate (float Cock0, float At60, float At120, float At180, 
                                                     float At240, float At300);

}; /* SpineAveragingClass */



/*******************************************************************************
*                                                                              *
*                      SpineAveragingClass::Calculate                          *
*                                                                              *
*       Take parameters; work out average.                                     *
*                                                                              *
*******************************************************************************/

void SpineAveragingClass::Calculate (float Cock0, float At60, float At120, 
                                       float At180, float At240, float At300)
{
  float Sum;
  
  DEBUG (FUNC, Cock0, At60, At120);
  DEBUG (FUNC, At180, At240, At300);

  // work out the average
  Sum = (Cock0 + At60 + At120 + At180 + At240 + At300);
  AverageForce = Sum / 6.0;

  // find the maximum - this is dumb code but it works !!fixme
  MisalignmentDirection = 0;
  MaxForce = Cock0;
  if (At60 > MaxForce)
  {
    MaxForce = At60;
    MisalignmentDirection = 60;
  }
  if (At120 > MaxForce)
  {
    MaxForce = At120;
    MisalignmentDirection = 120;
  }
  if (At180 > MaxForce)
  {
    MaxForce = At180;
    MisalignmentDirection = 180;
  }
  if (At240 > MaxForce)
  {
    MaxForce = At240;
    MisalignmentDirection = 240;
  }
  if (At300 > MaxForce)
  {
    MaxForce = At300;
    MisalignmentDirection = 300;
  }
  ForceErrorPercent = (MaxForce / AverageForce) * 100.0;

  DEBUG (FUNC, MaxForce, AverageForce, MisalignmentDirection);

} /* SpineAveragingClass::Calculate */



/*******************************************************************************
*                                                                              *
*                 SpineAveragingClass::SpineAveragingClass                     *
*                                                                              *
*       Initialise the class data.                                             *
*                                                                              *
*******************************************************************************/

SpineAveragingClass::SpineAveragingClass (void)
{
  // initialise class data
  AverageForce = 0;
  MisalignmentDirection = 0;
  ForceErrorPercent = 0;
  
} /* SpineAveragingClass::SpineAveragingClass */



/*******************************************************************************
********************************************************************************
**                                                                            **
**                             Sleep Routines                                 **
**                                                                            **
**      These routines put the device into a low power state after a          **
**      period of inactivity and wake it up again when the ok button is       **
**      pressed. This will help to save the battery - the HX711 and strain    **                                                             
**      guages in particular are power hungry.                                **
**                                                                            **
********************************************************************************
*******************************************************************************/

/*******************************************************************************
*                                                                              *
*                             OKButtonInterrupt                                *
*                                                                              *
*       Routine to service the interrupt on the OK button to wake up from      *
*       sleep.                                                                 *
*                                                                              *
*******************************************************************************/

void OKButtonInterrupt(void)
{
  // detach the interrupt to stop it from continuously firing while the 
  // OK button pin is low
  detachInterrupt (digitalPinToInterrupt (BUTTON_OK_PIN));
  
} /* OKButtonInterrupt */



/*******************************************************************************
*                                                                              *
*                                 CheckForSleep                                *
*                                                                              *
*       Check how long it is since something happened.  If it's too long       *                                                  
*       then put the devices into low power mode, set up the wakeup            *
*       interrupt and put the arduino into sleep mode.  When the arduino       *
*       wakes up then turn everything back on again.                           *
*                                                                              *
*******************************************************************************/
 
bool CheckForSleep (bool SomethingHappened)
{
  static unsigned long LastActivity = 0;

  DEBUG (FUNC, "something happened %i, last activity %u", 
                                        (int) SomethingHappened, LastActivity);  

  // if something has happened then reset the sleep timer and return
  if (SomethingHappened)
  {
    LastActivity = millis();
    return (false);
  }

  // if we haven't hit the timeout time yet then just return
  if (millis() < (LastActivity + SLEEPTIMEOUT))
    return (false);
  
  // put the load cells into low power mode
  ArrowSpine->PowerDownArrowScale ();

  // shut down the display
  LCD->Off ();

  // set up the ok button as an interrupt and attach handler
  attachInterrupt (digitalPinToInterrupt (BUTTON_OK_PIN), OKButtonInterrupt, 
                                                                        LOW);
  delay(100);

  // set up the arduino sleep mode and sleep it
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  
  sleep_enable();
  sleep_mode();

  // on waking up the first thing to do is disable sleep
  sleep_disable(); 

  // turn on the display
  LCD->On ();

  // turn on the load cells
  ArrowSpine->PowerUpArrowScale ();

  // we just woke up so restart the timeout
  LastActivity = millis();

  return (true);
  
} /* CheckForSleep */



/*******************************************************************************
********************************************************************************
**                                                                            **
**                                  setup                                     **
**                                                                            **
**      The setup function runs once when you reset or power the board.       **
**                                                                            **
********************************************************************************
*******************************************************************************/

void setup() 
{
  // configure unused digital pins - Arduino seemingly uses less power if 
  // the pins are not left floating - I don't know if this is really true
  // but it's no effort 
  pinMode(10, INPUT);        
  digitalWrite(10, HIGH);    
  pinMode(11, INPUT);        
  digitalWrite(11, HIGH);    
  pinMode(12, INPUT);        
  digitalWrite(12, HIGH);    
  pinMode(13, INPUT);        
  digitalWrite(13, HIGH);    
  pinMode(14, INPUT);        
  digitalWrite(14, HIGH);    
  pinMode(15, INPUT);        
  digitalWrite(15, HIGH);    
  pinMode(16, INPUT);        
  digitalWrite(16, HIGH);    
  pinMode(17, INPUT);        
  digitalWrite(17, HIGH);    
  pinMode(20, INPUT);        
  digitalWrite(20, HIGH);    
  pinMode(21, INPUT);        
  digitalWrite(21, HIGH);    

  // initialise everything
  Debug = new SerialDebug ("================ " ARROWSCALETITLE " - " 
                               ARROWSCALEVERSION " - " __DATE__ " " 
                               __TIME__ " ================");
  OnboardLED = new OnboardLEDControl ();
  LCD = new DisplayClass ();

  // Print a startup message to the LCD.
  LCD->Line (0, ARROWSCALETITLE);
  LCD->Line (1, ARROWSCALEVERSION);
  delay(ARROWSCALEDELAY);
  
  // Print another startup message identifying the builder if message exists
  #ifdef BUILDERMESSAGELINE1
    LCD->Line (0,BUILDERMESSAGELINE1);
    LCD->Line (1,BUILDERMESSAGELINE2);
    delay(BUILDERMESSAGEDELAY);
  #endif

  Buttons = new ButtonControl ();

  //check whether flag is 23" (PIN 9 HIGH) or 28" (PIN 9 LOW) and init the 
  // property (part of ButtonControl class)
  Buttons->Init23InchModeFlag();

  // SWITCH23INCHMODE will not be used anymore. Me neither, I don't know if 
  // this is really true that Arduino uses less power like that, but I like 
  // when it's no effort
  pinMode(SWITCH23INCHMODE, INPUT);        
  digitalWrite(SWITCH23INCHMODE, HIGH);  

  //Print whether it's 23" or 28" long mode
  if (Buttons->Get23InchMode())
    LCD->Line (0,MOUNTEDIN23);
  else 
    LCD->Line (0,MOUNTEDIN28);
   
  LCD->Line (1,"");
  delay(MOUTEDSIZEDELAY);
  
  NVM = new EEPROMControl;
  ArrowSpine = new ArrowSpineClass;

  LCD->DelayTheReadyMessage ();

  OnboardLED->Blink (3);

  DEBUG (FUNC);
  
} /* setup */


/*******************************************************************************
********************************************************************************
**                                                                            **
**                                  loop                                      **
**                                                                            **
**      The loop function runs over and over again forever.                   **
**                                                                            **
********************************************************************************
*******************************************************************************/

void loop() 
{
  static unsigned long LastSpineChangePress = 0;
  static bool MeasuringSpine = false;
  static bool Weighing = false;
  static float PeakForce [6] = {0, 0, 0, 0, 0, 0};
  float Force;
  float CoG;
  int Button = BUTTON_NONE;       // last button pressed
  bool SomethingHappened = false;
  SpineAveragingClass SpineAverage; 

  // update the LCD if it's necessary
  LCD->LazyUpdate ();

  // do the periodic read of the load cells
  ArrowSpine->Poll ();

  // look to see if any of the buttons have been pressed
  Button = Buttons->Poll ();

  switch (Button)
  {
    
    // calibrate the arrow scale using a 1kg reference weight
    case BUTTON_3LONG_CALIBRATE_BIGUP:
    
      if (ArrowSpine->AutoCalibrateArrowScale ())
        LCD->DelayTheReadyMessage();
      else
        LCD->LazyUpdate (true);

      SomethingHappened = true;
      
      break;

    // display the average spine
    case BUTTON_2_SPINE_UP:
    
      // calculate the average of the last six measurements (assumed to 
      // be six measurements on the same arrow at 60 degree intervals) -
      // the oldest will be considered to be 0 degrees (the cock?)
      SpineAverage.Calculate (PeakForce [5], PeakForce [4], PeakForce [3], 
                              PeakForce [2], PeakForce [1], PeakForce [0]);
      LCD->WriteAverageSpineOnScreen (SpineAverage.AverageForce, 
                    NVM->SpineMethodology, SpineAverage.MaxForce, 
                    SpineAverage.MisalignmentDirection,Buttons->Get23InchMode());
                    
      // wait for a key press
      Buttons->WaitFor (BUTTON_1_OK);

      SomethingHappened = true;

      LCD->LazyUpdate (true);
      
      break;
      
    // display / change the spine methodology 
    case BUTTON_2LONG_SPINE_UP:
      
      // if it's the first press for a long time then just re-display the 
      // current spine methodology but if the button presses are within the 
      // time interval then cycle around the spine methodologies changing the
      // current one in force
      if (millis () < (LastSpineChangePress + SPINECHANGEPRESSDELAY))
        NVM->SpineMethodology = NextSpineMethodology (NVM->SpineMethodology);
      LCD->Line (SpineMethodologyString (NVM->SpineMethodology, 
                                                          LONGSPINESTRING));
      
      LCD->DelayTheReadyMessage ();
      
      LastSpineChangePress = millis ();
      
      SomethingHappened = true;
      
      break;

    // initiate spine reading
    case BUTTON_1_OK:
      
      // zero the scale - the arrow should be in position when the scale is
      // zeroed so that the arrow weight does not contribute to the force
      // on the scale and affect the spine calculation
      LCD->Line (ZEROINGSCALE);
      ArrowSpine->ZeroArrowScale ();        

      // set spine measurement in progress - this is a continuous measurement
      MeasuringSpine = true;
      Weighing = false;

      // shuffle the reading history; initialise the most recent spine to 
      // zero and display it
      for (int i = 5; i >= 1; i--) PeakForce [i] = PeakForce [i-1];
      PeakForce [0] = 0;
      LCD->WriteSpineOnScreen (0, NVM->SpineMethodology, Buttons->Get23InchMode() );
      
      SomethingHappened = true;
      
      break;
   
    // constant weight reading
    case BUTTON_3_CALIBRATE_UP:
      
      // zero the scale - this gives a tare function
      LCD->Line (ZEROINGSCALE);
      ArrowSpine->ZeroArrowScale ();        

      // set weighing in progress - this is a continuous measurement
      Weighing = true;
      MeasuringSpine = false;

      LCD->WriteWeightOnScreen (0, 0, 0, Buttons->Get23InchMode());
      
      SomethingHappened = true;
      
      break;
   
    case BUTTON_NONE:
    default:
       break;
  }

  // if spine measurement is in progress then measure the arrow spine; 
  // if the spine measurement has increased then write it to the screen
  if (MeasuringSpine)
  {
    Force = ArrowSpine->ReadArrowScale ();
    if ((Force > SIGNIFICANTREADING) && (Force > PeakForce [0]))
    {
      LCD->WriteSpineOnScreen (Force, NVM->SpineMethodology,Buttons->Get23InchMode());
      PeakForce [0] = Force;
    }
  }

  // if weighing is in progress then write weight to the screen
  if (Weighing)
  {
    Force = ArrowSpine->ReadArrowScale ();
    CoG = ArrowSpine->ReadArrowCOG (Buttons->Get23InchMode());
    LCD->WriteWeightOnScreen (Force, CoG, ArrowSpine->GetArrowLength (),
                                                Buttons->Get23InchMode());
  }

  // update the EEPROM if necessary
  NVM->LazyUpdate ();

  // check if we need to sleep
  CheckForSleep (SomethingHappened);

} /* loop */
