#include           <Arduino.h>

///////////////////////CHRONO DEFINITIONS///////////////////////////////
#include           <Chrono.h>
Chrono             wait;                                                // Timer to use inspite of delay in Linear actuator
Chrono             samplingIN;                                          // Sampling every one sec for IN flow sensor
Chrono		   samplingOUT;											                    // Sampling every one sec for OUT flow sensor

/////////////////////PUMP DEFINITIONS///////////////////////////////////
#define	           pump				12			// Output pin to control pump state
#define            solenoid                     13                      // Output pin to control solenoid valve

//////////////////////NEXTION DEFINITIONS///////////////////////////////
#include           <Nextion.h>
#include           <SPI.h>
#include           <AltSoftSerial.h>
AltSoftSerial      HMISerial;                                           // RX PIN 8, TX PIN 9, PIN 10 PWM DISABLED 
/*
Nextion(BLUE)      TX                 =         8 (RX)
Nextion(YELLOW)    RX                 =         9 (TX)
*/
uint32_t           WaterIN            =         0;                      // Variabe to store the inlet water for comparison
uint32_t           VOLUME             =         0;                      // Variable to store the value of the VOLUME

//Declare pages
NexPage            page0              =         NexPage(0, 0, "page0");
NexPage            page1              =         NexPage(1, 0, "page1");
//Declare textboxes
NexText            tSet               =         NexText(0,4,"tSet");
NexText            tNow               =         NexText(0,8,"tNow");
NexText            tWtrRt             =         NexText(0,5,"tWtrRt");
NexText            tWtrIn             =         NexText(0,17,"tWtrIn");
NexNumber          nVol               =         NexNumber(0,13,"nVol");

NexText            tRnrt              =         NexText(1,7,"tRnrt");
NexText            tTtlrn             =         NexText(1,8,"tTtlrn");
NexText		   tIng		      =         NexText(1,18,"tIng");
NexText            tRuno              =         NexText(1,9,"tRuno");
NexText            tPerm              =         NexText(1,10,"tPerm");
NexText            tInc               =         NexText(1,20,"tInc");
NexText            tQuantita          =         NexText(1,22,"tQuantita");

///////////////////////2 FLOW SENSORS///////////////////////////////////
#define            flowsensorIN                 3                       // Sensor Input IN,  FLOW SENSOR INGRESSO
#define            flowsensorOUT                2                       // Sensor Input OUT, FLOW SENSOR SCARICO

volatile int       flow_frequencyIN;                                    // Measures flow sensor pulses IN  (Hz)// Pulse frequency (Hz) = Calibration_factor x Flow rate in L/min
float              volIN               =        0.0;                    // Calculated IN litres
float              l_minuteIN;                                          // Calculated IN liters/min

volatile int       flow_frequencyOUT;                                   // Measures flow sensor pulses OUT // Pulse C
float              volOUT              =        0.0;                    // Calculated OUT litres
float              l_minuteOUT;

float              Int_Pio;                                             // Variable for Intensita Pioggia
float              Pio_Ttl;                                             // Variable for Pioggia Totale
float              Permiab             =        0.0;                    // Agro Net permiability

float              Calibration_factorIN=        7.0;                    // Calibration factor for IN flow sensor; PulsePerSecPerLiter
float              Calibration_factorOUT=       7.0;                    // Calibration factor for OUT flow sensor

//////////////////////LINEAR ACTUATOR///////////////////////////////////
#define             RPWM                        5                      // Motor driving circuit output
#define             LPWM                        6
#define             sensorPin                   A0                      // Linear Act.(LA) potentiometer feedback
#define             potPin                      A1                      // External Trimpot (EXT) to adjust actuator length

int                 sensorVal;                                          // Analog reading of LA potentiometer
int                 potVal;                                             // Analog reading of EXT potentiometer

int                 Speed             =         157;                    // Customize speed, 255 max, 0 min >> max 13mm/sec:255; 5mm/sec:98; 8mm/sec: 157
int                 Buffer            =         10;						          // Addition gives buffer to prevent actuator from rapidly vibrating due to noisy data inputs

int                 nowinclination;                                     // Current inclination
int                 setinclination;                                     // Set inclination to achieve

int                 minLAReading      =         82;                     // Minimum LA pot feedback at zero extension
int                 maxLAReading      =         792;                    // Maximum LA pot feedback at full extended length

int                 minPotReading     =         0;                      // Minimum EXT pot value at knob 0
int                 maxPotReading     =         944;                    // Maximum EXT pot value at knob 10

//////////////////////LINEAR ACTUATOR///////////////////////////////////
void flowIN() {                                                         // Interrupt function for IN(ISR)
   flow_frequencyIN++;                                                  // Measurement one cycle per second (Hz); Pulse Count
}
void flowOUT() {                                                        // Interrupt function for OUT(ISR)
   flow_frequencyOUT++;
}

//////////////////////LINEAR ACTUATOR///////////////////////////////////
void driveActuator(int Direction, int Speed) {
   switch (Direction) {
     case -1:                                                           // Extension
       analogWrite(RPWM, Speed);
       analogWrite(LPWM, 0);
       break;

     case 0:                                                            // Stopping
       analogWrite(RPWM, 0);
       analogWrite(LPWM, 0);
       break;

     case 1:                                                            // Retraction
       analogWrite(RPWM, 0);
       analogWrite(LPWM, Speed);
       break;
  }
}

//Linear actuator (a)data collection,(b)serial display,(c)nextion display
void GetLAData() {
    potVal = map(analogRead(potPin), minPotReading, maxPotReading, minLAReading, maxLAReading); // Mapping EXT pot values to LA values
    sensorVal = analogRead(sensorPin);

    if (potVal > (sensorVal+Buffer)) {                                  // Addition gives buffer to prevent actuator from rapidly vibrating due to noisy data inputs
      driveActuator(1, Speed);
    }
    else if (potVal < (sensorVal-Buffer)) {
      driveActuator(-1, Speed);
    }
    else {
      driveActuator(0, Speed);
    }

    nowinclination = map(sensorVal, minLAReading, maxLAReading, 2, 35);
    setinclination = map(potVal, minLAReading, maxLAReading, 2, 35);

    wait.restart();                                                     // Add 100us delay between sensor readings
    while (wait.hasPassed(100)) {}
}

void SerialLAData() {
	  //Serial.print(analogRead(potPin));									             // Displays external Trimpot analog values
	  Serial.print("EXT Pot Set: ");									                 // Displays external Trimpot analog values mapped to LA values
    Serial.print(potVal);
    Serial.print("; LA Pot Now: ");										               // Displays LA values
    Serial.print(sensorVal);

    Serial.print("; Set angle: ");                                   // Display SET Inclination
    Serial.print(setinclination, 0);
    Serial.print("\xC2\xB0");                                        // Prints degree symbol on serial monitor
    Serial.print("; Now angle: ");
    Serial.print(nowinclination, 0);                                 // Display NOW Inclination
    Serial.println("\xC2\xB0.");
}

void DisplayLAData() {
    char setgradi[10];                                                 // Display SET Inclination
    dtostrf(setinclination, 3, 0, setgradi);
    tSet.setText(setgradi);

    char nowgradi[10];                                                  // Display NOW Inclination
    dtostrf(nowinclination, 3, 0, nowgradi);
    tNow.setText(nowgradi);
    tInc.setText(nowgradi);												                      // Display now inclination on page 1
}

//2 Flow Sensors (a)data collection,(b)serial display,(c)nextion display
////- IN : INGRESSO FLOW SENSOR
void GetFLOWINData() {
    if (samplingIN.hasPassed(1000)) {                                   // Every second, calculate and print litres/hour
     samplingIN.restart();

     l_minuteIN = (flow_frequencyIN / Calibration_factorIN);            // l/min calculation for flow sensor
     //l_minuteIN = l_minuteIN / 60.0;                                     // l/min rate is converted to l/sec
     volIN += l_minuteIN/60.0;                                          // liquid volume is updated for every one second, first l/min is converted to l/sec
	   flow_frequencyIN = 0;                                        // Reset Counter

	   WaterIN = (int) volIN;                                       // CASTING - Convert float volIN to int WaterIN for comparsion of user set value

	   Int_Pio = l_minuteIN*60.0;                                    // Application rate l/min conversion to l/hr for mm/hr values
	   Pio_Ttl = volIN*60.0;                                         // Total rain conversion to mm by multiplying with 60
    }
}

void SerialFLOWINData() {
	  Serial.print("Inlet Water: ");
    Serial.print(volIN, 2);                                             // Total volumeIN in liters
    Serial.print(" L;");

    Serial.print(" Inlet Water Rate: ");
    Serial.print(l_minuteIN, 2);                                         // Print litres/minute; DEC only give decimal places
    Serial.print(" L/min;");

    Serial.print(" Rain Intensity: ");
    Serial.print(Int_Pio, 0);
    Serial.print(" mm/hr;");

    Serial.print(" Total Rain: ");
    Serial.print(Pio_Ttl, 0);
    Serial.println(" mm.");
}

void DisplayFLOWINData() {
    char IntPiog[10]; 													                        // Display Rain Intensity
    dtostrf(Int_Pio, 4, 0, IntPiog);
    tRnrt.setText(IntPiog);

    char PiogTot[10];													                          // Display Total Rain
    dtostrf(Pio_Ttl, 4, 0, PiogTot);
    tTtlrn.setText(PiogTot);

    char Quant[10];                                                     // Display Inlet Water Rate
    dtostrf(l_minuteIN, 5, 2, Quant);
    tWtrRt.setText(Quant);
    tQuantita.setText(Quant);											                      // Display water rate on page 1

    char IngrAcq[10];                                                   // Display Inlet water volume
    dtostrf(volIN, 5, 2, IngrAcq);
    tIng.setText(IngrAcq);                                              // Display Inlet water volume on page 0
    tWtrIn.setText(IngrAcq);                                            // Display Inlet water volume on page 1
}
////- OUT : SCARICO FLOW SENSOR
void GetFLOWOUTData() {
    if (samplingOUT.hasPassed(1000)) {                                  // Every second, calculate and print litres/hour
      samplingOUT.restart();

	    l_minuteOUT = (flow_frequencyOUT / Calibration_factorOUT);
	    //l_minuteOUT = l_minuteOUT / 60.0;
	    volOUT += l_minuteOUT / 60.0;
	    flow_frequencyOUT = 0;                                     // Reset Counter
    }
}

void SerialFLOWOUTData() {
    Serial.print("Oulet Water: ");
    Serial.print(volOUT, 2);                                           // Total volumeOUT in liters
    Serial.print(" L;");

	  Serial.print(" Rain Runoff Rate ");
    Serial.print(l_minuteOUT, 2);                                      // Print litres/minute; DEC only give decimal places
    Serial.print(" L/min;");
}

void DisplayFLOWOUTData() {
    char ScaAcq[10];                                                     // Display out water
    dtostrf(volOUT, 5, 2, ScaAcq);
    tRuno.setText(ScaAcq);
}

void NetPermiability() {
	  Permiab = (1-(volOUT/volIN))*100;				// Formula for Net Permeability Calculation

	  Serial.print(" Net Permiability: ");
    Serial.print(Permiab, 0);                                            // Print permiability of net
    Serial.println(" %.");

    char Perm[10];                                                       // Display Net Permeability
    dtostrf(Permiab, 4, 0, Perm);
    tPerm.setText(Perm);
}

////////////////////////RUN ONCE////////////////////////////////////////
void setup() {
    // Start Serial communication
    Serial.begin(9600);													                        // Serial communication begin for Nextion
    
    // Pump control
    pinMode(pump, OUTPUT);                                               
    digitalWrite(pump, HIGH);                                           // Optocopuler relay is active LOW so setting pins HIGH in the beginning >> N/C STATE
    
    // Solenoid control
    pinMode(solenoid, OUTPUT);                                          
    digitalWrite(solenoid, HIGH);                                       // Normal relay is active LOW so setting pins HIGH in the beginning >> N/C STATE
    
    // Flow Sensors
    pinMode(flowsensorIN, INPUT);                                       
    digitalWrite(flowsensorIN, HIGH);                                   // Optional Internal Pull-Up
    attachInterrupt(digitalPinToInterrupt(flowsensorIN), flowIN, RISING);// Setup Interrupt

    pinMode(flowsensorOUT, INPUT);                                      
    digitalWrite(flowsensorOUT, HIGH);                                  // Optional Internal Pull-Up
    attachInterrupt(digitalPinToInterrupt(flowsensorOUT), flowOUT, RISING);// Setup Interrupt

    // Linear Actuator
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
    pinMode(sensorPin, INPUT);
    pinMode(potPin, INPUT);

    // Nextion Display
    nexInit();                                                          // Initialize Nextion Library

    // Wait for 3 sec for system stabilisation
    wait.restart();                                                     // Add 3 secs delay between sensor readings
    while (wait.hasPassed(3000)) {}
}

////////////////////////////////////////////////////////////////////////
void loop() {
    //GET SET TEST VOLUME
    nVol.getValue(&VOLUME);												                      // Read the value of the VOLUME SET
    wait.restart();                                                     // Add 0.5 sec delay to ensure value is taken properly
    while (wait.hasPassed(100)) {}
    nVol.getValue(&VOLUME);

    //LINEAR ACTUATOR
    GetLAData();                                                        // Linear Actuator data
    //SerialLAData();                                                     // Send LA results to serial port for debugging
    DisplayLAData();                                                    // Send LA results to NEXTION display
    
    while (WaterIN++ <= VOLUME) {                                       // Add 1 in WaterIN values becuase while float --> int conversion truncation happens
      //LINEAR ACTUATOR
      GetLAData();                                                      // Linear Actuator data
      //SerialLAData();                                                   // Send LA results to serial port for debugging
      DisplayLAData();                                                  // Send LA results to NEXTION display
	    //FLOW METER (1) IN - INGRESSO
	    GetFLOWINData();                                            // Flow SensorsIN data
      SerialFLOWINData();
      DisplayFLOWINData();                                              // Display FlowIN results to NEXTION display
      //FLOW METER (2) OUT - SCARICO
      GetFLOWOUTData();                                                 // Flow SensorsOUT data
      SerialFLOWOUTData();
      DisplayFLOWOUTData();                                             // Display FlowOUT results to NEXTION display
      //NET PERMIABILITY CALCULATION
      NetPermiability();												                        // Calculate and Display Net Permiability
    }
    //TURN OFF PUMP & SOLENOID
    digitalWrite(pump, LOW);											                      // Turn pump off as the test finishes by bringing to low as it is LOW level trigger
    digitalWrite(solenoid, LOW);                                        // Turn solenoid off as the test finishes by bringing to low as it is LOW level trigger
    //FLOW METER (2) OUT - SCARICO
    GetFLOWOUTData();                                                   // Flow SensorsOUT data
    SerialFLOWOUTData();
    DisplayFLOWOUTData();                                               // Display FlowOUT results to NEXTION display
    ///NET PERMIABILITY CALCULATION
    NetPermiability();												                          // Calculate and Display Net Permiability
}

