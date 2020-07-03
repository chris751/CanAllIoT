#include <math.h>;

bool debug = true;

int analogResistorPin = A0;
int raw = 0;
float Vin = 3.3;
float Vout = 0;
float R1 = 10000;
float R2 = 0;
float buffer = 0;

// Modules
const float basisResistance = 1000;
const float hallEffectModule = 2750;
const float PIRModule = 2000;
const float timerModule = 1000;
const float noiseTolerance = 3; 

// millis
const long moduleCheckInterval = 1000; 
unsigned long previousMillis = 0;  

// reusable variables
const int digitalSensorPin = D1; // used for digital sensor input
const int analogSensorPin = A1;  // used for analog sensor input
float ohms;

// PIR variables
int pirState = 0; // variable for reading the pushbutton status
bool pirCalled = false;
int pirCounter = 0;
int countSinceLastPublish;

// Potentiometer 
int timerVal = 0;       // variable to store the value coming from the sensor

// HALL EFFECT 
int hallEffectValue = 0;
// Smoothing 
const int numReadings = 10;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

const String HALL_EFFECT = "HALL_EFFECT";
const String PIR = "PIR";
const String TIMER = "TIMER";
const String SENSOR_CONFIG_STATE = "SENSOR_CONFIG_STATE";

bool hasSetKeepAlive = false;

void setup()
{
    // Smoothing
    for (int thisReading = 0; thisReading < numReadings; thisReading++) {
        readings[thisReading] = 0;
    }
    pinMode(digitalSensorPin, INPUT);
    pinMode(D7, OUTPUT);
    Serial.begin(9600);
}

int safetyCounter = 0;
int timeWithModuleCounter = 0;
int previousModule = -1;

void loop()
{
    unsigned long currentMillis = millis();
    if(currentMillis-previousMillis >= moduleCheckInterval) {
        previousMillis = currentMillis;
        ohms = measureOhms();
        safetyCounter++; 
    }
    int module = determineModule(ohms);
    
    if (ohms != 0.0 && safetyCounter > 60)
    {
        updateSensorState(module);
        safetyCounter = 0;
    }
    //  Serial.print("timeWithModuleCounter");
    // Serial.println(timeWithModuleCounter);

    if (module == previousModule || previousModule == -1) {
        timeWithModuleCounter++;
        previousModule = module;
    } else {
        previousModule = module;
        timeWithModuleCounter = 0;
    }
    if (timeWithModuleCounter > 100) {
        startSensorAction(module);
    }

    // force keep alive 
    if (Particle.connected()) {
		if (!hasSetKeepAlive) {
			hasSetKeepAlive = true;
			Particle.keepAlive(120);
		}
	}else {
		hasSetKeepAlive = false;		
	}
    
    delay(100);
}

void startSensorAction(int moduleState)
{
    switch (moduleState)
    {
    case 1: // Hall Effect
        getHallEffectInput();
        hallEffectAction();
        break;
    case 2: // PIR module
        getPirInput();
        break;
    case 3: // Timer module
        getTimerInput(); 
        break;
    case 4: // hall effect + timer module
        getTimerInput();
        getHallEffectInput();
        hallEffectAndTimerAction();
        break;
    }
}

int previousSensor = -1; 
int previousTimerVal = -1;
void updateSensorState(int currentSensorState) {
    if (previousSensor == -1 || previousSensor != currentSensorState){ // not state set yet or state is new --> lets update
        publish(SENSOR_CONFIG_STATE, String(currentSensorState)+","+String(timerVal)); // update trigger state
        previousSensor = currentSensorState;
        previousTimerVal = timerVal;
    } else if (previousTimerVal == -1 || timerVal != previousTimerVal ) {
        publish(SENSOR_CONFIG_STATE, String(currentSensorState)+","+String(timerVal)); // update trigger state
        previousTimerVal = timerVal;
    }
}

unsigned long firstTimeCalledMillis; 
bool firstTimeHigh = true;
bool published = false;

void hallEffectAndTimerAction() {
    if(hallEffectValue == HIGH) {
        if (firstTimeHigh) {
            firstTimeCalledMillis = millis();
            firstTimeHigh = false;
        }
        unsigned long currentMillis = millis();
        // if magnet is not present for specified time by timer module
        if(currentMillis-firstTimeCalledMillis >= timerVal * 1000 && !published) { 
            publish(HALL_EFFECT, String(timerVal));  
            published = true;    
        }
    } else { 
        firstTimeHigh = true;
        published = false;
    }
}

bool isNewHallEffectValue = true;

void hallEffectAction() {
   
    //Serial.println("isNewHallEffectValue: " + String(isNewHallEffectValue));
    if(hallEffectValue == HIGH && isNewHallEffectValue == true) { // magnet is not present
        publish(HALL_EFFECT, String(hallEffectValue));
        isNewHallEffectValue = false;
    } 
    else if (hallEffectValue == LOW) {
        isNewHallEffectValue = true;
    }
}

void getHallEffectInput() {
    hallEffectValue = digitalRead(digitalSensorPin);
    Serial.println(hallEffectValue);
}


void getTimerInput() {
   int timerSensorValue = analogRead(analogSensorPin);
   timerVal = timerSensorValue/(4096/300);
   Serial.print("timerVal: ");
   Serial.println(timerVal);
}

// millis
const long pirModuleCheckInterval = 60000; 
unsigned long pirPreviousMillis = 0;  

void getPirInput()
{
    if (debug){ Serial.println("Running PIR action..."); }
    pirState = digitalRead(digitalSensorPin);
    if (pirState == HIGH)
    {
        digitalWrite(D7, HIGH);
        if (!pirCalled)
        {
            pirCounter++;
            if(debug) {
                Serial.print("PIR counter: ");
                Serial.println(pirCounter);
            }
            pirCalled = true;
        }
    }
    else
    {
        pirCalled = false;
        digitalWrite(D7, LOW);
    }

    unsigned long currentMillis = millis();
    if(currentMillis-pirPreviousMillis >= pirModuleCheckInterval && pirCounter != 0) {
        pirPreviousMillis = currentMillis;
        if(debug){ Serial.println("Pir was activitaed with counter: " + String(pirCounter)); }
        publish(PIR, String(pirCounter));
        pirCounter = 0;
    }

    // int counterThreshold = 10;
    // if (pirCounter > counterThreshold)
    // {   
    //     if(debug){ Serial.println("PIR was activated " + String(counterThreshold) + "times, reseting counter"); }
    //     publish(PIR, String(pirCounter));
    //     pirCounter = 0;
    // }
}

bool publishSucces = false;
void publish(String name, String data)
{
    if (Particle.connected() == false) {
        Serial.println("Device not connected");
        Particle.connect();
        Serial.println("Connecting to particle cloud...");
        waitUntil(Particle.connected);
        delay(2000); // wait for the device to connect
        Serial.println("Device should now be connected");
    }
    // if (pPublish(name, data) == true){
    //     Serial.println("Published data succesfully");
    // } else {
    //     pPublish(name, data)
    // }
    while(!publishSucces) {
        publishSucces = pPublish(name, data);
    }
    publishSucces = false;
}

bool pPublish(String name, String data) {
    if (Particle.publish(name, data, PUBLIC) == true){
        Serial.println("Published Succesfully");
        return true;
    } else {
        Serial.println("Failed to Publish");
        return false;
    }
}

float calculateLowTolerance(float module)
{
    return module * (100 - noiseTolerance) / 100;
}

float calculateHighTolerance(float module)
{
    return module * (100 + noiseTolerance) / 100;
}

float basicResistanceCalculation()
{
    return basisResistance;
}

float basicResistanceCalculation(float module)
{
    float modulePart = (1 / module);
    float resPart = (1 / basisResistance);

    float stuff = (1 / module) + (1 / basisResistance);

    return pow(stuff, -1);
}

float basicResistanceCalculation(float module1, float module2)
{
    return pow((1 / basisResistance) + (1 / module1) + (1 / module2), -1);
}

int determineModule(int ohm)
{
    if (ohm > calculateLowTolerance(basicResistanceCalculation()) && ohm < calculateHighTolerance(basicResistanceCalculation()))
    {
        Serial.println("NO MODULE CONNECTED");
        return 0;
    }
    else if (ohm > calculateLowTolerance(basicResistanceCalculation(hallEffectModule)) && ohm < calculateHighTolerance(basicResistanceCalculation(hallEffectModule)))
    {
        Serial.println("HALL EFFECT MODULE IS CONNECTED");
        return 1;
    }
    else if (ohm > calculateLowTolerance(basicResistanceCalculation(PIRModule)) && ohm < calculateHighTolerance(basicResistanceCalculation(PIRModule)))
    {
        Serial.println("PIR MODULE IS CONNECTED");
        return 2;
    }
    else if (ohm > calculateLowTolerance(basicResistanceCalculation(timerModule)) && ohm < calculateHighTolerance(basicResistanceCalculation(timerModule)))
    {
        Serial.println("Timer MODULE IS CONNECTED");
        return 3;
    }
    else if (ohm > calculateLowTolerance(basicResistanceCalculation(hallEffectModule, timerModule)) && ohm < calculateHighTolerance(basicResistanceCalculation(hallEffectModule, timerModule)))
    {
        Serial.println("Hall & Timer MODULE IS CONNECTED");
        return 4;
    }
    else
    {
        Serial.println("NOISE DETECTED");
        return 400;
    }
}

float measureOhms()
{
    raw = calculateAverage();
    if (raw)
    {
        buffer = raw * Vin;
        Vout = (buffer) / 4096;
        buffer = Vout / (Vin - Vout);
        R2 = R1 / buffer;
        // Serial.print("RAW: ");
        // Serial.println(raw);
        // Serial.print("Vout: ");
        // Serial.println(Vout);
        Serial.print("R2: ");
        Serial.println(R2);
        return R2;
    }
    else
    {
        return 0.0;
    }
}



int calculateAverage() {

        // subtract the last reading:
    total = total - readings[readIndex];
    // read from the sensor:
    readings[readIndex] = analogRead(analogResistorPin);
    // add the reading to the total:
    total = total + readings[readIndex];
    // advance to the next position in the array:
    readIndex = readIndex + 1;

    // if we're at the end of the array...
    if (readIndex >= numReadings) {
        // ...wrap around to the beginning:
        readIndex = 0;
    }

    // calculate the average:
    average = total / numReadings;
    // send it to the computer as ASCII digits
    //Serial.println(average);
    return average;
}