#include <Arduino.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
using namespace std;
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

// Pins
int analogPin = 36; // potentiometer wiper (middle terminal) connected to analog pin 3  
int ledPin = 5;

// EMG Data
int analogVal = 0;  // variable to store the value read
int average = 0;
float delayEMG = 0.2; // 0.2 =< delayEMG < 1 (Nyquist Freq)
int window = 100; // window*delayEMG <= delayIMU
int windowAvr = window*1000;
int frequency = 0;
int amplitude = 0;
bool posState = false;

// IMU Data
Adafruit_BNO055 bno = Adafruit_BNO055(55);
float pitch;
float threshHigh = 77; // Good Muscle Angle Region
float threshLow = 70; // Good Muscle Angle Region
float delayIMU = 20;

// BLE Packet
uint8_t sendData[12]; // pitch(float/4), frequency(int/4), amplitude(int/4)

bool first = true;

BLEServer *pServer = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914c"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a9"

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;
    };

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
    }
};

BLECharacteristic *pCharacteristic;

void setup()
{
    pinMode(analogPin, INPUT);
    pinMode(ledPin, OUTPUT);
    Serial.begin(9600);

    // Create the BLE Device
    BLEDevice::init("PostureBelt");

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create a BLE Characteristic
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_NOTIFY |
            BLECharacteristic::PROPERTY_INDICATE);

    // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
    // Create a BLE Descriptor
    pCharacteristic->addDescriptor(new BLE2902());

    // Start the service
    pService->start();

    // Start advertising
    pServer->getAdvertising()->start();
    Serial.println("Waiting a client connection to notify...");
    Wire.begin();
    delay(1000);
    bno.setMode(bno.OPERATION_MODE_IMUPLUS);
    delay(1000);

    for(int i = 0; i < windowAvr; i++) {
        average += analogRead(analogPin); // read the input pin
    }
    average /= (windowAvr);
    Serial.print("Average: ");
    Serial.println(average);
}


void loop()
{
    //Read EMG value
    frequency = 0;
    amplitude = 0;
    for(int i = 0; i < window; i++) { // calculating frequency and amplitude
        analogVal = analogRead(analogPin) - average; // read the input pin
        amplitude += analogVal*analogVal;
        if (analogVal > 0) {
            if (!posState) {
                frequency++;
            }
            posState = true;
        } else {
            if (posState) {
                frequency++;
            }
            posState = false;
        }
        delay(delayEMG);
    }

    pitch = bno.getVector(Adafruit_BNO055::VECTOR_EULER).y();
    /*
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << 0);
    Wire.endTransmission();
    */

    Serial.print("Pitch: ");
    Serial.print(pitch);
    Serial.print("\tEMG Recent Value: ");
    Serial.print(analogVal);
    Serial.print("\tEMG Frequency: ");
    Serial.print(frequency);
    Serial.print("\tEMG Amplitude: ");
    Serial.println(amplitude);

    //Build Packet
    sendData[0] = (float) pitch;
    sendData[4] = (float) frequency;
    sendData[8] = (float) amplitude;

    if (pitch > threshHigh)
    {
        digitalWrite(ledPin, HIGH);
    }
    else if (pitch > threshLow)
    {
        digitalWrite(ledPin, LOW);
    }
    else
    {
        digitalWrite(ledPin, HIGH);
    }

    // notify changed value
    if (deviceConnected)
    {
        pCharacteristic->setValue((uint8_t *)sendData, 12);
        pCharacteristic->notify();
        // value++;
        // delay(20); // bluetooth stack will go into congestion, if too many packets are sent
    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected)
    {
        delay(500);                  // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected)
    {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
    delay(delayIMU - (delayEMG*window));
}