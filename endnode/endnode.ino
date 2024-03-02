/*
 * Project: River monitoring with LoRa
 * Device: RAK LoRa Endnode T1
 * Author: Wilson Cosmo
 */

#include <Arduino.h>
#include <SX126x-RAK4630.h> //http://librarymanager/All#SX126x
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//for RAK5811 module:
#include <Wire.h>
#ifdef _VARIANT_RAK4630_
#include <Adafruit_TinyUSB.h>
#endif

#define NO_OF_SAMPLES 32

// Function declarations
void OnTxDone(void);
void OnTxTimeout(void);

#ifdef NRF52_SERIES
#define LED_BUILTIN 35
#endif

// Sensor port:
#define ONE_WIRE_BUS WB_IO2
#define DO_OUTPUT WB_IO4
#define SOLAR_OUTPUT WB_A1
#define PIN_VBAT WB_A0
OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);

// Define LoRa parameters
#define RF_FREQUENCY 915000000	// Hz
#define TX_OUTPUT_POWER 22		// dBm
#define LORA_BANDWIDTH 0		// [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 12 // [SF7..SF12]
#define LORA_CODINGRATE 4		// [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8	// Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0	// Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 30000
#define TX_TIMEOUT_VALUE 30000

static RadioEvents_t RadioEvents;

#define address 99              //default I2C ID number for EZO RTD Circuit.
#define address2 102              //default I2C ID number for EZO RTD Circuit.

//battery------------------------------------------



uint32_t vbat_pin = PIN_VBAT;

#define VBAT_MV_PER_LSB (0.73242188F) // 3.0V ADC range and 12 - bit ADC resolution = 3000mV / 4096
#define VBAT_DIVIDER_COMP (1.73)      // Compensation factor for the VBAT divider, depend on the board
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)


float readVBAT(void) //Get RAW Battery Voltage
{
    float raw;

    // Get the raw 12-bit, 0..3000mV ADC value
    raw = analogRead(vbat_pin);

    return raw * REAL_VBAT_MV_PER_LSB;
}


uint8_t mvToPercent(float mvolts) //Convert from raw mv to percentage
{
    if (mvolts < 3300)
        return 0;

    if (mvolts < 3600)
    {
        mvolts -= 3300;
        return mvolts / 30;
    }

    mvolts -= 3600;
    return 10 + (mvolts * 0.15F); // thats mvolts /6.66666666
}


uint8_t mvToLoRaWanBattVal(float mvolts) //get LoRaWan Battery value
{
    if (mvolts < 3300)
        return 0;

    if (mvolts < 3600)
    {
        mvolts -= 3300;
        return mvolts / 30 * 2.55;
    }

    mvolts -= 3600;
    return (10 + (mvolts * 0.15F)) * 2.55;
}
//------------------------------------------


//other variables:
#define pkt_size 59 //24 default, 59 by datasheet

//Payload variables
int cc = 1; //counter
uint8_t payload[pkt_size]; //payload
String s_payload = ""; //payload - String format
String d_id = "T1"; //ID - Endnode
float solar_v = 0; //solar output in volts
int t_send = 60000; //time between payloads, default of the project: 60000
int e_id = 50; //Tag - experiment number

float vbat_mv = 3.7;
float ph = -7; //pH
float temp = -127; //temperatura
float d_oxyg = 10.50;
float do_cal = 0; //DO calibration
float phSleep = 0.0;
float tempSleep = 0.0;

void setup()
{
  
	// Initialize Serial for debug output
	time_t timeout = millis();
	Serial.begin(115200);
	while (!Serial)
	{
		if ((millis() - timeout) < 5000)
		{
            delay(100);
        }
        else
        {
            break;
        }
	}
 
	Serial.println("=====================================");    
  Serial.print("Device: RAK LoRa Endnode - ");
  Serial.println(d_id);  
  Serial.println("=====================================");

  /* WisBLOCK 5811 Power On*/
  pinMode(WB_IO1, OUTPUT);
  digitalWrite(WB_IO1, HIGH);
  /* WisBLOCK 5811 Power On*/

  pinMode(SOLAR_OUTPUT, INPUT_PULLDOWN); //solar sensor output
  pinMode(DO_OUTPUT, INPUT_PULLDOWN); //solar sensor output

  analogReference(AR_INTERNAL_3_0);
  analogOversampling(128);
 
  
	// Initialize LoRa chip.
	lora_rak4630_init();
  
	// Initialize the Radio callbacks
	RadioEvents.TxDone = OnTxDone;
	RadioEvents.RxDone = NULL;
	RadioEvents.TxTimeout = OnTxTimeout;
	RadioEvents.RxTimeout = NULL;
	RadioEvents.RxError = NULL;
	RadioEvents.CadDone = NULL;

	// Initialize the Radio
	Radio.Init(&RadioEvents);

	// Set Radio channel
	Radio.SetChannel(RF_FREQUENCY);

	// Set Radio TX configuration
	Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
					  LORA_SPREADING_FACTOR, LORA_CODINGRATE,
					  LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
					  true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);
	
  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);

  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14

  // Let the ADC settle
  delay(1);

  // Get a single ADC sample and throw it away
  readVBAT();  

  sensors.begin(); 

  Wire.begin(); 

  DO_cal();

}

void loop()
{  
  float vbat_mv = readVBAT();  //tension in mV
  //sensors.requestTemperatures(); //read temperature sensor
  solar_v = getSolarV(); //read solar panel tension  
  //d_oxyg = getDO(); //DO
  ph = Read_from_I2C("R"); //pH
  phSleep = Read_from_I2C("Sleep"); //pH
  temp = Read_from_I2C2("R"); //temperatura
  tempSleep = Read_from_I2C2("Sleep"); //temperatura
  	
  
  /*
  Pkg sequence:
  <PKG_ID>;<Exp_ID>;<V_Bat>;<Temperature>;<PH>;<DO>;<V_Solar>
  */    
  s_payload = d_id + ";" + String(e_id) + ";" + String((vbat_mv/1000),2) + ";" + String(temp) + ";" + String(ph) + ";" + String(d_oxyg,2) + ";" + String(solar_v,2) + ";";

  cc = cc + 1;
  send();
  delay(t_send);
}


void OnTxDone(void) //Function to be executed on Radio Tx Done event
{  
  Serial.print("Sent: ");
  Serial.println((char*)payload); //feedback in serial output  
}

void OnTxTimeout(void) //Function to be executed on Radio Tx Timeout event
{
	Serial.println("OnTxTimeout");
}

void send()
{  
  s_payload.getBytes(payload, pkt_size);
  Radio.Send(payload, pkt_size);  
}

float getSolarV() //Function to read the Tension output from the Solar Panel
{
  int i;
  int mcu_ain_raw = 0;  
  int average_raw;
  float mcu_ain_voltage;
  float voltage_sensor;               // variable to store the value coming from the sensor
  float real_v;

  for (i = 0; i < NO_OF_SAMPLES; i++)
  {
    mcu_ain_raw += analogRead(SOLAR_OUTPUT);       // the input pin A1 for the potentiometer
  }
  
  average_raw = mcu_ain_raw / i;
  mcu_ain_voltage = average_raw * 3.0 / 1024;   //raef 3.0V / 10bit ADC
  voltage_sensor = mcu_ain_voltage / 0.6;     //WisBlock RAK5811 (0 ~ 5V).   Input signal reduced to 6/10 and output
  real_v = voltage_sensor/0.8; //based on tension division  

  return real_v;
}

//DO start

float getDO() //Function to read the DO output from the Sensor
{
  float read_do = do_cal * DO_voltage();
  return read_do;
}

float DO_voltage(){
  float voltage_mV = 0;
  float volt_avg_len = 1000;

  for (int i = 0; i < volt_avg_len; ++i) {
    voltage_mV += analogRead(DO_OUTPUT) / 1024.0 * 5000.0;
  }
  
  voltage_mV /= volt_avg_len;

  return voltage_mV;
}

void DO_cal(){
  float full_sat_voltage = DO_voltage(); //primeira voltagem ~ 100
  float cc = 100/full_sat_voltage;
  do_cal = cc;
}

//end DO

float Read_from_I2C(String cmd)
{
  byte received_from_computer = 0; //we need to know how many characters have been received.
  byte serial_event = 0;           //a flag to signal when data has been received from the pc/mac/other.
  byte code = 0;                   //used to hold the I2C response code.
  char RTD_data[20];               //we make a 20 byte character array to hold incoming data from the RTD circuit.
  byte in_char = 0;                //used as a 1 byte buffer to store in bound bytes from the RTD Circuit.
  byte i = 0;                      //counter used for RTD_data array.
  int time_ = 1000;                 //used to change the delay needed depending on the command sent to the EZO Class RTD Circuit.
  float tmp_float;                 //float var used to hold the float value of the RTD.

  float reading_float;
  int flag = 6;
  int timeout = 0;

  int str_len = cmd.length() + 1; 
  char computerdata[str_len];
  cmd.toCharArray(computerdata, str_len);
    
                                                      //if a command was sent to the EZO device.
    for (i = 0; i <= str_len; i++) {                               //set all char to lower case, this is just so this exact sample code can recognize the "sleep" command.
      computerdata[i] = tolower(computerdata[i]);                                 //"Sleep" ≠ "sleep"
    }
    i=0;                                                                          //reset i, we will need it later 
    if (computerdata[0] == 'c' || computerdata[0] == 'r')time_ = 600;             //if a command has been sent to calibrate or take a reading we wait 600ms so that the circuit has time to take the reading.
    else time_ = 250;                                                             //if any other command has been sent we wait only 250ms.


    Wire.beginTransmission(address);                                              //call the circuit by its ID number.
    Wire.write(computerdata);                                                     //transmit the command that was sent through the serial port.
    Wire.endTransmission();                                                       //end the I2C data transmission.


    if (strcmp(computerdata, "sleep") != 0) {                                     //if the command that has been sent is NOT the sleep command, wait the correct amount of time and request data.
                                                                                  //if it is the sleep command, we do nothing. Issuing a sleep command and then requesting data will wake the RTD circuit.
      while(flag != 0){
        delay(time_);                                                               //wait the correct amount of time for the circuit to complete its instruction.

        Wire.requestFrom(address, 20, 1);                                           //call the circuit and request 20 bytes (this may be more than we need)
        code = Wire.read();                                                         //the first byte is the response code, we read this separately.

        switch (code) {                       //switch case based on what the response code is.
          case 1: 
            flag = 0;                            //decimal 1.
            //Serial.println("Success");        //means the command was successful.          
            delay(2000);
            break;                            //exits the switch case.

          case 2:
            flag = 1;
            //decimal 2.
            //Serial.println("Failed");         //means the command has failed.
            break;                            //exits the switch case.

          case 254:                         //decimal 254.
            flag = 2;
            //Serial.println("Pending");        //means the command has not yet been finished calculating.
            break;                            //exits the switch case.

          case 255:                           //decimal 255.
            flag = 3;
            //Serial.println("No Data");        //means there is no further data to send.
            break;                            //exits the switch case.
        }
        timeout++;
        if(timeout == 3){
          flag = 0;
        }
      }
      while (Wire.available()) {            //are there bytes to receive.
        in_char = Wire.read();              //receive a byte.
        RTD_data[i] = in_char;              //load this byte into our array.
        i += 1;                             //incur the counter for the array element.
        if (in_char == 0) {                 //if we see that we have been sent a null command.
          i = 0;                            //reset the counter i to 0.
          break;                            //exit the while loop.
        }
      }

      //Serial.println(RTD_data);             //print the data.
    }
    serial_event = false;                   //reset the serial event flag.
    reading_float=atof(RTD_data);
    flag = 6;
    if (timeout == 3){
      reading_float = -1;
    }
    timeout = 0;
    return reading_float;                                                                                                                                                                                                                                                                                                                
}


float Read_from_I2C2(String cmd)
{
  byte received_from_computer = 0; //we need to know how many characters have been received.
  byte serial_event = 0;           //a flag to signal when data has been received from the pc/mac/other.
  byte code = 0;                   //used to hold the I2C response code.
  char RTD_data[20];               //we make a 20 byte character array to hold incoming data from the RTD circuit.
  byte in_char = 0;                //used as a 1 byte buffer to store in bound bytes from the RTD Circuit.
  byte i = 0;                      //counter used for RTD_data array.
  int time_ = 800;                 //used to change the delay needed depending on the command sent to the EZO Class RTD Circuit.
  float tmp_float;                 //float var used to hold the float value of the RTD.

  float reading_float;
  int flag = 6;
  int timeout = 0;

  int str_len = cmd.length() + 1; 
  char computerdata[str_len];
  cmd.toCharArray(computerdata, str_len);
    
                                                      //if a command was sent to the EZO device.
    for (i = 0; i <= str_len; i++) {                               //set all char to lower case, this is just so this exact sample code can recognize the "sleep" command.
      computerdata[i] = tolower(computerdata[i]);                                 //"Sleep" ≠ "sleep"
    }
    i=0;                                                                          //reset i, we will need it later 
    if (computerdata[0] == 'c' || computerdata[0] == 'r')time_ = 600;             //if a command has been sent to calibrate or take a reading we wait 600ms so that the circuit has time to take the reading.
    else time_ = 250;                                                             //if any other command has been sent we wait only 250ms.


    Wire.beginTransmission(address2);                                              //call the circuit by its ID number.
    Wire.write(computerdata);                                                     //transmit the command that was sent through the serial port.
    Wire.endTransmission();                                                       //end the I2C data transmission.


    if (strcmp(computerdata, "sleep") != 0) {                                     //if the command that has been sent is NOT the sleep command, wait the correct amount of time and request data.
                                                                                  //if it is the sleep command, we do nothing. Issuing a sleep command and then requesting data will wake the RTD circuit.
      while(flag != 0){
        delay(time_);                                                               //wait the correct amount of time for the circuit to complete its instruction.

        Wire.requestFrom(address2, 20, 1);                                           //call the circuit and request 20 bytes (this may be more than we need)
        code = Wire.read();                                                         //the first byte is the response code, we read this separately.

        switch (code) {                       //switch case based on what the response code is.
          case 1: 
            flag = 0;                            //decimal 1.
            //Serial.println("Success");        //means the command was successful.          
            delay(2000);
            break;                            //exits the switch case.

          case 2:
            flag = 1;
            //decimal 2.
            //Serial.println("Failed");         //means the command has failed.
            break;                            //exits the switch case.

          case 254:                         //decimal 254.
            flag = 2;
            //Serial.println("Pending");        //means the command has not yet been finished calculating.
            break;                            //exits the switch case.

          case 255:                           //decimal 255.
            flag = 3;
            //Serial.println("No Data");        //means there is no further data to send.
            break;                            //exits the switch case.
        }
        timeout++;
        if(timeout == 3){
          flag = 0;
        }
      }
      while (Wire.available()) {            //are there bytes to receive.
        in_char = Wire.read();              //receive a byte.
        RTD_data[i] = in_char;              //load this byte into our array.
        i += 1;                             //incur the counter for the array element.
        if (in_char == 0) {                 //if we see that we have been sent a null command.
          i = 0;                            //reset the counter i to 0.
          break;                            //exit the while loop.
        }
      }

      //Serial.println(RTD_data);             //print the data.
    }
    serial_event = false;                   //reset the serial event flag.
    reading_float=atof(RTD_data);
    flag = 6;
    if (timeout == 3){
      reading_float = -1;
    }
    timeout = 0;
    return reading_float;  
}
