// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       Roundhouse Control.ino
    Created:	12/28/2018 3:46:36 PM
    Author:     Bob Gamble

  Designed by Gianni Barbotti (ITALY) on May 13 2018

This sketch was tried using:
1) the DCC_Decoder, Wire, Adafruit_PWMServoDriver libraries;
2) the sketch published at the link:
https://www.dccinterface.com/how-to/arduino-dcc-interface-accessory-decoder/
gave me the necessary starting point to prepare this project in order to achieve the use of micro-servos in the railway model to move the switches with slow motion.
3) the example sketch made available for the connection
https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library/blob/master/examples/servo/servo.ino
4) DCC control station Lenz LZV100 + Lenz LH100 + Lenz LHI101F it also works on the DCC NCE control station. I did not try it on the Digitrax DCS 100
5) serial-USB hama adapter to connect LHI101F to the PC on the USB port
6) the DCC Shield board for Arduino Uno designed by Luca Dentella, who gave it to me, sent for free and I thank you again.
in addition to the DCC interface for Ardino that I purchased a
https://www.dccinterface.com/how-to/connecting-interface-arduino/
and that also works on Arduino Mega 2560 R3 (I could not try the other because it no longer goes, my fault).
Both work beautifully. The advantage given by using the interface
designed by Luca Dentella is that it is a shield for Arduino Uno, to connect it to the Arduino and so on.
7) 2 x Adafruit PWM / 16 Bit 16 Bit Servo Driver - I2C Interface - PCA9685

N.B: if a servo is in the "positive" state and the same state is reactivated,
	 the servo will move rapidly in the opposite direction and then take the programmed position regularly
	 in the same way if it is in the "negative" state. This happens automatically.
	 In the sketch, therefore, I included the servo status check so that what is described above does not happen.

IMPORTANT OBSERVATION:
		  Whenever the Arduino "goes out" the sketch will lose the memory of the servos
		  status and therefore they will suffer the problem described above.
		  Further development will include storing the status of the servos on an SD card.
*/


#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <NmraDcc.h>

NmraDcc Dcc;

uint8_t lastDirection = 0xFF;

#define DCC_PIN 2 // define Pin for DCC and Arduino data transfer

// ****  important message    *****
//N.B. : The values of the following six lines are very important and are the only ones
// they must be modified according to the desired rotation for the servos and the
// Adafruit cards, then the number of servos you want to manage
// All other instructions should not be changed, unless you want to
// change the logic of the sketch

// int i_max_servo    = 31;  // maximum number of servos managed ( if Adafruit berakout are 2)

#define i_max_servo 10   // modify as desired
// #define num_ch_srv 16     //  number of channels beakout servos NOT modify!!!


Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40); // to manage servos from 1  to the 16(addresses 100-115)

/*
	 For each of the additionalbreakout Adafruit add the statement
	 Adafruit_PWMServoDriver pwmN = Adafruit_PWMServoDriver(0x4N);
	 where N is the progressive of the Adafruit breakout added


Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41); // to manage servos from 16 to the 31(addresses 116-131)
*/

volatile bool bInterruptDetected = false;
bool NextServo = true;
bool ServoActive = false;
int ActiveServo = 0;
uint8_t Lights = 0;
uint8_t Cycle = 0;

typedef struct
{
	int address;
	int servonum;
	bool active;
	int status;
	int ServoMin;
	int ServoMax;
	int Position;
}
DCCAccessoryAddress;
DCCAccessoryAddress sAddresses[i_max_servo];


void ConfigureServos()
// int status_servo[i_max_servo]; //memorize the status of servo  :  0 = if the switch are in deviate position
//                                                                1 = if the switch are in correct position
//                                                                2 = status switch on start sketch
// #define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
// #define SERVOMAX  400 // this is the 'maximum' pulse length count (out of 4096)

{
	sAddresses[0].address = 600;
	sAddresses[0].servonum = 0;
	sAddresses[0].active = false;
	sAddresses[0].status = 2;
	sAddresses[0].ServoMin = 250;
	sAddresses[0].ServoMax = 500;
	sAddresses[0].Position = 250;

	sAddresses[1].address = 601;
	sAddresses[1].servonum = 1;
	sAddresses[1].active = false;
	sAddresses[1].status = 2;
	sAddresses[1].ServoMin = 265;
	sAddresses[1].ServoMax = 500;
	sAddresses[1].Position = 265;

	sAddresses[2].address = 602;
	sAddresses[2].servonum = 2;
	sAddresses[2].active = false;
	sAddresses[2].status = 2;
	sAddresses[2].ServoMin = 235;
	sAddresses[2].ServoMax = 500;
	sAddresses[2].Position = 235;

	sAddresses[3].address = 603;
	sAddresses[3].servonum = 3;
	sAddresses[3].active = false;
	sAddresses[3].status = 2;
	sAddresses[3].ServoMin = 215;
	sAddresses[3].ServoMax = 490;
	sAddresses[3].Position = 215;

	sAddresses[4].address = 604;
	sAddresses[4].servonum = 4;
	sAddresses[4].active = false;
	sAddresses[4].status = 2;
	sAddresses[4].ServoMin = 240;
	sAddresses[4].ServoMax = 500;
	sAddresses[4].Position = 240;

	sAddresses[5].address = 605;
	sAddresses[5].servonum = 5;
	sAddresses[5].active = false;
	sAddresses[5].status = 2;
	sAddresses[5].ServoMin = 250;
	sAddresses[5].ServoMax = 500;
	sAddresses[5].Position = 250;

	sAddresses[6].address = 606;
	sAddresses[6].servonum = 6;
	sAddresses[6].active = false;
	sAddresses[6].status = 2;
	sAddresses[6].ServoMin = 255;
	sAddresses[6].ServoMax = 500;
	sAddresses[6].Position = 255;

	sAddresses[7].address = 607;
	sAddresses[7].servonum = 7;
	sAddresses[7].active = false;
	sAddresses[7].status = 2;
	sAddresses[7].ServoMin = 235;
	sAddresses[7].ServoMax = 500;
	sAddresses[7].Position = 235;

	sAddresses[8].address = 608;
	sAddresses[8].servonum = 8;
	sAddresses[8].active = false;
	sAddresses[8].status = 2;
	sAddresses[8].ServoMin = 245;
	sAddresses[8].ServoMax = 500;
	sAddresses[8].Position = 245;

	sAddresses[9].address = 609;
	sAddresses[9].servonum = 9;
	sAddresses[9].active = false;
	sAddresses[9].status = 2;
	sAddresses[9].ServoMin = 225;
	sAddresses[9].ServoMax = 500;
	sAddresses[9].Position = 225;
/*
	sAddresses[10].address = 610;
	sAddresses[10].servonum = 10;
	sAddresses[10].active = false;
	sAddresses[10].status = 2;
	sAddresses[10].ServoMin = 250;
	sAddresses[10].ServoMax = 500;
	sAddresses[10].Position = 250;

	sAddresses[11].address = 611;
	sAddresses[11].servonum = 11;
	sAddresses[11].active = false;
	sAddresses[11].status = 2;
	sAddresses[11].ServoMin = 250;
	sAddresses[11].ServoMax = 500;
	sAddresses[11].Position = 250;

	sAddresses[12].address = 612;
	sAddresses[12].servonum = 12;
	sAddresses[12].active = false;
	sAddresses[12].status = 2;
	sAddresses[12].ServoMin = 250;
	sAddresses[12].ServoMax = 500;
	sAddresses[12].Position = 250;

	sAddresses[13].address = 613;
	sAddresses[13].servonum = 13;
	sAddresses[13].active = false;
	sAddresses[13].status = 2;
	sAddresses[13].ServoMin = 250;
	sAddresses[13].ServoMax = 500;
	sAddresses[13].Position = 250;

	sAddresses[14].address = 614;
	sAddresses[14].servonum = 14;
	sAddresses[14].active = false;
	sAddresses[14].status = 2;
	sAddresses[14].ServoMin = 250;
	sAddresses[14].ServoMax = 500;
	sAddresses[14].Position = 250;

	sAddresses[15].address = 615;
	sAddresses[15].servonum = 15;
	sAddresses[15].active = false;
	sAddresses[15].status = 2;
	sAddresses[15].ServoMin = 250;
	sAddresses[15].ServoMax = 500;
	sAddresses[15].Position = 250;
	*/
}

// This function is called whenever a normal DCC Turnout Packet is received
void notifyDccAccTurnoutOutput(uint16_t Addr, uint8_t Direction, uint8_t OutputPower)
{
	// Convert NMRA packet address format to human address //

	//int pairAddress = (Direction >> 1) + 1;

	int outputInPair = Direction & 0x01;

//	int i_dec;

	Serial.print("notifyDccAccTurnoutOutput: ");
	Serial.print(Addr, DEC);
	Serial.print(',');
	Serial.print(Direction, DEC);
	Serial.print(',');
	Serial.print(OutputPower, HEX);
	Serial.print(',');
	Serial.print(" outputInPair = ");
	Serial.println(outputInPair, DEC);

	if ((Addr == 700))
	{
		if (Direction == 1)
		{
			digitalWrite(7, HIGH); // turn on
		}
		else
		{
			digitalWrite(7, LOW); // turn off
		}
	}
	if ((Addr == 701))
	{
		if (Direction == 1)
		{
			digitalWrite(8, HIGH); // turn on
		}
		else
		{
			digitalWrite(8, LOW); // turn off
		}
	}
	for (int i = 0; i < (sizeof(sAddresses) / sizeof(DCCAccessoryAddress)); i++)
	{
		//if ((Addr == sAddresses[i].address) && ((Addr != lastAddr) || (Direction != sAddresses[i].status)) && OutputPower)
		if ((Addr == sAddresses[i].address)  && (Direction != sAddresses[i].status) )
		{
		//	lastAddr = Addr;
			
			Serial.print(F("Activating Servo : "));
			Serial.println(i, DEC);

			sAddresses[i].active = true;
			sAddresses[i].status = Direction & 0x01;

			if (Direction)
			{
				Serial.print(F("Opening : "));
				Serial.println(sAddresses[i].address, DEC);
				break;
			}
			else
			{
				Serial.print(F("Closing : "));
				Serial.println(sAddresses[i].address, DEC);
				break;
			}
		}
	}
};
void setupDCCDecoder()
{
	Serial.println(F("Setting up DCC Decorder..."));

	// Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
	Dcc.pin(0, 2, 1);

	// Call the main DCC Init function to enable the DCC Receiver
	Dcc.init(MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 0);
}

void setup()
{
	Serial.begin(115200);
	while (!Serial);   // Wait for the USB Device to Enumerate

	Serial.print(F("Start sketch "));
	Serial.println("- decoder ready");

	pinMode(7, OUTPUT);
	digitalWrite(7, HIGH); // turn off
	pinMode(8, OUTPUT);
	digitalWrite(8, HIGH); // turn off

	ConfigureServos();
	
	setupDCCDecoder();

	  // Inizialize Pin 2 

	/* 
	Dcc.pin(digitalPinToInterrupt(DCC_PIN), DCC_PIN, 1);

	Dcc.init(MAN_ID_DIY, 1, FLAGS_DCC_ACCESSORY_DECODER, 0);  // Serial.println("- decoder ready");
	*/

  // Inizialize PCA9685 breakout 1

	pwm1.begin();

 Serial.println("pwml begun ");
	// frequency set to 60 Hz  - NOT modify!!!!

	pwm1.setPWMFreq(60);   // frequency set to 60 Hz  - NOT modify!!!!
  
 Serial.println("pwml frequency set");
	delay(10);

	// Inizialize PCA9685 breakout 2
	/*
	pwm2.begin();
	pwm2.setPWMFreq(60);   // frequency set to 60 Hz  - NOT modify!!!!
	*/
}


void loop()
{
	// demo code - set up next servo to run
	// Serial.println("Loop ");
	ServoActive = false;
	for (int i = 0; i < (sizeof(sAddresses) / sizeof(DCCAccessoryAddress)); i++)
	{
		if (sAddresses[i].active)
		{
			ServoActive = true;
		}
	}
	if (ServoActive == false)
	{
		sAddresses[ActiveServo].active = true;
		sAddresses[ActiveServo].status = Cycle;
		ActiveServo++;
		// Serial.println("Cycled ");
		if (ActiveServo >= i_max_servo)
		{
			ActiveServo = 0;
		  if (Cycle==1)
			 Cycle =0;
		  else
			 Cycle = 1;
		
			if (Lights == 1)
			{
			  digitalWrite(7, HIGH); // turn on
			  digitalWrite(8, HIGH); // turn on
			  Lights = 0;
			}
			else
			{
			  digitalWrite(7, LOW); // turn off
			  digitalWrite(8, LOW); // turn off
			  Lights = 1;
			}
		}
	}
	// Drive each servo one at a time
	for (int i = 0; i < (sizeof(sAddresses) / sizeof(DCCAccessoryAddress)); i++)
	{

		if (sAddresses[i].active ) 
		{
			Serial.print("Servo Active: ");
			Serial.print(sAddresses[i].servonum, DEC);
			Serial.print(',');
			Serial.print(sAddresses[i].status, DEC);
			Serial.print(',');
			Serial.println(sAddresses[i].Position, DEC);
			
			pwm1.setPWM(sAddresses[i].servonum, 0, sAddresses[i].Position);
			if ( (sAddresses[i].status == 1))
			{
				sAddresses[i].Position++;
				if (sAddresses[i].Position >= sAddresses[i].ServoMax)
				{
					sAddresses[i].active = false;
					Serial.print("Servo Open: ");
					Serial.print(sAddresses[i].servonum, DEC);
					Serial.print(',');
					Serial.print(sAddresses[i].status, DEC);
					Serial.print(',');
					Serial.println(sAddresses[i].Position, DEC);			
				}
				
			}
			else
			{
				sAddresses[i].Position--;
				if (sAddresses[i].Position <= sAddresses[i].ServoMin)
				{
					sAddresses[i].active = false;
					Serial.print("Servo Closed: ");
					Serial.print(sAddresses[i].servonum, DEC);
					Serial.print(',');
					Serial.print(sAddresses[i].status, DEC);
					Serial.print(',');
					Serial.println(sAddresses[i].Position, DEC);
				}
			}
		}
	}
	delay(20);
//	Dcc.process();
}
