// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       Roundhouse Control.ino
    Created:    12/28/2018 3:46:36 PM
    Author:     Bob Gamble and others from posted code examples

This sketch uses:
1) the DCC_Decoder, Wire, Adafruit_PWMServoDriver libraries;
2) PWM / 16 Bit 16 Bit Servo Driver - I2C Interface - PCA9685

Note if a servo is in the "positive" state and the same state is reactivated,
 the servo will move rapidly in the opposite direction and then take the programmed position regularly
 in the same way if it is in the "negative" state. This happens automatically.
 In the sketch, therefore, I included the servo status check so that what is described above does not happen.

IMPORTANT OBSERVATION:
Whenever the Arduino "goes out" the sketch will lose the memory of the servo
status and therefore they will suffer the problem described above.
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <NmraDcc.h>

NmraDcc Dcc;

uint8_t lastDirection = 0xFF;

#define DCC_PIN 2	 // define Pin for DCC and Arduino data transfer

#define i_max_servo 10   // modify as desired, you can have 16 for each PCA9685

Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40); // to manage servos from 1  to 16 (addresses 100-115)

volatile bool bInterruptDetected = false;

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
// memory for the status of servo:  
//			0 = deviate position
//                                         1 = correct position
//                                         2 = status on start sketch

{	// these are status arrays for each servo
	sAddresses[0].address = 600;	// DCC address for this servo
	sAddresses[0].servonum = 0;	// number of servo in the code
	sAddresses[0].active = false;	// servo in use flag
	sAddresses[0].status = 2;		// flag for opening or closing of servo
	sAddresses[0].ServoMin = 250;	// position of servo at close
	sAddresses[0].ServoMax = 500;	// position of servo at open
	sAddresses[0].Position = 250;	// current position

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

/*	repeat the above construct for any additional servos implemented 	*/
}

// This function is called whenever a normal DCC Turnout Packet is received

void notifyDccAccTurnoutOutput(uint16_t Addr, uint8_t Direction, uint8_t OutputPower)
{
	// Convert NMRA packet address format to human address //

	int outputInPair = Direction & 0x01;

// the following prints to the serial monitor for debugging purposes
	Serial.print("notifyDccAccTurnoutOutput: ");
	Serial.print(Addr, DEC);
	Serial.print(',');
	Serial.print(Direction, DEC);
	Serial.print(',');
	Serial.print(OutputPower, HEX);
	Serial.print(',');
	Serial.print(" outputInPair = ");
	Serial.println(outputInPair, DEC);

	if ((Addr == 700)) // hard coded address for controlling LED lights from pin 7
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
	if ((Addr == 701)) // hard coded address for controlling LED lights from pin 8
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
		if ((Addr == sAddresses[i].address)  && (Direction != sAddresses[i].status) )
		{
					
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
	digitalWrite(7, LOW); // turn off
	pinMode(8, OUTPUT);
	digitalWrite(8, LOW); // turn off

	ConfigureServos();
	
	setupDCCDecoder();	

  // Initialize PCA9685

	pwm1.begin();
	pwm1.setPWMFreq(60);   // frequency set to 60 Hz  - NOT modify!!!!
	delay(10);
}

void loop()
/* this is the loop the code continuously executes after everything is initialized. When there is DCC traffic the code is interrupted to process the message and if there is a command to a servo that respective status array is updated. The changes to the status array will cause this code loop to move the servo as commanded */
{
	// Drive each servo one at a time
	for (int i = 0; i < (sizeof(sAddresses) / sizeof(DCCAccessoryAddress)); i++)
	{
		if (sAddresses[i].active ) 
		{	
			pwm1.setPWM(sAddresses[i].servonum, 0, sAddresses[i].Position);
			if ( (sAddresses[i].status == 1))
			{
				sAddresses[i].Position++;
				if (sAddresses[i].Position == sAddresses[i].ServoMax)
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
				if (sAddresses[i].Position == sAddresses[i].ServoMin)
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
	delay(20);  // increase the delay to slow the rate of the servo movement
	Dcc.process();
}
