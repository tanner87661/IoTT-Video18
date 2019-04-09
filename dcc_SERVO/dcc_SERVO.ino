#include <NmraDcc.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

NmraDcc  Dcc ;

const int DccAckPin = 3 ; //not connected
//int firstServo = 21; //no longer used
#define numServos 16

typedef struct {
  uint16_t servoAddr;
  uint16_t  minPos;
  uint16_t  maxPos;
  uint16_t  targetPos;
  uint16_t  currPos;
  uint8_t  moveDelay;
  uint32_t lastMove;
} myServo;

myServo servoArray[numServos] = {
                            {24, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,10,0},
                            {25, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0},
                            {50, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0},
                            {51, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0},
                            
                            {10, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0},
                            {11, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0},
                            {16, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0},
                            {17, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0},
                            
                            {18, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0},
                            {23, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0},
                            {33, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0},
                            {34, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0},
                            
                            {10800, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0},
                            {10802, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0},
                            {10804, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0},
                            {10806, SERVOMIN, SERVOMAX,SERVOMIN, SERVOMIN+1,0,0},
  
                          };

// This function is called whenever a normal DCC Turnout Packet is received and we're in Output Addressing Mode
void notifyDccAccTurnoutOutput( uint16_t Addr, uint8_t Direction, uint8_t OutputPower )
{
  Serial.print("notifyDccAccTurnoutOutput: ") ;
  Serial.print(Addr,DEC) ;
  Serial.print(',');
  Serial.print(Direction,DEC) ;
  Serial.print(',');
  Serial.println(OutputPower, HEX) ;
  for (int i=0; i < numServos; i++)
    if (Addr == servoArray[i].servoAddr)
      if (Direction == 0)
        servoArray[i].targetPos = servoArray[i].minPos;
      else
        servoArray[i].targetPos = servoArray[i].maxPos;
}

// This function is called whenever a DCC Signal Aspect Packet is received
void notifyDccSigOutputState( uint16_t Addr, uint8_t State)
{
  Addr += 10000;
  Serial.print("notifyDccSigOutputState: ") ;
  Serial.print(Addr,DEC) ;
  Serial.print(',');
  Serial.println(State, HEX) ;
  for (int i=0; i < numServos; i++)
    if (Addr == servoArray[i].servoAddr)
      servoArray[i].targetPos = min(servoArray[i].maxPos, servoArray[i].minPos +(State * ((servoArray[i].maxPos - servoArray[i].minPos)/8)));
}

void processLocations()
{
  for (int i=0; i < numServos; i++)
    if (servoArray[i].targetPos != servoArray[i].currPos) 
    {
      if ((servoArray[i].lastMove + servoArray[i].moveDelay < millis()))
      {
        if (servoArray[i].targetPos > servoArray[i].currPos)
        {
          servoArray[i].currPos++;
          pwm.setPWM(i, 0, servoArray[i].currPos);
        }
        else
        {
          servoArray[i].currPos--;
          pwm.setPWM(i, 0, servoArray[i].currPos);
        }
        servoArray[i].lastMove = millis(); 
      }
    }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  // Configure the DCC CV Programing ACK pin for an output
  pinMode( DccAckPin, OUTPUT );

  Serial.println("NMRA DCC Example 1");
  
  // Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
  Dcc.pin(0, 2, 1);
  
  // Call the main DCC Init function to enable the DCC Receiver
  Dcc.init( MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER | CV29_OUTPUT_ADDRESS_MODE, 0 );

  Serial.println("Init Done");

  pwm.begin();
  
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

}

void loop() {
  // put your main code here, to run repeatedly:
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();
  processLocations();
}
