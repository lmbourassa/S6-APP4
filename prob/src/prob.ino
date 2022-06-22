#include "message.h"
#include "crc16.h"

#define MANCHESTER_BIT_PERIOD_US 300
#define HALF_PERIOD  MANCHESTER_BIT_PERIOD_US/2

SYSTEM_THREAD(ENABLED);

Message message;
Message packet;
Message reception;
Message dissassembled;

const uint8_t txPin = D4;
const uint8_t rxPin = D2;
const uint8_t scramblePin = A4;
const uint8_t msgPin = A3;

enum rxState
{
  NEW,
  CLOCK,
  RECEIVING,
}rxState;

system_tick_t chronoStart, period;
bool bitValue = 0;

Timer timer(10, finishReception);

Thread disassemblyThread("Disassembly", disassemblyFunc);
Thread extractionThread("Extraction", extractionFunc);
Thread insertionThread("Insertion", insertionFunc);
Thread assemblyThread("Assembly", assemblyFunc);
Thread TransmissionThread("Transmission", transmissionFunc, OS_THREAD_PRIORITY_CRITICAL);

void setup()
{
  Serial.begin(9600);
  waitFor(Serial.isConnected, 30000);

  rxState = NEW;
  attachInterrupt(rxPin, receptionFunc, FALLING, 0);
}

void loop()
{
  delay(100);
}

void finishReception(void)
{
  timer.stopFromISR();
  
  rxState = NEW;
  bitValue = 0;

  attachInterrupt(rxPin, receptionFunc, FALLING, 0);

  reception.sendMessage();
}

void receptionFunc(void)
{
  system_tick_t chronoTemp = micros();

  timer.resetFromISR();

  switch (rxState)
  {
    case CLOCK:
    {
      period = chronoTemp - chronoStart;
      period -= period/4; // ajusting for error

      chronoStart = chronoTemp;

      rxState = RECEIVING;

      attachInterrupt(rxPin, receptionFunc, CHANGE, 0);

      reception.setMessage((uint8_t*)&bitValue, 1);

      break;
    }

    case RECEIVING:
    {
      if((chronoTemp - chronoStart) < period)
      {
        bitValue = !bitValue;

        break;
      }

      bitValue = !bitValue;

      chronoStart = chronoTemp;

      reception.setMessage((uint8_t*)&bitValue, 1);

      break;
    }
  
    default:
    {
      chronoStart = chronoTemp;

      rxState = CLOCK;

      reception.resetLength();

      timer.startFromISR();

      reception.setMessage((uint8_t*)&bitValue, 1);
      
      break;
    }
  }
}

void disassemblyFunc(void)
{
  while(true)
  {
    uint8_t* pkt = reception.getMessage();
    uint8_t length = reception.getLength();

    dissassembled.resetLength();

    uint8_t temp = 0x00;
    uint8_t mask = 0x80;


    for(uint8_t i = 0; i < length; i++)
    {
      if(1 == pkt[i])
      {
        temp |= mask;
      }

      mask = mask >> 1;

      if(0x00 == mask)
      {
        mask = 0x80;
        dissassembled.setMessage(&temp, 1);
        temp = 0x00;
      }

    }

    dissassembled.sendMessage();

    os_thread_yield();
  }
}

void extractionFunc(void)
{
  while(true)
  {
    uint8_t* msg = dissassembled.getMessage();

    uint8_t actualLength = msg[3];
    uint8_t actualMessage[actualLength];
    uint8_t crcParts[2];

    memcpy(actualMessage, &(msg[4]), actualLength);
    memcpy(crcParts, &(msg[actualLength + 4]), 2);

    uint16_t rxcrc = (((uint16_t)crcParts[0]) << 8) | crcParts[1];

    uint16_t crc = crc16(actualMessage, actualLength);

    if(crc == rxcrc)
    {
      Serial.printlnf("Received: %s", actualMessage);
    }

    else
    {
      Serial.println("CRC error");
    }

    os_thread_yield();
  }
}

void insertionFunc(void)
{
  pinMode(msgPin, INPUT_PULLUP);

  while(true)
  {
    if(digitalRead(msgPin))
    {
      uint8_t testMsg[22] = "Congolexicomatisation";
      message.resetLength();
      message.setMessage(testMsg, sizeof(testMsg));
    }

    else
    {
      uint8_t testMsg[19] = "Bonjour Etienne :)";
      message.resetLength();
      message.setMessage(testMsg, sizeof(testMsg));
    }

    message.sendMessage();

    delay(2000);
  }
}

void assemblyFunc(void)
{
  // pinMode(scramblePin, INPUT_PULLUP);

  while(true)
  {
    uint8_t* msg = message.getMessage();
    uint8_t length = message.getLength();

    uint16_t crc = crc16(msg, length);

    uint8_t pkt[length + 7];
    pkt[0] = 0;                                     // Preambule
    pkt[1] = 0b01111110;                            // Start
    pkt[2] = 0x0;                                   // Type + Flag
    pkt[3] = length;                                // Message length
    memcpy(&pkt[4], msg, length);  // Message
    pkt[length + 4] = crc >> 8;                     // CRC16 MSB
    pkt[length + 5] = crc;                          // CRC16 LSB
    pkt[length + 6] = 0b01111110;                   // End

    // if(digitalRead(scramblePin))
    // {
    //   pkt[4] = 0xFF;
    // }

    packet.resetLength();
    packet.setMessage(pkt, sizeof(pkt));
    packet.sendMessage();

    os_thread_yield();
  }
}

void transmissionFunc(void)
{
  pinMode(txPin, OUTPUT);

  while(true)
  {
    uint8_t* pkt = packet.getMessage();
    uint8_t length = packet.getLength();

    for(uint8_t i = 0; i < length; i++)
    {
      for(uint8_t mask = 0x80; mask > 0x00; mask >>= 1)
      {
        if(mask == (mask & pkt[i]))
        {
          sendOne();
        }

        else
        {
          sendZero();
        }
      }
    }

    os_thread_yield();
  }
}

void sendZero()
{
  delayMicroseconds(HALF_PERIOD);
  digitalWrite(txPin, HIGH);

  delayMicroseconds(HALF_PERIOD);
  digitalWrite(txPin, LOW);
}

void sendOne()
{
  delayMicroseconds(HALF_PERIOD);
  digitalWrite(txPin, LOW);

  delayMicroseconds(HALF_PERIOD);
  digitalWrite(txPin, HIGH);
}