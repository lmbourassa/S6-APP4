#include "message.h"
#include "crc16.h"

SYSTEM_THREAD(ENABLED);

Message message;
Message packet;

const uint8_t txPin = D4;
const uint8_t rxPin = D2;

enum rxState
{
  NEW,
  CLOCK,
  ZERO,
  ONE
}rxState;

system_tick_t chronoStart, chronoStop, chrono, period, error;
uint8_t bitCount = 0;

// Thread receptionThread("Reception", receptionFunc);
//Thread disassemblyThread("Disassembly", disassemblyFunc);
//Thread extractionThread("Extraction", extractionFunc);
Thread insertionThread("Insertion", insertionFunc);
Thread assemblyThread("Assembly", assemblyFunc);
Thread TransmissionThread("Transmission", transmissionFunc, OS_THREAD_PRIORITY_CRITICAL);

void setup()
{
  Serial.begin(9600);
  waitFor(Serial.isConnected, 30000);

  attachInterrupt(rxPin, receptionFunc, FALLING, 0);
}

void loop()
{
  delay(100);
}

void receptionFunc(void)
{
  switch (rxState)
  {
    case CLOCK:
    {
      chronoStop = micros();

      period = chronoStop - chronoStart;
      chronoStart = chronoStop;
      error = period/4;

      bitCount++;

      if(7 == bitCount)
      {
        bitCount = 0;
        rxState = ZERO;

        detachInterrupt(rxPin);
        attachInterrupt(rxPin, receptionFunc, CHANGE, 0);
      }

      break;
    }

    case ZERO:
    {
      chronoStop = micros();
      chrono = chronoStop - chronoStart;
      chronoStart = chronoStop;
      bitCount++;

      if(chronoStop > (period + error))
      {
        rxState = ONE;

        Serial.print("1");
      }

      else
      {
        rxState = ZERO;
        Serial.print("0");
      }

      break;
    }

    case ONE:
    {
      chronoStop = micros();
      chrono = chronoStop - chronoStart;
      chronoStart = chronoStop;
      bitCount++;

      if(chronoStop > (period + error))
      {
        rxState = ZERO;

        Serial.print("0");
      }

      else
      {
        rxState = ONE;
        Serial.print("1");
      }

      break;
    }
  
    default:
    {
      chronoStart = micros();

      rxState = CLOCK;

      detachInterrupt(rxPin);
      attachInterrupt(rxPin, receptionFunc, FALLING, 0);
      
      break;
    }
  }

  if(bitCount >= 88)
  {
    rxState = NEW;

    detachInterrupt(rxPin);
    attachInterrupt(rxPin, receptionFunc, FALLING, 0);

    bitCount = 0;

    Serial.println();
    Serial.printlnf("period: %lu us + %lu us", period, error);
  }
}

void disassemblyFunc(void)
{
  while(true)
  {
    WITH_LOCK(Serial)
    {
      Serial.println("Disassembly");
    }

    os_thread_yield();
  }
}

void extractionFunc(void)
{
  while(true)
  {
    WITH_LOCK(Serial)
    {
      Serial.println("Extraction");
    }

    os_thread_yield();
  }
}

void insertionFunc(void)
{
  while(true)
  {
    uint8_t testMsg[5] = "test";
    message.setMessage(testMsg, sizeof(testMsg));

    delay(2000);
  }
}

void assemblyFunc(void)
{
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

    packet.setMessage(pkt, sizeof(pkt));

    os_thread_yield();
  }
}

void transmissionFunc(void)
{
  uint16_t clk = {0x5555};
  uint16_t man[80] = {0};
  uint8_t mask = 0x1;

  pinMode(txPin, OUTPUT);

  while(true)
  {
    uint8_t* pkt = packet.getMessage();
    uint8_t length = packet.getLength();

    // for(uint8_t i = 0; i < length; i++)
    // {
    //     uint16_t temp = 0;
        
    //     for(uint8_t j = 0; j < 8; j++)
    //     {
    //         uint16_t a = ((pkt[i] >> (7 - j)) & mask);
    //         temp = temp << 2;
    //         temp |=  ((a << 1) | a);
    //     }

    //     man[i] = clk ^ temp;
    // }

    for(uint8_t i = 0; i < length; i++)
    {
      for(uint16_t mask = 0x8000; mask > 0x0000; mask >>= 1)
      {
        // if(mask == (mask & man[i]))
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
  delayMicroseconds(1000);
  digitalWrite(txPin, HIGH);

  delayMicroseconds(1000);
  digitalWrite(txPin, LOW);
}

void sendOne()
{
  delayMicroseconds(1000);
  digitalWrite(txPin, LOW);

  delayMicroseconds(1000);
  digitalWrite(txPin, HIGH);
}