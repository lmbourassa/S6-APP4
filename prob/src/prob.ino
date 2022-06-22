#include "message.h"
#include "crc16.h"

SYSTEM_THREAD(ENABLED);

Message message;
Message packet;
Message reception;

const uint8_t txPin = D4;
const uint8_t rxPin = D2;

enum rxState
{
  NEW,
  CLOCK,
  ZERO,
  ONE
}rxState;

system_tick_t chronoStart, period;
uint8_t bitCount = 0;

Timer timer(10, finishReception);

// Thread receptionThread("Reception", receptionFunc);
Thread disassemblyThread("Disassembly", disassemblyFunc);
//Thread extractionThread("Extraction", extractionFunc);
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

  attachInterrupt(rxPin, receptionFunc, FALLING, 0);

  bitCount = 0;

  Serial.println();
  Serial.printlnf("period: %lu us", period);

  reception.sendMessage();
}

void receptionFunc(void)
{
  system_tick_t chronoTemp = micros();
  system_tick_t chrono;

  timer.resetFromISR();

  switch (rxState)
  {
    case CLOCK:
    {
      period = chronoTemp - chronoStart;
      period += period/4; // ajusting for error

      chronoStart = chronoTemp;

      rxState = ZERO;

      break;
    }

    case ZERO:
    {
      chrono = chronoTemp - chronoStart;
      chronoStart = chronoTemp;
      bitCount++;

      if(chrono > period)
      {
        rxState = ONE;

        attachInterrupt(rxPin, receptionFunc, RISING, 0);
      }

      break;
    }

    case ONE:
    {
      chrono = chronoTemp - chronoStart;
      chronoStart = chronoTemp;
      bitCount++;

      if(chrono > period)
      {
        rxState = ZERO;

        attachInterrupt(rxPin, receptionFunc, FALLING, 0);
      }

      break;
    }
  
    default:
    {
      chronoStart = chronoTemp;

      rxState = CLOCK;

      reception.resetLength();

      timer.startFromISR();
      
      break;
    }
  }

  reception.setMessage((uint8_t*)&rxState, 1);
}

void disassemblyFunc(void)
{
  while(true)
  {
    uint8_t* pkt = reception.getMessage();
    uint8_t length = reception.getLength();

    for(uint8_t i = 0; i < length; i++)
    {
      if(ONE == pkt[i])
      {
        Serial.print("1");
      }

      else
      {
        Serial.print("0");
      }
    }

    Serial.println();
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
    message.resetLength();
    message.setMessage(testMsg, sizeof(testMsg));
    message.sendMessage();

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
      // Serial.printlnf("%02X\n", pkt[i]);

      for(uint8_t mask = 0x80; mask > 0x00; mask >>= 1)
      {
        if(mask == (mask & pkt[i]))
        {
          // Serial.print("1");
          sendOne();
        }

        else
        {
          // Serial.print("0");
          sendZero();
        }
      }
    }

    // Serial.println();
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