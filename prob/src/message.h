#ifndef MESSAGE_H
#define MESSAGE_H

#include <spark_wiring_thread.h>

class Message
{
private:
    uint8_t data[1024];
    uint8_t length;
    Mutex hasDataLock; // Locked when has no data, unlocked when has data

public:
    Message();
    void setMessage(uint8_t* msg, uint8_t length);
    uint8_t* getMessage();
    uint8_t getLength();
    void sendMessage();
    void resetLength();
};

#endif