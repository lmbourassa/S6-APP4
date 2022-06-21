#include "message.h"
#include <string.h>

Message::Message()
{
    length = 0;
    hasDataLock.lock();
}

void Message::setMessage(uint8_t* msg, uint8_t length)
{
    memcpy(&(data[this->length]), msg, length);

    this->length += length;
}

uint8_t* Message::getMessage()
{
    hasDataLock.lock();

    return data;
}

uint8_t Message::getLength()
{
    return length;
}

void Message::sendMessage()
{
    hasDataLock.unlock();
}

void Message::resetLength()
{
    length = 0;
}