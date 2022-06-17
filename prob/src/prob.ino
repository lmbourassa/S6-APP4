SYSTEM_THREAD(ENABLED);

Mutex mutex_serial;

Thread receptionThread("Reception", receptionFunc);
Thread disassemblyThread("Disassembly", disassemblyFunc);
Thread extractionThread("Extraction", extractionFunc);
Thread insertionThread("Insertion", insertionFunc);
Thread assemblyThread("Assembly", assemblyFunc);
Thread dispatchThread("Dispatch", dispatchFunc);

void setup()
{
  Serial.begin(9600);
  waitFor(Serial.isConnected, 30000);
}

void loop()
{
  delay(100);
}

void receptionFunc(void)
{
  while(true)
  {
    mutex_serial.lock();
    Serial.println("Reception");
    mutex_serial.unlock();

    os_thread_yield();
  }
}

void disassemblyFunc(void)
{
  while(true)
  {
    mutex_serial.lock();
    Serial.println("Disassembly");
    mutex_serial.unlock();

    os_thread_yield();
  }
}

void extractionFunc(void)
{
  while(true)
  {
    mutex_serial.lock();
    Serial.println("Extraction");
    mutex_serial.unlock();

    os_thread_yield();
  }
}

void insertionFunc(void)
{
  while(true)
  {
    mutex_serial.lock();
    Serial.println("Insertion");
    mutex_serial.unlock();

    os_thread_yield();
  }
}

void assemblyFunc(void)
{
  while(true)
  {
    mutex_serial.lock();
    Serial.println("Assembly");
    mutex_serial.unlock();

    os_thread_yield();
  }
}

void dispatchFunc(void)
{
  while(true)
  {
    mutex_serial.lock();
    Serial.println("Dispatch");
    mutex_serial.unlock();

    os_thread_yield();
  }
}