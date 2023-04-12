#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define BUF_SIZE 1024
volatile bool is_read_complete = false;
char* buf = nullptr;

void read_serial(void *arg)
{
  while(1)
  {
    // Serial.print("************** High Water Mark (stack size left): ");
    // Serial.println(uxTaskGetStackHighWaterMark(NULL));
    // Serial.print("************** Heap Size left: ");
    // Serial.println(xPortGetFreeHeapSize());
    if(!is_read_complete)
    {
      buf = (char*) pvPortMalloc(BUF_SIZE);
      if(buf)
      {
        memset(buf,'\0',(size_t)BUF_SIZE);
        Serial.read(buf,BUF_SIZE);
        is_read_complete = true;
      }
    }
  }
}

void print_serial_message(void *)
{
  while(1)
  {
    if(is_read_complete)
    {
      Serial.println(buf);
      vPortFree(buf);
      is_read_complete = false;
    }
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  xTaskCreatePinnedToCore(
      read_serial,
      "read serial",
      1024,
      NULL,
      1,
      NULL,
      1);

  xTaskCreatePinnedToCore(
      print_serial_message,
      "print_serial_message",
      1024,
      NULL,
      1,
      NULL,
      1);
    
  // vTaskDelete(NULL);

}

void loop()
{
  // put your main code here, to run repeatedly:
}