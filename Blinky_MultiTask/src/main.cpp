#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
// #include "soc/timer_group_struct.h"
// #include "soc/timer_group_reg.h"

#define LED_PIN 2

//default rate
static int led_blink_rate = 1000;

void serial_read(void* arg)
{
  Serial.print("\n----serial_read() is running on core-> ");
  Serial.println(xPortGetCoreID());
  Serial.println("Enter the LED blink delay in ms: ");
  while(1)
  {
    int tmp = Serial.parseInt();
    if(tmp > 0)
    {
      led_blink_rate = tmp;
      Serial.print("Updated LED delay to: ");
      Serial.println(led_blink_rate);
    }
  }
}

void blink_led(void* arg)
{
  Serial.print("\n----blink_led() is running on core-> ");
  Serial.println(xPortGetCoreID());
  while(1)
  {
    digitalWrite(LED_PIN,HIGH);
    vTaskDelay(led_blink_rate/portTICK_PERIOD_MS);
    digitalWrite(LED_PIN,LOW);
    vTaskDelay(led_blink_rate/portTICK_PERIOD_MS);
  }
}


void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN,OUTPUT);
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS); //wait for serial connection
  xTaskCreatePinnedToCore(
    serial_read,
    "read value from serial",
    1024,
    nullptr,
    1,
    nullptr,
    1
  );

   xTaskCreatePinnedToCore(
    blink_led,
    "blink led",
    1024,
    nullptr,
    1,
    nullptr,
    1
  );
  //delete setup and loop task
  vTaskDelete(NULL);
}

void loop() {
  // to avoid watchdog reset error (only occurs when using dual core)
  // TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
  // TIMERG0.wdt_feed=1;
  // TIMERG0.wdt_wprotect=0;
}