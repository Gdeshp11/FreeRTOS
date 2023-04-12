#include <Arduino.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define LED_PIN 2
static SemaphoreHandle_t mutex;

void blink_led(void* arg)
{
  int delay = *(int*)arg;
  xSemaphoreGive(mutex);

  Serial.print("task arg:: ");
  Serial.println(*(int*)delay);

  while(1)
  {
    digitalWrite(LED_PIN,HIGH);
    vTaskDelay(delay/portTICK_PERIOD_MS);
    digitalWrite(LED_PIN,LOW);
    vTaskDelay(delay/portTICK_PERIOD_MS);
  }  
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED_PIN,OUTPUT);
  mutex = xSemaphoreCreateMutex();

  while(Serial.available() <=0);
  int delay_arg = Serial.parseInt();

  Serial.print("Sending delay arg to task: ");
  Serial.println(delay_arg);
  
  xSemaphoreTake(mutex,portMAX_DELAY);
  
  xTaskCreatePinnedToCore(
      blink_led,
      "blink led",
      1024,
      (void*)&delay_arg,
      1,
      NULL,
      1);
  
  xSemaphoreTake(mutex,portMAX_DELAY);

  Serial.println("Setup() done!");
}

void loop() {
  // put your main code here, to run repeatedly:
   vTaskDelay(1000 / portTICK_PERIOD_MS);
}