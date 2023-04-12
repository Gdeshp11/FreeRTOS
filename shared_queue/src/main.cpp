#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define QUEUE_SIZE 100
#define MSG_LEN 100
#define LED_PIN 2

static QueueHandle_t msg_queue1;
static QueueHandle_t msg_queue2;


void read_serial(void* arg)
{
  char buf[10] = {0};
  int blink_delay;

  while(1)
  {
    memset(buf,0,10);
    // print 'blinked' after 100 led blinks
    if(xQueueReceive(msg_queue2,(void*)&buf,0) == pdTRUE)
    {
      Serial.println(buf);
    }

    String ip = Serial.readString();

    if(ip.indexOf("delay") >= 0)
    {
      blink_delay = ip.substring(ip.indexOf(" ") + 1).toInt();
      Serial.print("parsed delay from serial: ");
      Serial.println(blink_delay);
      if(xQueueSend(msg_queue1,(void*)&blink_delay,10) != pdTRUE)
      {
        Serial.println("can't add delay to msg_queue1: full!");
      }
    }
    else if(!ip.isEmpty())
    {
      Serial.print("User Input: ");
      Serial.println(ip);
    }
  }

}

void blink_led(void* arg)
{
  uint8_t blink_counter = 0;
  const char* msg = "Blinked";
  int led_blink_delay = 1000;

  while(1)
  {
    if(xQueueReceive(msg_queue1,(void*)&led_blink_delay,0) == pdTRUE)
    {
      Serial.print("xQueueReceive->updated led blink delay to: ");
      Serial.println(led_blink_delay);
    }
    if(blink_counter<100)
    {
      digitalWrite(LED_PIN,HIGH);
      vTaskDelay(led_blink_delay/portTICK_PERIOD_MS);
      digitalWrite(LED_PIN,LOW);
      vTaskDelay(led_blink_delay/portTICK_PERIOD_MS);
      blink_counter++;
    }
    else if(blink_counter == 100)
    {
      blink_counter = 0;
      if(xQueueSend(msg_queue2,(void*)msg,10) != pdTRUE)
      {
        Serial.println("can't add \"Blinked\" to msg_queue2: Full!");
      }
    }
  }
}




void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    pinMode(LED_PIN,OUTPUT);
    vTaskDelay(1000/portTICK_PERIOD_MS);

    msg_queue1 = xQueueCreate(QUEUE_SIZE,sizeof(int));
    msg_queue2 = xQueueCreate(QUEUE_SIZE,MSG_LEN);

  xTaskCreatePinnedToCore(
      read_serial,
      "read_serial",
      1024,
      NULL,
      1,
      NULL,
      1);

  xTaskCreatePinnedToCore(
      blink_led,
      "blink_led",
      1024,
      NULL,
      1,
      NULL,
      0);
      
      // vTaskDelete(NULL);
}

void loop() {
  // put your main code here, to run repeatedly:
}