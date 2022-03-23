/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "usb/usb_host.h"
#include "class_driver.hpp"
#include "freertos/queue.h"
#include "MIDI.h"
#include "Arduino.h"

#define DAEMON_TASK_PRIORITY    2
#define CLASS_TASK_PRIORITY     3
#define SERIAL_MIDI_TASK_PRIORITY 3
static const char *TAG = "DAEMON";

//extern void class_driver_task(void *arg);

SemaphoreHandle_t signaling_sem;
TaskHandle_t daemon_task_hdl;
TaskHandle_t class_driver_task_hdl;
TaskHandle_t serial_midi_task_hdl;



static void host_lib_daemon_task(void *arg)
{
    SemaphoreHandle_t signaling_sem = (SemaphoreHandle_t)arg;

    ESP_LOGI(TAG, "Installing USB Host Library");
    usb_host_config_t host_config = {
        // .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_config));

    //Signal to the class driver task that the host library is installed
    xSemaphoreGive(signaling_sem);
    vTaskDelay(10); //Short delay to let client task spin up

    bool has_clients = true;
    bool has_devices = true;
    while (has_clients || has_devices ) {
        uint32_t event_flags;
        ESP_ERROR_CHECK(usb_host_lib_handle_events(portMAX_DELAY, &event_flags));
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            has_clients = false;
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            has_devices = false;
        }
    }
    ESP_LOGI(TAG, "No more clients and devices");

    //Uninstall the USB Host Library
    ESP_ERROR_CHECK(usb_host_uninstall());
    //Wait to be deleted
    xSemaphoreGive(signaling_sem);
    vTaskSuspend(NULL);
}

static void serial_midi_task(void *arg)
{
  struct MIDI_input_message Serial_MIDI_message = {0,0,0,0};
  MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, SERIAL_MIDI);
  SERIAL_MIDI.begin(MIDI_CHANNEL_OMNI);
  while(1)
  {
    if (SERIAL_MIDI.read())
    {
      uint8_t type = SERIAL_MIDI.getType();
      uint8_t channel = SERIAL_MIDI.getChannel();
      uint8_t data1 = SERIAL_MIDI.getData1();
      uint8_t data2 = SERIAL_MIDI.getData2();
      Serial_MIDI_message = { type, channel, data1, data2 };
      if (xQueueSendToBack(xMIDI_input_queue, (void *)&Serial_MIDI_message, (TickType_t)1) != pdPASS)
      {
      ESP_LOGE("Serial MIDI", "Error: MIDI Input Queue Full when pushing Serial MIDI");
      }
      ESP_LOGI("Serial MIDI", "type: %02x, channel: %02x, data1: %02x, data2:%02x", type, channel, data1, data2);
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void setup()
{
  signaling_sem = xSemaphoreCreateBinary();
  xMIDI_input_queue = xQueueCreate(30, sizeof(MIDI_input_message *));
  
  if (xMIDI_input_queue == NULL)
  {
    ESP_LOGE("", "Error, queue creation failed");
  }
    MIDI_USB_Task_Param midi_task_param = {
        xMIDI_input_queue,
        signaling_sem
    };
    //Create daemon task
    xTaskCreatePinnedToCore(host_lib_daemon_task,
                            "daemon",
                            4096,
                            (void *)signaling_sem,
                            DAEMON_TASK_PRIORITY,
                            &daemon_task_hdl,
                            0);
    //Create the class driver task
    xTaskCreatePinnedToCore(class_driver_task,
                            "class",
                            4096,
                            (void *)&midi_task_param,
                            CLASS_TASK_PRIORITY,
                            &class_driver_task_hdl,
                            0);
    xTaskCreatePinnedToCore(serial_midi_task,
                            "serial_midi",
                            4096,
                            NULL,
                            tskIDLE_PRIORITY,
                            &serial_midi_task_hdl,
                            0);
}
void loop()
{
  vTaskDelay(pdMS_TO_TICKS(2));     //Add a short delay to let the tasks run

  //Wait for the tasks to complete
  for (int i = 0; i < 2; i++) {
      xSemaphoreTake(signaling_sem, portMAX_DELAY);
  }

  //Delete the tasks
  vTaskDelete(class_driver_task_hdl);
  vTaskDelete(daemon_task_hdl);
  ESP.restart();
}