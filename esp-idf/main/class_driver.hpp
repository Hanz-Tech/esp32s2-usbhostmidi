/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "show_desc.hpp"

#define CLIENT_NUM_EVENT_MSG        5

#define ACTION_OPEN_DEV             0x01
#define ACTION_GET_DEV_INFO         0x02
#define ACTION_GET_DEV_DESC         0x04
#define ACTION_GET_CONFIG_DESC      0x08
#define ACTION_GET_STR_DESC         0x10
#define ACTION_CLOSE_DEV            0x20
#define ACTION_EXIT                 0x40

bool isMIDI = false;
bool isMIDIReady = false;

const size_t MIDI_IN_BUFFERS = 1;
const size_t MIDI_OUT_BUFFERS = 8;
usb_transfer_t *MIDIOut = NULL;
usb_transfer_t *MIDIIn[MIDI_IN_BUFFERS] = {NULL};
QueueHandle_t xMIDI_input_queue = NULL;
// usb_host_client_handle_t driver_obj.client_hdl;
// usb_device_handle_t driver_obj.dev_hdl;


typedef struct MIDI_USB_Task_Param
{
    QueueHandle_t queue;
    SemaphoreHandle_t signaling_sem;
} midi_driver_task_params;

struct MIDI_input_message
{
  uint8_t type;
  uint8_t channel;
  uint8_t data1;
  uint8_t data2;
} xMessage;

struct MIDI_input_message USB_MIDI_message = {0,0,0,0};

typedef struct {
    usb_host_client_handle_t client_hdl;
    uint8_t dev_addr;
    usb_device_handle_t dev_hdl;
    uint32_t actions;
} class_driver_t;

class_driver_t driver_obj = {0};

static const char *TAG_CLASS = "USB_DRIVER";

static void midi_transfer_cb(usb_transfer_t *transfer)
{
  // ESP_LOGI("", "midi_transfer_cb context: %d", transfer->context);
  if (driver_obj.dev_hdl == transfer->device_handle)
  {
    int in_xfer = transfer->bEndpointAddress & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK;
    if ((transfer->status == 0) && in_xfer)
    {
      uint8_t *const p = transfer->data_buffer;
      for (int i = 0; i < transfer->actual_num_bytes; i += 4)
      {
        if ((p[i] + p[i + 1] + p[i + 2] + p[i + 3]) == 0)
          break;
        USB_MIDI_message.channel = p[i];
        USB_MIDI_message.type = p[i + 1];
        USB_MIDI_message.data1 = p[i + 2];
        USB_MIDI_message.data2 = p[i + 3];
        if (xQueueSendToBack(xMIDI_input_queue, (void *)&USB_MIDI_message, (TickType_t)1) != pdPASS)
        {
          ESP_LOGE(TAG_CLASS, "Error: MIDI Input Queue Full when pushing USB MIDI");
        }
        ESP_LOGI(TAG_CLASS, "usb_midi: %02x %02x %02x %02x", p[i], p[i + 1], p[i + 2], p[i + 3]);
      }
      esp_err_t err = usb_host_transfer_submit(transfer);
      if (err != ESP_OK)
      {
        ESP_LOGI(TAG_CLASS, "usb_host_transfer_submit In fail: %x", err);
      }
    }
    else
    {
      ESP_LOGI(TAG_CLASS, "transfer->status %d", transfer->status);
    }
  }
}

void check_interface_desc_MIDI(const void *p)
{
  const usb_intf_desc_t *intf = (const usb_intf_desc_t *)p;
  // USB MIDI
  if ((intf->bInterfaceClass == USB_CLASS_AUDIO) && (intf->bInterfaceSubClass == 3) && (intf->bInterfaceProtocol == 0))
  {
    isMIDI = true;
    ESP_LOGI(TAG_CLASS, "Claiming a MIDI device!");
    esp_err_t err = usb_host_interface_claim(driver_obj.client_hdl, driver_obj.dev_hdl, intf->bInterfaceNumber, intf->bAlternateSetting);
    if (err != ESP_OK)
      ESP_LOGI(TAG_CLASS, "usb_host_interface_claim failed: %x", err);
  }
}

void prepare_endpoints(const void *p)
{
  const usb_ep_desc_t *endpoint = (const usb_ep_desc_t *)p;
  esp_err_t err;

  // must be bulk for MIDI
  if ((endpoint->bmAttributes & USB_BM_ATTRIBUTES_XFERTYPE_MASK) != USB_BM_ATTRIBUTES_XFER_BULK)
  {
    ESP_LOGI(TAG_CLASS, "Not bulk endpoint: 0x%02x", endpoint->bmAttributes);
    return;
  }
  if (endpoint->bEndpointAddress & USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK)
  {
    for (int i = 0; i < MIDI_IN_BUFFERS; i++)
    {
      err = usb_host_transfer_alloc(endpoint->wMaxPacketSize, 0, &MIDIIn[i]);
      if (err != ESP_OK)
      {
        MIDIIn[i] = NULL;
        ESP_LOGI(TAG_CLASS, "usb_host_transfer_alloc In fail: %x", err);
      }
      else
      {
        MIDIIn[i]->device_handle = driver_obj.dev_hdl;
        MIDIIn[i]->bEndpointAddress = endpoint->bEndpointAddress;
        MIDIIn[i]->callback = midi_transfer_cb;
        MIDIIn[i]->context = (void *)i;
        MIDIIn[i]->num_bytes = endpoint->wMaxPacketSize;
        esp_err_t err = usb_host_transfer_submit(MIDIIn[i]);
        if (err != ESP_OK)
        {
          ESP_LOGI(TAG_CLASS, "usb_host_transfer_submit In fail: %x", err);
        }
      }
    }
  }
  else
  {
    err = usb_host_transfer_alloc(endpoint->wMaxPacketSize, 0, &MIDIOut);
    if (err != ESP_OK)
    {
      MIDIOut = NULL;
      ESP_LOGI(TAG_CLASS, "usb_host_transfer_alloc Out fail: %x", err);
      return;
    }
    ESP_LOGI(TAG_CLASS, "Out data_buffer_size: %d", MIDIOut->data_buffer_size);
    MIDIOut->device_handle = driver_obj.dev_hdl;
    MIDIOut->bEndpointAddress = endpoint->bEndpointAddress;
    MIDIOut->callback = midi_transfer_cb;
    MIDIOut->context = NULL;
    //    MIDIOut->flags |= USB_TRANSFER_FLAG_ZERO_PACK;
  }
  isMIDIReady = ((MIDIOut != NULL) && (MIDIIn[0] != NULL));
}

void show_config_desc_full(const usb_config_desc_t *config_desc)
{
  // Full decode of config desc.
  const uint8_t *p = &config_desc->val[0];
  uint8_t bLength;
  for (int i = 0; i < config_desc->wTotalLength; i += bLength, p += bLength)
  {
    bLength = *p;
    if ((i + bLength) <= config_desc->wTotalLength)
    {
      const uint8_t bDescriptorType = *(p + 1);
      switch (bDescriptorType)
      {
      case USB_B_DESCRIPTOR_TYPE_DEVICE:
        ESP_LOGI(TAG_CLASS, "USB Device Descriptor should not appear in config");
        break;
      case USB_B_DESCRIPTOR_TYPE_CONFIGURATION:
        show_config_desc(p);
        break;
      case USB_B_DESCRIPTOR_TYPE_STRING:
        ESP_LOGI(TAG_CLASS, "USB string desc TBD");
        break;
      case USB_B_DESCRIPTOR_TYPE_INTERFACE:
        show_interface_desc(p);
        if (!isMIDI)
          check_interface_desc_MIDI(p);
        break;
      case USB_B_DESCRIPTOR_TYPE_ENDPOINT:
        if (isMIDI && !isMIDIReady)
        {
          show_endpoint_desc(p);
          prepare_endpoints(p);
        }
        break;
      case USB_B_DESCRIPTOR_TYPE_DEVICE_QUALIFIER:
        // Should not be in config?
        ESP_LOGI(TAG_CLASS, "USB device qual desc TBD");
        break;
      case USB_B_DESCRIPTOR_TYPE_OTHER_SPEED_CONFIGURATION:
        // Should not be in config?
        ESP_LOGI(TAG_CLASS, "USB Other Speed TBD");
        break;
      case USB_B_DESCRIPTOR_TYPE_INTERFACE_POWER:
        // Should not be in config?
        ESP_LOGI(TAG_CLASS, "USB Interface Power TBD");
        break;
      default:
        ESP_LOGI(TAG_CLASS, "Unknown USB Descriptor Type: 0x%x", *p);
        break;
      }
    }
    else
    {
      ESP_LOGI(TAG_CLASS, "USB Descriptor invalid");
      return;
    }
  }
}

static void client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg)
{
    class_driver_t *driver_obj = (class_driver_t *)arg;
    switch (event_msg->event) {
        case USB_HOST_CLIENT_EVENT_NEW_DEV:
            if (driver_obj->dev_addr == 0) {
                driver_obj->dev_addr = event_msg->new_dev.address;
                //Open the device next
                driver_obj->actions |= ACTION_OPEN_DEV;
            }
            break;
        case USB_HOST_CLIENT_EVENT_DEV_GONE:
            if (driver_obj->dev_hdl != NULL) {
                //Cancel any other actions and close the device next
                driver_obj->actions = ACTION_CLOSE_DEV;
            }
            break;
        default:
            //Should never occur
            abort();
    }
}

static void action_open_dev(class_driver_t *driver_obj)
{
    assert(driver_obj->dev_addr != 0);
    ESP_LOGI(TAG_CLASS, "Opening device at address %d", driver_obj->dev_addr);
    ESP_ERROR_CHECK(usb_host_device_open(driver_obj->client_hdl, driver_obj->dev_addr, &driver_obj->dev_hdl));
    //Get the device's information next
    driver_obj->actions &= ~ACTION_OPEN_DEV;
    driver_obj->actions |= ACTION_GET_DEV_INFO;
}

static void action_get_info(class_driver_t *driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(TAG_CLASS, "Getting device information");
    usb_device_info_t dev_info;
    ESP_ERROR_CHECK(usb_host_device_info(driver_obj->dev_hdl, &dev_info));
    ESP_LOGI(TAG_CLASS, "\t%s speed", (dev_info.speed == USB_SPEED_LOW) ? "Low" : "Full");
    ESP_LOGI(TAG_CLASS, "\tbConfigurationValue %d", dev_info.bConfigurationValue);
    //Todo: Print string descriptors

    //Get the device descriptor next
    driver_obj->actions &= ~ACTION_GET_DEV_INFO;
    driver_obj->actions |= ACTION_GET_DEV_DESC;
}

static void action_get_dev_desc(class_driver_t *driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(TAG_CLASS, "Getting device descriptor");
    const usb_device_desc_t *dev_desc;
    ESP_ERROR_CHECK(usb_host_get_device_descriptor(driver_obj->dev_hdl, &dev_desc));
    //usb_print_device_descriptor(dev_desc);
    //Get the device's config descriptor next
    driver_obj->actions &= ~ACTION_GET_DEV_DESC;
    driver_obj->actions |= ACTION_GET_CONFIG_DESC;
}

static void action_get_config_desc(class_driver_t *driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    ESP_LOGI(TAG_CLASS, "Getting config descriptor");
    const usb_config_desc_t *config_desc;
    ESP_ERROR_CHECK(usb_host_get_active_config_descriptor(driver_obj->dev_hdl, &config_desc));
    //usb_print_config_descriptor(config_desc, NULL);
    show_config_desc_full(config_desc);
    //Get the device's string descriptors next
    driver_obj->actions &= ~ACTION_GET_CONFIG_DESC;
    driver_obj->actions |= ACTION_GET_STR_DESC;
}

static void action_get_str_desc(class_driver_t *driver_obj)
{
    assert(driver_obj->dev_hdl != NULL);
    usb_device_info_t dev_info;
    ESP_ERROR_CHECK(usb_host_device_info(driver_obj->dev_hdl, &dev_info));
    // if (dev_info.str_desc_manufacturer) {
    //     ESP_LOGI(TAG_CLASS, "Getting Manufacturer string descriptor");
    //     //usb_print_string_descriptor(dev_info.str_desc_manufacturer);
    // }
    // if (dev_info.str_desc_product) {
    //     ESP_LOGI(TAG_CLASS, "Getting Product string descriptor");
    //     //usb_print_string_descriptor(dev_info.str_desc_product);
    // }
    // if (dev_info.str_desc_serial_num) {
    //     ESP_LOGI(TAG_CLASS, "Getting Serial Number string descriptor");
    //     //usb_print_string_descriptor(dev_info.str_desc_serial_num);
    // }
    //Nothing to do until the device disconnects
    driver_obj->actions &= ~ACTION_GET_STR_DESC;
}

static void aciton_close_dev(class_driver_t *driver_obj)
{
    ESP_ERROR_CHECK(usb_host_device_close(driver_obj->client_hdl, driver_obj->dev_hdl));
    driver_obj->dev_hdl = NULL;
    driver_obj->dev_addr = 0;
    //We need to exit the event handler loop
    driver_obj->actions &= ~ACTION_CLOSE_DEV;
    driver_obj->actions |= ACTION_EXIT;
}

void class_driver_task(void *params)
{
    MIDI_USB_Task_Param * task_params = (MIDI_USB_Task_Param *) params;
    SemaphoreHandle_t signaling_sem = (SemaphoreHandle_t)task_params->signaling_sem;
    xMIDI_input_queue = (QueueHandle_t)task_params->queue;

    //Wait until daemon task has installed USB Host Library
    xSemaphoreTake(signaling_sem, portMAX_DELAY);
    
    ESP_LOGI(TAG_CLASS, "Registering Client");
    usb_host_client_config_t client_config = {
        .is_synchronous = false,    //Synchronous clients currently not supported. Set this to false
        .max_num_event_msg = CLIENT_NUM_EVENT_MSG,
        .async = {
            .client_event_callback = client_event_cb,
            .callback_arg = (void *)&driver_obj,
        },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&client_config, &driver_obj.client_hdl));

    while (1) {
        if (driver_obj.actions == 0) {
            usb_host_client_handle_events(driver_obj.client_hdl, portMAX_DELAY);
        } else {
            if (driver_obj.actions & ACTION_OPEN_DEV) {
                action_open_dev(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_DEV_INFO) {
                action_get_info(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_DEV_DESC) {
                action_get_dev_desc(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_CONFIG_DESC) {
                action_get_config_desc(&driver_obj);
            }
            if (driver_obj.actions & ACTION_GET_STR_DESC) {
                action_get_str_desc(&driver_obj);
            }
            if (driver_obj.actions & ACTION_CLOSE_DEV) {
                aciton_close_dev(&driver_obj);
            }
            if (driver_obj.actions & ACTION_EXIT) {
                break;
            }
        }
    }

    ESP_LOGI(TAG_CLASS, "Deregistering Client");
    ESP_ERROR_CHECK(usb_host_client_deregister(driver_obj.client_hdl));

    //Wait to be deleted
    xSemaphoreGive(signaling_sem);
    vTaskSuspend(NULL);
}
