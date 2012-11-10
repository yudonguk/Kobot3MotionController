/**
  ******************************************************************************
  * @file    usb_desc.c
  * @author  MCD Application Team
  * @version V3.4.0
  * @date    29-June-2012
  * @brief   Descriptors for Custom HID Demo
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_desc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USB Standard Device Descriptor */
const uint8_t CustomHID_DeviceDescriptor[CUSTOMHID_SIZ_DEVICE_DESC] =
  {
    0x12,                       /*bLength */
    USB_DEVICE_DESCRIPTOR_TYPE, /*bDescriptorType*/
    0x00,                       /*bcdUSB */
    0x02,
    0x00,                       /*bDeviceClass*/
    0x00,                       /*bDeviceSubClass*/
    0x00,                       /*bDeviceProtocol*/
    0x40,                       /*bMaxPacketSize40*/
    0x75,                       /*idVendor (0x5975)*/
    0x59,
    0x4D,                       /*idProduct = 0x4B4D*/
    0x4B,
    0x00,                       /*bcdDevice rel. 1.00*/
    0x01,
    1,                          /*Index of string descriptor describing
                                              manufacturer */
    2,                          /*Index of string descriptor describing
                                             product*/
    3,                          /*Index of string descriptor describing the
                                             device serial number */
    0x01                        /*bNumConfigurations*/
  }
  ; /* CustomHID_DeviceDescriptor */


/* USB Configuration Descriptor */
/*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor */
const uint8_t CustomHID_ConfigDescriptor[CUSTOMHID_SIZ_CONFIG_DESC] =
  {
    0x09, /* bLength: Configuration Descriptor size */
    USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: Configuration */
    CUSTOMHID_SIZ_CONFIG_DESC,
    /* wTotalLength: Bytes returned */
    0x00,
    0x01,         /* bNumInterfaces: 1 interface */
    0x01,         /* bConfigurationValue: Configuration value */
    0x00,         /* iConfiguration: Index of string descriptor describing
                                 the configuration*/
    0xC0,         /* bmAttributes: Bus powered */
    0x32,         /* MaxPower 100 mA: this current is used for detecting Vbus */

    /************** Descriptor of Custom HID interface ****************/
    /* 09 */
    0x09,         /* bLength: Interface Descriptor size */
    USB_INTERFACE_DESCRIPTOR_TYPE,/* bDescriptorType: Interface descriptor type */
    0x00,         /* bInterfaceNumber: Number of Interface */
    0x00,         /* bAlternateSetting: Alternate setting */
    0x02,         /* bNumEndpoints */
    0x03,         /* bInterfaceClass: HID */
    0x00,         /* bInterfaceSubClass : 1=BOOT, 0=no boot */
    0x00,         /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
    0,            /* iInterface: Index of string descriptor */
    /******************** Descriptor of Custom HID HID ********************/
    /* 18 */
    0x09,         /* bLength: HID Descriptor size */
    HID_DESCRIPTOR_TYPE, /* bDescriptorType: HID */
    0x10,         /* bcdHID: HID Class Spec release number */
    0x01,
    0x00,         /* bCountryCode: Hardware target country */
    0x01,         /* bNumDescriptors: Number of HID class descriptors to follow */
    0x22,         /* bDescriptorType */
    CUSTOMHID_SIZ_REPORT_DESC,/* wItemLength: Total length of Report descriptor */
    0x00,
    /******************** Descriptor of Custom HID endpoints ******************/
    /* 27 */
    0x07,          /* bLength: Endpoint Descriptor size */
    USB_ENDPOINT_DESCRIPTOR_TYPE, /* bDescriptorType: */

    0x81,          /* bEndpointAddress: Endpoint Address (IN) */
    0x03,          /* bmAttributes: Interrupt endpoint */
    CUSTOMHID_SIZ_ENDPOINT1_IN_MAX_PACKET,          /* wMaxPacketSize: 64 Bytes max */
    0x00,
    0x01,          /* bInterval: Polling Interval (1 ms) */
    /* 34 */
    	
    0x07,	/* bLength: Endpoint Descriptor size */
    USB_ENDPOINT_DESCRIPTOR_TYPE,	/* bDescriptorType: */
			/*	Endpoint descriptor type */
    0x01,	/* bEndpointAddress: */
			/*	Endpoint Address (OUT) */
    0x03,	/* bmAttributes: Interrupt endpoint */
    CUSTOMHID_SIZ_ENDPOINT1_OUT_MAX_PACKET,	/* wMaxPacketSize: 64 Bytes max  */
    0x00,
    0x01,	/* bInterval: Polling Interval (1 ms) */
    /* 41 */
  }; /* CustomHID_ConfigDescriptor */

const uint8_t CustomHID_ReportDescriptor[CUSTOMHID_SIZ_REPORT_DESC] =
  {
	// (7 Byte)
    0x06, 0xFF, 0x00,      /* USAGE_PAGE (Vendor Page: 0x00FF) */
    0x09, 0x06,            /* USAGE Generic Device Controls  */
    0xa1, 0x01,            /* COLLECTION (Application)       */
    /* 6 */

    //Report 1 (15 Byte)
    0x85, 0x01,			//Report ID 1
    0x09, 0x06,			//Usage Generic Device Controls
    0x15, 0x00,			//Logical minimum 0
    0x26, 0xFF, 0x00, 	//Logical maximum 255
    0x75, 0x08,			//Report size 8bits
    0x95, 6,			//Report count
    0x81, 0x02,			//Input (Data, Var, Abs)

    //Report 1 (15 Byte)
    0x85, 0x01,			//Report ID 1
    0x09, 0x06,			//Usage Generic Device Controls
    0x15, 0x00,			//Logical minimum 0
    0x26, 0xFF, 0x00, 	//Logical maximum 255
    0x75, 0x08,			//Report size 8bits
    0x95, 6,			//Report count
    0x91, 0x02,			//Output (Data, Var, Abs)

    //Report 2 (15 Byte)
    0x85, 0x02,			//Report ID 2
    0x09, 0x06,			//Usage Generic Device Controls
    0x15, 0x00,			//Logical minimum 0
    0x26, 0xFF, 0x00, 	//Logical maximum 255
    0x75, 0x08,			//Report size 8bits
    0x95, 12,			//Report count
    0x81, 0x02,			//Input (Data, Var, Abs)

    //Report 2 (15 Byte)
    0x85, 0x02,			//Report ID 2
    0x09, 0x06,			//Usage Generic Device Controls
    0x15, 0x00,			//Logical minimum 0
    0x26, 0xFF, 0x00, 	//Logical maximum 255
    0x75, 0x08,			//Report size 8bits
    0x95, 12,			//Report count
    0x91, 0x02,			//Output (Data, Var, Abs)

    //Report 3 (15 Byte)
    0x85, 0x03,			//Report ID 3
    0x09, 0x06,			//Usage Generic Device Controls
    0x15, 0x00,			//Logical minimum 0
    0x26, 0xFF, 0x00, 	//Logical maximum 255
    0x75, 0x08,			//Report size 8bits
    0x95, 18,			//Report count
    0x81, 0x02,			//Input (Data, Var, Abs)

    //Report 3 (15 Byte)
    0x85, 0x03,			//Report ID 3
    0x09, 0x06,			//Usage Generic Device Controls
    0x15, 0x00,			//Logical minimum 0
    0x26, 0xFF, 0x00, 	//Logical maximum 255
    0x75, 0x08,			//Report size 8bits
    0x95, 18,			//Report count
    0x91, 0x02,			//Output (Data, Var, Abs)

    //Report 4 (15 Byte)
    0x85, 0x03,			//Report ID 3
    0x09, 0x06,			//Usage Generic Device Controls
    0x15, 0x00,			//Logical minimum 0
    0x26, 0xFF, 0x00, 	//Logical maximum 255
    0x75, 0x08,			//Report size 8bits
    0x95, 24,			//Report count
    0x81, 0x02,			//Input (Data, Var, Abs)

    //Report 4 (15 Byte)
    0x85, 0x03,			//Report ID 3
    0x09, 0x06,			//Usage Generic Device Controls
    0x15, 0x00,			//Logical minimum 0
    0x26, 0xFF, 0x00, 	//Logical maximum 255
    0x75, 0x08,			//Report size 8bits
    0x95, 24,			//Report count
    0x91, 0x02,			//Output (Data, Var, Abs)

    //(1 Byte)
    0xc0 	          /*     END_COLLECTION	             */
  }; /* CustomHID_ReportDescriptor */

/* USB String Descriptors (optional) */
const uint8_t CustomHID_StringLangID[CUSTOMHID_SIZ_STRING_LANGID] =
  {
    CUSTOMHID_SIZ_STRING_LANGID,
    USB_STRING_DESCRIPTOR_TYPE,
    0x09,
    0x04
  }
  ; /* LangID = 0x0409: U.S. English */

const uint8_t CustomHID_StringVendor[CUSTOMHID_SIZ_STRING_VENDOR] =
  {
    CUSTOMHID_SIZ_STRING_VENDOR, /* Size of Vendor string */
    USB_STRING_DESCRIPTOR_TYPE,  /* bDescriptorType*/
    /* Manufacturer: "STMicroelectronics" */
    'I', 0, 'I', 0, 'C', 0, ' ', 0, 'L', 0, 'a', 0, 'b', 0, ',', 0,
    ' ', 0, 'U', 0, 'n', 0, 'i', 0, 'v', 0, '.', 0, 'K', 0, 'a', 0,
    'n', 0, 'g', 0, 'w', 0, 'o', 0,'n', 0
  };

const uint8_t CustomHID_StringProduct[CUSTOMHID_SIZ_STRING_PRODUCT] =
  {
    CUSTOMHID_SIZ_STRING_PRODUCT,          /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
    'K', 0, 'o', 0, 'b', 0, 'o', 0, 't', 0, '3', 0, ' ', 0,
    'M', 0, 'o', 0, 't', 0, 'i', 0, 'o', 0, 'n', 0, ' ', 0,
    'B', 0, 'o', 0, 'a', 0, 'r', 0, 'd', 0
  };
uint8_t CustomHID_StringSerial[CUSTOMHID_SIZ_STRING_SERIAL] =
  {
    CUSTOMHID_SIZ_STRING_SERIAL,           /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
    'S', 0, 'T', 0, 'M', 0,'3', 0,'2', 0, '1', 0, '0', 0
  };

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

