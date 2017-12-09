//Copyright 2007-2013
//Allwinner Technology Co., Ltd. <www.allwinnertech.com>
//Jerry Wang <wangflord@allwinnertech.com>


#include "usb_base.h"
#include "usb_module.h"
extern sunxi_usb_setup_req_t *sunxi_udev_active;









int sunxi_usb_dev_register(uint dev_name)
{
    int ret = 0;
    sunxi_usb_dbg("sunxi_usb_dev_register\n");
    switch( dev_name ){  
      #ifdef SUNXI_USB_DEVICE_MASS
      case SUNXI_USB_DEVICE_MASS:
        sunxi_usb_dbg("register SUNXI_USB_DEVICE_MASS begin\n");
        sunxi_usb_module_reg(SUNXI_USB_DEVICE_MASS);
        sunxi_usb_dbg("register SUNXI_USB_DEVICE_MASS ok\n");
      break;
      #endif
                          
      //setup_req_1
      #ifdef SUNXI_USB_DEVICE_EFEX
      case SUNXI_USB_DEVICE_EFEX:
        sunxi_usb_module_reg(SUNXI_USB_DEVICE_EFEX);
        //setup_req_2
      break;
      #endif
                                          
      #ifdef SUNXI_USB_DEVICE_FASTBOOT
      case SUNXI_USB_DEVICE_FASTBOOT:
        sunxi_usb_module_reg(SUNXI_USB_DEVICE_FASTBOOT);
      break;
      #endif
                       
      #ifdef SUNXI_USB_DEVICE_BURN
      case SUNXI_USB_DEVICE_BURN:
        sunxi_usb_module_reg(SUNXI_USB_DEVICE_BURN);
      break;
      #endif
                  
      default:
        ret = -1;
      break;
    }
    return ret;
}













