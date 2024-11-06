

#include "usbd_code.h"
#include "usb_port.h"

#include <rtthread.h>
#include <rtdevice.h>
#include "drv_common.h"

#if 1


#define USB_NO_DATA                   (-1)            /*!< For Device.Transfer.sByteLength                  */
/* Private function prototypes -----------------------------------------------------------------------------*/
static void _USBDCore_Suspend(USBDCore_TypeDef *pCore);
static void _USBDCore_Reset(USBDCore_TypeDef *pCore);
static void _USBDCore_Resume(USBDCore_TypeDef *pCore);

static void usbd_ept_init(USBDCore_TypeDef *udev);
static void usbd_eptn_out_handler(USBDCore_TypeDef *udev, USBD_EPTn_Enum EPTn);
/*********************************************************************************************************//**
  * @brief  USB Interrupt Service Routine.
  * @param  pCore: pointer of USB Device
  * @retval None
  ***********************************************************************************************************/
/* USB的中断回调函数 */
void usbd_irq_handler(USBDCore_TypeDef *pCore)
{
    u32 USBISRFlag = API_USB_GET_INT();
    u32 USBEPTISRFlag;
    USBD_EPTn_Enum EPTn;

#if (USBDCORE_DEBUG == 1)
    u32 USBAddr = HT_USB->DEVAR;
#endif

    /*--------------------------------------------------------------------------------------------------------*/
    /* USB SOF Interrupt                                                                                      */
    /*--------------------------------------------------------------------------------------------------------*/
    if (API_USB_IS_SOF_INT(USBISRFlag))
    {
        usbd_sof_callback(pCore);
        API_USB_CLR_SOF_INT();
    }

    /*--------------------------------------------------------------------------------------------------------*/
    /* USB SUSPEND Interrupt                                                                                  */
    /*--------------------------------------------------------------------------------------------------------*/
    if (API_USB_IS_SUSPEND_INT(USBISRFlag))
    {
        API_USB_CLR_SUSPEND_INT();
        usbd_suspend_callback(pCore);
        _USBDCore_Suspend(pCore);
    }

    /*--------------------------------------------------------------------------------------------------------*/
    /* USB RESET Interrupt（复位）                                                                             */
    /*--------------------------------------------------------------------------------------------------------*/
    if (API_USB_IS_RESET_INT(USBISRFlag))
    {
        if (API_USB_IS_FRES_INT(USBISRFlag))    //强制复位
        {
            API_USB_CLR_FRES_INT();
        }
        else    //正常复位
        {
            usbd_reset_callback(pCore);
            _USBDCore_Reset(pCore);
        }
        API_USB_CLR_RESET_INT();
    }

    /*--------------------------------------------------------------------------------------------------------*/
    /* USB RESUME Interrupt（恢复中断）                                                                        */
    /*--------------------------------------------------------------------------------------------------------*/
    if (API_USB_IS_RESUME_INT(USBISRFlag))
    {
        usbd_resume_callback(pCore);
        _USBDCore_Resume(pCore);
        API_USB_CLR_RESUME_INT();
    }

    /*--------------------------------------------------------------------------------------------------------*/
    /* USB Endpoint 0 interrupt                                                                               */
    /*--------------------------------------------------------------------------------------------------------*/
    if (API_USB_IS_EPTn_INT(USBISRFlag, USBD_EPT0))
    {
        USBEPTISRFlag = API_USB_EPTn_GET_INT(USBD_EPT0);

        /*------------------------------------------------------------------------------------------------------*/
        /* Control SETUP Stage（请求控制）                                                                       */
        /*------------------------------------------------------------------------------------------------------*/
        if (API_USB_IS_SETUP_INT(USBEPTISRFlag))
        {
            API_USB_READ_SETUP(&(pCore->Device.Request));               /* Read SETUP Command data from USB Buffer*/
            usbd_setup_callback(pCore);
            API_USB_CLR_SETUP_INT();                                    /* Clear SETUP Interrupt                  */
        }

        /*------------------------------------------------------------------------------------------------------*/
        /* Control Endpoint 0 IN（端点0 IN）                                                                     */
        /*------------------------------------------------------------------------------------------------------*/
        if (API_USB_EPTn_IS_IN_INT(USBEPTISRFlag))
        {
            usbd_ep0_in_callback(pCore);
            API_USB_EPTn_CLR_IN_INT(USBD_EPT0);
        }

        /*------------------------------------------------------------------------------------------------------*/
        /* Control Endpoint 0 OUT（端点0 OUT）                                                                   */
        /*------------------------------------------------------------------------------------------------------*/
        if (API_USB_EPTn_IS_OUT_INT(USBEPTISRFlag))
        {
            /*----------------------------------------------------------------------------------------------------*/
            /* Clear interrupt flag before USBDCore_ControlOUT is meaning since USBDCore_ControlOUT clear NAKRX   */
            /* bit which will cause another interrupt occur.                                                      */
            /*----------------------------------------------------------------------------------------------------*/
            API_USB_EPTn_CLR_OUT_INT(USBD_EPT0);
//            usbd_eptn_out_handler(pCore,USBD_EPT0);
            usbd_ep0_out_callback(pCore);
        }

        /*------------------------------------------------------------------------------------------------------*/
        /* Clear Control Endpoint 0 global interrupt                                                            */
        /*------------------------------------------------------------------------------------------------------*/
        API_USB_CLR_EPTn_INT(USBD_EPT0);

    } /* if (API_USB_IS_EP_INT(USBISRFlag, USBD_EPT0))                                                        */


    /*--------------------------------------------------------------------------------------------------------*/
    /* USB Endpoint n call back function                                                                      */
    /*--------------------------------------------------------------------------------------------------------*/
    while ((EPTn = API_USB_GET_EPT_NUM(API_USB_GET_INT())) != USBD_NOEPT)
    {
        USBEPTISRFlag = API_USB_EPTn_GET_INT((USBD_EPTn_Enum)EPTn);

        if (API_USB_EPTn_IS_INT(USBEPTISRFlag))
        {
            API_USB_EPTn_CLR_INT(EPTn);
            API_USB_CLR_EPTn_INT(EPTn);

            /* 在此处调用端点输入输出回调函数*/
            if (USBEPTISRFlag & IDTXIF)
            {
                /* 在此处调用端点输入函数 */
                usbd_ep_in_callback(pCore, (USBD_EPTn_Enum)EPTn);
            }
            else
            {
                /* 在此处调用端点输出函数 */
//                usbd_eptn_out_handler(pCore,EPTn);
                usbd_ep_out_callback(pCore, (USBD_EPTn_Enum)EPTn);
            }

        }
    } /* while ((EPTn = API_USB_GET_EPTn_NUM(API_USB_GET_INT())) != USBD_NOEPT)                               */

    return;
}
/*********************************************************************************************************//**
  * @brief  USB Core initialization.
  * @param  pCore: pointer of USB Device
  * @retval None
  ***********************************************************************************************************/
void USBDCore_Init(USBDCore_TypeDef *pCore)
{
    pCore->Info.CurrentStatus = USER_USB_STATE_POWERED;
    API_USB_INIT(pCore->pDriver);
    
    /* 端点信息初始化 */
    usbd_ept_init(pCore);
    
    return;
}
/*********************************************************************************************************//**
  * @brief  USB Core Main Routine for application.
  * @param  pCore: pointer of USB Device
  * @retval None
  ***********************************************************************************************************/
void USBDCore_MainRoutine(USBDCore_TypeDef *pCore)
{
    API_USB_POWER_UP(pCore->pDriver, pCore->Info.CurrentFeature.Bits.bSelfPowered);

    if (pCore->Info.CurrentStatus == USER_USB_STATE_SUSPENDED)
    {
        /*------------------------------------------------------------------------------------------------------*/
        /* System Low Power call back function                                                                  */
        /*------------------------------------------------------------------------------------------------------*/
        if (pCore->Power.CallBack_Suspend.func != NULL)
        {
            __DBG_USBPrintf("%06ld >LOWPOWER\r\n", ++__DBG_USBCount);

            pCore->Power.CallBack_Suspend.func(pCore->Power.CallBack_Suspend.uPara);

            __DBG_USBPrintf("%06ld <LOWPOWER\r\n", ++__DBG_USBCount);
        }
    }

    return;
}
/*********************************************************************************************************//**
  * @brief  USB Suspend
  * @param  pCore: pointer of USB Device
  * @retval None
  ***********************************************************************************************************/
static void _USBDCore_Suspend(USBDCore_TypeDef *pCore)
{
    /*--------------------------------------------------------------------------------------------------------*/
    /* When Device has been suspended, Change CurrentStatus as SUSPEND and then USBDCore_PowerHandler will    */
    /* turn off chip power.                                                                                   */
    /*--------------------------------------------------------------------------------------------------------*/
    if (pCore->Info.CurrentStatus >= USER_USB_STATE_POWERED)
    {
        API_USB_POWER_OFF();
        pCore->Info.LastStatus = pCore->Info.CurrentStatus;
        pCore->Info.CurrentStatus = USER_USB_STATE_SUSPENDED;
    }

    return;
}
/*********************************************************************************************************//**
  * @brief  USB Reset
  * @param  pCore: pointer of USB Device
  * @retval None
  ***********************************************************************************************************/
static void _USBDCore_Reset(USBDCore_TypeDef *pCore)
{
    USBD_Driver_TypeDef *pDrv = (USBD_Driver_TypeDef *)pCore->pDriver;

    pCore->Device.Transfer.sByteLength = USB_NO_DATA;
    pCore->Info.uCurrentConfiguration = 0;
    pCore->Info.uCurrentInterface = 0;
    pCore->Info.CurrentFeature.Bits.bRemoteWakeup = 0;
    pCore->Info.CurrentStatus = USER_USB_STATE_DEFAULT;
    pCore->Info.uIsDiscardClearFeature = FALSE;

    API_USB_DEINIT();

    API_USB_POWER_ON();

    /* Endpoint 0 initialization                                                                              */
    API_USB_EPTn_INIT(USBD_EPT0, pCore->pDriver); // To be modify, init from desc

    /* Enable USB interrupt                                                                                   */
    API_USB_ENABLE_INT(pDrv->uInterruptMask);

    return;
}
/*********************************************************************************************************//**
  * @brief  USB Resume
  * @param  pCore: pointer of USB Device
  * @retval None
  ***********************************************************************************************************/
static void _USBDCore_Resume(USBDCore_TypeDef *pCore)
{
    API_USB_POWER_ON();
    pCore->Info.CurrentStatus = pCore->Info.LastStatus;
    return;
}
/****************************************************************************************************************************/
void usbd_ep_enable(USBDCore_TypeDef *pCore, uint8_t ept_addr)
{
    USBD_Driver_TypeDef *pDrv = (USBD_Driver_TypeDef *)pCore->pDriver;
    /* 端点使能 */
    pDrv->ept[ept_addr&0x7f].CFGR.bits.EPEN = 1;
    /* 端点初始化和失能 */
    API_USB_EPTn_INIT((USBD_EPTn_Enum)(ept_addr&0x7f), pCore->pDriver); // To be modify, init from desc
}
void usbd_ep_disable(USBDCore_TypeDef *pCore, uint8_t ept_addr)
{
    USBD_Driver_TypeDef *pDrv = (USBD_Driver_TypeDef *)pCore->pDriver;
    pDrv->ept[ept_addr&0x7f].CFGR.bits.EPEN = 0;
    /* 端点失能 */
    API_USB_EPTn_INIT((USBD_EPTn_Enum)(ept_addr&0x7f), pCore->pDriver); // To be modify, init from desc
}

static void usbd_ept_init(USBDCore_TypeDef *udev)
{
    uint8_t ept_num = 0;
    usb_ept_info *ept_info;
    for(ept_num = 0;ept_num < 8;ept_num++)
    {
        ept_info = &udev->ept_io[ept_num];
        
        ept_info->maxpacket = 64;
        
        ept_info->status = 1;
        
        ept_info->total_len = 0;
        ept_info->trans_len = 0;
        ept_info->trans_buf = NULL;
    }
}
/**
  * @brief  usb endpoint receive data
  * @param  udev: to the structure of usbd_core_type
  * @param  ept_addr: endpoint number
  * @param  buffer: receive data buffer
  * @param  len: receive data length
  * @retval none
  */
void usbd_ept_recv(USBDCore_TypeDef *udev, uint8_t ept_addr, uint8_t *buffer, uint16_t len)
{
  /* get endpoint info struct and register */
  usb_ept_info *ept_info = &udev->ept_io[ept_addr & 0x7F];
  uint32_t trs_len = 0;

   /* set receive data buffer and length */
  ept_info->trans_buf = buffer;
  ept_info->total_len = len;
  ept_info->trans_len = 0;

  if(ept_info->total_len > ept_info->maxpacket)
  {
    trs_len = ept_info->maxpacket;
    ept_info->total_len -= trs_len;
  }
  else
  {
    trs_len = len;
    ept_info->total_len = 0;
  }

  ept_info->trans_len = trs_len;
  /* set rx status valid */
  ept_info->status = TRUE;
}

/* 主机接收数据 */
void usbd_eptn_in_handler(USBDCore_TypeDef *udev, USBD_EPTn_Enum EPTn)
{

    
    
    
}
/* 主机发送数据 */
static void usbd_eptn_out_handler(USBDCore_TypeDef *udev, USBD_EPTn_Enum EPTn)
{
    uint16_t length = 0;
    /* 获取指定端点的信息 */
    usb_ept_info *ept_info = &udev->ept_io[EPTn];
    
    length = (EPTn == USBD_EPT0) ? USBD_EPTGetTransferCount(EPTn, USBD_CNTOUT):USBD_EPTGetTransferCount(EPTn, USBD_CNTB0);
    if((ept_info->trans_buf == NULL)&&(length > 0))
    {
        rt_kprintf("usbd_eptn_out_handler error!\n");
        rt_kprintf("ep num = %d!\n",EPTn);
        rt_kprintf("data length = %d!\n",length);
        return;
    }
    else
    {
        length = USBDCore_EPTReadOUTData(EPTn, (uint32_t *)ept_info->trans_buf, length);
        /* set received data length */
        ept_info->trans_len += length;    //读取到的数据长度增加
        ept_info->trans_buf += length;    //偏移缓存的地址
        /* 这里是读取完数据后，让USB内核准备读取下一次数据 */
        /* 这里调用的条件分别为，需要读取的数据长度为0或者读取到的数据小于最大值或者读取的是端点0的数据 */
        if(ept_info->total_len == 0 || length < ept_info->maxpacket || EPTn == USBD_EPT0)
        {
            /* out transfer complete */
            if(EPTn == USBD_EPT0)
                usbd_ep0_out_callback(udev);
            else
                usbd_ep_out_callback(udev, (USBD_EPTn_Enum)EPTn);
        }
        else  //如果以上条件都不满足，则根据新的地址信息和长度
        {
            /* endpoint continue receive data  */
            usbd_ept_recv(udev, EPTn, ept_info->trans_buf, ept_info->total_len);
        }
    }
}



#endif /* BSP_USING_USBD */
