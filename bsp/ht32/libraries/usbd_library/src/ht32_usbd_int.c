/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-07-11     QT-one       first version
 */

#include "ht32_usbd_int.h"

#ifdef RT_USING_USB_DEVICE
//#include "ht32_usbd_core.h"



static void USBPLL_Configuration(void);
static void USBVRG_Configuration(void);
static void Suspend(u32 uPara);


static USBDCore_TypeDef *int_p_usbd_code;
static USBD_Driver_TypeDef gUSBDriver;
static u32 gIsLowPowerAllowed = TRUE;

/*
    Ϊ�˲��ƻ�ԭ����usb_code�ļ��е�USB�жϻص�����
    �˴��ع�USB�жϻص�����
    �����ع�USB���жϻص��������⣬����Ҫ��дһЩ�жϻص����ܺ���
*/
/*
    RTT��USB�жϻص���������
    USB�жϺ��� -> USB�жϻص����� -> USB��ع��ܻص�����
    -> RTT��USB��ع��ܻص����� -> ����USB�ж�

    RTT��USB�����������̣�RTT��USB�߳��������̣�
    USB�߳������ȴ�USB��ȡ����Ϣ�����е���Ϣ
    -> USB�ж�ͨ���ص����������յ�����Ϣ����USB����Ϣ����
    -> RTT��USB��ع��ܻص�������ȡ��USB�жϵ���Ϣ��������USB��Ϣ���е�״̬
    -> USB�˳��жϣ�USB��ȡ����Ϣ���߳����������
    -> USB�̸߳��ݻ�ȡ��״ִ̬�ж�Ӧ�Ĺ���
    -> USB�߳�ͨ��USB�����ӿ�ʵ�ֶ�Ӧ�Ĺ���

*/
/*
    ����RTT��USB�жϻص����̺�RTT��USB�߳�ִ�й���
    �ó����USB������Ҫʵ�ֵ�������Ҫ����
    1��USB�߳�ʵ�ֹ�������Ҫ�ĵ��õ���USB�����ӿں���
    2��USB�жϻص�����ʹ�õ���RTT��USB�����ĺ���������νӲ���

    �������������Ƚ���Ҫ�Ĺ����⣬����Ҫ������µ�һЩ��Ҫ����
    1��USB��ʼ������
    2��USB�豸ע�ắ��

    ������Ϲ�����Ҫ�漰�������ļ�Ϊ��
    1��drv_usbd.c
    2��ht32_usbd_int.c

    �������ļ������ݷ����Լ��ļ��������£�
    drv_usbd.c
        ��Ҫ����ʵ��USB�Ĳ����ӿڵ�ʵ��
        �Լ���ʼ��������USB�豸ע�ắ��
        �����Զ����USB�ں˹��ص��ڸ��ļ���
        ����USB���жϺ���Ҳ��д�ڸ��ļ���
        ������
            ht32_usbd_core.c
            ht32_usbd_int.c
            �Լ�RTT������ļ�

    ht32_usbd_int.c
        ��Ҫ����ʵ��USB�жϻص��Լ��ص�����
        �к�RTT��USB������������νӲ���
        ���ļ��������USB�ĳ�ʼ���ú����Լ�
        USB�������뻽�ѵ���غ���
        ������
            ht32_usbd_core.c
            �Լ�RTT������ļ�
*/

/* ֡��ʼ��SOF���жϻص� */
void usbd_sof_callback(USBDCore_TypeDef *pCore)
{
    udcd_t udcd = (udcd_t)pCore->pdata;
    rt_usbd_sof_handler(udcd);
}
/* USB��λ�ж� */
void usbd_reset_callback(USBDCore_TypeDef *pCore)
{
    udcd_t udcd = (udcd_t)pCore->pdata;
    rt_usbd_reset_handler(udcd);
}

/* USB��ͣ(�Ͽ�����)�ж� */
void usbd_suspend_callback(USBDCore_TypeDef *pCore)
{
    udcd_t udcd = (udcd_t)pCore->pdata;
    rt_usbd_disconnect_handler(udcd);
}

/* USB�ָ����������ӣ��ж� */
void usbd_resume_callback(USBDCore_TypeDef *pCore)
{
    udcd_t udcd = (udcd_t)pCore->pdata;
    rt_usbd_connect_handler(udcd);
}

/* USB�˵�0�ж� */
/* �˵�0�����ж� */
void usbd_setup_callback(USBDCore_TypeDef *pCore)
{
    udcd_t udcd = (udcd_t)pCore->pdata;
    rt_usbd_ep0_setup_handler(udcd, (struct urequest *)&pCore->Device.Request);
}

/* �˵�0�����жϣ����Թ��������˵������жϣ� */
void usbd_ep0_in_callback(USBDCore_TypeDef *pCore)
{
    udcd_t udcd = (udcd_t)pCore->pdata;
    rt_usbd_ep0_in_handler(udcd);
}

/* �˵�0����жϣ����Թ��������˵�����жϣ� */
void usbd_ep0_out_callback(USBDCore_TypeDef *pCore)
{
    udcd_t udcd = (udcd_t)pCore->pdata;
    rt_usbd_ep0_out_handler(udcd, pCore->Device.Transfer.sByteLength);
}

/* USB�����˵��ж� */
/* �����˵������ж� */
void usbd_ep_in_callback(USBDCore_TypeDef *pCore, USBD_EPTn_Enum EPTn)
{
    udcd_t udcd = (udcd_t)pCore->pdata;
    rt_usbd_ep_in_handler(udcd, EPTn | 0x80, pCore->Device.Transfer.sByteLength);
}

/* �����˵�����ж� */
void usbd_ep_out_callback(USBDCore_TypeDef *pCore, USBD_EPTn_Enum EPTn)
{
    udcd_t udcd = (udcd_t)pCore->pdata;
    rt_usbd_ep_out_handler(udcd, EPTn, pCore->Device.Transfer.sByteLength);
}




//rt_err_t rt_usbd_set_feature(udevice_t device, rt_uint16_t value, rt_uint16_t index);
//rt_err_t rt_usbd_clear_feature(udevice_t device, rt_uint16_t value, rt_uint16_t index);
//rt_err_t rt_usbd_ep_set_stall(udevice_t device, uep_t ep);
//rt_err_t rt_usbd_ep_clear_stall(udevice_t device, uep_t ep);
//rt_err_t rt_usbd_ep0_set_stall(udevice_t device);
//rt_err_t rt_usbd_ep0_clear_stall(udevice_t device);


/*********************************************************************************************************//**
  * @brief  Configure USB.
  * @retval None
  ***********************************************************************************************************/
static void USB_Configuration(USBDCore_TypeDef *pCore)
{
    CKCU_PeripClockConfig_TypeDef CKCUClock = {{ 0 }};
    CKCUClock.Bit.USBD       = 1;
    CKCUClock.Bit.EXTI       = 1;
    CKCU_PeripClockConfig(CKCUClock, ENABLE);
    
    int_p_usbd_code = pCore;

#if (LIBCFG_CKCU_USB_PLL)
    USBPLL_Configuration();
#endif

#if (LIBCFG_PWRCU_VREG)
    USBVRG_Configuration();                               /* Voltage of USB setting                           */
#endif

    pCore->pDriver = (u32 *)&gUSBDriver;                /* Initiate memory pointer of USB driver            */
    pCore->Power.CallBack_Suspend.func  = Suspend;      /* Install suspend call back function into USB core */

//    gUSBCore.pDriver = (u32 *)&gUSBDriver;                /* Initiate memory pointer of USB driver            */
//    gUSBCore.Power.CallBack_Suspend.func  = Suspend;      /* Install suspend call back function into USB core */
    //gUSBCore.Power.CallBack_Suspend.uPara = (u32)NULL;
    
    /* ��������ʼ�� */
//    USBDDesc_Init(&pCore->Device.Desc);                 /* Initiate memory pointer of descriptor            */
    /* USB���ʼ�� */
//    USBDClass_Init(&(pCore->Class));                      /* Initiate USB Class layer                         */
    /* USB�ں˳�ʼ�� */
    USBDCore_Init(pCore);                             /* Initiate USB Core layer                          */

    /* !!! NOTICE !!!
       Must turn on if the USB clock source is from HSI (PLL clock Source)
    */
#if 0
    {
        /* Turn on HSI auto trim function                                                                       */
        CKCU_HSIAutoTrimClkConfig(CKCU_ATC_USB);
        CKCU_HSIAutoTrimCmd(ENABLE);
    }
#endif

    NVIC_EnableIRQ(USB_IRQn);                             /* Enable USB device interrupt                      */
}

#if (LIBCFG_CKCU_USB_PLL)
/*********************************************************************************************************//**
 * @brief  Configure USB PLL
 * @retval None
 ************************************************************************************************************/
static void USBPLL_Configuration(void)
{
    {
        /* USB PLL configuration                                                                                */

        /* !!! NOTICE !!!
           Notice that the local variable (structure) did not have an initial value.
           Please confirm that there are no missing members in the parameter settings below in this function.
        */
        CKCU_PLLInitTypeDef PLLInit;

        PLLInit.ClockSource = CKCU_PLLSRC_HSE;  // CKCU_PLLSRC_HSE or CKCU_PLLSRC_HSI
#if (LIBCFG_CKCU_USB_PLL_96M)
        PLLInit.CFG = CKCU_USBPLL_8M_96M;
#else
        PLLInit.CFG = CKCU_USBPLL_8M_48M;
#endif
        PLLInit.BYPASSCmd = DISABLE;
        CKCU_USBPLLInit(&PLLInit);
    }

    CKCU_USBPLLCmd(ENABLE);

    while (CKCU_GetClockReadyStatus(CKCU_FLAG_USBPLLRDY) == RESET);
    CKCU_USBClockConfig(CKCU_CKUSBPLL);
}
#endif

#if (LIBCFG_PWRCU_VREG)
/*********************************************************************************************************//**
 * @brief  Configure USB Voltage
 * @retval None
 ************************************************************************************************************/
static void USBVRG_Configuration(void)
{
    CKCU_PeripClockConfig_TypeDef CKCUClock = {{ 0 }};
    CKCUClock.Bit.BKP                   = 1;
    CKCU_PeripClockConfig(CKCUClock, ENABLE);

    PWRCU_SetVREG(PWRCU_VREG_3V3);

    /* !!! NOTICE !!!
       USB LDO should be enabled (PWRCU_VREG_ENABLE) if the MCU VDD > 3.6 V.
    */
    PWRCU_VREGConfig(PWRCU_VREG_BYPASS);
}
#endif

#define REMOTE_WAKEUP      (0)
/*********************************************************************************************************//**
  * @brief  Suspend call back function which enter DeepSleep1
  * @param  uPara: Parameter for Call back function
  * @retval None
  ***********************************************************************************************************/
static void Suspend(u32 uPara)
{
#if (REMOTE_WAKEUP == 1)
    u32 IsRemoteWakeupAllowed;
#endif

    if (gIsLowPowerAllowed)
    {

#if (REMOTE_WAKEUP == 1)
        /* Disable EXTI interrupt to prevent interrupt occurred after wakeup                                    */
        EXTI_IntConfig(KEY1_BUTTON_EXTI_CHANNEL, DISABLE);
        IsRemoteWakeupAllowed = USBDCore_GetRemoteWakeUpFeature(&gUSBCore);

        if (IsRemoteWakeupAllowed == TRUE)
        {
            /* Enable EXTI wake event and clear wakeup flag                                                       */
            EXTI_WakeupEventConfig(KEY1_BUTTON_EXTI_CHANNEL, EXTI_WAKEUP_LOW_LEVEL, ENABLE);
            EXTI_ClearWakeupFlag(KEY1_BUTTON_EXTI_CHANNEL);
        }
#endif

        __DBG_USBPrintf("%06ld >DEEPSLEEP\r\n", ++__DBG_USBCount);

        // Add your procedure here which disable related IO to reduce power consumption
        // ..................
        //
        
        if ((int_p_usbd_code->Info.CurrentStatus == USB_STATE_SUSPENDED) && ((HT_USB->CSR & 0xC0) == 0x40))   // D+ = 1, D- = 0
        {
            /* For Bus powered device, you must enter DeepSleep1 when device has been suspended. For self-powered */
            /* device, you may decide to enter DeepSleep1 or not depended on your application.                    */

            /* For the convenient during debugging and evaluation stage, the USBDCore_LowPower() is map to a null */
            /* function by default. In the real product, you must map this function to the low power function of  */
            /* firmware library by setting USBDCORE_ENABLE_LOW_POWER as 1 (in the ht32fxxxx_usbdconf.h file).     */
            USBDCore_LowPower();
        }

        // Add your procedure here which recovery related IO for application
        // ..................
        //

        __DBG_USBPrintf("%06ld <DEEPSLEEP\r\n", ++__DBG_USBCount);

#if (REMOTE_WAKEUP == 1)
        if (EXTI_GetWakeupFlagStatus(KEY1_BUTTON_EXTI_CHANNEL) == SET)
        {
            __DBG_USBPrintf("%06ld WAKEUP\r\n", ++__DBG_USBCount);
            if (IsRemoteWakeupAllowed == TRUE && USBDCore_IsSuspend(&gUSBCore) == TRUE)
            {
                USBDCore_TriggerRemoteWakeup();
            }
        }

        if (IsRemoteWakeupAllowed == TRUE)
        {
            /* Disable EXTI wake event and clear wakeup flag                                                      */
            EXTI_WakeupEventConfig(KEY1_BUTTON_EXTI_CHANNEL, EXTI_WAKEUP_LOW_LEVEL, DISABLE);
            EXTI_ClearWakeupFlag(KEY1_BUTTON_EXTI_CHANNEL);
        }

        /* Clear EXTI edge flag and enable EXTI interrupt                                                       */
        EXTI_ClearEdgeFlag(KEY1_BUTTON_EXTI_CHANNEL);
        EXTI_IntConfig(KEY1_BUTTON_EXTI_CHANNEL, ENABLE);
#endif
    }

    return;
}



















#endif /* RT_USING_USB_DEVICE */
