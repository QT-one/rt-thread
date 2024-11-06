/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-07-09     QT-one       first version
 */

#include "drv_usbd.h"

#define USBD_TEST_PIN                GET_PIN(A, 0)

#ifdef RT_USING_USB_DEVICE
#if !defined(BSP_USING_USBD)
    #error "Please define at least one BSP_USING_USBD"
#endif

#if defined(BSP_USING_USBD)
//#include "ht32_usbd_core.h"
//#include "usbd_code.h"
#include "usb_port.h"
#endif

/* usb对象结构体 */
struct ht32_usbd
{
    char *name;
    USBDCore_TypeDef *p_usbd_code;
    IRQn_Type irq;
};

__ALIGN4 USBDCore_TypeDef p_usbd;
/* internal mount point */
/* 内部挂载点 */
static struct ht32_usbd *p_usbd_instance = RT_NULL;

/* Endpoint Function List */
/* 端点功能列表 */
static struct ep_id endpoint_pool[] =
{
    {0x00, USB_EP_ATTR_CONTROL,   USB_DIR_INOUT, 64, ID_ASSIGNED  },
    {0x01, USB_EP_ATTR_BULK,      USB_DIR_IN,    64, ID_UNASSIGNED},
    {0x02, USB_EP_ATTR_BULK,      USB_DIR_OUT,   64, ID_UNASSIGNED},
    {0x03, USB_EP_ATTR_INT,       USB_DIR_IN,    64, ID_UNASSIGNED},
    {0x04, USB_EP_ATTR_INT,       USB_DIR_OUT,   64, ID_UNASSIGNED},
    {0x05, USB_EP_ATTR_ISOC,      USB_DIR_IN,    64, ID_UNASSIGNED},
    {0x06, USB_EP_ATTR_ISOC,      USB_DIR_OUT,   64, ID_UNASSIGNED},
    {0x07, USB_EP_ATTR_TYPE_MASK, USB_DIR_MASK,  64, ID_UNASSIGNED},
};

/* usbd Peripheral List */
/* usbd设备列表 */
static struct ht32_usbd usbd_config[] =
{
#ifdef BSP_USING_USBD
    {
        .name           = BSP_USING_USBD_NAME,
        .p_usbd_code    = NULL,
        .irq            = USB_IRQn,
    },
#endif
};


void usbd_test_pin(void)
{
    static uint8_t pin_flag = 0;
    (pin_flag == 0)?(HT_GPIOA->SRR = GPIO_PIN_0):(HT_GPIOA->RR = GPIO_PIN_0);
    pin_flag = (pin_flag == 0)?(1):(0);
    
}

/* 帧起始（SOF）中断回调 */
void usbd_sof_callback(USBDCore_TypeDef *pCore)
{
    udcd_t udcd = (udcd_t)pCore->pdata;
    rt_usbd_sof_handler(udcd);
}
/* USB复位中断 */
void usbd_reset_callback(USBDCore_TypeDef *pCore)
{
    udcd_t udcd = (udcd_t)pCore->pdata;
    rt_usbd_reset_handler(udcd);
}

/* USB暂停(断开连接)中断 */
void usbd_suspend_callback(USBDCore_TypeDef *pCore)
{
    udcd_t udcd = (udcd_t)pCore->pdata;
    rt_usbd_disconnect_handler(udcd);
}

/* USB恢复（重新连接）中断 */
void usbd_resume_callback(USBDCore_TypeDef *pCore)
{
    udcd_t udcd = (udcd_t)pCore->pdata;
    rt_usbd_connect_handler(udcd);
}

/* USB端点0中断 */
/* 端点0控制中断 */
void usbd_setup_callback(USBDCore_TypeDef *pCore)
{
    udcd_t udcd = (udcd_t)pCore->pdata;
    rt_usbd_ep0_setup_handler(udcd, (struct urequest *)&pCore->Device.Request);
}

/* 端点0输入中断（可以归入其他端点输入中断） */
void usbd_ep0_in_callback(USBDCore_TypeDef *pCore)
{
    udcd_t udcd = (udcd_t)pCore->pdata;
    rt_usbd_ep0_in_handler(udcd);
}

/* 端点0输出中断（可以归入其他端点输出中断） */
void usbd_ep0_out_callback(USBDCore_TypeDef *pCore)
{
    udcd_t udcd = (udcd_t)pCore->pdata;
    
    /* 这里尝试将使用pCore->Device.Transfer.sByteLength = pCore->Device.Request.wLength*/
    /* 此处是导致USB枚举过慢的原因 */
    pCore->Device.Transfer.sByteLength = pCore->Device.Request.wLength;
    rt_usbd_ep0_out_handler(udcd, pCore->Device.Transfer.sByteLength);
    
//    rt_usbd_ep0_out_handler(udcd, pCore->ept_io->trans_len);
}

/* USB其他端点中断 */
/* 其他端点输入中断 */
void usbd_ep_in_callback(USBDCore_TypeDef *pCore, USBD_EPTn_Enum EPTn)
{
    udcd_t udcd = (udcd_t)pCore->pdata;
    /* 这里应该确定发送的数据有多长，然后将发送的数据给到回调函数 */
    pCore->Device.Transfer.sByteLength = 0;
    rt_usbd_ep_in_handler(udcd, EPTn | 0x80, pCore->Device.Transfer.sByteLength);
}

/* 其他端点输出中断 */
void usbd_ep_out_callback(USBDCore_TypeDef *pCore, USBD_EPTn_Enum EPTn)
{

    udcd_t udcd = (udcd_t)pCore->pdata;
    /* 这里应该读取到端点接收到的数据有多长，然后将该数据给到回调函数 */
//    pCore->Device.Transfer.sByteLength = USBD_EPTGetTransferCount(EPTn,USBD_CNTB0);
    rt_usbd_ep_out_handler(udcd, EPTn, pCore->ept_io->trans_len);
}

/*********************************************************************************************************************/

/* 设置地址 */
static rt_err_t ht32_set_address(rt_uint8_t address)
{
    /* 设置设备地址 */
    p_usbd_instance->p_usbd_code->Info.CurrentStatus = USER_USB_STATE_ADDRESS;
    API_USB_SET_ADDR(address);
    return RT_EOK;
}
/* 配置设置(这里是根据端点的地址来开启端点) */
static rt_err_t ht32_set_config(rt_uint8_t address)
{
    return RT_EOK;
}
/* 端点暂停 */
static rt_err_t ht32_ep_set_stall(rt_uint8_t address)
{
    if(0==(address&0x7f))
        API_USB_EPTn_SEND_STALL((USBD_EPTn_Enum)(address & 0x7f));
    else
        API_USB_EPTn_SET_HALT((USBD_EPTn_Enum)(address & 0x7f));
    return RT_EOK;
}
/* 端点重启 */
static rt_err_t ht32_ep_clear_stall(rt_uint8_t address)
{
    if(0==(address&0x7f))
    {
        
    }
    else
    {
        API_USB_EPTn_CLR_HALT((USBD_EPTn_Enum)(address & 0x7f));
        API_USB_EPTn_CLR_DTG((USBD_EPTn_Enum)(address & 0x7f));
    }
    return RT_EOK;
}
/* 端点使能 */
static rt_err_t ht32_ep_enable(struct uendpoint *ep)
{
    /* 固件库中暂时没有找到相关功能 */
    RT_ASSERT(ep != RT_NULL);
    RT_ASSERT(ep->ep_desc != RT_NULL);
    usbd_ep_enable(p_usbd_instance->p_usbd_code, ep->ep_desc->bEndpointAddress);
    return RT_EOK;
}
/* 端点失能 */
static rt_err_t ht32_ep_disable(struct uendpoint *ep)
{
    /* 固件库中暂时没有找到相关功能 */
    RT_ASSERT(ep != RT_NULL);
    RT_ASSERT(ep->ep_desc != RT_NULL);
    usbd_ep_disable(p_usbd_instance->p_usbd_code, ep->ep_desc->bEndpointAddress);
    return RT_EOK;
}
/* 端点接收数据准备(这里只是准备，并不是在这里读取数据，数据是在中断中读取的) */
static rt_ssize_t ht32_ep_read_prepare(rt_uint8_t address, void *buffer, rt_size_t size)
{
    static uint8_t read_flag = 0;
    uint8_t read_buff[64] = {0};
    /*
        USB读取数据的流程如下：
        1、上次读取数据的最后会让USB内核准备下次要将数据读取到指定的地址
        2、本次读取完数据后，会
    */

    /* 从该函数中读取数据 *//* USB无法连接的错误点 */
    /* 从目前的情况来看，猜测是ep0->request.buffer没有申请到有效的内存所导致的 */
    /* 下一步的查找目标就是查看ep0->request.buffer有没有申请到有效的内存 */
    uint16_t length = 0;
//    length = ((address & 0x7f) == USBD_EPT0) ? USBD_EPTGetTransferCount((address & 0x7f), USBD_CNTOUT):USBD_EPTGetTransferCount((address & 0x7f), USBD_CNTB0);
//    rt_kprintf("data length = %d!\n",length);
//    if((size > 64)||((buffer == NULL)&&(size > 0)))
//    {
//        rt_kprintf("ep_read_prepare error!\n");
//        return -1;
//    }
//    else
//    {
//        rt_kprintf("ep num = %d!\n",address & 0x7f);
//        rt_kprintf("data length = %d!\n",size);
    if(read_flag == 0)
    {
        read_flag = 1;
        length = USBDCore_EPTReadOUTData((USBD_EPTn_Enum)(address & 0x7f), (uint32_t *)read_buff, size);
        /* 此处的错误是由于buffer的数据越界造成的 */
        rt_memcpy(buffer, read_buff, length);
        read_flag = 0;
    }
    else
    {
        rt_kprintf("ep_read_prepare error!\n");
    }
//        USBDCore_EPTReadOUTData((USBD_EPTn_Enum)(address & 0x7f), (uint32_t *)buffer, size);
//        rt_kprintf("read end!\n");
//    }
    
//    usbd_ept_recv(p_usbd_instance->p_usbd_code, address, buffer, size);
    return size;
    
//    return RT_EOK;
}
/* 端点读数据 */
static rt_ssize_t ht32_ep_read(rt_uint8_t address, void *buffer)
{
    /* 不从这个函数里读取数据 */
    rt_size_t size = 0;
    RT_ASSERT(buffer != RT_NULL);
//    USBDCore_EPTReadOUTData((USBD_EPTn_Enum)(address & 0x7f), (uint32_t *)buffer, size);
    return size;
}
/* 端点写数据 */
static rt_ssize_t ht32_ep_write(rt_uint8_t address, void *buffer, rt_size_t size)
{
    /* 使用该函数向USB端点写入数据 */
    return USBDCore_EPTWriteINData((USBD_EPTn_Enum)(address & 0x7f), (uint32_t *)buffer, size);
//    return RT_EOK;
}
/* 端点0发送状态 */
static rt_err_t ht32_ep0_send_status(void)
{
    uint8_t Date = 0;
    /* 发送端点0的状态 */
    API_USB_EPTn_WRITE_IN(USBD_EPT0, (u32*)&Date, 0);
    return RT_EOK;
}
/* USB暂停 */
static rt_err_t ht32_suspend(void)
{
    return RT_EOK;
}
/* USB唤醒 */
static rt_err_t ht32_wakeup(void)
{
    return RT_EOK;
}

/* USB设备接口函数 */
const static struct udcd_ops _udc_ops =
{
    .set_address            = ht32_set_address,
    .set_config             = ht32_set_config,
    .ep_set_stall           = ht32_ep_set_stall,
    .ep_clear_stall         = ht32_ep_clear_stall,
    .ep_enable              = ht32_ep_enable,
    .ep_disable             = ht32_ep_disable,
    .ep_read_prepare        = ht32_ep_read_prepare,
    .ep_read                = ht32_ep_read,
    .ep_write               = ht32_ep_write,
    .ep0_send_status        = ht32_ep0_send_status,
    .suspend                = ht32_suspend,
    .wakeup                 = ht32_wakeup,
};

static void usbd_mainroutine(void)
{
    /* USB启动程序 */
    USBDCore_MainRoutine(p_usbd_instance->p_usbd_code);
}
/* USB设备初始化函数 */
static rt_err_t ht32_dcd_init(rt_device_t device)
{
    /* USB对象和接口初始化，以及开启USB中断 */
    USB_Configuration(p_usbd_instance->p_usbd_code);
    
    rt_thread_idle_sethook(usbd_mainroutine);

    return RT_EOK;
}


/* USB设备注册函数 */
int ht32_usbd_register(void)
{
    rt_size_t obj_num;
    rt_err_t result = 0;
    int index;
    USBDCore_TypeDef *p_usbd_core = &p_usbd;
    /* 计算有多少个USB设备 */
    obj_num = sizeof(usbd_config) / sizeof(struct ht32_usbd);

    for (index = 0; index < obj_num; index++)
    {
        /* 申请一个udcd对象内存并清空 */
        udcd_t udcd = (udcd_t)rt_malloc(sizeof(struct udcd));
        if (udcd == RT_NULL)
        {
            rt_kprintf("udcd malloc failed\r\n");
            return -RT_ERROR;
        }
        rt_memset((void *)udcd, 0, sizeof(struct udcd));

        /* 申请一个USB内核对象内存并清空 */
//        USBDCore_TypeDef *p_usbd_core = (USBDCore_TypeDef *)rt_malloc(sizeof(USBDCore_TypeDef));
//        if (p_usbd_core == RT_NULL)
//        {
//            rt_kprintf("usbd_core malloc failed\r\n");
//            return -RT_ERROR;
//        }
//        rt_memset((void *)p_usbd_core, 0, sizeof(USBDCore_TypeDef));

        /* 为申请的udcd对象赋值 */
        udcd->parent.type = RT_Device_Class_USBDevice;      //设备类型
        udcd->parent.init = ht32_dcd_init;                  //设备初始化函数

        udcd->parent.user_data = p_usbd_core;               //设备的私有指针，一般保存的都是用户自己定义的对象结构体指针，这里保存的是AT自定义的USB内核指针
        udcd->ops = &_udc_ops;                              //usb操作接口结构体指针

        
        /* AT自己USB内核对象的私有指针（该指针是为了适配RTT时增加的）*/
        p_usbd_core->pdata = udcd;
        usbd_config[index].p_usbd_code = p_usbd_core;       //这里把申请到的自己定义的USB内核指针给赋值到自己定义的USB对象指针中

        /* register endpoint infomation */
        udcd->ep_pool = endpoint_pool;                      //USB端点信息（这里给的是USB端点功能列表）
        udcd->ep0.id = &endpoint_pool[0];                   //USB端点0的信息

        /* USB设备注册 */
        /* 参数：设备驱动结构体，设备名称，设备功能标志*/
        result = rt_device_register((rt_device_t)udcd, usbd_config[index].name, 0);
        RT_ASSERT(result == RT_EOK);

        /* 注册完成后把自定义的USB对象结构体指针挂载到文件内部的设备挂载点（方便后续在文件中进行操作） */
        p_usbd_instance = &usbd_config[index];

        /* RTT的USB内核初始化 */
        result = rt_usb_device_init();
        RT_ASSERT(result == RT_EOK);
    }

    return result;
}
INIT_DEVICE_EXPORT(ht32_usbd_register);







/* USB中断函数 */
void USB_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* 中断回调函数 */
    usbd_irq_handler(p_usbd_instance->p_usbd_code);

    /* leave interrupt */
    rt_interrupt_leave();
}

#endif /* RT_USING_USB_DEVICE */
