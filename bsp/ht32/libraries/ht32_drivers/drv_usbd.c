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

/* usb����ṹ�� */
struct ht32_usbd
{
    char *name;
    USBDCore_TypeDef *p_usbd_code;
    IRQn_Type irq;
};

__ALIGN4 USBDCore_TypeDef p_usbd;
/* internal mount point */
/* �ڲ����ص� */
static struct ht32_usbd *p_usbd_instance = RT_NULL;

/* Endpoint Function List */
/* �˵㹦���б� */
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
/* usbd�豸�б� */
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
    
    /* ���ﳢ�Խ�ʹ��pCore->Device.Transfer.sByteLength = pCore->Device.Request.wLength*/
    /* �˴��ǵ���USBö�ٹ�����ԭ�� */
    pCore->Device.Transfer.sByteLength = pCore->Device.Request.wLength;
    rt_usbd_ep0_out_handler(udcd, pCore->Device.Transfer.sByteLength);
    
//    rt_usbd_ep0_out_handler(udcd, pCore->ept_io->trans_len);
}

/* USB�����˵��ж� */
/* �����˵������ж� */
void usbd_ep_in_callback(USBDCore_TypeDef *pCore, USBD_EPTn_Enum EPTn)
{
    udcd_t udcd = (udcd_t)pCore->pdata;
    /* ����Ӧ��ȷ�����͵������ж೤��Ȼ�󽫷��͵����ݸ����ص����� */
    pCore->Device.Transfer.sByteLength = 0;
    rt_usbd_ep_in_handler(udcd, EPTn | 0x80, pCore->Device.Transfer.sByteLength);
}

/* �����˵�����ж� */
void usbd_ep_out_callback(USBDCore_TypeDef *pCore, USBD_EPTn_Enum EPTn)
{

    udcd_t udcd = (udcd_t)pCore->pdata;
    /* ����Ӧ�ö�ȡ���˵���յ��������ж೤��Ȼ�󽫸����ݸ����ص����� */
//    pCore->Device.Transfer.sByteLength = USBD_EPTGetTransferCount(EPTn,USBD_CNTB0);
    rt_usbd_ep_out_handler(udcd, EPTn, pCore->ept_io->trans_len);
}

/*********************************************************************************************************************/

/* ���õ�ַ */
static rt_err_t ht32_set_address(rt_uint8_t address)
{
    /* �����豸��ַ */
    p_usbd_instance->p_usbd_code->Info.CurrentStatus = USER_USB_STATE_ADDRESS;
    API_USB_SET_ADDR(address);
    return RT_EOK;
}
/* ��������(�����Ǹ��ݶ˵�ĵ�ַ�������˵�) */
static rt_err_t ht32_set_config(rt_uint8_t address)
{
    return RT_EOK;
}
/* �˵���ͣ */
static rt_err_t ht32_ep_set_stall(rt_uint8_t address)
{
    if(0==(address&0x7f))
        API_USB_EPTn_SEND_STALL((USBD_EPTn_Enum)(address & 0x7f));
    else
        API_USB_EPTn_SET_HALT((USBD_EPTn_Enum)(address & 0x7f));
    return RT_EOK;
}
/* �˵����� */
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
/* �˵�ʹ�� */
static rt_err_t ht32_ep_enable(struct uendpoint *ep)
{
    /* �̼�������ʱû���ҵ���ع��� */
    RT_ASSERT(ep != RT_NULL);
    RT_ASSERT(ep->ep_desc != RT_NULL);
    usbd_ep_enable(p_usbd_instance->p_usbd_code, ep->ep_desc->bEndpointAddress);
    return RT_EOK;
}
/* �˵�ʧ�� */
static rt_err_t ht32_ep_disable(struct uendpoint *ep)
{
    /* �̼�������ʱû���ҵ���ع��� */
    RT_ASSERT(ep != RT_NULL);
    RT_ASSERT(ep->ep_desc != RT_NULL);
    usbd_ep_disable(p_usbd_instance->p_usbd_code, ep->ep_desc->bEndpointAddress);
    return RT_EOK;
}
/* �˵��������׼��(����ֻ��׼�����������������ȡ���ݣ����������ж��ж�ȡ��) */
static rt_ssize_t ht32_ep_read_prepare(rt_uint8_t address, void *buffer, rt_size_t size)
{
    static uint8_t read_flag = 0;
    uint8_t read_buff[64] = {0};
    /*
        USB��ȡ���ݵ��������£�
        1���ϴζ�ȡ���ݵ�������USB�ں�׼���´�Ҫ�����ݶ�ȡ��ָ���ĵ�ַ
        2�����ζ�ȡ�����ݺ󣬻�
    */

    /* �Ӹú����ж�ȡ���� *//* USB�޷����ӵĴ���� */
    /* ��Ŀǰ������������²���ep0->request.bufferû�����뵽��Ч���ڴ������µ� */
    /* ��һ���Ĳ���Ŀ����ǲ鿴ep0->request.buffer��û�����뵽��Ч���ڴ� */
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
        /* �˴��Ĵ���������buffer������Խ����ɵ� */
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
/* �˵������ */
static rt_ssize_t ht32_ep_read(rt_uint8_t address, void *buffer)
{
    /* ��������������ȡ���� */
    rt_size_t size = 0;
    RT_ASSERT(buffer != RT_NULL);
//    USBDCore_EPTReadOUTData((USBD_EPTn_Enum)(address & 0x7f), (uint32_t *)buffer, size);
    return size;
}
/* �˵�д���� */
static rt_ssize_t ht32_ep_write(rt_uint8_t address, void *buffer, rt_size_t size)
{
    /* ʹ�øú�����USB�˵�д������ */
    return USBDCore_EPTWriteINData((USBD_EPTn_Enum)(address & 0x7f), (uint32_t *)buffer, size);
//    return RT_EOK;
}
/* �˵�0����״̬ */
static rt_err_t ht32_ep0_send_status(void)
{
    uint8_t Date = 0;
    /* ���Ͷ˵�0��״̬ */
    API_USB_EPTn_WRITE_IN(USBD_EPT0, (u32*)&Date, 0);
    return RT_EOK;
}
/* USB��ͣ */
static rt_err_t ht32_suspend(void)
{
    return RT_EOK;
}
/* USB���� */
static rt_err_t ht32_wakeup(void)
{
    return RT_EOK;
}

/* USB�豸�ӿں��� */
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
    /* USB�������� */
    USBDCore_MainRoutine(p_usbd_instance->p_usbd_code);
}
/* USB�豸��ʼ������ */
static rt_err_t ht32_dcd_init(rt_device_t device)
{
    /* USB����ͽӿڳ�ʼ�����Լ�����USB�ж� */
    USB_Configuration(p_usbd_instance->p_usbd_code);
    
    rt_thread_idle_sethook(usbd_mainroutine);

    return RT_EOK;
}


/* USB�豸ע�ắ�� */
int ht32_usbd_register(void)
{
    rt_size_t obj_num;
    rt_err_t result = 0;
    int index;
    USBDCore_TypeDef *p_usbd_core = &p_usbd;
    /* �����ж��ٸ�USB�豸 */
    obj_num = sizeof(usbd_config) / sizeof(struct ht32_usbd);

    for (index = 0; index < obj_num; index++)
    {
        /* ����һ��udcd�����ڴ沢��� */
        udcd_t udcd = (udcd_t)rt_malloc(sizeof(struct udcd));
        if (udcd == RT_NULL)
        {
            rt_kprintf("udcd malloc failed\r\n");
            return -RT_ERROR;
        }
        rt_memset((void *)udcd, 0, sizeof(struct udcd));

        /* ����һ��USB�ں˶����ڴ沢��� */
//        USBDCore_TypeDef *p_usbd_core = (USBDCore_TypeDef *)rt_malloc(sizeof(USBDCore_TypeDef));
//        if (p_usbd_core == RT_NULL)
//        {
//            rt_kprintf("usbd_core malloc failed\r\n");
//            return -RT_ERROR;
//        }
//        rt_memset((void *)p_usbd_core, 0, sizeof(USBDCore_TypeDef));

        /* Ϊ�����udcd����ֵ */
        udcd->parent.type = RT_Device_Class_USBDevice;      //�豸����
        udcd->parent.init = ht32_dcd_init;                  //�豸��ʼ������

        udcd->parent.user_data = p_usbd_core;               //�豸��˽��ָ�룬һ�㱣��Ķ����û��Լ�����Ķ���ṹ��ָ�룬���ﱣ�����AT�Զ����USB�ں�ָ��
        udcd->ops = &_udc_ops;                              //usb�����ӿڽṹ��ָ��

        
        /* AT�Լ�USB�ں˶����˽��ָ�루��ָ����Ϊ������RTTʱ���ӵģ�*/
        p_usbd_core->pdata = udcd;
        usbd_config[index].p_usbd_code = p_usbd_core;       //��������뵽���Լ������USB�ں�ָ�����ֵ���Լ������USB����ָ����

        /* register endpoint infomation */
        udcd->ep_pool = endpoint_pool;                      //USB�˵���Ϣ�����������USB�˵㹦���б�
        udcd->ep0.id = &endpoint_pool[0];                   //USB�˵�0����Ϣ

        /* USB�豸ע�� */
        /* �������豸�����ṹ�壬�豸���ƣ��豸���ܱ�־*/
        result = rt_device_register((rt_device_t)udcd, usbd_config[index].name, 0);
        RT_ASSERT(result == RT_EOK);

        /* ע����ɺ���Զ����USB����ṹ��ָ����ص��ļ��ڲ����豸���ص㣨����������ļ��н��в����� */
        p_usbd_instance = &usbd_config[index];

        /* RTT��USB�ں˳�ʼ�� */
        result = rt_usb_device_init();
        RT_ASSERT(result == RT_EOK);
    }

    return result;
}
INIT_DEVICE_EXPORT(ht32_usbd_register);







/* USB�жϺ��� */
void USB_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* �жϻص����� */
    usbd_irq_handler(p_usbd_instance->p_usbd_code);

    /* leave interrupt */
    rt_interrupt_leave();
}

#endif /* RT_USING_USB_DEVICE */
