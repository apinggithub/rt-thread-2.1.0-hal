/*
 * File      : usart_wifi_esp12f.c
 * This file is part of RT-Thread RTOS
 * Copyright by Shanghai Real-Thread Electronic Technology Co.,Ltd
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2014-07-31     aozima       the first version
 * 2014-09-18     aozima       update command & response.
 */

#include <rtthread.h>
#include <drivers/usart.h>

#include <netif/ethernetif.h>
#include <netif/etharp.h>
#include <lwip/icmp.h>
#include "lwipopts.h"

#define WIFI_DEBUG_ON
// #define ETH_RX_DUMP
// #define ETH_TX_DUMP

#ifdef WIFI_DEBUG_ON
#define WIFI_DEBUG         rt_kprintf("[ESP-12F] ");rt_kprintf
//#define USART_DEBUG         rt_kprintf("[USART] ");rt_kprintf
#define USART_DEBUG(...)
#else
#define WIFI_DEBUG(...)
#define WSART_DEBUG(...)
#endif /* #ifdef WIFI_DEBUG_ON */

/********************************* ESP-12F **************************************/
#include "usart_wifi_esp12f.h"

/* tools */
#define node_entry(node, type, member) \
    ((type *)((char *)(node) - (unsigned long)(&((type *)0)->member)))
#define member_offset(type, member) \
    ((unsigned long)(&((type *)0)->member))

#define MAX_USART_PACKET_SIZE     (member_offset(struct usart_data_packet, buffer) + USART_MAX_DATA_LEN)
#define MAX_USART_BUFFER_SIZE     (sizeof(struct usart_response) + MAX_USART_PACKET_SIZE)
#define MAX_ADDR_LEN 6

struct esp12f_wifi
{
    /* inherit from ethernet device */
    struct eth_device parent;

    struct rt_usart_device *rt_usart_device;

    /* interface address info. */
    rt_uint8_t  dev_addr[MAX_ADDR_LEN];         /* hw address   */
    rt_uint8_t  active;

    struct rt_mempool usart_tx_mp;
    struct rt_mempool usart_rx_mp;

    struct rt_mailbox usart_tx_mb;
    struct rt_mailbox eth_rx_mb;

    int usart_tx_mb_pool[USART_TX_POOL_SIZE + 1];
    int eth_rx_mb_pool[USART_RX_POOL_SIZE + 1];

    int esp12f_cmd_mb_pool[3];
    struct rt_mailbox esp12f_cmd_mb;
    uint32_t last_cmd;

    ALIGN(4)
    rt_uint8_t usart_tx_mempool[(sizeof(struct usart_data_packet) + 4) * USART_TX_POOL_SIZE];
    ALIGN(4)
    rt_uint8_t usart_rx_mempool[(sizeof(struct usart_data_packet) + 4) * USART_RX_POOL_SIZE];

    ALIGN(4)
    uint8_t usart_hw_rx_buffer[MAX_USART_BUFFER_SIZE];

    /* status for ESP12F */
    esp12f_ap_info ap_info;  /* AP info for conn. */
    esp12f_ap_info *ap_scan; /* AP list for SCAN. */
    uint32_t ap_scan_count;
};
static struct esp12f_wifi esp12f_wifi_device;
static struct rt_event usart_wifi_data_event;




void rt_usart_configure()
{
}
void rt_usart_send(struct rt_usart_device *driver,rt_uint16_t len)
{
}
void rt_usart_take_bus()
{
}
void usart_wifi_hw_init()
{
}
void usart_wifi_int_cmd(rt_uint8_t reg)
{
}
rt_uint8_t usart_wifi_is_busy(void)
{
	return 0;
}

static void resp_handler(struct esp12f_wifi *wifi_device, struct esp12f_resp *resp)
{
    struct esp12f_resp *resp_return = RT_NULL;

    switch (resp->cmd)
    {
    case ESP12F_CMD_INIT:
        WIFI_DEBUG("resp_handler ESP12F_CMD_INIT\n");
        resp_return = (struct esp12f_resp *)rt_malloc(member_offset(struct esp12f_resp, resp) + sizeof(esp12f_resp_init)); //TODO:
        if(resp_return == RT_NULL) break;
        memcpy(resp_return, resp, member_offset(struct esp12f_resp, resp) + sizeof(esp12f_resp_init));

        WIFI_DEBUG("sn:%-*.*s\n", sizeof(resp->resp.init.sn), sizeof(resp->resp.init.sn), resp->resp.init.sn);
        WIFI_DEBUG("version:%-*.*s\n", sizeof(resp->resp.init.version), sizeof(resp->resp.init.version), resp->resp.init.version);

        rt_memcpy(wifi_device->dev_addr, resp->resp.init.mac, 6);
        break;

    case ESP12F_CMD_SCAN:
        if( resp->len == sizeof(esp12f_ap_info) )
        {
            esp12f_ap_info *ap_scan = rt_realloc(wifi_device->ap_scan, sizeof(esp12f_ap_info) * (wifi_device->ap_scan_count + 1) );
            if(ap_scan != RT_NULL)
            {
                memcpy( &ap_scan[wifi_device->ap_scan_count], &resp->resp.ap_info, sizeof(esp12f_ap_info) );

                //dump
                if(1)
                {
#ifdef WIFI_DEBUG_ON
                    esp12f_ap_info *ap_info = &resp->resp.ap_info;
                    WIFI_DEBUG("SCAN SSID:%-32.32s\n", ap_info->ssid);
                    WIFI_DEBUG("SCAN BSSID:%02X-%02X-%02X-%02X-%02X-%02X\n",
                               ap_info->bssid[0],
                               ap_info->bssid[1],
                               ap_info->bssid[2],
                               ap_info->bssid[3],
                               ap_info->bssid[4],
                               ap_info->bssid[5]);
                    WIFI_DEBUG("SCAN rssi:%ddBm\n", ap_info->rssi);
                    WIFI_DEBUG("SCAN rate:%dMbps\n", ap_info->max_data_rate/1000);
                    WIFI_DEBUG("SCAN channel:%d\n", ap_info->channel);
                    WIFI_DEBUG("SCAN security:%08X\n\n", ap_info->security);
#endif /* WIFI_DEBUG_ON */
                }

                wifi_device->ap_scan_count++;
                wifi_device->ap_scan = ap_scan;
            }

            return; /* wait for next ap */
        }
        break;
    case ESP12F_CMD_JOIN:
    case ESP12F_CMD_EASY_JOIN:
        WIFI_DEBUG("resp_handler ESP12F_CMD_EASY_JOIN\n");
        resp_return = (struct esp12f_resp *)rt_malloc(member_offset(struct esp12f_resp, resp) + sizeof(esp12f_resp_join)); //TODO:
        if(resp_return == RT_NULL) break;
        memcpy(resp_return, resp, member_offset(struct esp12f_resp, resp) + sizeof(esp12f_resp_join));

        if( resp->result == 0 )
        {
            memcpy(&wifi_device->ap_info, &resp_return->resp.ap_info, sizeof(esp12f_resp_join));
            wifi_device->active = 1;
            eth_device_linkchange(&wifi_device->parent, RT_TRUE);
        }
        else
        {
            WIFI_DEBUG("ESP12F_CMD_EASY_JOIN result: %d\n", resp->result );
        }

        //dupm
        if(1)
        {
#ifdef WIFI_DEBUG_ON
            esp12f_ap_info *ap_info = &resp->resp.ap_info;
            WIFI_DEBUG("JOIN SSID:%-32.32s\n", ap_info->ssid);
            WIFI_DEBUG("JOIN BSSID:%02X-%02X-%02X-%02X-%02X-%02X\n",
                       ap_info->bssid[0],
                       ap_info->bssid[1],
                       ap_info->bssid[2],
                       ap_info->bssid[3],
                       ap_info->bssid[4],
                       ap_info->bssid[5]);
            WIFI_DEBUG("JOIN rssi:%ddBm\n", ap_info->rssi);
            WIFI_DEBUG("JOIN rate:%dMbps\n", ap_info->max_data_rate/1000);
            WIFI_DEBUG("JOIN channel:%d\n", ap_info->channel);
            WIFI_DEBUG("JOIN security:%08X\n\n", ap_info->security);
#endif /* WIFI_DEBUG_ON */
        }
        break;

    case ESP12F_CMD_RSSI:
        // TODO: client RSSI.
    {
        esp12f_ap_info *ap_info = &resp->resp.ap_info;
        wifi_device->ap_info.rssi = ap_info->rssi;
        WIFI_DEBUG("current RSSI: %d\n", wifi_device->ap_info.rssi);
    }
    break;

    case ESP12F_CMD_SOFTAP:
    {
        if( resp->result == 0 )
        {
            ;
            wifi_device->active = 1;
            eth_device_linkchange(&wifi_device->parent, RT_TRUE);
        }
        else
        {
            WIFI_DEBUG("ESP12F_CMD_EASY_JOIN result: %d\n", resp->result );
        }

    }
    break;

    default:
        WIFI_DEBUG("resp_handler %d\n", resp->cmd);
        break;
    }


    if(resp->cmd == wifi_device->last_cmd)
    {
        rt_mb_send(&wifi_device->esp12f_cmd_mb, (rt_uint32_t)resp_return);
        return;
    }
    else
    {
        rt_free(resp_return);
    }
}

static rt_err_t esp12f_cmd(struct esp12f_wifi *wifi_device, uint32_t cmd, void *args)
{
    rt_err_t result = RT_EOK;
    rt_int32_t timeout = ESP12F_CMD_TIMEOUT;

    struct usart_data_packet *data_packet;
    struct esp12f_cmd *wifi_cmd = RT_NULL;
    struct esp12f_resp *resp = RT_NULL;

    wifi_device->last_cmd = cmd;

    data_packet = (struct usart_data_packet *)rt_mp_alloc(&wifi_device->usart_tx_mp, RT_WAITING_FOREVER);
    wifi_cmd = (struct esp12f_cmd *)data_packet->buffer;

    wifi_cmd->cmd = cmd;
    wifi_cmd->len = 0;

    if( cmd == ESP12F_CMD_INIT )
    {
        wifi_cmd->len = sizeof(esp12f_cmd_init);
    }
    else if( cmd == ESP12F_CMD_SCAN )
    {
        wifi_cmd->len = 0;
        timeout += RT_TICK_PER_SECOND*10;

        if(wifi_device->ap_scan)
        {
            rt_free(wifi_device->ap_scan);
            wifi_device->ap_scan = RT_NULL;
            wifi_device->ap_scan_count = 0;
        }
    }
    else if( cmd == ESP12F_CMD_JOIN )
    {
        wifi_cmd->len = sizeof(esp12f_cmd_join);
    }
    else if( cmd == ESP12F_CMD_EASY_JOIN )
    {
        wifi_cmd->len = sizeof(esp12f_cmd_easy_join);
        timeout += RT_TICK_PER_SECOND*5;
    }
    else if( cmd == ESP12F_CMD_RSSI )
    {
        wifi_cmd->len = sizeof(esp12f_cmd_rssi);
    }
    else if( cmd == ESP12F_CMD_SOFTAP )
    {
        wifi_cmd->len = sizeof(esp12f_cmd_softap);
    }
    else
    {
        WIFI_DEBUG("unkown ESP12F CMD %d\n", cmd);
        result = -RT_ENOSYS;
        rt_mp_free(data_packet);
        data_packet = RT_NULL;
    }

    if(data_packet == RT_NULL)
    {
        goto _exit;
    }

    if(wifi_cmd->len)
        memcpy(&wifi_cmd->params, args, wifi_cmd->len);

    data_packet->data_type = data_type_cmd;
    data_packet->data_len = member_offset(struct esp12f_cmd, params) + wifi_cmd->len;

    rt_mb_send(&wifi_device->usart_tx_mb, (rt_uint32_t)data_packet);
    rt_event_send(&usart_wifi_data_event, 1);

    result = rt_mb_recv(&wifi_device->esp12f_cmd_mb,
                        (rt_uint32_t *)&resp,
                        timeout);

    if ( result != RT_EOK )
    {
        WIFI_DEBUG("CMD %d error, resultL %d\n", cmd, result );
    }

    if(resp != RT_NULL)
        result = resp->result;

_exit:
    wifi_device->last_cmd = 0;
    if(resp) rt_free(resp);
    return result;
}

static rt_err_t usart_wifi_transfer(struct esp12f_wifi *dev)
{
    struct pbuf *p = RT_NULL;
    struct usart_cmd_request cmd;
    struct usart_response resp;

    rt_err_t result;
    const struct usart_data_packet *data_packet = RT_NULL;

    struct esp12f_wifi *wifi_device = (struct esp12f_wifi *)dev;
    struct rt_usart_device *rt_usart_device = wifi_device->rt_usart_device;

    usart_wifi_int_cmd(0);
    while (usart_wifi_is_busy());
    USART_DEBUG("sequence start!\n");

    memset(&cmd, 0, sizeof(struct usart_cmd_request));
    cmd.magic1 = CMD_MAGIC1;
    cmd.magic2 = CMD_MAGIC2;

    cmd.flag |= CMD_FLAG_MRDY;

    result = rt_mb_recv(&wifi_device->usart_tx_mb,
                        (rt_uint32_t *)&data_packet,
                        0);
    if ((result == RT_EOK) && (data_packet != RT_NULL) && (data_packet->data_len > 0))
    {
        cmd.M2S_len = data_packet->data_len + member_offset(struct usart_data_packet, buffer);
        //USART_DEBUG("cmd.M2S_len = %d\n", cmd.M2S_len);
    }

    //rt_usart_send(rt_usart_device,&cmd, sizeof(cmd));
    while (usart_wifi_is_busy());

    {
        struct rt_usart_message message;
        uint32_t max_data_len = 0;

        /* setup message */
        message.send_buf = RT_NULL;
        //message.recv_buf = &resp;
        message.length = sizeof(resp);
        message.cs_take = 1;
        message.cs_release = 0;

        rt_usart_take_bus(rt_usart_device);

        /* transfer message */
        //rt_usart_device->bus->ops->xfer(rt_usart_device, &message);

        if ((resp.magic1 != RESP_MAGIC1) || (resp.magic2 != RESP_MAGIC2))
        {
            USART_DEBUG("bad resp magic, abort!\n");
            goto _bad_resp_magic;
        }

        if (resp.flag & RESP_FLAG_SRDY)
        {
            USART_DEBUG("RESP_FLAG_SRDY\n");
            max_data_len = cmd.M2S_len;
        }

        if (resp.S2M_len)
        {
            USART_DEBUG("resp.S2M_len: %d\n", resp.S2M_len);
            if (resp.S2M_len > MAX_USART_PACKET_SIZE)
            {
                USART_DEBUG("resp.S2M_len %d > %d(MAX_USART_PACKET_SIZE), drop!\n", resp.S2M_len, MAX_USART_PACKET_SIZE);
                resp.S2M_len = 0;//drop
            }

            if (resp.S2M_len > max_data_len)
                max_data_len = resp.S2M_len;
        }

        if (max_data_len == 0)
        {
            USART_DEBUG("no rx or tx data!\n");
        }

        //USART_DEBUG("max_data_len = %d\n", max_data_len);

_bad_resp_magic:
        /* setup message */
        /*message.send_buf = data_packet;//&tx_buffer;
        message.recv_buf = wifi_device->usart_hw_rx_buffer;//&rx_buffer;
        message.length = max_data_len;
        message.cs_take = 0;
        message.cs_release = 1;*/

        /* transfer message */
        //rt_usart_device->bus->ops->xfer(rt_usart_device, &message);

        //rt_usart_release_bus(rt_usart_device);

        if (cmd.M2S_len && (resp.flag & RESP_FLAG_SRDY))
        {
            rt_mp_free((void *)data_packet);
        }

        if ((resp.S2M_len) && (resp.S2M_len <= MAX_USART_PACKET_SIZE))
        {
            data_packet = (struct usart_data_packet *)wifi_device->usart_hw_rx_buffer;
            if (data_packet->data_type == data_type_eth_data)
            {

                if (wifi_device->active)
                {
                    p = pbuf_alloc(PBUF_LINK, data_packet->data_len, PBUF_RAM);
                    pbuf_take(p, (rt_uint8_t *)data_packet->buffer, data_packet->data_len);

                    rt_mb_send(&wifi_device->eth_rx_mb, (rt_uint32_t)p);
                    eth_device_ready((struct eth_device *)dev);
                }
                else
                {
                    USART_DEBUG("!active, RX drop.\n");
                }
            }
            else if (data_packet->data_type == data_type_resp)
            {
                USART_DEBUG("data_type_resp\n");
                resp_handler(dev, (struct esp12f_resp *)data_packet->buffer);
            }
            else
            {
                USART_DEBUG("data_type: %d, %dbyte\n",
                          data_packet->data_type,
                          data_packet->data_len);
            }
        }
    }
    usart_wifi_int_cmd(1);

    USART_DEBUG("sequence finish!\n\n");

    if ((cmd.M2S_len == 0) && (resp.S2M_len == 0))
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

#if defined(ETH_RX_DUMP) ||  defined(ETH_TX_DUMP)
static void packet_dump(const char *msg, const struct pbuf *p)
{
    const struct pbuf* q;
    rt_uint32_t i,j;
    rt_uint8_t *ptr = p->payload;

    rt_kprintf("%s %d byte\n", msg, p->tot_len);

    i=0;
    for(q=p; q != RT_NULL; q= q->next)
    {
        ptr = q->payload;

        for(j=0; j<q->len; j++)
        {
            if( (i%8) == 0 )
            {
                rt_kprintf("  ");
            }
            if( (i%16) == 0 )
            {
                rt_kprintf("\r\n");
            }
            rt_kprintf("%02x ",*ptr);

            i++;
            ptr++;
        }
    }
    rt_kprintf("\n\n");
}
#endif /* dump */

/********************************* RT-Thread Ethernet interface begin **************************************/
static rt_err_t esp12f_wifi_init(rt_device_t dev)
{
	
    return RT_EOK;
}

static rt_err_t esp12f_wifi_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t esp12f_wifi_close(rt_device_t dev)
{
    return RT_EOK;
}

static rt_size_t esp12f_wifi_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_size_t esp12f_wifi_write(rt_device_t dev, rt_off_t pos, const void *buffer, rt_size_t size)
{
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_err_t esp12f_wifi_control(rt_device_t dev, rt_uint8_t cmd, void *args)
{
    struct esp12f_wifi *wifi_device = (struct esp12f_wifi *)dev;
    rt_err_t result = RT_EOK;

    if (cmd == NIOCTL_GADDR)
    {
        memcpy(args, wifi_device->dev_addr, 6);
    }
    else
    {
        result = esp12f_cmd(wifi_device, cmd, args);
    }

    return result;
}

/* transmit packet. */
rt_err_t esp12f_wifi_tx(rt_device_t dev, struct pbuf *p)
{
    rt_err_t result = RT_EOK;
    struct usart_data_packet *data_packet;
    struct esp12f_wifi *wifi_device = (struct esp12f_wifi *)dev;

    if (!wifi_device->active)
    {
        WIFI_DEBUG("!active, TX drop!\n");
        return RT_EOK;
    }

    /* get free tx buffer */
    data_packet = (struct usart_data_packet *)rt_mp_alloc(&wifi_device->usart_tx_mp, RT_WAITING_FOREVER);
    if (data_packet != RT_NULL)
    {
        data_packet->data_type = data_type_eth_data;
        data_packet->data_len = p->tot_len;

        pbuf_copy_partial(p, data_packet->buffer, data_packet->data_len, 0);

        rt_mb_send(&wifi_device->usart_tx_mb, (rt_uint32_t)data_packet);
        rt_event_send(&usart_wifi_data_event, 1);
    }
    else
        return -RT_ERROR;

#ifdef ETH_TX_DUMP
    packet_dump("TX dump", p);
#endif /* ETH_TX_DUMP */

    /* Return SUCCESS */
    return result;
}

/* reception packet. */
struct pbuf *esp12f_wifi_rx(rt_device_t dev)
{
    struct pbuf *p = RT_NULL;
    struct esp12f_wifi *wifi_device = (struct esp12f_wifi *)dev;

    if (rt_mb_recv(&wifi_device->eth_rx_mb, (rt_uint32_t *)&p, 0) != RT_EOK)
    {
        return RT_NULL;
    }

#ifdef ETH_RX_DUMP
    if(p)
        packet_dump("RX dump", p);
#endif /* ETH_RX_DUMP */

    return p;
}
/********************************* RT-Thread Ethernet interface end **************************************/

static void usart_wifi_data_thread_entry(void *parameter)
{
    rt_uint32_t e;
    rt_err_t result;

    while (1)
    {
        /* receive first event */
        if (rt_event_recv(&usart_wifi_data_event,
                          1,
                          RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_FOREVER,
                          &e) != RT_EOK)
        {
            continue;
        }

        result = usart_wifi_transfer(&esp12f_wifi_device);

        if (result == RT_EOK)
        {
            rt_event_send(&usart_wifi_data_event, 1);
        }
    }
}

rt_err_t rt_hw_wifi_init(const char *usart_device_name, wifi_mode_t mode)
{
	  
    /* align and struct size check. */
    RT_ASSERT( (USART_MAX_DATA_LEN & 0x03) == 0);
    RT_ASSERT( sizeof(struct esp12f_resp) <= USART_MAX_DATA_LEN);

    memset(&esp12f_wifi_device, 0, sizeof(struct esp12f_wifi));

    esp12f_wifi_device.rt_usart_device = (struct rt_usart_device *)rt_device_find(usart_device_name);

    if (esp12f_wifi_device.rt_usart_device == RT_NULL)
    {
        USART_DEBUG("usart device %s not found!\r\n", usart_device_name);
        return -RT_ENOSYS;
    }

    /* config usart */
    {
        struct rt_usart_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_USART_MODE_0 | RT_USART_MSB; /* USART Compatible: Mode 0. */
        cfg.max_hz = 15 * 1000000; /* 10M */
        rt_usart_configure(esp12f_wifi_device.rt_usart_device, &cfg);
    }

    esp12f_wifi_device.parent.parent.init       = esp12f_wifi_init;
    esp12f_wifi_device.parent.parent.open       = esp12f_wifi_open;
    esp12f_wifi_device.parent.parent.close      = esp12f_wifi_close;
    esp12f_wifi_device.parent.parent.read       = esp12f_wifi_read;
    esp12f_wifi_device.parent.parent.write      = esp12f_wifi_write;
    esp12f_wifi_device.parent.parent.control    = esp12f_wifi_control;
    esp12f_wifi_device.parent.parent.user_data  = RT_NULL;

    esp12f_wifi_device.parent.eth_rx     = esp12f_wifi_rx;
    esp12f_wifi_device.parent.eth_tx     = esp12f_wifi_tx;

    rt_mp_init(&esp12f_wifi_device.usart_tx_mp,
               "usart_tx",
               &esp12f_wifi_device.usart_tx_mempool[0],
               sizeof(esp12f_wifi_device.usart_tx_mempool),
               sizeof(struct usart_data_packet));

    rt_mp_init(&esp12f_wifi_device.usart_rx_mp,
               "usart_rx",
               &esp12f_wifi_device.usart_rx_mempool[0],
               sizeof(esp12f_wifi_device.usart_rx_mempool),
               sizeof(struct usart_data_packet));

    rt_mb_init(&esp12f_wifi_device.usart_tx_mb,
               "usart_tx",
               &esp12f_wifi_device.usart_tx_mb_pool[0],
               USART_TX_POOL_SIZE,
               RT_IPC_FLAG_PRIO);

    rt_mb_init(&esp12f_wifi_device.eth_rx_mb,
               "eth_rx",
               &esp12f_wifi_device.eth_rx_mb_pool[0],
               USART_TX_POOL_SIZE,
               RT_IPC_FLAG_PRIO);

    rt_mb_init(&esp12f_wifi_device.esp12f_cmd_mb,
               "wifi_cmd",
               &esp12f_wifi_device.esp12f_cmd_mb_pool[0],
               sizeof(esp12f_wifi_device.esp12f_cmd_mb_pool) / 4,
               RT_IPC_FLAG_PRIO);
    rt_event_init(&usart_wifi_data_event, "wifi", RT_IPC_FLAG_FIFO);

    usart_wifi_hw_init();

    {
        rt_thread_t tid;


        tid = rt_thread_create("wifi",
                               usart_wifi_data_thread_entry,
                               RT_NULL,
                               2048,
                               RT_THREAD_PRIORITY_MAX - 2,
                               20);

        if (tid != RT_NULL)
            rt_thread_startup(tid);
    }

    /* init: get mac address */
    {
        esp12f_cmd_init init;
        init.mode = mode;
        WIFI_DEBUG("wifi_control ESP12F_CMD_INIT\n");
        esp12f_wifi_control((rt_device_t)&esp12f_wifi_device,
                           ESP12F_CMD_INIT,
                           (void *)&init); // 0: firmware, 1: STA, 2:AP

    }

    /* register eth device */
    eth_device_init(&(esp12f_wifi_device.parent), "w0");
    eth_device_linkchange(&esp12f_wifi_device.parent, RT_FALSE);

    return RT_EOK;
}

void usart_wifi_isr(int vector)
{
    /* enter interrupt */
    rt_interrupt_enter();

    USART_DEBUG("usart_wifi_isr\n");
    rt_event_send(&usart_wifi_data_event, 1);

    /* leave interrupt */
    rt_interrupt_leave();
}

/********************************* ESP12F tools **************************************/
rt_err_t esp12f_join(const char * SSID, const char * passwd)
{
    rt_err_t result;
    rt_device_t wifi_device;
    esp12f_cmd_easy_join easy_join;

    wifi_device = rt_device_find("w0");
    if(wifi_device == RT_NULL)
        return -RT_ENOSYS;

    strncpy( easy_join.ssid, SSID, sizeof(easy_join.ssid) );
    strncpy( easy_join.passwd, passwd, sizeof(easy_join.passwd) );

    result = rt_device_control(wifi_device,
                               ESP12F_CMD_EASY_JOIN,
                               (void *)&easy_join);

    return result;
}

rt_err_t esp12f_softap(const char * SSID, const char * passwd,uint32_t security,uint32_t channel)
{
    rt_err_t result;
    rt_device_t wifi_device;
    esp12f_cmd_softap softap;

    wifi_device = rt_device_find("w0");
    if(wifi_device == RT_NULL)
        return -RT_ENOSYS;

    strncpy( softap.ssid, SSID, sizeof(softap.ssid) );
    strncpy( softap.passwd, passwd, sizeof(softap.passwd) );

    softap.security = security;
    softap.channel = channel;
    result = rt_device_control(wifi_device,
                               ESP12F_CMD_SOFTAP,
                               (void *)&softap);

    return result;
}

int32_t esp12f_rssi(void)
{
    rt_err_t result;
    struct esp12f_wifi * wifi_device;

    wifi_device = (struct esp12f_wifi *)rt_device_find("w0");

    if(wifi_device == RT_NULL)
        return 0;

    if(wifi_device->active == 0)
        return 0;

    // SCAN
    result = rt_device_control((rt_device_t)wifi_device,
                               ESP12F_CMD_RSSI,
                               RT_NULL);

    if(result == RT_EOK)
    {
        return wifi_device->ap_info.rssi;
    }

    return 0;
}

#ifdef RT_USING_FINSH
#include <finsh.h>

static rt_err_t esp12f_scan(void)
{
    rt_err_t result;
    struct esp12f_wifi * wifi_device;

    wifi_device = (struct esp12f_wifi *)rt_device_find("w0");

    rt_kprintf("\nCMD ESP12F_CMD_SCAN \n");
    result = rt_device_control((rt_device_t)wifi_device,
                               ESP12F_CMD_SCAN,
                               RT_NULL);

    rt_kprintf("CMD ESP12F_CMD_SCAN result:%d\n", result);

    if(result == RT_EOK)
    {
        uint32_t i;
        esp12f_ap_info *ap_info;

        for(i=0; i<wifi_device->ap_scan_count; i++)
        {
            ap_info = &wifi_device->ap_scan[i];
            rt_kprintf("AP #%02d SSID: %-32.32s\n", i, ap_info->ssid );
        }
    }

    return result;
}
FINSH_FUNCTION_EXPORT(esp12f_scan, SACN and list AP.);
FINSH_FUNCTION_EXPORT(esp12f_join, ESP12F join to AP.);
FINSH_FUNCTION_EXPORT(esp12f_rssi, get ESP12F current AP rssi.);

#endif // RT_USING_FINSH
