#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/usb/cdc.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/usb/cdc.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/list.h>
#include <linux/etherdevice.h> 
#include "CR_net_device.h"

//TODO it's may be changed int the future
#define CR_READY(cr)	(cr && cr->dev)

#define cr_set_control(cr, control) \
	cr_ctrl_msg(cr, USB_CDC_REQ_SET_CONTROL_LINE_STATE, control, NULL, 0)
#define cr_set_line(cr, line) \
	cr_ctrl_msg(cr, USB_CDC_REQ_SET_LINE_CODING, 0, line, sizeof *(line))
#define cr_send_break(cr, ms) \
	cr_ctrl_msg(cr, USB_CDC_REQ_SEND_BREAK, ms, NULL, 0)

//CR
//struct cr *cr;
static struct usb_driver CR_usb_driver;

char* ifname = "cr%d";

//device method prototypes
static int	CR_open(struct net_device *dev); //register memory, IRQ, I/O port resource
static int	CR_stop(struct net_device *dev); //reverse operation of open()
static int	CR_hard_start_xmit(struct sk_buff *skb, struct net_device *dev); //upper layer send packet(sk_buff) to this net_device_driver
static void	CR_tx_timeout(struct net_device *dev); //a packet transmission is timeout
struct net_device_stats *CR_getstats(struct net_device *dev); // called when ifconfig

static const struct net_device_ops CR_netdev_ops;

static int cr_ctrl_msg(struct cr *cr, int request, int value, void *buf, int len);
static int cr_wb_alloc(struct cr *cr);
static int cr_wb_is_avail(struct cr *cr);
static int cr_write_start(struct cr *cr, int wbn);

static void reset_buffer(struct myRXBuf *buf);
static unsigned long pop_skb_from_buffer(void *dst, struct myRXBuf *buf);
static unsigned long push_data_to_buffer(struct myRXBuf *buf, void *src, unsigned long len);
static unsigned long packet_length_if_ready(struct myRXBuf *buf);

/*
 * Register memory, IRQ, IO port resource.
 */
static int CR_open(struct net_device *netdev)
{
	int i;
	struct net_priv *net_priv;
	struct cr *cr;

	printk("%s:entered\n", __FUNCTION__);
	//open CC1111 USB interface

	net_priv = netdev_priv(netdev);
	cr = net_priv->cr;

	//initialize the netdev private data transmision spinlock
	spin_lock_init(&net_priv->tx_lock);
//	spin_lock_init(&net_priv->rx_lock);
	spin_lock_init(&net_priv->stat_lock);

	mutex_lock(&cr->mutex);

	cr->ctrlurb->dev = cr->dev;
	if (usb_submit_urb(cr->ctrlurb, GFP_KERNEL)) {
		dbg("usb_submit_urb(ctrl irq) failed");
		printk("goto bail_out\n");
		goto bail_out;
	}

	if (0 > cr_set_control(cr, cr->ctrlout = ACM_CTRL_DTR | ACM_CTRL_RTS) &&
	    (cr->ctrl_caps & USB_CDC_CAP_LINE))
	{
		printk("goto full_bailout\n");
		goto full_bailout;
	}

	INIT_LIST_HEAD(&cr->spare_read_urbs);
	INIT_LIST_HEAD(&cr->spare_read_bufs);
	INIT_LIST_HEAD(&cr->filled_read_bufs);

	for (i = 0; i < cr->rx_buflimit; i++)
		list_add(&(cr->ru[i].list), &cr->spare_read_urbs);
	for (i = 0; i < cr->rx_buflimit; i++)
		list_add(&(cr->rb[i].list), &cr->spare_read_bufs);

	cr->throttle = 0;

	tasklet_schedule(&cr->urb_task);
	
	mutex_unlock(&cr->mutex);

	//TODO

 	if(!netif_queue_stopped(netdev))
	{
		printk("!netif_queue_stopped\n");
		netif_start_queue(netdev);
	}
	else
	{
		printk("netif_queue_stopped\n");
		netif_wake_queue(netdev);
	}

	return 0;

full_bailout:
	usb_kill_urb(cr->ctrlurb);
bail_out:
	mutex_unlock(&cr->mutex);

	return -1;
}

static int CR_stop(struct net_device *netdev) //reverse operation of open()
{
	int i, nr;
	struct net_priv *net_priv;
	struct cr *cr;

	printk("%s:entered\n", __FUNCTION__);
	//close CC1111 USB interface

	net_priv = netdev_priv(netdev);
	cr = net_priv->cr;
	nr = cr->rx_buflimit;

	if(netdev)
	{
		if (!netif_queue_stopped(netdev))
		{
			printk("!netif_queue_stopped\n");
			netif_stop_queue(netdev);
		}
		else
		{
			printk("netif_queue_stopped\n");
		}
	}

	mutex_lock(&cr->mutex);

	cr_set_control(cr, cr->ctrlout = 0);

	usb_kill_urb(cr->ctrlurb);
	for (i = 0; i < CR_NW; i++)
		usb_kill_urb(cr->wb[i].urb);
	tasklet_disable(&cr->urb_task);
	for (i = 0; i < nr; i++)
		usb_kill_urb(cr->ru[i].urb);
	tasklet_enable(&cr->urb_task);

	mutex_unlock(&cr->mutex);

	//TODO

	return 0;
}

static int CR_hard_start_xmit(struct sk_buff *skb, struct net_device *netdev) 
{
	//upper layer send packet(sk_buff) to this net_device_driver

	int stat;
	int count, to_send;
	int wbn;
	unsigned long flags1;

	struct net_priv *net_priv;
	struct cr *cr;
	struct cr_wb *wb;

	printk("%s:entered\n", __FUNCTION__);
	//queue
	net_priv = netdev_priv(netdev);
	cr = net_priv->cr;

	count = 0;
	printk("remain wb: %d\n", cr_wb_is_avail(cr));

	while(count<skb->len)
	{
//		spin_lock_irqsave(&cr->write_lock, flags1);
		spin_lock(&cr->write_lock);
		wbn = cr_wb_alloc(cr);
		if (wbn < 0) {
//			spin_unlock_irqrestore(&cr->write_lock, flags1);
			spin_unlock(&cr->write_lock);

			netif_stop_queue(netdev);
			printk("NETDEV_TX_BUSY\n");
			return NETDEV_TX_BUSY;
		}
		wb = &cr->wb[wbn];
	
		if(count==0)
		{
			to_send = ( (skb->len) > cr->writesize-3) ? (cr->writesize-3) : (skb->len);
			memset(wb->buf, '@', 1);
			memset(wb->buf+1, (skb->len)>>8, 1);
			memset(wb->buf+2, (skb->len)&0x0ff, 1);
			memcpy((wb->buf)+3, (skb->data)+count, to_send);
			wb->len = to_send+3;
		}
		else
		{
			to_send = ( (skb->len-count) > cr->writesize) ? (cr->writesize) : (skb->len-count);
			memcpy(wb->buf, (skb->data)+count, to_send);
			wb->len = to_send;
		}

//		spin_unlock_irqrestore(&cr->write_lock, flags1);
		spin_unlock(&cr->write_lock);
		stat = cr_write_start(cr, wbn);
		if(stat>=0)
		{
			count+=to_send;
		}
	}

	spin_lock(&net_priv->stat_lock);
	net_priv->tx_packet++;
	net_priv->tx_byte += count;
	spin_unlock(&net_priv->stat_lock);

	printk("%d bytes packet is sent\n", count);

	dev_kfree_skb(skb);

	//TODO
	return 0;
}

static void CR_tx_timeout(struct net_device *netdev) //a packet transmission is timeout
{
	printk("%s:entered\n", __FUNCTION__);
	//TODO
}

struct net_device_stats *CR_getstats(struct net_device *netdev) // called when ifconfig
{
	printk("%s:entered\n", __FUNCTION__);
	//TODO
}

static int CR_set_mac_address(struct net_device *netdev, void *addr)
{
	memcpy(netdev->dev_addr, addr, ETH_ALEN);
	return 0;
}

static const struct net_device_ops CR_netdev_ops = {
	.ndo_open		= CR_open,
	.ndo_stop		= CR_stop,
//TODO	.ndo_do_ioctl		= CR_ioctl,
	.ndo_start_xmit		= CR_hard_start_xmit,
	.ndo_tx_timeout 	= CR_tx_timeout,
//TODO	.ndo_set_multicast_list = CR_set_multicast,
	.ndo_set_mac_address	= CR_set_mac_address,

//TODO	.ndo_change_mtu		= eth_change_mtu,
//TODO	.ndo_validate_addr	= eth_validate_addr,
};


static int cr_ctrl_msg(struct cr *cr, int request, int value,
							void *buf, int len)
{
	int retval = usb_control_msg(cr->dev, usb_sndctrlpipe(cr->dev, 0),
		request, USB_RT_ACM, value,
		cr->control->altsetting[0].desc.bInterfaceNumber,
		buf, len, 5000);
	printk("cr_control_msg: rq: 0x%02x val: %#x len: %#x result: %d\n",
						request, value, len, retval);
	return retval < 0 ? retval : 0;
}
/* devices aren't required to support these requests.
 * the cdc acm descriptor tells whether they do...
 */

/* control interface reports status changes with "interrupt" transfers */
static void cr_ctrl_irq(struct urb *urb)
{
	struct cr *cr = urb->context;

	int retval;
	int status = urb->status;

	switch (status) {
	case 0:
		printk("%s:state OK\n", __FUNCTION__);
		/* success */
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		return;
	default:
		goto exit;
	}

	if (!CR_READY(cr))
		goto exit;

exit:
	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval)
		dev_err(&urb->dev->dev, "%s - usb_submit_urb failed with "
			"result %d", __func__, retval);
}


/* deliver data from NIC to CC1111 : cr_wb_alloc, 
cr_wb_is_avail, cr_write_done, cr_start_wb
, cr_write_start, cr_write_bulk:START*/

static int cr_wb_alloc(struct cr *cr)
{
	int i, wbn;
	struct cr_wb *wb;

	wbn = 0;
	i = 0;
	for (;;) {
		wb = &cr->wb[wbn];
		if (!wb->use) {
			wb->use = 1;
			printk("valid wb --\n");
			return wbn;
		}
		wbn = (wbn + 1) % CR_NW;
		if (++i >= CR_NW)
			return -1;
	}
}

static int cr_wb_is_avail(struct cr *cr)
{
	int i, n;
	unsigned long flags;

	n = CR_NW;
	spin_lock_irqsave(&cr->write_lock, flags);
	for (i = 0; i < CR_NW; i++)
		n -= cr->wb[i].use;
	spin_unlock_irqrestore(&cr->write_lock, flags);

	printk("valid wb num=%d\n", n);
	return n;
}

/*
 * Finish write. Caller must hold cr->write_lock
 */
static void cr_write_done(struct cr *cr, struct cr_wb *wb)
{
	wb->use = 0;
	cr->transmitting--;
	usb_autopm_put_interface_async(cr->control);
	printk("valid wb ++\n");
}

static int cr_start_wb(struct cr *cr, struct cr_wb *wb)
{
	int rc;

	cr->transmitting++;

	wb->urb->transfer_buffer = wb->buf;
	wb->urb->transfer_dma = wb->dmah;
	wb->urb->transfer_buffer_length = wb->len;
	wb->urb->dev = cr->dev;

	rc = usb_submit_urb(wb->urb, GFP_ATOMIC);
	if (rc < 0) {
		dbg("usb_submit_urb(write bulk) failed: %d", rc);
		cr_write_done(cr, wb);
	}
	return rc;
}

static int cr_write_start(struct cr *cr, int wbn)
{
	unsigned long flags;
	struct cr_wb *wb = &cr->wb[wbn];
	int rc;

	spin_lock_irqsave(&cr->write_lock, flags);
	if (!cr->dev) {
		wb->use = 0;
		spin_unlock_irqrestore(&cr->write_lock, flags);
		printk("valid wb ++\n");
		return -ENODEV;
	}

	dbg("%s susp_count: %d", __func__, cr->susp_count);
	usb_autopm_get_interface_async(cr->control);
	if (cr->susp_count) {
		if (!cr->delayed_wb)
			cr->delayed_wb = wb;
		else
			usb_autopm_put_interface_async(cr->control);
		spin_unlock_irqrestore(&cr->write_lock, flags);
		return 0;	/* A white lie */
	}
	usb_mark_last_busy(cr->dev);

	rc = cr_start_wb(cr, wb);
	spin_unlock_irqrestore(&cr->write_lock, flags);

	return rc;

}


static void cr_write_bulk(struct urb *urb)
{
	struct cr_wb *wb = urb->context;
	struct cr *cr = wb->instance;
	unsigned long flags;

	spin_lock_irqsave(&cr->write_lock, flags);

	cr_write_done(cr, wb);
	spin_unlock_irqrestore(&cr->write_lock, flags);

	if(netif_queue_stopped(cr->netdev))
	{
		if(cr_wb_is_avail(cr)>CR_NW/2)
		{
			netif_wake_queue(cr->netdev);
		}
	}
}

/* 
 * deliver data from NIC to CC1111 : cr_wb_alloc, 
 * cr_wb_is_avail, cr_write_done, cr_start_wb
 * cr_write_start, cr_write_bulk:END
 */


/* wrap the operations into static functions:START */
static int cr_write_buffers_alloc(struct cr *cr)
{
	int i;
	struct cr_wb *wb;

	for (wb = &cr->wb[0], i = 0; i < CR_NW; i++, wb++) {
		wb->buf = usb_alloc_coherent(cr->dev, cr->writesize, GFP_KERNEL,
		    &wb->dmah);
		if (!wb->buf) {
			while (i != 0) {
				--i;
				--wb;
				usb_free_coherent(cr->dev, cr->writesize,
				    wb->buf, wb->dmah);
			}
			return -ENOMEM;
		}
	}
	return 0;
}

static void cr_write_buffers_free(struct cr *cr)
{
	int i;
	struct cr_wb *wb;
	struct usb_device *usb_dev = interface_to_usbdev(cr->control);

	for (wb = &cr->wb[0], i = 0; i < CR_NW; i++, wb++)
		usb_free_coherent(usb_dev, cr->writesize, wb->buf, wb->dmah);
}

static void cr_read_buffers_free(struct cr *cr)
{
	struct usb_device *usb_dev = interface_to_usbdev(cr->control);
	int i, n = cr->rx_buflimit;

	for (i = 0; i < n; i++)
		usb_free_coherent(usb_dev, cr->readsize,
					cr->rb[i].base, cr->rb[i].dma);
}
/* wrap the operations into static functions:END */

/* deliver data from CC1111 to NIC: cr_read_bulk, cr_rx_tasklet: START*/
static void cr_read_bulk(struct urb *urb)
{
	struct cr_rb *buf;
	struct cr_ru *rcv = urb->context;
	struct cr *cr = rcv->instance;
	int status = urb->status;

	dbg("Entering acm_read_bulk with status %d", status);

	if (!CR_READY(cr)) {
		dev_dbg(&cr->data->dev, "Aborting, cr not ready");
		return;
	}
	usb_mark_last_busy(cr->dev);

	if (status)
		dev_dbg(&cr->data->dev, "bulk rx status %d\n", status);

	buf = rcv->buffer;
	buf->size = urb->actual_length;

	if (likely(status == 0)) {
		spin_lock(&cr->read_lock);
		cr->processing++;

		// push the URB to available queue for further usage
		list_add_tail(&rcv->list, &cr->spare_read_urbs);

		// push the data buffer to waiting buffer for passing to upper layer
		list_add_tail(&buf->list, &cr->filled_read_bufs);

		spin_unlock(&cr->read_lock);
	} else {
		/* we drop the buffer due to an error */
		spin_lock(&cr->read_lock);

		// put the URB to available queue for further usage
		list_add_tail(&rcv->list, &cr->spare_read_urbs);

		// drop the data buffer for further usage
		list_add(&buf->list, &cr->spare_read_bufs);

		spin_unlock(&cr->read_lock);
		/* nevertheless the tasklet must be kicked unconditionally
		so the queue cannot dry up */
	}

	if (likely(!cr->susp_count))
		tasklet_schedule(&cr->urb_task);

}

static void cr_rx_tasklet(unsigned long _cr)
{
	struct net_priv *net_priv;
	/* [Tracing Note] should be a struct cr pointer instead of void pointer */
	struct cr *cr = (void *)_cr;
	struct cr_rb *buf;

	struct cr_ru *rcv;
	unsigned long flags;
	unsigned char throttled;
	unsigned long tmp_len;
	int buf_size;
	char* skb_data_mem;

	printk("%s:entered\n", __FUNCTION__);

	net_priv = netdev_priv(cr->netdev);

	dbg("Entering cr_rx_tasklet");

	if (!CR_READY(cr)) {
		dbg("cr_rx_tasklet: CR not ready");
		return;
	}

	spin_lock_irqsave(&cr->throttle_lock, flags);
	throttled = cr->throttle;
	spin_unlock_irqrestore(&cr->throttle_lock, flags);
	if (throttled) {
		dbg("cr_rx_tasklet: throttled");
		return;
	}

next_buffer:
	spin_lock_irqsave(&cr->read_lock, flags);
	if (list_empty(&cr->filled_read_bufs)) {
		// all data buffer are passed to upper layer
		spin_unlock_irqrestore(&cr->read_lock, flags);
		goto urbs;
	}

	//pop a data buffer from the FIFO queue
	buf = list_entry(cr->filled_read_bufs.next,
			 struct cr_rb, list);
	list_del(&buf->list);
	spin_unlock_irqrestore(&cr->read_lock, flags);

	dbg("cr_rx_tasklet: procesing buf 0x%p, size = %d", buf, buf->size);

	// check the device is throttle or not
	spin_lock_irqsave(&cr->throttle_lock, flags);
	throttled = cr->throttle;
	spin_unlock_irqrestore(&cr->throttle_lock, flags);

	if (!throttled)
	{		
		buf_size=buf->size;
		if(push_data_to_buffer(&net_priv->buf, buf->base, buf_size)==0)
		{
			printk("push_data_to_buffer() returns 0\n");

			// put back the data buffer to the FIFO queue
			spin_lock_irqsave(&cr->read_lock, flags);
			list_add(&buf->list, &cr->filled_read_bufs);
			spin_unlock_irqrestore(&cr->read_lock, flags);

			goto urbs;
		}

		while( (tmp_len=packet_length_if_ready(&net_priv->buf))!=0 )
		{
			printk("%s: tmp_len is %ld\n", __FUNCTION__, tmp_len);

			net_priv->rx_skb = netdev_alloc_skb(cr->netdev, tmp_len+2);
			if(!net_priv->rx_skb)
			{
				printk("alloc rx_skb fail\n");
			}
			else
			{
				printk("alloc rx_skb success\n");
				skb_reserve(net_priv->rx_skb, 2); //Align IP on 16 byte boundaries
				net_priv->rx_skb->dev = cr->netdev;
				net_priv->rx_skb->ip_summed = CHECKSUM_UNNECESSARY;  //don't check it

				skb_data_mem = skb_put(net_priv->rx_skb, tmp_len);

				if(pop_skb_from_buffer(skb_data_mem, &net_priv->buf)!=0)
				{
					net_priv->rx_skb->protocol = eth_type_trans(net_priv->rx_skb, cr->netdev);
					netif_rx(net_priv->rx_skb);
//					net_priv->rx_skb = NULL;
				}
				else
				{
					//TODO
				}
			}
		}
	}
	else
	{
		// put back the data buffer to the FIFO queue
		spin_lock_irqsave(&cr->read_lock, flags);
		list_add(&buf->list, &cr->filled_read_bufs);
		spin_unlock_irqrestore(&cr->read_lock, flags);

		return;
	}

	spin_lock_irqsave(&cr->read_lock, flags);
	// put the data buffer to available queue for further usage
	list_add(&buf->list, &cr->spare_read_bufs);
	spin_unlock_irqrestore(&cr->read_lock, flags);

	goto next_buffer;

urbs:

	// reuse all available buffer
	while (!list_empty(&cr->spare_read_bufs))
	{
		spin_lock_irqsave(&cr->read_lock, flags);
		if (list_empty(&cr->spare_read_urbs))
		{
			cr->processing = 0;
			spin_unlock_irqrestore(&cr->read_lock, flags);
			return;
		}

		// pop a available URB
		rcv = list_entry(cr->spare_read_urbs.next,
				 struct cr_ru, list);
		list_del(&rcv->list);
		spin_unlock_irqrestore(&cr->read_lock, flags);

		// pop a available data buffer
		buf = list_entry(cr->spare_read_bufs.next,
				 struct cr_rb, list);
		list_del(&buf->list);

		// match the URB and buffer
		rcv->buffer = buf;

		usb_fill_bulk_urb(rcv->urb, cr->dev,
				  cr->rx_endpoint,
				  buf->base,
				  cr->readsize,
				  cr_read_bulk, rcv);

		rcv->urb->transfer_dma = buf->dma;
		rcv->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

		/* This shouldn't kill the driver as unsuccessful URBs are
		   returned to the free-urbs-pool and resubmited ASAP */
		spin_lock_irqsave(&cr->read_lock, flags);
		if (cr->susp_count ||
				usb_submit_urb(rcv->urb, GFP_ATOMIC) < 0)
		{

			//If the URB submit process is failed, put the URB and data buffer to avai queue
			list_add(&buf->list, &cr->spare_read_bufs);
			list_add(&rcv->list, &cr->spare_read_urbs);
			cr->processing = 0;
			spin_unlock_irqrestore(&cr->read_lock, flags);
			return;
		}
		else
		{
			spin_unlock_irqrestore(&cr->read_lock, flags);
			dbg("cr_rx_tasklet: sending urb 0x%p, rcv 0x%p, buf 0x%p", rcv->urb, rcv, buf);
		}
	}

	// the reading process is done
	spin_lock_irqsave(&cr->read_lock, flags);
	cr->processing = 0;
	spin_unlock_irqrestore(&cr->read_lock, flags);
}
/* deliver data from CC1111 to NIC: cr_read_bulk, cr_rx_tasklet: END*/

static void reset_buffer(struct myRXBuf *buf)
{
	buf->head=0;
	buf->tail=0;
	buf->length=0;

	buf->is_writing=0;
	buf->is_reading=0;
	buf->is_checking=0;
	buf->is_ready=0;

	buf->is_accessing=0;

	return;
}

static unsigned long pop_skb_from_buffer(void* dst, struct myRXBuf *buf)
{
	unsigned long readCount;
	unsigned long len;
	unsigned long flags;

	if(packet_length_if_ready(buf)!=0)
	{
		len = ((buf->data[(buf->head+1)%myRXBufSize])<<8) + (buf->data[(buf->head+2)%myRXBufSize]);
		readCount = ((buf->head+len+2)>=myRXBufSize)?((myRXBufSize-1)-(buf->head+3)+1):(len);

		if(readCount==len)
		{
			memcpy(dst, &(buf->data[(buf->head+3)%myRXBufSize]), len);
		}
		else
		{
			memcpy(dst, &(buf->data[(buf->head+3)%myRXBufSize]), readCount);
			memcpy(dst+readCount, &(buf->data[0]), len-readCount);
		}

		buf->head = (buf->head+3+len)%myRXBufSize;
		buf->length -= (3+len);

		buf->is_ready=0;
	}
	else
	{
		len=0;
	}

	printk("%s:buf head=%ld, tail=%ld, length=%ld, return len=%ld\n"
		, __FUNCTION__, buf->head, buf->tail, buf->length, len);

	return len;
}

static unsigned long push_data_to_buffer(struct myRXBuf *buf, void* src, unsigned long len)
{
	unsigned long writeCount;

	if( (myRXBufSize-buf->length) >= len )
	{
		writeCount=((buf->tail+len-1)>=myRXBufSize)?((myRXBufSize-1)-(buf->tail)+1):(len);
		if(writeCount==len)
		{
			memcpy( &(buf->data[buf->tail]), src, len);
		}
		else
		{
			memcpy( &(buf->data[buf->tail]), src, writeCount);
			memcpy( &(buf->data[0]), src+writeCount, len-writeCount);
		}

		buf->tail = (buf->tail+len)%myRXBufSize;
		buf->length += len;


	}
	else
	{
		printk("The buffer is full\n");
		len=0;
	}

	printk("%s:buf head=%ld, tail=%ld, length=%ld, return len=%ld\n"
		, __FUNCTION__, buf->head, buf->tail, buf->length, len);
	return len;
}

static unsigned long packet_length_if_ready(struct myRXBuf *buf)
{
	unsigned long len;
	unsigned long flags;

	if(buf->is_ready!=0)
	{
		len = buf->packet_length;
		return len;
	}

packet_check_start:

	while((buf->data[buf->head])!='@')
	{
		if(buf->length<=3)
		{
			printk("packet is too little to check \n");
			printk("%s:head=%ld, tail=%ld, length=%ld, return len=%ld\n"
				, __FUNCTION__, buf->head, buf->tail, buf->length, (unsigned long)0);
			return 0;
		}

		buf->head = (buf->head+1)%myRXBufSize;
		buf->length--;
	}

	len = ((buf->data[(buf->head+1)%myRXBufSize])<<8)+((buf->data[(buf->head+2)%myRXBufSize])&0x0ff);
	if(buf->length>=len+3)
	{
		if( ((buf->data[(buf->head+2+len+1)%myRXBufSize])=='@')
			||((buf->head+3+len)%myRXBufSize == buf->tail) )
		{
			buf->is_ready=1;
			buf->packet_length=len;
			printk("%s:A packet with length=%ld is ready.\n"
				, __FUNCTION__, len);
			return len;
		}
		else
		{
			buf->head = (buf->head+1)%myRXBufSize;
			buf->length--;

			goto packet_check_start;
		}
	}
	else
	{
		printk("%s:head=%ld, tail=%ld, length=%ld, return len=%ld\n"
			, __FUNCTION__, buf->head, buf->tail, buf->length, (unsigned long)0);
		printk("%s:A packet with length=%ld is expected.\n"
			, __FUNCTION__, len);
		return 0;
	}
}

static int CR_usb_init(struct usb_interface *intf, const struct usb_device_id *id)
{
	int i;
	struct cr *cr;
	struct net_priv *net_priv;

	//interface
	struct usb_interface *control_interface;
	struct usb_interface *data_interface;
	int call_interface_num = -1;
	int data_interface_num;

	//endpoint
	struct usb_endpoint_descriptor *epctrl = NULL;
	struct usb_endpoint_descriptor *epread = NULL;
	struct usb_endpoint_descriptor *epwrite = NULL;

	struct usb_cdc_union_desc *union_header = NULL;
	struct usb_cdc_country_functional_desc *cfd = NULL;
	struct usb_device *usb_dev = interface_to_usbdev(intf);
	unsigned char *buffer = intf->altsetting->extra;
	int buflen = intf->altsetting->extralen;

	//function
	u8 ac_management_function = 0;
	u8 call_management_function = 0;
	int ctrlsize, readsize;
	int combined_interfaces = 0;
	int num_rx_buf;

	struct net_device *CR_netdev;

	printk("%s:entered\n", __FUNCTION__);
	printk("%s:Address of Interface: %p\n", __FUNCTION__, intf);

	num_rx_buf = CR_NR;


	if (!buffer)
	{
		dev_err(&intf->dev, "Weird descriptor references\n");
		return -EINVAL;
	}

	printk("%s: buflen:%d\n", __FUNCTION__, buflen);
	if (!buflen)
	{
		if (intf->cur_altsetting->endpoint &&
				intf->cur_altsetting->endpoint->extralen &&
				intf->cur_altsetting->endpoint->extra)
		{
			dev_err(&intf->dev,
				"Seeking extra descriptors on endpoint\n");
			buflen = intf->cur_altsetting->endpoint->extralen;
			buffer = intf->cur_altsetting->endpoint->extra;
		}
		else
		{
			dev_err(&intf->dev,
				"Zero length descriptor references\n");
			return -EINVAL;
		}
	}

//Probing setting
	while (buflen > 0)
	{
		if (buffer[1] != USB_DT_CS_INTERFACE)
		{
			dev_err(&intf->dev, "skipping garbage\n");
			goto next_desc;
		}

		switch (buffer[2]) {
		case USB_CDC_UNION_TYPE: /* we've found it */
			printk("%s: USB_CDC_UNION_TYPE\n", __FUNCTION__);
			if (union_header)
			{
				dev_err(&intf->dev, "More than one "
					"union descriptor, skipping ...\n");
				goto next_desc;
			}
			union_header = (struct usb_cdc_union_desc *)buffer;
			break;
		case USB_CDC_COUNTRY_TYPE: /* export through sysfs*/
			printk("%s: USB_CDC_COUNTRY_TYPE\n", __FUNCTION__);
			cfd = (struct usb_cdc_country_functional_desc *)buffer;
			break;
		case USB_CDC_HEADER_TYPE: /* maybe check version */
			printk("%s: USB_CDC_HEADER_TYPE\n", __FUNCTION__);
			break; /* for now we ignore it */
		case USB_CDC_ACM_TYPE:
			ac_management_function = buffer[3];
			printk("%s: USB_CDC_ACM_TYPE: ac_management_function = %d\n"
							, __FUNCTION__, ac_management_function);
			break;
		case USB_CDC_CALL_MANAGEMENT_TYPE:
			call_management_function = buffer[3];
			call_interface_num = buffer[4];
			printk("%s: USB_CDC_CALL_MANAGEMENT_TYPE: call_management_function = %d\n"
							, __FUNCTION__,  call_management_function);
			printk("%s: USB_CDC_CALL_MANAGEMENT_TYPE: call_interface_num = %d\n"
							, __FUNCTION__, call_interface_num);

			if ((call_management_function & 3) != 3)
				dev_err(&intf->dev, "This device cannot do calls on its own. It is not a modem.\n");
			break;
		default:
			/* there are LOTS more CDC descriptors that
			 * could legitimately be found here.
			 */
			dev_dbg(&intf->dev, "Ignoring descriptor: "
					"type %02x, length %d\n",
					buffer[2], buffer[0]);
			break;
		}
next_desc:
		buflen -= buffer[0];
		buffer += buffer[0];
	}

	control_interface = usb_ifnum_to_if(usb_dev, union_header->bMasterInterface0);
	data_interface = usb_ifnum_to_if(usb_dev, (data_interface_num = union_header->bSlaveInterface0));
	if (!control_interface || !data_interface) {
		dev_dbg(&intf->dev, "no interfaces\n");
		return -ENODEV;
	}

	if (!union_header) {
		printk("%s: NO UNION HEADER\n", __FUNCTION__);
	}

	if (!combined_interfaces && intf != control_interface)
		return -ENODEV;

	if (!combined_interfaces && usb_interface_claimed(data_interface)) {
		/* valid in this context */
		dev_dbg(&intf->dev, "The data interface isn't available\n");
		return -EBUSY;
	}

	if (data_interface->cur_altsetting->desc.bNumEndpoints < 2)
		return -EINVAL;

	epctrl = &control_interface->cur_altsetting->endpoint[0].desc;
	epread = &data_interface->cur_altsetting->endpoint[0].desc;
	epwrite = &data_interface->cur_altsetting->endpoint[1].desc;

	printk("%s: Data Interface Num = %d\n", __FUNCTION__, data_interface_num);
	printk("%s: Call Interface Num = %d\n", __FUNCTION__, call_interface_num);
	printk("%s: Address of Control Interface: %p\n", __FUNCTION__, control_interface);
	printk("%s: Address of Data Interface: %p\n", __FUNCTION__, data_interface);

	if (!usb_endpoint_dir_in(epread)) {
		printk("!usb_endpoint_dir_in(epread)\n");
	}

	cr = kzalloc(sizeof(struct cr), GFP_KERNEL); // = kmalloc+memset
	if (cr == NULL) {
		dev_dbg(&intf->dev, "out of memory (cr kzalloc)\n");
		goto alloc_fail;
	}

//initialize cr structure
	ctrlsize = le16_to_cpu(epctrl->wMaxPacketSize);
	readsize = le16_to_cpu(epread->wMaxPacketSize);
	cr->combined_interfaces = combined_interfaces;
	cr->control = control_interface;
	cr->data = data_interface;
	cr->dev = usb_dev;
	cr->ctrl_caps = ac_management_function;
	cr->ctrlsize = ctrlsize;
	cr->readsize = readsize;
	cr->writesize = 32;
	cr->rx_buflimit = num_rx_buf;

	printk("%s: readsize:%d\n", __FUNCTION__, cr->readsize);
	printk("%s: writesize:%d\n", __FUNCTION__, cr->writesize);

//ISR, work
	cr->urb_task.func = cr_rx_tasklet;
	cr->urb_task.data = (unsigned long)cr;
	init_waitqueue_head(&cr->drain_wait);

//spinlock, mutex
	spin_lock_init(&cr->throttle_lock);
	spin_lock_init(&cr->write_lock);
	spin_lock_init(&cr->read_lock);
	mutex_init(&cr->mutex);
	
//IN endpoint
	cr->rx_endpoint = usb_rcvbulkpipe(usb_dev, epread->bEndpointAddress);
	cr->is_int_ep = usb_endpoint_xfer_int(epread);

	if (cr->is_int_ep){
		cr->bInterval = epread->bInterval;
		printk("%s: IN Endpoint is of interrupt transfer type.\n", __FUNCTION__);
	}
	else{
		printk("%s: IN Endpoint is not of interrupt transfer type.\n", __FUNCTION__);
	}

//allocate IN buffer
	for (i = 0; i < num_rx_buf; i++) {
		struct cr_rb *rb = &(cr->rb[i]);

		rb->base = usb_alloc_coherent(cr->dev, readsize,
				GFP_KERNEL, &rb->dma);
		if (!rb->base) {
			dev_dbg(&intf->dev,
				"out of memory (read bufs usb_alloc_coherent)\n");
			goto alloc_fail7;
		}
	}

//allocate IN URB
	for (i = 0; i < num_rx_buf; i++) {
		struct cr_ru *rcv = &(cr->ru[i]);

		rcv->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (rcv->urb == NULL) {
			dev_dbg(&intf->dev,
				"out of memory (read urbs usb_alloc_urb)\n");
			goto alloc_fail7;
		}

		rcv->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		rcv->instance = cr;
	}

//OUT endpoint

	if (usb_endpoint_xfer_int(epwrite)){
		printk("%s: OUT Endpoint type : interrupt\n", __FUNCTION__);
	}
	else{
		printk("%s: OUT Endpoint type : bulk\n", __FUNCTION__);
	}

//allocate OUT buffer
	if (cr_write_buffers_alloc(cr) < 0) {
		dev_dbg(&intf->dev, "out of memory (write buffer alloc)\n");
		goto alloc_fail4;
	}

//allocate OUT URB
	for (i = 0; i < CR_NW; i++) {
		struct cr_wb *snd = &(cr->wb[i]);

		snd->urb = usb_alloc_urb(0, GFP_KERNEL);
		if (snd->urb == NULL) {
			dev_dbg(&intf->dev,
				"out of memory (write urbs usb_alloc_urb)");
			goto alloc_fail7;
		}

		if (usb_endpoint_xfer_int(epwrite)){
			usb_fill_int_urb(snd->urb, usb_dev,
				usb_sndbulkpipe(usb_dev, epwrite->bEndpointAddress),
				NULL, cr->writesize, cr_write_bulk, snd, epwrite->bInterval);
		}
		else{			
			usb_fill_bulk_urb(snd->urb, usb_dev,
				usb_sndbulkpipe(usb_dev, epwrite->bEndpointAddress),
				NULL, cr->writesize, cr_write_bulk, snd);
		}

		snd->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		snd->instance = cr;
	}

//allocate CTL URB
	cr->ctrlurb = usb_alloc_urb(0, GFP_KERNEL);
	if (!cr->ctrlurb) {
		dev_dbg(&intf->dev, "out of memory (ctrlurb kmalloc)\n");
		goto alloc_fail5;
	}
	usb_fill_int_urb(cr->ctrlurb, usb_dev,
			 usb_rcvintpipe(usb_dev, epctrl->bEndpointAddress),
			 cr->ctrl_buffer, ctrlsize, cr_ctrl_irq, cr,
			 /* works around buggy devices */
			 epctrl->bInterval ? epctrl->bInterval : 0xff);
	cr->ctrlurb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	cr->ctrlurb->transfer_dma = cr->ctrl_dma;


	usb_set_intfdata(intf, cr);

	//TODO
	usb_driver_claim_interface(&CR_usb_driver, data_interface, cr);
	usb_set_intfdata(data_interface, cr);
	usb_get_intf(control_interface);

	/* Allocate net_dev
	 * [Tracing Note]
	 * using sizeof(&cr) but dereference with net_priv
	 * which is pretty weird.
	 */
	CR_netdev = alloc_etherdev(sizeof(&cr));
	if(!CR_netdev) 
		goto error;
	if(dev_alloc_name(CR_netdev, ifname)<0){
		printk("dev_alloc_name() failed\n");
		goto error;
	}

	/* Basic Init of the net device, such as
	 * netdev_ops, device name and so on.
	 */
	CR_netdev->netdev_ops = &CR_netdev_ops;
	CR_netdev->watchdog_timeo = HZ*3; /* 3 second timeout */
	CR_set_mac_address(CR_netdev, "0EDCBA");
	cr->netdev = CR_netdev;

	/* [Tracing Note] 
	 * deference with type struct net_priv, this one confuse me QAQ 
	 */
	net_priv = netdev_priv(CR_netdev);
	net_priv->cr = cr;
	net_priv->tx_packet=0;
	net_priv->tx_byte=0;

	reset_buffer(&net_priv->buf);
	/* register the net device */
	if (register_netdev(CR_netdev) != 0) {
		printk("register_netdev() failed\n");
		goto error;
	}

	return 0;

error:
full_bailout:
	usb_kill_urb(cr->ctrlurb);

alloc_fail8:
	for (i = 0; i < CR_NW; i++)
		usb_free_urb(cr->wb[i].urb);

alloc_fail7:
	cr_read_buffers_free(cr);
	for (i = 0; i < num_rx_buf; i++)
		usb_free_urb(cr->ru[i].urb);
	usb_free_urb(cr->ctrlurb);
alloc_fail5:
	cr_write_buffers_free(cr);

alloc_fail4:
	usb_free_coherent(usb_dev, ctrlsize, cr->ctrl_buffer, cr->ctrl_dma);

alloc_fail:
	return -ENODEV;	

}

static void CR_usb_disconnect(struct usb_interface *intf, pm_message_t message)
{
	struct usb_device *usb_dev = interface_to_usbdev(intf);
	struct cr *cr = usb_get_intfdata(intf);

	printk("%s:entered\n", __FUNCTION__);
	printk("%s:Address of Interface: %p\n", __FUNCTION__, intf);
	//TODO

	/* sibling interface is already cleaning up */
	if (!cr){
		printk("%s: the sibling interface is already cleaning up\n", __FUNCTION__);
		return;
	}
	else{
		//TODO fake close START

		//TODO fake close END
	}

	cr->dev = NULL;
	usb_set_intfdata(cr->control, NULL);
	usb_set_intfdata(cr->data, NULL);

	cr_write_buffers_free(cr);
	usb_free_coherent(usb_dev, cr->ctrlsize, cr->ctrl_buffer, cr->ctrl_dma);
	cr_read_buffers_free(cr);

	usb_driver_release_interface(&CR_usb_driver, intf == cr->control ?
					cr->data : cr->control);
	if(cr->netdev)
		unregister_netdev(cr->netdev);

	//TODO
}

static int CR_usb_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct net_device *pnetdev=usb_get_intfdata(intf);
	struct usb_device *usb_dev = interface_to_usbdev(intf);

	printk("%s:entered\n", __FUNCTION__);
	//TODO
}

static int CR_usb_resume(struct usb_interface *intf)
{
	struct net_device *pnetdev=usb_get_intfdata(intf);

	printk("%s:entered\n", __FUNCTION__);
	//TODO
}

//From TI firmware
/**
 * USB_INTERFACE_INFO - macro used to describe a class of usb interfaces
 * @cl: bInterfaceClass value
 * @sc: bInterfaceSubClass value
 * @pr: bInterfaceProtocol value
 *
 * This macro is used to create a struct usb_device_id that matches a
 * specific class of interfaces.
 */

/*
                DB deviceDescEnd - deviceDesc
                DB DESC_TYPE_DEVICE ; bDescriptorType
                DB 00H, 02H         ; bcdUSB
                DB CDC_DEVICE       ; bDeviceClass
                DB 00H              ; bDeviceSubClass
                DB 00H              ; bDeviceProtocol
                DB EP0_PACKET_SIZE
                DB 51H, 04H         ; idVendor Texas Instruments
                #if (chip==2531)
                DB 0A8H, 16H        ; idProduct CC2531
                #elif (chip==2511)
                DB 0A4H, 16H        ; idProduct CC2511
                #else
                DB 0A6H, 16H        ; idProduct CC1111
                #endif
                DB 09H, 00H         ; bcdDevice
                DB 01H              ; iManufacturer
                DB 02H              ; iProduct
                DB 03H              ; iSerialNumber
                DB 01H              ; bNumConfigurations
*/

/**
 * USB_INTERFACE_INFO - macro used to describe a class of usb interfaces
 * @cl: bInterfaceClass value 
 * @sc: bInterfaceSubClass value
 * @pr: bInterfaceProtocol value
 * 
 */

static struct usb_device_id product_ids[] = {
	{ USB_INTERFACE_INFO(USB_CLASS_COMM, USB_CDC_SUBCLASS_ACM,
		USB_CDC_ACM_PROTO_AT_V25TER) },
	{ }
};
MODULE_DEVICE_TABLE(usb, product_ids);

static struct usb_driver CR_usb_driver = {
	.name = (char*)DRV_NAME,
	.id_table = product_ids,
	.probe = CR_usb_init,
	.disconnect = CR_usb_disconnect,
//	.suspend = CR_usb_suspend,
//	.resume = CR_usb_resume,
//	.supports_autosuspend = 1,
};

static int __init CR_init(void)
{
	printk("%s:entered\n", __FUNCTION__);

	return usb_register(&CR_usb_driver);
}

static void __exit CR_exit(void)
{
	printk("%s:entered\n", __FUNCTION__);

	usb_deregister(&CR_usb_driver);
}

module_init(CR_init);
module_exit(CR_exit);
MODULE_LICENSE("GPL");
