//definition
#define DRV_NAME "CR NIC"
#define CR_NW  64
#define CR_NR  64

/*
 * Requests.
 */
#define USB_RT_ACM              (USB_TYPE_CLASS | USB_RECIP_INTERFACE)

/*
* Output control lines.
*/
#define ACM_CTRL_DTR            0x01
#define ACM_CTRL_RTS            0x02

/*
 * Input control lines and line errors.
 */
#define ACM_CTRL_DCD            0x01
#define ACM_CTRL_DSR            0x02
#define ACM_CTRL_BRK            0x04
#define ACM_CTRL_RI             0x08

#define ACM_CTRL_FRAMING        0x10
#define ACM_CTRL_PARITY         0x20
#define ACM_CTRL_OVERRUN        0x40

struct cr_wb {
	unsigned char *buf;
	dma_addr_t dmah;
	int len;
	int use;
	struct urb		*urb;
	struct cr		*instance;
};

struct cr_rb {
	struct list_head	list;
	int			size;
	unsigned char		*base;
	dma_addr_t		dma;
};

struct cr_ru {
	struct list_head	list;
	struct cr_rb		*buffer;
	struct urb		*urb;
	struct cr		*instance;
};

//#define myRXBufSize 0x10000
#define myRXBufSize 0x1000
struct myRXBuf
{
	char data[myRXBufSize*2];
	unsigned long head;
	unsigned long tail;
	unsigned long length;

	char is_writing;
	char is_reading;
	char is_checking;
	char is_ready;

	spinlock_t writing_lock;
	spinlock_t reading_lock;
	spinlock_t checking_lock;

	char is_accessing;
	spinlock_t access_lock;

	unsigned long packet_length;

};

struct cr{

	struct net_device *netdev;
	struct usb_device *dev;				/* the corresponding usb device */
	struct usb_interface *control;			/* control interface */
	struct usb_interface *data;			/* data interface */
	struct urb *ctrlurb;				/* urbs */
	u8 *ctrl_buffer;				/* buffers of urbs */ //??
	dma_addr_t ctrl_dma;				/* dma handles of buffers */ //??
	struct cr_wb wb[CR_NW];
	struct cr_ru ru[CR_NR];
	struct cr_rb rb[CR_NR];
	int rx_buflimit;
	int rx_endpoint;
	spinlock_t read_lock;
	struct list_head spare_read_urbs;
	struct list_head spare_read_bufs;
	struct list_head filled_read_bufs;
	int write_used;					/* number of non-empty write buffers */
	int processing;					//??
	int transmitting;				//??
	spinlock_t write_lock;
	struct mutex mutex;
	struct work_struct work;			/* work queue entry for line discipline waking up */ //??
	wait_queue_head_t drain_wait;			/* close processing */
	struct tasklet_struct urb_task;                 /* rx processing */
	spinlock_t throttle_lock;			/* synchronize throtteling and read callback */
	unsigned int ctrlin;				/* input control lines (DCD, DSR, RI, break, overruns) */
	unsigned int ctrlout;				/* output control lines (DTR, RTS) */
	unsigned int writesize;				/* max packet size for the output bulk endpoint */
	unsigned int readsize,ctrlsize;			/* buffer sizes for freeing */
	unsigned int minor;				/* acm minor number */
	unsigned char throttle;				/* throttled by tty layer */
	unsigned char clocal;				/* termios CLOCAL */
	unsigned int ctrl_caps;				/* control capabilities from the class specific header */
	unsigned int susp_count;			/* number of suspended interfaces */
	int combined_interfaces:1;			/* control and data collapsed */
	int is_int_ep:1;				/* interrupt endpoints contrary to spec used */
	u8 bInterval;
	struct cr_wb *delayed_wb;			/* write queued for a device about to be woken */
};


struct net_priv
{
	struct cr *cr;
	spinlock_t tx_lock;
//	spinlock_t rx_lock;
	spinlock_t stat_lock;
	int tx_packet;
	int tx_byte;
	struct sk_buff *rx_skb;
	struct myRXBuf buf;
};
