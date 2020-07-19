/*
 * f_kc_serial.c - generic USB serial function driver
 *
 * Copyright (C) 2003 Al Borchers (alborchers@steinerpoint.com)
 * Copyright (C) 2008 by David Brownell
 * Copyright (C) 2008 by Nokia Corporation
 *
 * This software is distributed under the terms of the GNU General
 * Public License ("GPL") as published by the Free Software Foundation,
 * either version 2 of that License or (at your option) any later version.
 */

/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include "usb_gadget_xport.h"

#include "u_serial.h"
#include "gadget_chips.h"


/*
 * This function packages a simple "generic serial" port with no real
 * control mechanisms, just raw data transfer over two bulk endpoints.
 *
 * Because it's not standardized, this isn't as interoperable as the
 * CDC ACM driver.  However, for many purposes it's just as functional
 * if you can arrange appropriate host side drivers.
 */
#define KC_GSERIAL_NO_PORTS 3

struct kc_f_gser {
	struct gserial			port;
	u8				ctrl_id,data_id;
	u8				port_num;

	u8				online;
	enum transport_type		transport;

	u8				pending;
	spinlock_t			lock;
	struct usb_ep			*notify;
	struct usb_request		*notify_req;

	struct usb_cdc_line_coding	port_line_coding;

	/* SetControlLineState request */
	u16				port_handshake_bits;
#define KC_ACM_CTRL_RTS	(1 << 1)	/* unused with full duplex */
#define KC_ACM_CTRL_DTR	(1 << 0)	/* host is ready for data r/w */

	/* SerialState notification */
	u16				serial_state;
#define KC_ACM_CTRL_OVERRUN	(1 << 6)
#define KC_ACM_CTRL_PARITY	(1 << 5)
#define KC_ACM_CTRL_FRAMING	(1 << 4)
#define KC_ACM_CTRL_RI		(1 << 3)
#define KC_ACM_CTRL_BRK		(1 << 2)
#define KC_ACM_CTRL_DSR		(1 << 1)
#define KC_ACM_CTRL_DCD		(1 << 0)
};

static unsigned int kc_no_tty_ports;
static unsigned int kc_no_smd_ports;
static unsigned int kc_no_hsic_sports;
static unsigned int kc_no_hsuart_sports;
static unsigned int kc_nr_ports;

static struct kc_port_info {
	enum transport_type	transport;
	unsigned		port_num;
	unsigned char		client_port_num;
} kc_gserial_ports[KC_GSERIAL_NO_PORTS];

static inline struct kc_f_gser *kc_func_to_gser(struct usb_function *f)
{
	return container_of(f, struct kc_f_gser, port.func);
}

static inline struct kc_f_gser *kc_port_to_gser(struct gserial *p)
{
	return container_of(p, struct kc_f_gser, port);
}

/*-------------------------------------------------------------------------*/

/* interface descriptor: */

static struct usb_interface_descriptor kc_gser_control_interface_desc = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	/* .bInterfaceNumber = DYNAMIC */
	.bNumEndpoints =	1,
	.bInterfaceClass =	USB_CLASS_COMM,
	.bInterfaceSubClass =	USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol =	USB_CDC_ACM_PROTO_AT_V25TER,
	/* .iInterface = DYNAMIC */
};

static struct usb_interface_descriptor kc_gser_data_interface_desc = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	/* .bInterfaceNumber = DYNAMIC */
	.bNumEndpoints =	2,
	.bInterfaceClass =	USB_CLASS_CDC_DATA,
	.bInterfaceSubClass =	0,
	.bInterfaceProtocol =	0,
	.iInterface = 0,
};

static struct usb_cdc_header_desc kc_gser_header_desc = {
	.bLength =		sizeof(kc_gser_header_desc),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_HEADER_TYPE,
	.bcdCDC =		cpu_to_le16(0x0110),
};

static struct usb_cdc_call_mgmt_descriptor
kc_gser_call_mgmt_descriptor = {
	.bLength =		sizeof(kc_gser_call_mgmt_descriptor),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_CALL_MANAGEMENT_TYPE,
	.bmCapabilities =	USB_CDC_COMM_FEATURE|USB_CDC_CAP_LINE,
	/* .bDataInterface = DYNAMIC */
};

static struct usb_cdc_acm_descriptor kc_gser_descriptor = {
	.bLength =		sizeof(kc_gser_descriptor),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_ACM_TYPE,
	.bmCapabilities =	USB_CDC_CAP_LINE,
};

static struct usb_cdc_union_desc kc_gser_union_desc = {
	.bLength =		sizeof(kc_gser_union_desc),
	.bDescriptorType =	USB_DT_CS_INTERFACE,
	.bDescriptorSubType =	USB_CDC_UNION_TYPE,
	/* .bMasterInterface0 =	DYNAMIC */
	/* .bSlaveInterface0 =	DYNAMIC */
};

/* full speed support: */

static struct usb_endpoint_descriptor kc_gser_fs_notify_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	0x0040,
	.bInterval =		0x10,
};

static struct usb_endpoint_descriptor kc_gser_fs_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.bInterval =		0x20,
};

static struct usb_endpoint_descriptor kc_gser_fs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.bInterval =		0x20,
};

static struct usb_descriptor_header *kc_gser_fs_function[] = {
	(struct usb_descriptor_header *) &kc_gser_control_interface_desc,
	(struct usb_descriptor_header *) &kc_gser_header_desc,
	(struct usb_descriptor_header *) &kc_gser_call_mgmt_descriptor,
	(struct usb_descriptor_header *) &kc_gser_descriptor,
	(struct usb_descriptor_header *) &kc_gser_union_desc,
	(struct usb_descriptor_header *) &kc_gser_fs_notify_desc,
	(struct usb_descriptor_header *) &kc_gser_data_interface_desc,
	(struct usb_descriptor_header *) &kc_gser_fs_in_desc,
	(struct usb_descriptor_header *) &kc_gser_fs_out_desc,
	NULL,
};

/* high speed support: */

static struct usb_endpoint_descriptor kc_gser_hs_notify_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =	0x0040,
	.bInterval =		0x10,
};

static struct usb_endpoint_descriptor kc_gser_hs_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
	.bInterval =		0x20,
};

static struct usb_endpoint_descriptor kc_gser_hs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(512),
	.bInterval =		0x20,
};

static struct usb_descriptor_header *kc_gser_hs_function[] = {
	(struct usb_descriptor_header *) &kc_gser_control_interface_desc,
	(struct usb_descriptor_header *) &kc_gser_header_desc,
	(struct usb_descriptor_header *) &kc_gser_call_mgmt_descriptor,
	(struct usb_descriptor_header *) &kc_gser_descriptor,
	(struct usb_descriptor_header *) &kc_gser_union_desc,
	(struct usb_descriptor_header *) &kc_gser_hs_notify_desc,
	(struct usb_descriptor_header *) &kc_gser_data_interface_desc,
	(struct usb_descriptor_header *) &kc_gser_hs_in_desc,
	(struct usb_descriptor_header *) &kc_gser_hs_out_desc,
	NULL,
};


static int kc_gport_setup(struct usb_configuration *c)
{
	int ret = 0;
	int port_idx;
	int i;

	pr_debug("%s: kc_no_tty_ports: %u"
		" kc_no_smd_ports: %u kc_no_hsic_sports: %u no_hsuart_ports: %u kc_nr_ports: %u\n",
			__func__, kc_no_tty_ports, kc_no_smd_ports,
			kc_no_hsic_sports, kc_no_hsuart_sports, kc_nr_ports);

	if (kc_no_tty_ports) {
		for (i = 0; i < kc_no_tty_ports; i++) {
			ret = gserial_alloc_line(&kc_gserial_ports[i].client_port_num);
			if (ret)
				return ret;
		}
	}

	if (kc_no_smd_ports)
		ret = gsmd_setup(c->cdev->gadget, kc_no_smd_ports);
	if (kc_no_hsic_sports) {
		port_idx = ghsic_data_setup(kc_no_hsic_sports, USB_GADGET_SERIAL);
		if (port_idx < 0)
			return port_idx;

		for (i = 0; i < kc_nr_ports; i++) {
			if (kc_gserial_ports[i].transport ==
					USB_GADGET_XPORT_HSIC) {
				kc_gserial_ports[i].client_port_num = port_idx;
				port_idx++;
			}
		}

		/*clinet port num is same for data setup and ctrl setup*/
		ret = ghsic_ctrl_setup(kc_no_hsic_sports, USB_GADGET_SERIAL);
		if (ret < 0)
			return ret;
		return 0;
	}
	if (kc_no_hsuart_sports) {
		port_idx = ghsuart_data_setup(kc_no_hsuart_sports,
					USB_GADGET_SERIAL);
		if (port_idx < 0)
			return port_idx;

		for (i = 0; i < kc_nr_ports; i++) {
			if (kc_gserial_ports[i].transport ==
					USB_GADGET_XPORT_HSUART) {
				kc_gserial_ports[i].client_port_num = port_idx;
				port_idx++;
			}
		}

		return 0;
	}
	return ret;
}

static int kc_gport_connect(struct kc_f_gser *gser)
{
	unsigned	port_num;
	int		ret;

	pr_debug("%s: transport: %s kc_f_gser: %p gserial: %p port_num: %d\n",
			__func__, xport_to_str(gser->transport),
			gser, &gser->port, gser->port_num);

	port_num = kc_gserial_ports[gser->port_num].client_port_num;

	switch (gser->transport) {
	case USB_GADGET_XPORT_TTY:
		gserial_connect(&gser->port, port_num);
		break;
	case USB_GADGET_XPORT_SMD:
		gsmd_connect(&gser->port, port_num);
		break;
	case USB_GADGET_XPORT_HSIC:
		ret = ghsic_ctrl_connect(&gser->port, port_num);
		if (ret) {
			pr_err("%s: ghsic_ctrl_connect failed: err:%d\n",
					__func__, ret);
			return ret;
		}
		ret = ghsic_data_connect(&gser->port, port_num);
		if (ret) {
			pr_err("%s: ghsic_data_connect failed: err:%d\n",
					__func__, ret);
			ghsic_ctrl_disconnect(&gser->port, port_num);
			return ret;
		}
		break;
	case USB_GADGET_XPORT_HSUART:
		ret = ghsuart_data_connect(&gser->port, port_num);
		if (ret) {
			pr_err("%s: ghsuart_data_connect failed: err:%d\n",
					__func__, ret);
			return ret;
		}
		break;
	default:
		pr_err("%s: Un-supported transport: %s\n", __func__,
				xport_to_str(gser->transport));
		return -ENODEV;
	}

	return 0;
}

static int kc_gport_disconnect(struct kc_f_gser *gser)
{
	unsigned port_num;

	pr_debug("%s: transport: %s kc_f_gser: %p gserial: %p port_num: %d\n",
			__func__, xport_to_str(gser->transport),
			gser, &gser->port, gser->port_num);

	port_num = kc_gserial_ports[gser->port_num].client_port_num;

	switch (gser->transport) {
	case USB_GADGET_XPORT_TTY:
		gserial_disconnect(&gser->port);
		break;
	case USB_GADGET_XPORT_SMD:
		gsmd_disconnect(&gser->port, port_num);
		break;
	case USB_GADGET_XPORT_HSIC:
		ghsic_ctrl_disconnect(&gser->port, port_num);
		ghsic_data_disconnect(&gser->port, port_num);
		break;
	case USB_GADGET_XPORT_HSUART:
		ghsuart_data_disconnect(&gser->port, port_num);
		break;
	default:
		pr_err("%s: Un-supported transport:%s\n", __func__,
				xport_to_str(gser->transport));
		return -ENODEV;
	}

	return 0;
}

static void kc_gser_complete_set_line_coding(struct usb_ep *ep,
		struct usb_request *req)
{
	struct kc_f_gser            *gser = ep->driver_data;
	struct usb_composite_dev *cdev = gser->port.func.config->cdev;

	if (req->status != 0) {
		DBG(cdev, "gser ttyGS%d completion, err %d\n",
				gser->port_num, req->status);
		return;
	}

	/* normal completion */
	if (req->actual != sizeof(gser->port_line_coding)) {
		DBG(cdev, "gser ttyGS%d short resp, len %d\n",
				gser->port_num, req->actual);
		usb_ep_set_halt(ep);
	} else {
		struct usb_cdc_line_coding	*value = req->buf;
		gser->port_line_coding = *value;
	}
}
/*-------------------------------------------------------------------------*/

static int
kc_gser_setup(struct usb_function *f, const struct usb_ctrlrequest *ctrl)
{
	struct kc_f_gser            *gser = kc_func_to_gser(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	struct usb_request	 *req = cdev->req;
	int			 value = -EOPNOTSUPP;
	u16			 w_index = le16_to_cpu(ctrl->wIndex);
	u16			 w_value = le16_to_cpu(ctrl->wValue);
	u16			 w_length = le16_to_cpu(ctrl->wLength);

	switch ((ctrl->bRequestType << 8) | ctrl->bRequest) {

	/* SET_LINE_CODING ... just read and save what the host sends */
	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_REQ_SET_LINE_CODING:
		if (w_length != sizeof(struct usb_cdc_line_coding))
			goto invalid;

		value = w_length;
		cdev->gadget->ep0->driver_data = gser;
		req->complete = kc_gser_complete_set_line_coding;
		break;

	/* GET_LINE_CODING ... return what host sent, or initial value */
	case ((USB_DIR_IN | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_REQ_GET_LINE_CODING:
		value = min_t(unsigned, w_length,
				sizeof(struct usb_cdc_line_coding));
		memcpy(req->buf, &gser->port_line_coding, value);
		break;

	/* SET_CONTROL_LINE_STATE ... save what the host sent */
	case ((USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE) << 8)
			| USB_CDC_REQ_SET_CONTROL_LINE_STATE:

		value = 0;
		gser->port_handshake_bits = w_value;
		if (gser->port.notify_modem) {
			unsigned port_num =
				kc_gserial_ports[gser->port_num].client_port_num;

			gser->port.notify_modem(&gser->port,
					port_num, w_value);
		}
		break;

	default:
invalid:
		DBG(cdev, "invalid control req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
	}

	/* respond with data transfer or status phase? */
	if (value >= 0) {
		DBG(cdev, "gser ttyGS%d req%02x.%02x v%04x i%04x l%d\n",
			gser->port_num, ctrl->bRequestType, ctrl->bRequest,
			w_value, w_index, w_length);
		req->zero = 0;
		req->length = value;
		value = usb_ep_queue(cdev->gadget->ep0, req, GFP_ATOMIC);
		if (value < 0)
			ERROR(cdev, "gser response on ttyGS%d, err %d\n",
					gser->port_num, value);
	}

	/* device either stalls (value < 0) or reports success */
	return value;
}

static int kc_gser_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct kc_f_gser		*gser = kc_func_to_gser(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int rc = 0;

	/* we know alt == 0, so this is an activation or a reset */

	if (intf == gser->ctrl_id) {
		if (gser->notify->driver_data) {
			DBG(cdev, "reset generic ctl ttyGS%d\n", gser->port_num);
			usb_ep_disable(gser->notify);
		}

		if (!gser->notify->desc) {
			if (config_ep_by_speed(cdev->gadget, f, gser->notify)) {
				gser->notify->desc = NULL;
				return -EINVAL;
			}
		}
		rc = usb_ep_enable(gser->notify);

		if (rc) {
			ERROR(cdev, "can't enable %s, result %d\n",
						gser->notify->name, rc);
			return rc;
		}
		gser->notify->driver_data = gser;
	} else if (intf == gser->data_id) {

		if (gser->port.in->driver_data) {
			DBG(cdev, "reset generic data ttyGS%d\n", gser->port_num);
			kc_gport_disconnect(gser);
		}
		if (!gser->port.in->desc || !gser->port.out->desc) {
			DBG(cdev, "activate generic ttyGS%d\n", gser->port_num);
			if (config_ep_by_speed(cdev->gadget, f, gser->port.in) ||
			    config_ep_by_speed(cdev->gadget, f, gser->port.out)) {
				gser->port.in->desc = NULL;
				gser->port.out->desc = NULL;
				return -EINVAL;
			}
		}

		kc_gport_connect(gser);

		gser->online = 1;
	}
	return rc;
}

static void kc_gser_disable(struct usb_function *f)
{
	struct kc_f_gser	*gser = kc_func_to_gser(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	DBG(cdev, "generic ttyGS%d deactivated\n", gser->port_num);

	kc_gport_disconnect(gser);

	usb_ep_fifo_flush(gser->notify);
	usb_ep_disable(gser->notify);
	gser->notify->driver_data = NULL;
	gser->online = 0;
}

static int kc_gser_notify(struct kc_f_gser *gser, u8 type, u16 value,
		void *data, unsigned length)
{
	struct usb_ep			*ep = gser->notify;
	struct usb_request		*req;
	struct usb_cdc_notification	*notify;
	const unsigned			len = sizeof(*notify) + length;
	void				*buf;
	int				status;
	struct usb_composite_dev *cdev = gser->port.func.config->cdev;

	req = gser->notify_req;
	gser->notify_req = NULL;
	gser->pending = false;

	req->length = len;
	notify = req->buf;
	buf = notify + 1;

	notify->bmRequestType = USB_DIR_IN | USB_TYPE_CLASS
			| USB_RECIP_INTERFACE;
	notify->bNotificationType = type;
	notify->wValue = cpu_to_le16(value);
	notify->wIndex = cpu_to_le16(gser->ctrl_id);
	notify->wLength = cpu_to_le16(length);
	memcpy(buf, data, length);

	status = usb_ep_queue(ep, req, GFP_ATOMIC);
	if (status < 0) {
		ERROR(cdev, "gser ttyGS%d can't notify serial state, %d\n",
				gser->port_num, status);
		gser->notify_req = req;
	}

	return status;
}

static int kc_gser_notify_serial_state(struct kc_f_gser *gser)
{
	int			 status;
	unsigned long flags;
	struct usb_composite_dev *cdev = gser->port.func.config->cdev;

	spin_lock_irqsave(&gser->lock, flags);
	if (gser->notify_req) {
		DBG(cdev, "gser ttyGS%d serial state %04x\n",
				gser->port_num, gser->serial_state);
		status = kc_gser_notify(gser, USB_CDC_NOTIFY_SERIAL_STATE,
				0, &gser->serial_state,
					sizeof(gser->serial_state));
	} else {
		gser->pending = true;
		status = 0;
	}
	spin_unlock_irqrestore(&gser->lock, flags);
	return status;
}

static void kc_gser_notify_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct kc_f_gser *gser = req->context;
	u8	      doit = false;
	unsigned long flags;

	/* on this call path we do NOT hold the port spinlock,
	 * which is why ACM needs its own spinlock
	 */
	spin_lock_irqsave(&gser->lock, flags);
	if (req->status != -ESHUTDOWN)
		doit = gser->pending;
	gser->notify_req = req;
	spin_unlock_irqrestore(&gser->lock, flags);

	if (doit && gser->online)
		kc_gser_notify_serial_state(gser);
}
static void kc_gser_connect(struct gserial *port)
{
	struct kc_f_gser *gser = kc_port_to_gser(port);

	gser->serial_state |= KC_ACM_CTRL_DSR | KC_ACM_CTRL_DCD;
	kc_gser_notify_serial_state(gser);
}

unsigned int kc_gser_get_dtr(struct gserial *port)
{
	struct kc_f_gser *gser = kc_port_to_gser(port);

	if (gser->port_handshake_bits & KC_ACM_CTRL_DTR)
		return 1;
	else
		return 0;
}

unsigned int kc_gser_get_rts(struct gserial *port)
{
	struct kc_f_gser *gser = kc_port_to_gser(port);

	if (gser->port_handshake_bits & KC_ACM_CTRL_RTS)
		return 1;
	else
		return 0;
}

unsigned int kc_gser_send_carrier_detect(struct gserial *port, unsigned int yes)
{
	struct kc_f_gser *gser = kc_port_to_gser(port);
	u16			state;

	state = gser->serial_state;
	state &= ~KC_ACM_CTRL_DCD;
	if (yes)
		state |= KC_ACM_CTRL_DCD;

	gser->serial_state = state;
	return kc_gser_notify_serial_state(gser);

}

unsigned int kc_gser_send_ring_indicator(struct gserial *port, unsigned int yes)
{
	struct kc_f_gser *gser = kc_port_to_gser(port);
	u16			state;

	state = gser->serial_state;
	state &= ~KC_ACM_CTRL_RI;
	if (yes)
		state |= KC_ACM_CTRL_RI;

	gser->serial_state = state;
	return kc_gser_notify_serial_state(gser);

}
static void kc_gser_disconnect(struct gserial *port)
{
	struct kc_f_gser *gser = kc_port_to_gser(port);

	gser->serial_state &= ~(KC_ACM_CTRL_DSR | KC_ACM_CTRL_DCD);
	kc_gser_notify_serial_state(gser);
}

static int kc_gser_send_break(struct gserial *port, int duration)
{
	struct kc_f_gser *gser = kc_port_to_gser(port);
	u16			state;

	state = gser->serial_state;
	state &= ~KC_ACM_CTRL_BRK;
	if (duration)
		state |= KC_ACM_CTRL_BRK;

	gser->serial_state = state;
	return kc_gser_notify_serial_state(gser);
}

static int kc_gser_send_modem_ctrl_bits(struct gserial *port, int ctrl_bits)
{
	struct kc_f_gser *gser = kc_port_to_gser(port);

	gser->serial_state = ctrl_bits;

	return kc_gser_notify_serial_state(gser);
}

/*-------------------------------------------------------------------------*/

/* serial function driver setup/binding */

static int
kc_gser_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct kc_f_gser		*gser = kc_func_to_gser(f);
	int			status;
	struct usb_ep		*ep;

	/* allocate instance-specific interface IDs, and patch descriptors */
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	gser->ctrl_id = status;

	kc_gser_control_interface_desc.bInterfaceNumber = status;
	kc_gser_union_desc .bMasterInterface0 = status;

	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	gser->data_id = status;

	kc_gser_data_interface_desc.bInterfaceNumber = status;
	kc_gser_union_desc.bSlaveInterface0 = status;
	kc_gser_call_mgmt_descriptor.bDataInterface = status;

	status = -ENODEV;

	/* allocate instance-specific endpoints */
	ep = usb_ep_autoconfig(cdev->gadget, &kc_gser_fs_in_desc);
	if (!ep)
		goto fail;
	gser->port.in = ep;
	ep->driver_data = cdev;	/* claim */

	ep = usb_ep_autoconfig(cdev->gadget, &kc_gser_fs_out_desc);
	if (!ep)
		goto fail;
	gser->port.out = ep;
	ep->driver_data = cdev;	/* claim */

	ep = usb_ep_autoconfig(cdev->gadget, &kc_gser_fs_notify_desc);
	if (!ep)
		goto fail;
	gser->notify = ep;
	ep->driver_data = cdev;	/* claim */
	/* allocate notification */
	gser->notify_req = gs_alloc_req(ep,
			sizeof(struct usb_cdc_notification) + 2,
			GFP_KERNEL);
	if (!gser->notify_req)
		goto fail;

	gser->notify_req->complete = kc_gser_notify_complete;
	gser->notify_req->context = gser;

	/* copy descriptors, and track endpoint copies */
	f->fs_descriptors = usb_copy_descriptors(kc_gser_fs_function);
    

	if (!f->fs_descriptors)
		goto fail;

	/* support all relevant hardware speeds... we expect that when
	 * hardware is dual speed, all bulk-capable endpoints work at
	 * both speeds
	 */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		kc_gser_hs_in_desc.bEndpointAddress =
				kc_gser_fs_in_desc.bEndpointAddress;
		kc_gser_hs_out_desc.bEndpointAddress =
				kc_gser_fs_out_desc.bEndpointAddress;
		kc_gser_hs_notify_desc.bEndpointAddress =
				kc_gser_fs_notify_desc.bEndpointAddress;

		/* copy descriptors, and track endpoint copies */
		f->hs_descriptors = usb_copy_descriptors(kc_gser_hs_function);

		if (!f->hs_descriptors)
			goto fail;

	}
	DBG(cdev, "generic ttyGS%d: %s speed IN/%s OUT/%s\n",
			gser->port_num,
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			gser->port.in->name, gser->port.out->name);
	return 0;

fail:
	if (f->fs_descriptors)
		usb_free_descriptors(f->fs_descriptors);

	if (gser->notify_req)
		gs_free_req(gser->notify, gser->notify_req);

	/* we might as well release our claims on endpoints */
	if (gser->notify)
		gser->notify->driver_data = NULL;

	/* we might as well release our claims on endpoints */
	if (gser->port.out)
		gser->port.out->driver_data = NULL;
	if (gser->port.in)
		gser->port.in->driver_data = NULL;

	ERROR(cdev, "%s: can't bind, err %d\n", f->name, status);

	return status;
}

static void
kc_gser_unbind(struct usb_configuration *c, struct usb_function *f)
{
	struct kc_f_gser *gser = kc_func_to_gser(f);

	if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);
	if (gadget_is_superspeed(c->cdev->gadget))
		usb_free_descriptors(f->ss_descriptors);
	usb_free_descriptors(f->fs_descriptors);

	gs_free_req(gser->notify, gser->notify_req);
	kfree(kc_func_to_gser(f));
}

/**
 * gser_bind_config - add a generic serial function to a configuration
 * @c: the configuration to support the serial instance
 * @port_num: /dev/ttyGS* port this interface will use
 * Context: single threaded during gadget setup
 *
 * Returns zero on success, else negative errno.
 *
 * Caller must have called @gserial_setup() with enough ports to
 * handle all the ones it binds.  Caller is also responsible
 * for calling @gserial_cleanup() before module unload.
 */
int kc_gser_bind_config(struct usb_configuration *c, u8 port_num)
{
	struct kc_f_gser	*gser;
	int		status;

	/* REVISIT might want instance-specific strings to help
	 * distinguish instances ...
	 */

	/* allocate and initialize one new instance */
	gser = kzalloc(sizeof *gser, GFP_KERNEL);
	if (!gser)
		return -ENOMEM;

	spin_lock_init(&gser->lock);

	gser->port_num = port_num;

	gser->port.func.name = "kc_gser";
	gser->port.func.bind = kc_gser_bind;
	gser->port.func.unbind = kc_gser_unbind;
	gser->port.func.set_alt = kc_gser_set_alt;
	gser->port.func.disable = kc_gser_disable;
	gser->transport		= kc_gserial_ports[port_num].transport;

	/* We support only three ports for now */
	if (port_num == 0)
		gser->port.func.name = "modem";
	else if (port_num == 1)
		gser->port.func.name = "nmea";
	else
		gser->port.func.name = "modem2";
	gser->port.func.setup = kc_gser_setup;
	gser->port.connect = kc_gser_connect;
	gser->port.get_dtr = kc_gser_get_dtr;
	gser->port.get_rts = kc_gser_get_rts;
	gser->port.send_carrier_detect = kc_gser_send_carrier_detect;
	gser->port.send_ring_indicator = kc_gser_send_ring_indicator;
	gser->port.send_modem_ctrl_bits = kc_gser_send_modem_ctrl_bits;
	gser->port.disconnect = kc_gser_disconnect;
	gser->port.send_break = kc_gser_send_break;
	
    gser->port_line_coding.dwDTERate = 0x2580;
    gser->port_line_coding.bCharFormat = USB_CDC_1_STOP_BITS;
    gser->port_line_coding.bParityType = USB_CDC_NO_PARITY;
    gser->port_line_coding.bDataBits = 8;

	status = usb_add_function(c, &gser->port.func);
	if (status)
		kfree(gser);
	return status;
}

/**
 * gserial_init_port - bind a gserial_port to its transport
 */
static int kc_gserial_init_port(int port_num, const char *name,
		const char *port_name)
{
	enum transport_type transport;
	int ret = 0;

	if (port_num >= KC_GSERIAL_NO_PORTS)
		return -ENODEV;

	transport = str_to_xport(name);
	pr_debug("%s, port:%d, transport:%s\n", __func__,
			port_num, xport_to_str(transport));

	kc_gserial_ports[port_num].transport = transport;
	kc_gserial_ports[port_num].port_num = port_num;

	switch (transport) {
	case USB_GADGET_XPORT_TTY:
		kc_gserial_ports[port_num].client_port_num = kc_no_tty_ports;
		kc_no_tty_ports++;
		break;
	case USB_GADGET_XPORT_SMD:
		kc_gserial_ports[port_num].client_port_num = kc_no_smd_ports;
		kc_no_smd_ports++;
			break;
	case USB_GADGET_XPORT_HSIC:
//		ghsic_ctrl_set_port_name(port_name, name);
//		ghsic_data_set_port_name(port_name, name);

		/*client port number will be updated in gport_setup*/
		kc_no_hsic_sports++;
		break;
	case USB_GADGET_XPORT_HSUART:
		/*client port number will be updated in gport_setup*/
		kc_no_hsuart_sports++;
		break;
	default:
		pr_err("%s: Un-supported transport transport: %u\n",
				__func__, kc_gserial_ports[port_num].transport);
		return -ENODEV;
	}

	kc_nr_ports++;

	return ret;
}
