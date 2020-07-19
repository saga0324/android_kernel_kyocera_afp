/*
 * f_nmea.c - USB nmea function driver
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
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include "usb_gadget_xport.h"

#include "u_serial.h"
#include "gadget_chips.h"

#include <linux/usb/cdc.h>

/*
 * This function packages a simple "generic serial" port with no real
 * control mechanisms, just raw data transfer over two bulk endpoints.
 *
 * Because it's not standardized, this isn't as interoperable as the
 * CDC ACM driver.  However, for many purposes it's just as functional
 * if you can arrange appropriate host side drivers.
 */
#define NMEA_NO_PORTS 1

struct f_nmea {
	struct gserial			port;
	u8				data_id;
	u8				port_num;

	u8				online;
	enum transport_type		transport;
};

static unsigned int no_tty_nmea_ports;
static unsigned int no_sdio_nmea_ports;
static unsigned int no_smd_nmea_ports;
static unsigned int no_hsic_nmea_sports;
static unsigned int no_hsuart_nmea_sports;
static unsigned int nr_nmea_ports;

static struct nmea_port_info {
	enum transport_type	transport;
	unsigned		port_num;
	unsigned char		client_port_num;
} nmea_ports[NMEA_NO_PORTS];

static inline struct f_nmea *func_to_nmea(struct usb_function *f)
{
	return container_of(f, struct f_nmea, port.func);
}
/*-------------------------------------------------------------------------*/

/* interface descriptor: */

static struct usb_interface_descriptor nmea_interface_desc = {
	.bLength =		USB_DT_INTERFACE_SIZE,
	.bDescriptorType =	USB_DT_INTERFACE,
	/* .bInterfaceNumber = DYNAMIC */
	.bNumEndpoints =	2,
	.bInterfaceClass =	0x02,
	.bInterfaceSubClass =	0x0A,
	.bInterfaceProtocol =	0x01,
	/* .iInterface = DYNAMIC */
};
#if 1
static struct usb_cdc_header_desc nmea_header_desc = {
	.bLength            =	sizeof(struct usb_cdc_header_desc),
	.bDescriptorType    =	0x24,
	.bDescriptorSubType =	0x00,
	.bcdCDC             =	__constant_cpu_to_le16(0x0110),
};

static struct usb_cdc_mdlm_desc nmea_mdlm_desc = {
	.bLength = sizeof(struct usb_cdc_mdlm_desc),
	.bDescriptorType = 0x24,
	.bDescriptorSubType = 0x12,
	.bcdVersion = __constant_cpu_to_le16(0x0100),
	.bGUID[0] = 0xC2,
	.bGUID[1] = 0x29,
	.bGUID[2] = 0x9F,
	.bGUID[3] = 0xCC,
	.bGUID[4] = 0xD4,
	.bGUID[5] = 0x89,
	.bGUID[6] = 0x40,
	.bGUID[7] = 0x66,
	.bGUID[8] = 0x89,
	.bGUID[9] = 0x2B,
	.bGUID[10] = 0x10,
	.bGUID[11] = 0xC3,
	.bGUID[12] = 0x41,
	.bGUID[13] = 0xDD,
	.bGUID[14] = 0x98,
	.bGUID[15] = 0xA9,
};

static struct usb_endpoint_descriptor nmea_hs_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
	.bInterval 		= 0,
};

static struct usb_endpoint_descriptor nmea_fs_in_desc  = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize = 	__constant_cpu_to_le16(64),
	.bInterval 		= 0,
};

static struct usb_endpoint_descriptor nmea_hs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
	.bInterval 		= 0,
};

static struct usb_endpoint_descriptor nmea_fs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(64),
	.bInterval 		= 0,
};

static struct usb_descriptor_header *nmea_fs_function[] = {
	(struct usb_descriptor_header *) &nmea_interface_desc,
	(struct usb_descriptor_header *) &nmea_header_desc,
	(struct usb_descriptor_header *) &nmea_mdlm_desc,
	(struct usb_descriptor_header *) &nmea_fs_in_desc,
	(struct usb_descriptor_header *) &nmea_fs_out_desc,
	NULL,
};

static struct usb_descriptor_header *nmea_hs_function[] = {
	(struct usb_descriptor_header *) &nmea_interface_desc,
	(struct usb_descriptor_header *) &nmea_header_desc,
	(struct usb_descriptor_header *) &nmea_mdlm_desc,
	(struct usb_descriptor_header *) &nmea_hs_in_desc,
	(struct usb_descriptor_header *) &nmea_hs_out_desc,
	NULL,
};

#else
/* full speed support: */
static struct usb_endpoint_descriptor nmea_fs_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_endpoint_descriptor nmea_fs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_OUT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
};

static struct usb_descriptor_header *nmea_fs_function[] = {
	(struct usb_descriptor_header *) &nmea_interface_desc,
	(struct usb_descriptor_header *) &nmea_fs_in_desc,
	(struct usb_descriptor_header *) &nmea_fs_out_desc,
	NULL,
};

/* high speed support: */
static struct usb_endpoint_descriptor nmea_hs_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
};

static struct usb_endpoint_descriptor nmea_hs_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	__constant_cpu_to_le16(512),
};

static struct usb_descriptor_header *nmea_hs_function[] = {
	(struct usb_descriptor_header *) &nmea_interface_desc,
	(struct usb_descriptor_header *) &nmea_hs_in_desc,
	(struct usb_descriptor_header *) &nmea_hs_out_desc,
	NULL,
};

static struct usb_endpoint_descriptor nmea_ss_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(1024),
};

static struct usb_endpoint_descriptor nmea_ss_out_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bmAttributes =		USB_ENDPOINT_XFER_BULK,
	.wMaxPacketSize =	cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor nmea_ss_bulk_comp_desc = {
	.bLength =              sizeof nmea_ss_bulk_comp_desc,
	.bDescriptorType =      USB_DT_SS_ENDPOINT_COMP,
};

static struct usb_descriptor_header *nmea_ss_function[] = {
	(struct usb_descriptor_header *) &nmea_interface_desc,
	(struct usb_descriptor_header *) &nmea_ss_in_desc,
	(struct usb_descriptor_header *) &nmea_ss_bulk_comp_desc,
	(struct usb_descriptor_header *) &nmea_ss_out_desc,
	(struct usb_descriptor_header *) &nmea_ss_bulk_comp_desc,
	NULL,
};
#endif

/* string descriptors: */

static struct usb_string nmea_string_defs[] = {
	[0].s = "Generic Serial",
	{  } /* end of list */
};

static struct usb_gadget_strings nmea_string_table = {
	.language =		0x0409,	/* en-us */
	.strings =		nmea_string_defs,
};

static struct usb_gadget_strings *nmea_strings[] = {
	&nmea_string_table,
	NULL,
};

static int nmea_port_setup(struct usb_configuration *c)
{
	int ret = 0;
	int port_idx;
	int i;
    int f;
    int ret_t;

	pr_debug("%s: no_tty_nmea_ports: %u no_sdio_nmea_ports: %u"
		" no_smd_nmea_ports: %u no_hsic_nmea_sports: %u no_hsuart_ports: %u nr_nmea_ports: %u\n",
			__func__, no_tty_nmea_ports, no_sdio_nmea_ports, no_smd_nmea_ports,
			no_hsic_nmea_sports, no_hsuart_nmea_sports, nr_nmea_ports);

    if(no_tty_nmea_ports){
		for (f = 0; f < no_tty_nmea_ports; f++){
			ret_t = gserial_alloc_line(
					&nmea_ports[f].client_port_num);
			if (ret_t)
				return ret_t;
		}
    }

	if (no_sdio_nmea_ports)
		ret = gsdio_setup(c->cdev->gadget, no_sdio_nmea_ports);
	if (no_smd_nmea_ports)
		ret = gsmd_setup(c->cdev->gadget, no_smd_nmea_ports);
	if (no_hsic_nmea_sports) {
		port_idx = ghsic_data_setup(no_hsic_nmea_sports, USB_GADGET_SERIAL);
		if (port_idx < 0)
			return port_idx;

		for (i = 0; i < nr_nmea_ports; i++) {
			if (nmea_ports[i].transport ==
					USB_GADGET_XPORT_HSIC) {
				nmea_ports[i].client_port_num = port_idx;
				port_idx++;
			}
		}

		/*clinet port num is same for data setup and ctrl setup*/
		ret = ghsic_ctrl_setup(no_hsic_nmea_sports, USB_GADGET_SERIAL);
		if (ret < 0)
			return ret;
		return 0;
	}
	if (no_hsuart_nmea_sports) {
		port_idx = ghsuart_data_setup(no_hsuart_nmea_sports,
					USB_GADGET_SERIAL);
		if (port_idx < 0)
			return port_idx;

		for (i = 0; i < nr_nmea_ports; i++) {
			if (nmea_ports[i].transport ==
					USB_GADGET_XPORT_HSUART) {
				nmea_ports[i].client_port_num = port_idx;
				port_idx++;
			}
		}

		return 0;
	}
	return ret;
}

static int nmea_port_connect(struct f_nmea *nmea)
{
	unsigned	port_num;
	int		ret;

	pr_debug("%s: transport: %s f_nmea: %p nmeaport: %p port_num: %d\n",
			__func__, xport_to_str(nmea->transport),
			nmea, &nmea->port, nmea->port_num);

	port_num = nmea_ports[nmea->port_num].client_port_num;

	switch (nmea->transport) {
	case USB_GADGET_XPORT_TTY:
		gserial_connect(&nmea->port, port_num);
		break;
	case USB_GADGET_XPORT_SMD:
		gsmd_connect(&nmea->port, port_num);
		break;
	case USB_GADGET_XPORT_HSIC:
		ret = ghsic_ctrl_connect(&nmea->port, port_num);
		if (ret) {
			pr_err("%s: ghsic_ctrl_connect failed: err:%d\n",
					__func__, ret);
			return ret;
		}
		ret = ghsic_data_connect(&nmea->port, port_num);
		if (ret) {
			pr_err("%s: ghsic_data_connect failed: err:%d\n",
					__func__, ret);
			ghsic_ctrl_disconnect(&nmea->port, port_num);
			return ret;
		}
		break;
	case USB_GADGET_XPORT_HSUART:
		ret = ghsuart_data_connect(&nmea->port, port_num);
		if (ret) {
			pr_err("%s: ghsuart_data_connect failed: err:%d\n",
					__func__, ret);
			return ret;
		}
		break;
	default:
		pr_err("%s: Un-supported transport: %s\n", __func__,
				xport_to_str(nmea->transport));
		return -ENODEV;
	}

	return 0;
}

static int nmea_port_disconnect(struct f_nmea *nmea)
{
	unsigned port_num;

	pr_debug("%s: transport: %s f_nmea: %p nmeaport: %p port_num: %d\n",
			__func__, xport_to_str(nmea->transport),
			nmea, &nmea->port, nmea->port_num);

	port_num = nmea_ports[nmea->port_num].client_port_num;

	switch (nmea->transport) {
	case USB_GADGET_XPORT_TTY:
		gserial_disconnect(&nmea->port);
		break;
	case USB_GADGET_XPORT_SMD:
		gsmd_disconnect(&nmea->port, port_num);
		break;
	case USB_GADGET_XPORT_HSIC:
		ghsic_ctrl_disconnect(&nmea->port, port_num);
		ghsic_data_disconnect(&nmea->port, port_num);
		break;
	case USB_GADGET_XPORT_HSUART:
		ghsuart_data_disconnect(&nmea->port, port_num);
		break;
	default:
		pr_err("%s: Un-supported transport:%s\n", __func__,
				xport_to_str(nmea->transport));
		return -ENODEV;
	}

	return 0;
}

static int nmea_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
	struct f_nmea		*nmea = func_to_nmea(f);
	struct usb_composite_dev *cdev = f->config->cdev;
	int rc = 0;

	/* we know alt == 0, so this is an activation or a reset */

	if (nmea->port.in->driver_data) {
		DBG(cdev, "reset generic data ttyGS%d\n", nmea->port_num);
		nmea_port_disconnect(nmea);
	}
	if (!nmea->port.in->desc || !nmea->port.out->desc) {
		DBG(cdev, "activate generic ttyGS%d\n", nmea->port_num);
		if (config_ep_by_speed(cdev->gadget, f, nmea->port.in) ||
		    config_ep_by_speed(cdev->gadget, f, nmea->port.out)) {
			nmea->port.in->desc = NULL;
			nmea->port.out->desc = NULL;
			return -EINVAL;
		}
	}

	nmea_port_connect(nmea);

	nmea->online = 1;
	return rc;
}

static void nmea_disable(struct usb_function *f)
{
	struct f_nmea	*nmea = func_to_nmea(f);
	struct usb_composite_dev *cdev = f->config->cdev;

	DBG(cdev, "generic ttyGS%d deactivated\n", nmea->port_num);

	nmea_port_disconnect(nmea);

	nmea->online = 0;
}
/*-------------------------------------------------------------------------*/

/* serial function driver setup/binding */

static int
nmea_bind(struct usb_configuration *c, struct usb_function *f)
{
	struct usb_composite_dev *cdev = c->cdev;
	struct f_nmea		*nmea = func_to_nmea(f);
	int			status;
	struct usb_ep		*ep;

	/* allocate instance-specific interface IDs */
	status = usb_interface_id(c, f);
	if (status < 0)
		goto fail;
	nmea->data_id = status;
	nmea_interface_desc.bInterfaceNumber = status;

	status = -ENODEV;

	/* allocate instance-specific endpoints */
	ep = usb_ep_autoconfig(cdev->gadget, &nmea_fs_in_desc);
	if (!ep)
		goto fail;
	nmea->port.in = ep;
	ep->driver_data = cdev;	/* claim */

	ep = usb_ep_autoconfig(cdev->gadget, &nmea_fs_out_desc);
	if (!ep)
		goto fail;
	nmea->port.out = ep;
	ep->driver_data = cdev;	/* claim */

	/* copy descriptors, and track endpoint copies */
	f->fs_descriptors = usb_copy_descriptors(nmea_fs_function);

	if (!f->fs_descriptors)
		goto fail;

	/* support all relevant hardware speeds... we expect that when
	 * hardware is dual speed, all bulk-capable endpoints work at
	 * both speeds
	 */
	if (gadget_is_dualspeed(c->cdev->gadget)) {
		nmea_hs_in_desc.bEndpointAddress =
				nmea_fs_in_desc.bEndpointAddress;
		nmea_hs_out_desc.bEndpointAddress =
				nmea_fs_out_desc.bEndpointAddress;

		/* copy descriptors, and track endpoint copies */
		f->hs_descriptors = usb_copy_descriptors(nmea_hs_function);

		if (!f->hs_descriptors)
			goto fail;

	}
#if 0
	if (gadget_is_superspeed(c->cdev->gadget)) {
		nmea_ss_in_desc.bEndpointAddress =
			nmea_fs_in_desc.bEndpointAddress;
		nmea_ss_out_desc.bEndpointAddress =
			nmea_fs_out_desc.bEndpointAddress;

		/* copy descriptors, and track endpoint copies */
		f->ss_descriptors = usb_copy_descriptors(nmea_ss_function);
		if (!f->ss_descriptors)
			goto fail;
	}
#endif

	DBG(cdev, "generic ttyGS%d: %s speed IN/%s OUT/%s\n",
			nmea->port_num,
			gadget_is_superspeed(c->cdev->gadget) ? "super" :
			gadget_is_dualspeed(c->cdev->gadget) ? "dual" : "full",
			nmea->port.in->name, nmea->port.out->name);
	return 0;

fail:
	if (f->fs_descriptors)
		usb_free_descriptors(f->fs_descriptors);
	/* we might as well release our claims on endpoints */
	if (nmea->port.out)
		nmea->port.out->driver_data = NULL;
	if (nmea->port.in)
		nmea->port.in->driver_data = NULL;

	ERROR(cdev, "%s: can't bind, err %d\n", f->name, status);

	return status;
}

static void
nmea_unbind(struct usb_configuration *c, struct usb_function *f)
{
	if (gadget_is_dualspeed(c->cdev->gadget))
		usb_free_descriptors(f->hs_descriptors);
	if (gadget_is_superspeed(c->cdev->gadget))
		if (f->ss_descriptors)
			usb_free_descriptors(f->ss_descriptors);
	usb_free_descriptors(f->fs_descriptors);
	kfree(func_to_nmea(f));
}

/**
 * nmea_bind_config - add a generic serial function to a configuration
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
int nmea_bind_config(struct usb_configuration *c, u8 port_num)
{
	struct f_nmea	*nmea;
	int		status;

	/* REVISIT might want instance-specific strings to help
	 * distinguish instances ...
	 */

	/* maybe allocate device-global string ID */
	if (nmea_string_defs[0].id == 0) {
		status = usb_string_id(c->cdev);
		if (status < 0)
			return status;
		nmea_string_defs[0].id = status;
	}

	/* allocate and initialize one new instance */
	nmea = kzalloc(sizeof *nmea, GFP_KERNEL);
	if (!nmea)
		return -ENOMEM;

	nmea->port_num = port_num;

	nmea->port.func.name = "nmea";
	nmea->port.func.strings = nmea_strings;
	nmea->port.func.bind = nmea_bind;
	nmea->port.func.unbind = nmea_unbind;
	nmea->port.func.set_alt = nmea_set_alt;
	nmea->port.func.disable = nmea_disable;
	nmea->transport		= nmea_ports[port_num].transport;

	status = usb_add_function(c, &nmea->port.func);
	if (status)
		kfree(nmea);
	return status;
}

/**
 * nmea_init_port - bind a nmea_port to its transport
 */
static int nmea_init_port(int port_num, const char *name)
{
	enum transport_type transport;

	if (port_num >= NMEA_NO_PORTS)
		return -ENODEV;

	transport = str_to_xport(name);
	pr_debug("%s, port:%d, transport:%s\n", __func__,
			port_num, xport_to_str(transport));

	nmea_ports[port_num].transport = transport;
	nmea_ports[port_num].port_num = port_num;

	switch (transport) {
	case USB_GADGET_XPORT_TTY:
		nmea_ports[port_num].client_port_num = no_tty_nmea_ports;
		no_tty_nmea_ports++;
		break;
	case USB_GADGET_XPORT_SMD:
		nmea_ports[port_num].client_port_num = no_smd_nmea_ports;
		no_smd_nmea_ports++;
		break;
	case USB_GADGET_XPORT_HSIC:
		/*client port number will be updated in nmea_port_setup*/
		no_hsic_nmea_sports++;
		break;
	case USB_GADGET_XPORT_HSUART:
		/*client port number will be updated in nmea_port_setup*/
		no_hsuart_nmea_sports++;
		break;
	default:
		pr_err("%s: Un-supported transport transport: %u\n",
				__func__, nmea_ports[port_num].transport);
		return -ENODEV;
	}

	nr_nmea_ports++;

	return 0;
}
