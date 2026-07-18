// SPDX-License-Identifier: GPL-2.0+
/*
 * CDC-ECM (Ethernet Control Model) USB host ethernet driver
 *
 * Drives a USB peripheral that exposes the standard CDC-ECM function,
 * e.g. a Linux device running the g_ether / f_ecm gadget (such as a BMC
 * attached to the board's USB host port). Unlike the other drivers in
 * this directory it is matched by interface class, not vendor/product ID.
 *
 * ECM framing is trivial: one ethernet frame per bulk transfer, no
 * headers. The device's ethernet functional descriptor tells the host
 * which MAC address to use (iMACAddress string descriptor).
 */

#include <dm.h>
#include <hexdump.h>
#include <log.h>
#include <malloc.h>
#include <memalign.h>
#include <net.h>
#include <usb.h>
#include <linux/usb/cdc.h>
#include "usb_ether.h"

#define ECM_BULK_SEND_TIMEOUT_MS	100

struct ecm_eth_priv {
	struct ueth_data ueth;
	u8 ctrl_ifnum;			/* bInterfaceNumber, control intf */
	u8 data_ifnum;			/* bInterfaceNumber, data intf */
	u16 max_seg;			/* wMaxSegmentSize */
	int rx_urb_size;
	u8 mac[ETH_ALEN];		/* host MAC from iMACAddress */
};

/* iMACAddress: the MAC this host should use, as 12 hex digits */
static int ecm_read_mac_string(struct usb_device *udev, u8 index, u8 *mac)
{
	char mac_str[13];
	int i, hi, lo;

	i = usb_string(udev, index, mac_str, sizeof(mac_str));
	if (i < ETH_ALEN * 2) {
		debug("%s: cannot read iMACAddress string: %d\n", __func__, i);
		return i < 0 ? i : -EINVAL;
	}

	for (i = 0; i < ETH_ALEN; i++) {
		hi = hex_to_bin(mac_str[i * 2]);
		lo = hex_to_bin(mac_str[i * 2 + 1]);
		if (hi < 0 || lo < 0)
			return -EINVAL;
		mac[i] = hi << 4 | lo;
	}

	return 0;
}

/*
 * Walk the raw configuration descriptor to find the ECM control
 * interface and its class-specific functional descriptors. The parsed
 * copy in udev->config does not retain class-specific descriptors, so
 * they must be taken from the raw blob.
 */
static int ecm_parse_config(struct usb_device *udev,
			    struct ecm_eth_priv *priv)
{
	struct usb_cdc_ether_desc *ether = NULL;
	struct usb_cdc_union_desc *un;
	struct usb_descriptor_header *head;
	struct usb_interface_descriptor *ifd;
	unsigned char *buffer;
	bool in_ecm_ctrl = false, found = false;
	int index, len, ret;

	len = usb_get_configuration_len(udev, 0);
	if (len < 0)
		return len;

	buffer = memalign(ARCH_DMA_MINALIGN, len);
	if (!buffer)
		return -ENOMEM;

	index = usb_get_configuration_no(udev, 0, buffer, len);
	if (index < 0) {
		free(buffer);
		return index;
	}

	priv->data_ifnum = 0xff;
	for (index = 0; index + sizeof(*head) <= len; index += head->bLength) {
		head = (struct usb_descriptor_header *)&buffer[index];
		if (!head->bLength || index + head->bLength > len)
			break;

		switch (head->bDescriptorType) {
		case USB_DT_INTERFACE:
			if (head->bLength != USB_DT_INTERFACE_SIZE)
				break;
			ifd = (struct usb_interface_descriptor *)head;
			in_ecm_ctrl = !found &&
				ifd->bInterfaceClass == USB_CLASS_COMM &&
				ifd->bInterfaceSubClass ==
					USB_CDC_SUBCLASS_ETHERNET;
			if (in_ecm_ctrl) {
				priv->ctrl_ifnum = ifd->bInterfaceNumber;
				found = true;
			}
			break;
		case USB_DT_CS_INTERFACE:
			if (!in_ecm_ctrl || head->bLength < 3)
				break;
			switch (buffer[index + 2]) {
			case USB_CDC_UNION_TYPE:
				if (head->bLength <
				    sizeof(struct usb_cdc_union_desc))
					break;
				un = (struct usb_cdc_union_desc *)head;
				priv->data_ifnum = un->bSlaveInterface0;
				break;
			case USB_CDC_ETHERNET_TYPE:
				if (head->bLength < sizeof(*ether))
					break;
				ether = (struct usb_cdc_ether_desc *)head;
				break;
			}
			break;
		}
	}

	if (!found || !ether || !ether->iMACAddress) {
		debug("%s: no ECM ethernet functional descriptor\n", __func__);
		free(buffer);
		return -ENXIO;
	}

	/* Without a union descriptor, assume the data interface follows */
	if (priv->data_ifnum == 0xff)
		priv->data_ifnum = priv->ctrl_ifnum + 1;

	priv->max_seg = le16_to_cpu(ether->wMaxSegmentSize) ?: PKTSIZE;
	priv->rx_urb_size = ALIGN(priv->max_seg, 512);

	ret = ecm_read_mac_string(udev, ether->iMACAddress, priv->mac);
	free(buffer);

	return ret;
}

static int ecm_find_endpoints(struct usb_device *udev,
			      struct ecm_eth_priv *priv)
{
	struct ueth_data *ueth = &priv->ueth;
	struct usb_interface *iface = NULL;
	int i;

	for (i = 0; i < udev->config.no_of_if; i++) {
		if (udev->config.if_desc[i].desc.bInterfaceNumber ==
		    priv->data_ifnum) {
			iface = &udev->config.if_desc[i];
			break;
		}
	}
	if (!iface) {
		debug("%s: data interface %d not found\n", __func__,
		      priv->data_ifnum);
		return -ENXIO;
	}

	/*
	 * The bulk endpoints live in altsetting 1; usb_parse_config()
	 * accumulates all altsettings' endpoints into one interface slot,
	 * so they are visible here.
	 */
	for (i = 0; i < iface->no_of_ep; i++) {
		int ep_addr = iface->ep_desc[i].bEndpointAddress;

		if ((iface->ep_desc[i].bmAttributes &
		     USB_ENDPOINT_XFERTYPE_MASK) != USB_ENDPOINT_XFER_BULK)
			continue;

		if ((ep_addr & USB_DIR_IN) && !ueth->ep_in)
			ueth->ep_in = ep_addr & USB_ENDPOINT_NUMBER_MASK;
		else if (!(ep_addr & USB_DIR_IN) && !ueth->ep_out)
			ueth->ep_out = ep_addr & USB_ENDPOINT_NUMBER_MASK;
	}
	debug("%s: ep_in %d, ep_out %d\n", __func__, ueth->ep_in,
	      ueth->ep_out);

	if (!ueth->ep_in || !ueth->ep_out)
		return -ENXIO;

	return 0;
}

static int ecm_eth_start(struct udevice *dev)
{
	struct ecm_eth_priv *priv = dev_get_priv(dev);
	struct usb_device *udev = priv->ueth.pusb_dev;
	int ret;

	/*
	 * Open the receive path. A stall just means the device keeps its
	 * default (everything through), so it is not fatal.
	 */
	ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
			      USB_CDC_SET_ETHERNET_PACKET_FILTER,
			      USB_TYPE_CLASS | USB_RECIP_INTERFACE |
			      USB_DIR_OUT,
			      USB_CDC_PACKET_TYPE_DIRECTED |
			      USB_CDC_PACKET_TYPE_BROADCAST |
			      USB_CDC_PACKET_TYPE_MULTICAST,
			      priv->ctrl_ifnum, NULL, 0, USB_CNTL_TIMEOUT);
	if (ret < 0)
		debug("%s: SetEthernetPacketFilter failed: %d\n", __func__,
		      ret);

	return 0;
}

static void ecm_eth_stop(struct udevice *dev)
{
}

static int ecm_eth_send(struct udevice *dev, void *packet, int length)
{
	struct ecm_eth_priv *priv = dev_get_priv(dev);
	struct ueth_data *ueth = &priv->ueth;
	unsigned long pipe = usb_sndbulkpipe(ueth->pusb_dev, ueth->ep_out);
	int actual_len, maxpacket;
	int err;
	ALLOC_CACHE_ALIGN_BUFFER(unsigned char, msg, PKTSIZE + 1);

	if (length > PKTSIZE)
		return -ENOSPC;

	memcpy(msg, packet, length);

	/*
	 * ECM ends a frame with a short packet. If the frame is an exact
	 * multiple of wMaxPacketSize, pad it by one byte instead of
	 * queueing a separate ZLP; receivers ignore the trailing byte
	 * (same strategy as Linux usbnet).
	 */
	maxpacket = usb_maxpacket(ueth->pusb_dev, pipe);
	if (maxpacket > 0 && !(length % maxpacket)) {
		msg[length] = 0;
		length++;
	}

	err = usb_bulk_msg(ueth->pusb_dev, pipe, (void *)msg, length,
			   &actual_len, ECM_BULK_SEND_TIMEOUT_MS);
	debug("Tx: len = %u, actual = %u, err = %d\n", length, actual_len,
	      err);

	return err;
}

static int ecm_eth_recv(struct udevice *dev, int flags, uchar **packetp)
{
	struct ecm_eth_priv *priv = dev_get_priv(dev);
	struct ueth_data *ueth = &priv->ueth;
	uint8_t *ptr;
	int ret, len;

	len = usb_ether_get_rx_bytes(ueth, &ptr);
	debug("%s: first try, len=%d\n", __func__, len);
	if (!len) {
		if (!(flags & ETH_RECV_CHECK_DEVICE))
			return -EAGAIN;
		ret = usb_ether_receive(ueth, priv->rx_urb_size);
		if (ret)
			return ret;

		len = usb_ether_get_rx_bytes(ueth, &ptr);
		debug("%s: second try, len=%d\n", __func__, len);
	}

	/* One bulk transfer carries exactly one ethernet frame */
	*packetp = ptr;
	return len;
}

static int ecm_free_pkt(struct udevice *dev, uchar *packet, int packet_len)
{
	struct ecm_eth_priv *priv = dev_get_priv(dev);

	usb_ether_advance_rxbuf(&priv->ueth, -1);

	return 0;
}

static int ecm_read_rom_hwaddr(struct udevice *dev)
{
	struct ecm_eth_priv *priv = dev_get_priv(dev);
	struct eth_pdata *pdata = dev_get_plat(dev);

	memcpy(pdata->enetaddr, priv->mac, ETH_ALEN);

	return 0;
}

static int ecm_eth_probe(struct udevice *dev)
{
	struct usb_device *udev = dev_get_parent_priv(dev);
	struct ecm_eth_priv *priv = dev_get_priv(dev);
	struct eth_pdata *pdata = dev_get_plat(dev);
	struct ueth_data *ueth = &priv->ueth;
	int ret;

	ret = ecm_parse_config(udev, priv);
	if (ret)
		return ret;

	ret = ecm_find_endpoints(udev, priv);
	if (ret)
		return ret;

	ueth->pusb_dev = udev;
	ueth->ifnum = priv->data_ifnum;
	ueth->rxsize = priv->rx_urb_size;
	ueth->rxbuf = memalign(ARCH_DMA_MINALIGN, ueth->rxsize);
	if (!ueth->rxbuf)
		return -ENOMEM;

	/* The bulk endpoints only exist in altsetting 1 */
	ret = usb_set_interface(udev, priv->data_ifnum, 1);
	if (ret) {
		debug("%s: cannot select data altsetting: %d\n", __func__,
		      ret);
		free(ueth->rxbuf);
		return ret;
	}

	memcpy(pdata->enetaddr, priv->mac, ETH_ALEN);

	return 0;
}

static const struct eth_ops ecm_eth_ops = {
	.start	= ecm_eth_start,
	.send	= ecm_eth_send,
	.recv	= ecm_eth_recv,
	.free_pkt = ecm_free_pkt,
	.stop	= ecm_eth_stop,
	.read_rom_hwaddr = ecm_read_rom_hwaddr,
};

U_BOOT_DRIVER(ecm_eth) = {
	.name	= "ecm_eth",
	.id	= UCLASS_ETH,
	.probe	= ecm_eth_probe,
	.ops	= &ecm_eth_ops,
	.priv_auto	= sizeof(struct ecm_eth_priv),
	.plat_auto	= sizeof(struct eth_pdata),
};

static const struct usb_device_id ecm_eth_id_table[] = {
	{
		.match_flags = USB_DEVICE_ID_MATCH_INT_CLASS |
			       USB_DEVICE_ID_MATCH_INT_SUBCLASS |
			       USB_DEVICE_ID_MATCH_INT_PROTOCOL,
		.bInterfaceClass = USB_CLASS_COMM,
		.bInterfaceSubClass = USB_CDC_SUBCLASS_ETHERNET,
		.bInterfaceProtocol = USB_CDC_PROTO_NONE,
	},
	{ }		/* Terminating entry */
};

U_BOOT_USB_DEVICE(ecm_eth, ecm_eth_id_table);
