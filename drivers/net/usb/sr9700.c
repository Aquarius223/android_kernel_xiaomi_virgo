/*
 * SR9700_android one chip USB 2.0 ethernet devices
 */

//#define DEBUG

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/stddef.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/usb.h>
#include <linux/crc32.h>
#include <linux/usb/usbnet.h>

#include "sr9700.h"

#define	MII_BUG_FIX

static int sr_read(struct usbnet *dev, u8 reg, u16 length, void *data)
{
	void *buf;
	int err = -ENOMEM;

	netdev_dbg(dev->net, "sr_read() reg=0x%02x length=%d", reg, length);

	buf = kmalloc(length, GFP_KERNEL);
	if (!buf)
		goto out;

	err = usb_control_msg(dev->udev, usb_rcvctrlpipe(dev->udev, 0), 
				SR_RD_REGS, SR_REQ_RD_REG,
			    0, reg, buf, length, USB_CTRL_SET_TIMEOUT);
	if (err == length)
		memcpy(data, buf, length);
	else if (err >= 0)
		err = -EINVAL;
	kfree(buf);

 out:
	return err;
}

static int sr_write(struct usbnet *dev, u8 reg, u16 length, void *data)
{
	void *buf = NULL;
	int err = -ENOMEM;

	netdev_dbg(dev->net, "sr_write() reg=0x%02x, length=%d", reg, length);

	if (data) {
		buf = kmemdup(data, length, GFP_KERNEL);
		if (!buf)
			goto out;
	}

	err = usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0),
			      SR_WR_REGS, SR_REQ_WR_REG,
			      0, reg, buf, length, USB_CTRL_SET_TIMEOUT);
	kfree(buf);
	if (err >= 0 && err < length)
		err = -EINVAL;
 out:
	return err;
}

static int sr_read_reg(struct usbnet *dev, u8 reg, u8 *value)
{
	return sr_read(dev, reg, 1, value);
}

static int sr_write_reg(struct usbnet *dev, u8 reg, u8 value)
{
	netdev_dbg(dev->net, "sr_write_reg() reg=0x%02x, value=0x%02x", reg, value);
	return usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0),
			       SR_WR_REG, SR_REQ_WR_REG,
			       value, reg, NULL, 0, USB_CTRL_SET_TIMEOUT);
}

static void sr_write_async_callback(struct urb *urb)
{
	struct usb_ctrlrequest *req = (struct usb_ctrlrequest *)urb->context;

	if (urb->status < 0)
		printk(KERN_DEBUG "sr_write_async_callback() failed with %d\n", urb->status);

	kfree(req);
	usb_free_urb(urb);
}

static void sr_write_async_helper(struct usbnet *dev, u8 reg, u8 value, u16 length, void *data)
{
	struct usb_ctrlrequest *req;
	struct urb *urb;
	int status;

	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb) {
		netdev_err(dev->net, "Error allocating URB in sr_write_async_helper!");
		return;
	}

	req = kmalloc(sizeof(struct usb_ctrlrequest), GFP_ATOMIC);
	if (!req) {
		netdev_err(dev->net, "Failed to allocate memory for control request");
		usb_free_urb(urb);
		return;
	}

	req->bRequestType = SR_REQ_WR_REG;
	req->bRequest = length ? SR_WR_REGS : SR_WR_REG;
	req->wValue = cpu_to_le16(value);
	req->wIndex = cpu_to_le16(reg);
	req->wLength = cpu_to_le16(length);

	usb_fill_control_urb(urb, dev->udev, usb_sndctrlpipe(dev->udev, 0),
			     (void *)req, data, length,
			     sr_write_async_callback, req);

	status = usb_submit_urb(urb, GFP_ATOMIC);
	if (status < 0) {
		netdev_err(dev->net, "Error submitting the control message: status=%d",
		       status);
		kfree(req);
		usb_free_urb(urb);
	}

	return;
}

static void sr_write_async(struct usbnet *dev, u8 reg, u16 length, void *data)
{
	netdev_dbg(dev->net, "sr_write_async() reg=0x%02x length=%d", reg, length);

	sr_write_async_helper(dev, reg, 0, length, data);
}

static void sr_write_reg_async(struct usbnet *dev, u8 reg, u8 value)
{
	netdev_dbg(dev->net, "sr_write_reg_async() reg=0x%02x value=0x%02x", reg, value);

	sr_write_async_helper(dev, reg, value, 0, NULL);
}

static int sr_share_read_word(struct usbnet *dev, int phy, u8 reg, __le16 *value)
{
	int ret, i;

	mutex_lock(&dev->phy_mutex);

	sr_write_reg(dev, EPAR, phy ? (reg | 0x40) : reg);
	sr_write_reg(dev, EPCR, phy ? 0xc : 0x4);

	for (i = 0; i < SR_SHARE_TIMEOUT; i++) {
		u8 tmp;

		udelay(1);
		ret = sr_read_reg(dev, EPCR, &tmp);
		if (ret < 0)
			goto out;

		if ((tmp & 1) == 0)
			break;
	}

	if (i >= SR_SHARE_TIMEOUT) {
		netdev_err(dev->net, "%s read timed out!", phy ? "phy" : "eeprom");
		ret = -EIO;
		goto out;
	}

	sr_write_reg(dev, EPCR, 0x0);
	ret = sr_read(dev, EPDR, 2, value);

	netdev_dbg(dev->net, "read shared %d 0x%02x returned 0x%04x, %d",
	       phy, reg, *value, ret);

 out:
	mutex_unlock(&dev->phy_mutex);
	return ret;
}

static int sr_share_write_word(struct usbnet *dev, int phy, u8 reg, __le16 value)
{
	int ret, i;

	mutex_lock(&dev->phy_mutex);

	ret = sr_write(dev, EPDR, 2, &value);
	if (ret < 0)
		goto out;

	sr_write_reg(dev, EPAR, phy ? (reg | 0x40) : reg);
	sr_write_reg(dev, EPCR, phy ? 0x1a : 0x12);

	for (i = 0; i < SR_SHARE_TIMEOUT; i++) {
		u8 tmp;

		udelay(1);
		ret = sr_read_reg(dev, EPCR, &tmp);
		if (ret < 0)
			goto out;

		if ((tmp & 1) == 0)
			break;
	}

	if (i >= SR_SHARE_TIMEOUT) {
		netdev_err(dev->net, "%s write timed out!", phy ? "phy" : "eeprom");
		ret = -EIO;
		goto out;
	}

	sr_write_reg(dev, EPCR, 0x0);

out:
	mutex_unlock(&dev->phy_mutex);
	return ret;
}

static int sr_read_eeprom_word(struct usbnet *dev, u8 offset, void *value)
{
	return sr_share_read_word(dev, 0, offset, value);
}


static int sr9700_android_get_eeprom_len(struct net_device *dev)
{
	return SR_EEPROM_LEN;
}

static int sr9700_android_get_eeprom(struct net_device *net, struct ethtool_eeprom *eeprom, u8 * data)
{
	struct usbnet *dev = netdev_priv(net);
	__le16 *ebuf = (__le16 *) data;
	int i;

	if ((eeprom->offset % 2) || (eeprom->len % 2))
		return -EINVAL;

	for (i = 0; i < eeprom->len / 2; i++) {
		if (sr_read_eeprom_word(dev, eeprom->offset / 2 + i, &ebuf[i]) < 0)
			return -EINVAL;
	}
	return 0;
}

static int sr9700_android_mdio_read(struct net_device *netdev, int phy_id, int loc)
{
	struct usbnet *dev = netdev_priv(netdev);

	__le16 res;
#ifdef	MII_BUG_FIX
	int rc = 0;
#endif

	if (phy_id) {
		netdev_dbg(dev->net, "Only internal phy supported");
		return 0;
	}

#ifdef	MII_BUG_FIX
	if(loc == MII_BMSR){
			u8 value;
			sr_read_reg(dev, NSR, &value);
			if(value & NSR_LINKST) {
				rc = 1;
			}
	}
	sr_share_read_word(dev, 1, loc, &res);
	if(rc == 1)
			return (le16_to_cpu(res) | BMSR_LSTATUS);
	else
			return (le16_to_cpu(res) & ~BMSR_LSTATUS);
#else
	sr_share_read_word(dev, 1, loc, &res);
#endif

	netdev_dbg(dev->net,
	       "sr9700_android_mdio_read() phy_id=0x%02x, loc=0x%02x, returns=0x%04x",
	       phy_id, loc, le16_to_cpu(res));

	return le16_to_cpu(res);
}

static void sr9700_android_mdio_write(struct net_device *netdev, int phy_id, int loc, int val)
{
	struct usbnet *dev = netdev_priv(netdev);
	__le16 res = cpu_to_le16(val);

	if (phy_id) {
		netdev_dbg(dev->net, "Only internal phy supported");
		return;
	}

	netdev_dbg(dev->net,"sr9700_android_mdio_write() phy_id=0x%02x, loc=0x%02x, val=0x%04x",
	       phy_id, loc, val);

	sr_share_write_word(dev, 1, loc, res);
}

static void sr9700_android_get_drvinfo(struct net_device *net, struct ethtool_drvinfo *info)
{
	usbnet_get_drvinfo(net, info);
	info->eedump_len = SR_EEPROM_LEN;
}

static u32 sr9700_android_get_link(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);
	int rc = 0;
	u8 value = 0;

	sr_read_reg(dev, NSR, &value);
	if(value & NSR_LINKST) {
		rc = 1;
	}

	return rc;
}

static int sr9700_android_ioctl(struct net_device *net, struct ifreq *rq, int cmd)
{
	struct usbnet *dev = netdev_priv(net);

	return generic_mii_ioctl(&dev->mii, if_mii(rq), cmd, NULL);
}

static const struct ethtool_ops sr9700_android_ethtool_ops = {
	.get_drvinfo	= sr9700_android_get_drvinfo,
	.get_link	= sr9700_android_get_link,
	.get_msglevel	= usbnet_get_msglevel,
	.set_msglevel	= usbnet_set_msglevel,
	.get_eeprom_len	= sr9700_android_get_eeprom_len,
	.get_eeprom	= sr9700_android_get_eeprom,
	.get_settings	= usbnet_get_settings,
	.set_settings	= usbnet_set_settings,
	.nway_reset	= usbnet_nway_reset,
};

static void sr9700_android_set_multicast(struct net_device *net)
{
	struct usbnet *dev = netdev_priv(net);
	u8 *hashes = (u8 *) & dev->data;
	u8 rx_ctl = 0x31;

	memset(hashes, 0x00, SR_MCAST_SIZE);
	hashes[SR_MCAST_SIZE - 1] |= 0x80;

        if (net->flags & IFF_PROMISC) {
                rx_ctl |= 0x02;
        } else if (net->flags & IFF_ALLMULTI ||
                   netdev_mc_count(net) > SR_MCAST_MAX) {
                rx_ctl |= 0x04;
        } else if (!netdev_mc_empty(net)) {
                struct netdev_hw_addr *ha;
 
                netdev_for_each_mc_addr(ha, net) {
                        u32 crc = ether_crc(ETH_ALEN, ha->addr) >> 26;
                        hashes[crc >> 3] |= 1 << (crc & 0x7);
                }
        }

	sr_write_async(dev, MAR, SR_MCAST_SIZE, hashes);
	sr_write_reg_async(dev, RCR, rx_ctl);
}

static int sr9700_android_set_mac_address(struct net_device *net, void *p)
{
	struct sockaddr *addr = p;
	struct usbnet *dev = netdev_priv(net);

	if (!is_valid_ether_addr(addr->sa_data)) {
		dev_err(&net->dev, "not setting invalid mac address %pM\n",
								addr->sa_data);
		return -EINVAL;
	}

	memcpy(net->dev_addr, addr->sa_data, net->addr_len);
	sr_write_async(dev, PAR, 6, dev->net->dev_addr);

	return 0;
}

static const struct net_device_ops sr9700_android_netdev_ops = {
	.ndo_open		= usbnet_open,
	.ndo_stop		= usbnet_stop,
	.ndo_start_xmit		= usbnet_start_xmit,
	.ndo_tx_timeout		= usbnet_tx_timeout,
	.ndo_change_mtu		= usbnet_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_do_ioctl 		= sr9700_android_ioctl,
	.ndo_set_rx_mode	= sr9700_android_set_multicast,
	.ndo_set_mac_address	= sr9700_android_set_mac_address,
};

static int sr9700_android_bind(struct usbnet *dev, struct usb_interface *intf)
{
	int ret;

	ret = usbnet_get_endpoints(dev, intf);
	if (ret)
		goto out;

	dev->net->netdev_ops = &sr9700_android_netdev_ops;
	dev->net->ethtool_ops = &sr9700_android_ethtool_ops;
	dev->net->hard_header_len += SR_TX_OVERHEAD;
	dev->hard_mtu = dev->net->mtu + dev->net->hard_header_len;
	dev->rx_urb_size = dev->net->mtu + ETH_HLEN + SR_RX_OVERHEAD;

	dev->mii.dev = dev->net;
	dev->mii.mdio_read = sr9700_android_mdio_read;
	dev->mii.mdio_write = sr9700_android_mdio_write;
	dev->mii.phy_id_mask = 0x1f;
	dev->mii.reg_num_mask = 0x1f;

	sr_write_reg(dev, NCR, 1);
	udelay(20);

	if (sr_read(dev, PAR, ETH_ALEN, dev->net->dev_addr) < 0) {
		printk(KERN_ERR "Error reading MAC address\n");
		ret = -ENODEV;
		goto out;
	}

	sr_write_reg(dev, PRR, 1);
	mdelay(20);
	sr_write_reg(dev, PRR, 0);
	udelay(2 * 1000);

	sr9700_android_set_multicast(dev->net);

	sr9700_android_mdio_write(dev->net, dev->mii.phy_id, MII_BMCR, BMCR_RESET);
	sr9700_android_mdio_write(dev->net, dev->mii.phy_id, MII_ADVERTISE, ADVERTISE_ALL | ADVERTISE_CSMA | ADVERTISE_PAUSE_CAP);
	mii_nway_restart(&dev->mii);

out:
	return ret;
}

static int sr9700_android_rx_fixup(struct usbnet *dev, struct sk_buff *skb)
{
	u8 status;
	int len;

	if (unlikely(skb->len < SR_RX_OVERHEAD)) {
		dev_err(&dev->udev->dev, "unexpected tiny rx frame\n");
		return 0;
	}

	status = skb->data[0];
	len = (skb->data[1] | (skb->data[2] << 8)) - 4;

	if (unlikely(status & 0xbf)) {
		if (status & 0x01) dev->net->stats.rx_fifo_errors++;
		if (status & 0x02) dev->net->stats.rx_crc_errors++;
		if (status & 0x04) dev->net->stats.rx_frame_errors++;
		if (status & 0x20) dev->net->stats.rx_missed_errors++;
		if (status & 0x90) dev->net->stats.rx_length_errors++;
		return 0;
	}

	skb_pull(skb, 3);
	skb_trim(skb, len);

	return 1;
}

static struct sk_buff *sr9700_android_tx_fixup(struct usbnet *dev, struct sk_buff *skb, gfp_t flags)
{
	int len;
	len = skb->len;

	if (skb_headroom(skb) < SR_TX_OVERHEAD) {
		struct sk_buff *skb2;

		skb2 = skb_copy_expand(skb, SR_TX_OVERHEAD, 0, flags);
		dev_kfree_skb_any(skb);
		skb = skb2;
		if (!skb)
			return NULL;
	}

	__skb_push(skb, SR_TX_OVERHEAD);

	if ((skb->len % dev->maxpacket) == 0)
		len++;

	skb->data[0] = len;
	skb->data[1] = len >> 8;

	return skb;
}

static void sr9700_android_status(struct usbnet *dev, struct urb *urb)
{
	int link;
	u8 *buf;

	if (urb->actual_length < 8)
		return;

	buf = urb->transfer_buffer;

	link = !!(buf[0] & 0x40);
	if (netif_carrier_ok(dev->net) != link) {
		if (link) {
			netif_carrier_on(dev->net);
			usbnet_defer_kevent (dev, EVENT_LINK_RESET);
		}
		else
			netif_carrier_off(dev->net);
		netdev_dbg(dev->net, "Link Status is: %d", link);
	}
}

static int sr9700_android_link_reset(struct usbnet *dev)
{
	struct ethtool_cmd ecmd;

	mii_check_media(&dev->mii, 1, 1);
	mii_ethtool_gset(&dev->mii, &ecmd);

	netdev_dbg(dev->net, "link_reset() speed: %d duplex: %d",
	       ecmd.speed, ecmd.duplex);

	return 0;
}

static const struct driver_info sr9700_android_info = {
	.description	= "SR9700_ANDROID USB Ethernet",
	.flags		= FLAG_ETHER,
	.bind		= sr9700_android_bind,
	.rx_fixup	= sr9700_android_rx_fixup,
	.tx_fixup	= sr9700_android_tx_fixup,
	.status		= sr9700_android_status,
	.link_reset	= sr9700_android_link_reset,
	.reset		= sr9700_android_link_reset,
};

static const struct usb_device_id products[] = {
	{
	 USB_DEVICE(0x0fe6, 0x9700),	/* SR9700_ANDROID device */
	 .driver_info = (unsigned long)&sr9700_android_info,
	 },
	{},
};

MODULE_DEVICE_TABLE(usb, products);

static struct usb_driver sr9700_android_driver = {
	.name = "sr9700_android",
	.id_table = products,
	.probe = usbnet_probe,
	.disconnect = usbnet_disconnect,
	.suspend = usbnet_suspend,
	.resume = usbnet_resume,
};

static int __init sr9700_android_init(void)
{
	return usb_register(&sr9700_android_driver);
}

static void __exit sr9700_android_exit(void)
{
	usb_deregister(&sr9700_android_driver);
}

module_init(sr9700_android_init);
module_exit(sr9700_android_exit);

MODULE_AUTHOR("jokeliu <jokeliu@163.com>");
MODULE_DESCRIPTION("SR9700 one chip USB 2.0 ethernet devices on android platform");
MODULE_LICENSE("GPL");