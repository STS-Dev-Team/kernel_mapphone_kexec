/* drivers/net/pppoctcn.c
 *
 * Driver for PPP on C+W
 *
 * Copyright (C) 2009 Google, Inc.
 * Author: Tao Jun  <e13875@motorola.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* This driver handles China Unicomm C+W data packets between a UDP
 * socket and a PPP channel.
 * To keep things simple, only one session per socket is permitted.
 * Packets are sent via the socket, so it must keep connected to the
 * same address. Currently this only works on IPv4 due to the lack
 * of UDP encapsulation support in IPv6. */

#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/skbuff.h>
#include <linux/file.h>
#include <linux/netdevice.h>
#include <linux/net.h>
#include <linux/udp.h>
#include <linux/ppp_defs.h>
#include <linux/if_ppp.h>
#include <linux/if_pppox.h>
#include <linux/ppp_channel.h>
#include <net/tcp_states.h>

#define WPDIF_PROTOCOL_ID               0xC5
#define WPDIF_PROTOCOL_VERSION     0x01
#define WPDIF_PROTOCOL_RESERVED   0x0000

#define SKB_RESERVE_LENGTH       32
#define CW_TUNNEL_HEADER_LENGTH  8

#define PPP_ADDR	0xFF
#define PPP_CTRL	0x03

union unaligned {
	__u32 u32;
} __attribute__((packed));

static inline union unaligned *unaligned(void *ptr)
{
	return (union unaligned *)ptr;
}

/* see how many ordinary chars there are at the start of buf */
static inline int
scan_ordinary(unsigned int raccm, const unsigned char *buf, int count)
{
	int i, c;

	for (i = 0; i < count; ++i) {
		c = buf[i];
		if (c == PPP_ESCAPE || c == PPP_FLAG
				|| (c < 0x20 && (raccm & (1 << c)) != 0))
			break;
	}
	return i;
}

static inline struct sk_buff *pppoctcn_accm(const unsigned char *buf,
	int count, unsigned int raccm)
{
	int n;
	int c;
	struct sk_buff *skb;
	unsigned char *sp;
	bool isEscape = false;

	skb = dev_alloc_skb(count + SKB_RESERVE_LENGTH);
	if (!skb)
		goto nomem;

	if (count > 0 && buf[0] == PPP_FLAG) {
		buf++;
		count--;
	}

	while (count > 0) {
		n = scan_ordinary(raccm, buf, count);

		if (n > 0) {
			if (n > skb_tailroom(skb)) {
				/* packet overflowed MRU*/
				printk(KERN_ERR "packet overflowed MRU\n");
				kfree_skb(skb);
				goto nomem;
			} else {
				sp = skb_put(skb, n);
				memcpy(sp, buf, n);
				if (isEscape) {
					sp[0] ^= 0x20;
					isEscape = false;
				}
			}
		}

		if (n >= count)
			break;

		c = buf[n];

		if (c ==  PPP_FLAG) {
			/*end of ppp*/
			/*check fcs*/
			unsigned int fcs, len;
			unsigned char *p;
			p = skb->data;
			len = skb->len;
			fcs = PPP_INITFCS;
			for (; len > 0; --len)
				fcs = PPP_FCS(fcs, *p++);
			if (fcs != PPP_GOODFCS) {
				printk(KERN_ERR "wrong fcs for incoming packets\n");
				kfree_skb(skb);
				return NULL;
			}
			skb_trim(skb, skb->len - 2);
			return skb;
		} else if (c == PPP_ESCAPE) {
			isEscape = true;
		}

		++n;

		buf += n;
		count -= n;
	}
	kfree_skb(skb);
	return NULL;
nomem:
	printk(KERN_ERR "pppoctcn_accm: no memory (input pkt)\n");
	return NULL;
}

/****************************************************************
CT's tunnel header.
The first 8 bytes is UDP header.
The last 8 bytes is tunnel header

Bit0          15 16         Bit31
--------------------------------
Source Port     |   Dest Port
--------------------------------
Pkt Length      |   Checksum
--------------------------------
ProtoID|ProtoVer|   Reserved
--------------------------------
			Stream ID
--------------------------------

*****************************************************************/
static int pppoctcn_recv_core(struct sock *sk_udp, struct sk_buff *skb)
{
	struct sock *sk = (struct sock *)sk_udp->sk_user_data;
	struct pppoctcn_opt *opt = &pppox_sk(sk)->proto.ctcn;
	struct sk_buff *newskb;
	__u8 *ptr;

	/* FIXME jfx486
	 * If the packet is not for ppp, like UDP_KEEPALIVE
	 * or timeout or 200OK then we should use the orignal receiver
	 * Please refer to pppolac.c for details
	 */
	/* Put it back if it is a udp keep alive packet.
	if (skb->data[sizeof(struct udphdr)] & not_define_PPP_KEEPALIVE)
		return opt->backlog_rcv(sk_udp, skb); */

	/* Drop the packet if it is too short. */
	if (skb->len < sizeof(struct udphdr) + CW_TUNNEL_HEADER_LENGTH)
		goto drop;

	/* workaround linearize sk_buff if fragments still exist */
	if (skb->data_len)
		if (skb_linearize(skb) < 0)
			goto drop;

	/* Skip UDP header. */
	skb_pull(skb, sizeof(struct udphdr));

	/* Check the protocol id. */
	if (skb->data[0]  != WPDIF_PROTOCOL_ID)
		goto drop;

	/* Check the version. */
	if (skb->data[1]  != WPDIF_PROTOCOL_VERSION)
		goto drop;

	ptr = &skb->data[4];

	/* Skip tunnel header. */
	skb_pull(skb, 8);

	/* Check the stream id*/
	if (unaligned(ptr)->u32 != opt->streamid)
		goto drop;

	newskb = pppoctcn_accm(skb->data, skb->len, opt->raccm);
	if (newskb == NULL)
		goto drop;

	/*release old skb and use new one*/
	kfree_skb(skb);
	skb = newskb;

	/* Skip PPP address and control if they are present. */
	if (skb->len >= 2 && skb->data[0] == PPP_ADDR &&
			skb->data[1] == PPP_CTRL){
		skb_pull(skb, 2);
	}

	/* Fix PPP protocol if it is compressed. */
	if (skb->len >= 1 && skb->data[0] & 1)
		skb_push(skb, 1)[0] = 0;

	/* Finally, deliver the packet to PPP channel. */
	skb_orphan(skb);
	ppp_input(&pppox_sk(sk)->chan, skb);
	return NET_RX_SUCCESS;

drop:
	kfree_skb(skb);
	return NET_RX_DROP;
}

static int pppoctcn_recv(struct sock *sk_udp, struct sk_buff *skb)
{
	sock_hold(sk_udp);
	sk_receive_skb(sk_udp, skb, 0);
	return 0;
}

static struct sk_buff_head delivery_queue;

static void pppoctcn_xmit_core(struct work_struct *delivery_work)
{
	mm_segment_t old_fs = get_fs();
	struct sk_buff *skb;

	set_fs(KERNEL_DS);
	while ((skb = skb_dequeue(&delivery_queue))) {
		struct sock *sk_udp = skb->sk;
		struct kvec iov = {.iov_base = skb->data, .iov_len = skb->len};
		struct msghdr msg = {
			.msg_iov = (struct iovec *)&iov,
			.msg_iovlen = 1,
			.msg_flags = MSG_NOSIGNAL | MSG_DONTWAIT,
		};
		sk_udp->sk_prot->sendmsg(NULL, sk_udp, &msg, skb->len);
		kfree_skb(skb);
	}
	set_fs(old_fs);
}

static DECLARE_WORK(delivery_work, pppoctcn_xmit_core);

#define PUT_BYTE(buf, c, xaccm, count, islcp) do { \
	if ((islcp && c < 0x20) || (xaccm[c >> 5] & (1 << (c & 0x1f)))) { \
		*buf++ = PPP_ESCAPE;  \
		*buf++ = c ^ 0x20;    \
		count += 2;           \
	} else{                   \
		*buf++ = c;           \
		count++ ;            \
	}                         \
} while (0)

static int pppoctcn_xmit(struct ppp_channel *chan, struct sk_buff *skb)
{
	struct sock *sk_udp = (struct sock *)chan->private;
	struct pppoctcn_opt *opt = &pppox_sk(sk_udp->sk_user_data)->proto.ctcn;

	unsigned char *outbuf;
	unsigned char *buf;
	unsigned char c;
	struct sk_buff *newskb;
	int i, proto, islcp;
	int count = 0;
	int fcs = PPP_INITFCS;
	unsigned int xaccm[8];

	int buflen = skb->len * 2 + SKB_RESERVE_LENGTH;
	newskb = dev_alloc_skb(buflen);
	if (!newskb) {
		kfree_skb(skb);
		printk(KERN_ERR "pppoctcn_xmit: no memory\n");
		return 1;
	}
	outbuf = newskb->data;
	buf = outbuf + CW_TUNNEL_HEADER_LENGTH;

	xaccm[0] = opt->saccm;
	xaccm[1] = 0;
	xaccm[2] = 0;
	xaccm[3] = 0x60000000; /* must escape 0x7d, 0x7e */
	xaccm[4] = 0;
	xaccm[5] = 0;
	xaccm[6] = 0;
	xaccm[7] = 0;

	proto = (skb->data[0] << 8) + skb->data[1];
	islcp = proto == PPP_LCP && 1 <= skb->data[2] && skb->data[2] <= 7;

	/*add PPP_FLAG*/
	*buf++ = PPP_FLAG;
	count++;

	/*add PPP_ADDR PPP_CTRL*/
	PUT_BYTE(buf, PPP_ADDR, xaccm, count, islcp);
	fcs = PPP_FCS(fcs, PPP_ADDR);
	PUT_BYTE(buf, PPP_CTRL, xaccm, count, islcp);
	fcs = PPP_FCS(fcs, PPP_CTRL);

	for (i = 0 ; i < skb->len; i++) {
		PUT_BYTE(buf, skb->data[i], xaccm, count, islcp);
		fcs = PPP_FCS(fcs, skb->data[i]);
	}
	/* We have finished the packet.  Add the FCS and flag.*/
	fcs = ~fcs;
	c = fcs & 0xff;
	PUT_BYTE(buf, c, xaccm, count, islcp);
	c = (fcs >> 8) & 0xff;
	PUT_BYTE(buf, c, xaccm, count, islcp);
	*buf++ = PPP_FLAG;
	count++;

	/* release old skb;*/
	kfree_skb(skb);

	/* Install wPDIF header. */
	count += CW_TUNNEL_HEADER_LENGTH;
	outbuf[0] = WPDIF_PROTOCOL_ID;
	outbuf[1] = WPDIF_PROTOCOL_VERSION;
	outbuf[2] = 0;
	outbuf[3] = 0;
	outbuf[4] = (opt->streamid) & 0xFF;
	outbuf[5] = (opt->streamid  >> 8) & 0xFF;
	outbuf[6] = (opt->streamid  >> 16) & 0xFF;
	outbuf[7] = (opt->streamid  >> 24) & 0xFF;

	skb_put(newskb, count);

	/* Now send the packet via the delivery queue. */
	skb_set_owner_w(newskb, sk_udp);
	skb_queue_tail(&delivery_queue, newskb);
	schedule_work(&delivery_work);
	return 1;
}

static int pppoctcn_ioctl(struct ppp_channel *sock, unsigned int cmd,
		unsigned long arg)
{
	struct sock *sk_udp = (struct sock *)sock->private;
	struct pppoctcn_opt *opt = &pppox_sk(sk_udp->sk_user_data)->proto.ctcn;

	switch (cmd) {
	case PPPIOCSASYNCMAP: /*set send ACCM*/
		get_user(opt->saccm , (u32 __user *)arg);
		break;
	case PPPIOCSRASYNCMAP:/*set recevie ACCM*/
		get_user(opt->raccm , (u32 __user *)arg);
		break;
	}
	return 0;
}

/******************************************************************************/

static struct ppp_channel_ops pppoctcn_channel_ops = {
	.start_xmit = pppoctcn_xmit,
	.ioctl = pppoctcn_ioctl,
};

static int pppoctcn_connect(struct socket *sock, struct sockaddr *useraddr,
	int addrlen, int flags)
{
	struct sock *sk = sock->sk;
	struct pppox_sock *po = pppox_sk(sk);
	struct sockaddr_pppoctcn *addr = (struct sockaddr_pppoctcn *)useraddr;
	struct socket *sock_udp = NULL;
	struct sock *sk_udp = NULL;
	int error;

	if (addrlen != sizeof(struct sockaddr_pppoctcn))
		return -EINVAL;

	lock_sock(sk);
	error = -EALREADY;
	if (sk->sk_state != PPPOX_NONE)
		goto out;

	sock_udp = sockfd_lookup(addr->udp_socket, &error);
	if (!sock_udp)
		goto out;
	sk_udp = sock_udp->sk;
	lock_sock(sk_udp);

	/* Remove this check when IPv6 supports UDP encapsulation. */
	error = -EAFNOSUPPORT;
	if (sk_udp->sk_family != AF_INET)
		goto out;
	error = -EPROTONOSUPPORT;
	if (sk_udp->sk_protocol != IPPROTO_UDP)
		goto out;
	error = -EDESTADDRREQ;
	if (sk_udp->sk_state != TCP_ESTABLISHED)
		goto out;
	error = -EBUSY;
	if (udp_sk(sk_udp)->encap_type || sk_udp->sk_user_data)
		goto out;
	if (!sk_udp->sk_bound_dev_if) {
		struct dst_entry *dst = sk_dst_get(sk_udp);
		error = -ENODEV;
		if (!dst)
			goto out;
		sk_udp->sk_bound_dev_if = dst->dev->ifindex;
		dst_release(dst);
	}

	po->chan.hdrlen = 10;
	po->chan.private = sk_udp;
	po->chan.ops = &pppoctcn_channel_ops;
	po->chan.mtu = PPP_MTU - 100;
	po->proto.ctcn.streamid = ntohl(addr->streamid);
	po->proto.ctcn.backlog_rcv = sk_udp->sk_backlog_rcv;

	error = ppp_register_channel(&po->chan);
	if (error)
		goto out;

	sk->sk_state = PPPOX_CONNECTED;
	udp_sk(sk_udp)->encap_type = UDP_ENCAP_CTCNINUDP;
	udp_sk(sk_udp)->encap_rcv = pppoctcn_recv;
	sk_udp->sk_backlog_rcv = pppoctcn_recv_core;
	sk_udp->sk_user_data = sk;

out:
	if (sock_udp) {
		release_sock(sk_udp);
		if (error)
			sockfd_put(sock_udp);
	}
	release_sock(sk);
	return error;
}

static int pppoctcn_release(struct socket *sock)
{
	struct sock *sk = sock->sk;

	if (!sk)
		return 0;

	lock_sock(sk);
	if (sock_flag(sk, SOCK_DEAD)) {
		release_sock(sk);
		return -EBADF;
	}

	if (sk->sk_state != PPPOX_NONE) {
		struct sock *sk_udp = (struct sock *)pppox_sk(sk)->chan.private;
		lock_sock(sk_udp);
		pppox_unbind_sock(sk);
		sk_udp->sk_user_data = NULL;
		udp_sk(sk_udp)->encap_type = 0;
		udp_sk(sk_udp)->encap_rcv = NULL;
		sk_udp->sk_backlog_rcv = pppox_sk(sk)->proto.ctcn.backlog_rcv;
		sk_udp->sk_user_data = NULL;
		release_sock(sk_udp);
		sockfd_put(sk_udp->sk_socket);
	}

	sock_orphan(sk);
	sock->sk = NULL;
	release_sock(sk);
	sock_put(sk);
	return 0;
}

/******************************************************************************/

static struct proto pppoctcn_proto = {
	.name = "PPPOCTCN",
	.owner = THIS_MODULE,
	.obj_size = sizeof(struct pppox_sock),
};

static const struct proto_ops pppoctcn_proto_ops = {
	.family = PF_PPPOX,
	.owner = THIS_MODULE,
	.release = pppoctcn_release,
	.bind = sock_no_bind,
	.connect = pppoctcn_connect,
	.socketpair = sock_no_socketpair,
	.accept = sock_no_accept,
	.getname = sock_no_getname,
	.poll = sock_no_poll,
	.ioctl = pppox_ioctl,
	.listen = sock_no_listen,
	.shutdown = sock_no_shutdown,
	.setsockopt = sock_no_setsockopt,
	.getsockopt = sock_no_getsockopt,
	.sendmsg = sock_no_sendmsg,
	.recvmsg = sock_no_recvmsg,
	.mmap = sock_no_mmap,
};

static int pppoctcn_create(struct net *net, struct socket *sock)
{
	struct sock *sk;

	sk = sk_alloc(net, PF_PPPOX, GFP_KERNEL, &pppoctcn_proto);
	if (!sk)
		return -ENOMEM;

	sock_init_data(sock, sk);
	sock->state = SS_UNCONNECTED;
	sock->ops = &pppoctcn_proto_ops;
	sk->sk_protocol = PX_PROTO_OCTCN;
	sk->sk_state = PPPOX_NONE;
	return 0;
}

/******************************************************************************/

static struct pppox_proto pppoctcn_pppox_proto = {
	.create = pppoctcn_create,
	.owner = THIS_MODULE,
};

static int __init pppoctcn_init(void)
{
	int error;

	error = proto_register(&pppoctcn_proto, 0);
	if (error)
		return error;

	error = register_pppox_proto(PX_PROTO_OCTCN, &pppoctcn_pppox_proto);
	if (error)
		proto_unregister(&pppoctcn_proto);
	else
		skb_queue_head_init(&delivery_queue);
	return error;
}

static void __exit pppoctcn_exit(void)
{
	unregister_pppox_proto(PX_PROTO_OCTCN);
	proto_unregister(&pppoctcn_proto);
}

module_init(pppoctcn_init);
module_exit(pppoctcn_exit);

MODULE_DESCRIPTION("PPP on C+W");
MODULE_AUTHOR("Tao Jun <e13875@motorola.com>");
MODULE_LICENSE("GPL");
