/* include/linux/if_pppoctcn.h
 *
 * Header for PPP over China Telecomm C+W
 *
 * Copyright (C) 2009 Google, Inc.
 * Author: Chia-chi Yeh <chiachi@android.com>
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

#ifndef __LINUX_IF_PPPOCTCN_H
#define __LINUX_IF_PPPOCTCN_H

#include <linux/socket.h>
#include <linux/types.h>

struct sockaddr_pppoctcn {
	sa_family_t	sa_family;	/* AF_PPPOX */
	unsigned int	sa_protocol;	/* PX_PROTO_OLAC */
	int		udp_socket;
	__u32	streamid;
} __attribute__((packed));

#endif /* __LINUX_IF_PPPOCTCN_H */
