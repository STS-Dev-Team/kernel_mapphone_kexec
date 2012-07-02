/*
 * Copyright (C) 2010 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#ifndef __BMP085_H__
#define __BMP085_H__

#define BMP085_NAME				"bmp085"

#ifdef __KERNEL__
struct bmp085_platform_data {
	int poll_interval;
	int min_interval;
	int max_p;
	int min_p;
	int fuzz;
	int flat;
	char regulator_name[10];
};
#endif /* __KERNEL__ */

#endif  /* __BMP085_H__ */

