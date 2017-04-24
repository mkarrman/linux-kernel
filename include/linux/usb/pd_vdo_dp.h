 /*
 * Copyright 2017 Mats Karrman
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __LINUX_USB_PD_VDO_DP_H
#define __LINUX_USB_PD_VDO_DP_H

#include <linux/usb/pd_vdo.h>

#define USB_SID_DISPLAYPORT	0xff01		/* Here or in pd_vdo.h? */

/*
 * DisplayPort Mode request -> response
 *
 * Request is simply properly formatted SVDM header
 *
 * Response is 1-6 data objects (one per mode):
 * [0] :: SVDM header
 * [1] :: First mode
 * [2] :: Second mode
 * [3] :: ...
 *
 */

#define VDO_DPM(ufpd, dfpd, nusb20, rec, sign, cap)	\
	(((ufpd) << 16) | ((dfpd) << 8) |		\
	 ((nusb20) << 7) | ((rec) << 6) |		\
	 ((sign) << 2) | (cap))

#define PD_DPM_UFP_D_PIN(vdo)	(((vdo) >> 16) & 0xff)
#define PD_DPM_DFP_D_PIN(vdo)	(((vdo) >> 8) & 0xff)
#define PD_DPM_SIGN(vdo)	(((vdo) >> 2) & 0xf)
#define PD_DPM_CAP(vdo)		((vdo) & 0x3)

#define DPM_RECEPTACLE		(1 << 6)
#define DPM_USB20_NOT_USED	(1 << 7)

#define DPM_UFP_D_PIN_A		0x01
#define DPM_UFP_D_PIN_B		0x02
#define DPM_UFP_D_PIN_C		0x04
#define DPM_UFP_D_PIN_D		0x08
#define DPM_UFP_D_PIN_E		0x10

#define DPM_DFP_D_PIN_A		0x01
#define DPM_DFP_D_PIN_B		0x02
#define DPM_DFP_D_PIN_C		0x04
#define DPM_DFP_D_PIN_D		0x08
#define DPM_DFP_D_PIN_E		0x10
#define DPM_DFP_D_PIN_F		0x20

#define DPM_SIGN_SUPP_DP13      0x1
#define DPM_SIGN_SUPP_USB_GEN2  0x2

#define DPM_PORT_CAP_UFP_D	1
#define DPM_PORT_CAP_DFP_D	2
#define DPM_PORT_CAP_DFP_UFP_D  3

#endif /* __LINUX_USB_PD_VDO_DP_H */

