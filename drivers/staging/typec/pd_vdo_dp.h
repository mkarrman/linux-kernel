/*
 * Copyright 2015-2016 Google, Inc
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

#define CMD_DP_STATUS		16
#define CMD_DP_CONFIG		17

/*
 * DisplayPort modes capabilities
 * -------------------------------
 * <31:24> : SBZ
 * <23:16> : UFP_D pin assignment supported
 * <15:8>  : DFP_D pin assignment supported
 * <7>     : USB 2.0 signaling (0b=yes, 1b=no)
 * <6>     : Plug | Receptacle (0b == plug, 1b == receptacle)
 * <5:2>   : xxx1: Supports DPv1.3, xx1x Supports USB Gen 2 signaling
 *           Other bits are reserved.
 * <1:0>   : signal direction ( 00b=rsv, 01b=sink, 10b=src 11b=both )
 */
#define VDO_MODE_DP(snkp, srcp, usb, gdr, sign, sdir)			\
	(((snkp) & 0xff) << 16 | ((srcp) & 0xff) << 8			\
	 | ((usb) & 1) << 7 | ((gdr) & 1) << 6 | ((sign) & 0xF) << 2	\
	 | ((sdir) & 0x3))
#define PD_DP_PIN_CAPS(x) ((((x) >> 6) & 0x1) ? (((x) >> 16) & 0x3f)	\
			  : (((x) >> 8) & 0x3f))

#define PD_DP_UFP_D_PIN(x)	(((x) >> 16) & 0xff)
#define PD_DP_DFP_D_PIN(x)	(((x) >> 8) & 0xff)
#define PD_DP_USB20_NOT_USED(x)	(((x) >> 7) & 1)
#define PD_DP_RECEPTACLE(x)	(((x) >> 6) & 1)
#define PD_DP_SIGNAL(x)		(((x) >> 2) & 0xf)
#define PD_DP_PORT_CAP(x)	((x) & 0x3)

#define MODE_DP_PIN_A		0x01
#define MODE_DP_PIN_B		0x02
#define MODE_DP_PIN_C		0x04
#define MODE_DP_PIN_D		0x08
#define MODE_DP_PIN_E		0x10
#define MODE_DP_PIN_F		0x20

/* Pin configs B/D/F support multi-function */
#define MODE_DP_PIN_MF_MASK	0x2a
/* Pin configs A/B support BR2 signaling levels */
#define MODE_DP_PIN_BR2_MASK	0x3
/* Pin configs C/D/E/F support DP signaling levels */
#define MODE_DP_PIN_DP_MASK	0x3c

#define MODE_DP_V13		0x1
#define MODE_DP_GEN2		0x2

#define MODE_DP_SNK		0x1
#define MODE_DP_SRC		0x2
#define MODE_DP_BOTH		0x3

/*
 * DisplayPort Status VDO
 * ----------------------
 * <31:9> : SBZ
 * <8>    : IRQ_HPD : 1 == irq arrived since last message otherwise 0.
 * <7>    : HPD state : 0 = HPD_LOW, 1 == HPD_HIGH
 * <6>    : Exit DP Alt mode: 0 == maintain, 1 == exit
 * <5>    : USB config : 0 == maintain current, 1 == switch to USB from DP
 * <4>    : Multi-function preference : 0 == no pref, 1 == MF preferred.
 * <3>    : enabled : is DPout on/off.
 * <2>    : power low : 0 == normal or LPM disabled, 1 == DP disabled for LPM
 * <1:0>  : connect status : 00b ==  no (DFP|UFP)_D is connected or disabled.
 *          01b == DFP_D connected, 10b == UFP_D connected, 11b == both.
 */
#define VDO_DP_STATUS(irq, lvl, amode, usbc, mf, en, lp, conn)		\
	(((irq) & 1) << 8 | ((lvl) & 1) << 7 | ((amode) & 1) << 6	\
	 | ((usbc) & 1) << 5 | ((mf) & 1) << 4 | ((en) & 1) << 3	\
	 | ((lp) & 1) << 2 | ((conn & 0x3) << 0))

#define PD_VDO_DPSTS_HPD_IRQ(x)		(((x) >> 8) & 1)
#define PD_VDO_DPSTS_HPD_LVL(x)		(((x) >> 7) & 1)
#define PD_VDO_DPSTS_MF_PREF(x)		(((x) >> 4) & 1)

/* Per DisplayPort Spec v1.3 Section 3.3, in uS */
#define HPD_USTREAM_DEBOUNCE_LVL	2000
#define HPD_USTREAM_DEBOUNCE_IRQ	250
#define HPD_DSTREAM_DEBOUNCE_IRQ	750	/* between 500-1000us */

/*
 * DisplayPort Configure VDO
 * -------------------------
 * <31:24> : SBZ
 * <23:16> : SBZ
 * <15:8>  : Pin assignment requested.  Choose one from mode caps.
 * <7:6>   : SBZ
 * <5:2>   : signalling : 1h == DP v1.3, 2h == Gen 2
 *           Oh is only for USB, remaining values are reserved
 * <1:0>   : cfg : 00 == USB, 01 == DFP_D, 10 == UFP_D, 11 == reserved
 */
#define VDO_DP_CFG(pin, sig, cfg) \
	(((pin) & 0xff) << 8 | ((sig) & 0xf) << 2 | ((cfg) & 0x3))

#define PD_DP_CFG_DPON(x)	((((x) & 0x3) == 1) || (((x) & 0x3) == 2))
/*
 * Get the pin assignment mask
 * for backward compatibility, if it is null,
 * get the former sink pin assignment we used to be in <23:16>.
 */
#define PD_DP_CFG_PIN(x) ((((x) >> 8) & 0xff) ? (((x) >> 8) & 0xff) \
					      : (((x) >> 16) & 0xff))

/* USB-IF SIDs */
#define USB_SID_PD		0xff00	/* power delivery */
#define USB_SID_DISPLAYPORT	0xff01
#define USB_SID_MHL		0xff02	/* Mobile High-Definition Link */

#endif /* __LINUX_USB_PD_VDO_DP_H */
