/*
 * Copyright 2017  Mats Karrman <mats@southpole.se>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * USB Type-C Port Manager test device.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/usb/pd.h>
#include <linux/usb/pd_vdo.h>
#include <linux/usb/pd_vdo_dp.h>
#include <linux/usb/typec.h>

#include "tcpm.h"

#define PD_RETRY_COUNT 3

enum {
	TESTMODE_NONE = 0,
	TESTMODE_SNK,
	TESTMODE_SRC,
};

struct tcpmtest_data {
	struct device *dev;
	struct i2c_client *client;
	struct tcpm_port *port;
	struct tcpc_dev tcpc;

	bool controls_vbus;

	enum typec_cc_status cc1_status;
	enum typec_cc_status cc2_status;
	bool vbus_present;

	enum typec_cc_polarity polarity;
	bool vconn_enable;
	bool pd_rx_enable;
	enum typec_data_role data_role;
	enum typec_role power_role;

	// Simulator internals
	
	struct task_struct *thread;
	struct mutex lock;
	union {
		unsigned int word;
		struct {
			unsigned int mode_set:1;
			unsigned int msg_rx:1;
			unsigned int msg_tx:1;
			unsigned int vbus_chng:1;
		};
	} request;

	enum tcpm_transmit_type tx_type;
	struct pd_message tx_msg;
	struct pd_message rx_msg;
        unsigned int rx_id;

	enum pd_pdo_type src_cap_pdo_type[PDO_MAX_OBJECTS];

	struct {
		unsigned int mode;
	} sysfs;
};

static inline struct tcpmtest_data *tcpc_to_tcpmtest_data(struct tcpc_dev *tcpc)
{
	return container_of(tcpc, struct tcpmtest_data, tcpc);
}


/*************** Message log parsers *****************************************/

static const char *tcpmtest_cc2str(enum typec_cc_status cc)
{
	const char *cc_str;

	switch (cc) {
	case TYPEC_CC_RA: cc_str = "RA"; break;
	case TYPEC_CC_RD: cc_str = "RD"; break;
	case TYPEC_CC_RP_DEF: cc_str = "RP_DEF"; break;
	case TYPEC_CC_RP_1_5: cc_str = "RP_1_5"; break;
	case TYPEC_CC_RP_3_0: cc_str = "RP_3_0"; break;
	case TYPEC_CC_OPEN: cc_str = "OPEN"; break;
	default: cc_str = "<undefined>"; break;
	}

	return cc_str;
}

static const char *tcpmtest_txtype2str(enum tcpm_transmit_type type)
{
	const char *tx_type_str;

	switch (type) {
	case TCPC_TX_SOP: tx_type_str = "SOP"; break;
	case TCPC_TX_SOP_PRIME: tx_type_str = "SOP'"; break;
	case TCPC_TX_SOP_PRIME_PRIME: tx_type_str = "SOP''"; break;
	case TCPC_TX_SOP_DEBUG_PRIME: tx_type_str = "DEBUG'"; break;
	case TCPC_TX_SOP_DEBUG_PRIME_PRIME: tx_type_str = "DEBUG''"; break;
	case TCPC_TX_HARD_RESET: tx_type_str = "HARD_RESET"; break;
	case TCPC_TX_CABLE_RESET: tx_type_str = "CABLE_RESET"; break;
	case TCPC_TX_BIST_MODE_2: tx_type_str = "BIST_MODE_2"; break;
	default: tx_type_str = "<undefined>"; break;
	}

	return tx_type_str;
};

static const char *tcpmtest_header2typestr(u16 header)
{
	unsigned int cnt = pd_header_cnt(header);
	const char *type_str;

	if (!cnt) {
		/* Control message */
		switch (pd_header_type(header)) {
		case PD_CTRL_GOOD_CRC: type_str = "GOOD_CRC"; break;
		case PD_CTRL_GOTO_MIN: type_str = "GOTO_MIN"; break;
		case PD_CTRL_ACCEPT: type_str = "ACCEPT"; break;
		case PD_CTRL_REJECT: type_str = "REJECT"; break;
		case PD_CTRL_PING: type_str = "PING"; break;
		case PD_CTRL_PS_RDY: type_str = "PS_RDY"; break;
		case PD_CTRL_GET_SOURCE_CAP: type_str = "GET_SOURCE_CAP"; break;
		case PD_CTRL_GET_SINK_CAP: type_str = "GET_SINK_CAP"; break;
		case PD_CTRL_DR_SWAP: type_str = "DR_SWAP"; break;
		case PD_CTRL_PR_SWAP: type_str = "PR_SWAP"; break;
		case PD_CTRL_VCONN_SWAP: type_str = "VCONN_SWAP"; break;
		case PD_CTRL_WAIT: type_str = "WAIT"; break;
		case PD_CTRL_SOFT_RESET: type_str = "SOFT_RESET"; break;
		default: type_str = "<ctrl-rsvd>"; break;
		}
	} else {
		/* Data message */
		switch (pd_header_type(header)) {
		case PD_DATA_SOURCE_CAP: type_str = "SOURCE_CAP"; break;
		case PD_DATA_REQUEST: type_str = "REQUEST"; break;
		case PD_DATA_BIST: type_str = "BIST"; break;
		case PD_DATA_SINK_CAP: type_str = "SINK_CAP"; break;
		case PD_DATA_VENDOR_DEF: type_str = "VDM"; break;
		default: type_str = "<data-rsvd>"; break;
		}
	}

	return type_str;
}

static void tcpmtest_log_dobj_hex(struct tcpmtest_data *data, u32 dobj)
{
	dev_info(data->dev, "- 0x%08x\n", dobj);
}

static void tcpmtest_log_capability_pdo(struct tcpmtest_data *data,
					u32 pdo, bool src, unsigned int index)
{
	enum pd_pdo_type type = pdo_type(pdo);
	unsigned int drp, ss, ep, cc, drd, pc, volt, volt2, curr, pwr;

	data->src_cap_pdo_type[index] = type;
	switch (type) {
	case PDO_TYPE_FIXED:
		data->src_cap_pdo_type[index] = type;
		pc = (pdo >> 20) & 0x3;
		volt = pdo_fixed_voltage(pdo);
		curr = ((pdo >> PDO_FIXED_CURR_SHIFT) & PDO_CURR_MASK) * 10;
		if (index == 0) {
			drp = (pdo & PDO_FIXED_DUAL_ROLE) ? 1 : 0;
			ss = (pdo & PDO_FIXED_SUSPEND) ? 1 : 0;
			ep = (pdo & PDO_FIXED_EXTPOWER) ? 1 : 0;
			cc = (pdo & PDO_FIXED_USB_COMM) ? 1 : 0;
			drd = (pdo & PDO_FIXED_DATA_SWAP) ? 1 : 0;
			if (src)
				dev_info(data->dev,
					 "- FIX:DRP=%u:SS=%u:EP=%u:CC=%u"
					 ":DRD=%u:PC=%u:%umV:%umA\n",
					 drp, ss, ep, cc, drd, pc, volt, curr);
			else /* snk */
				dev_info(data->dev,
					 "- FIX:DRP=%u:HC=%u:EP=%u:CC=%u"
					 ":DRD=%u:%umV:%umA\n",
					 drp, ss, ep, cc, drd, volt, curr);
		} else {
			if (src)
				dev_info(data->dev,
					 "- FIX:PC=%u:%umV:%umA\n",
					 pc, volt, curr);
			else /* snk */
				dev_info(data->dev,
					 "- FIX:%umV:%umA\n",
					 volt, curr);
		}
		break;

	case PDO_TYPE_BATT:
		volt = pdo_max_voltage(pdo);
		volt2 = pdo_min_voltage(pdo);
		pwr = pdo_max_power(pdo);
		dev_info(data->dev,
			 "- BAT:%umV:%umV:%umW\n", volt, volt2, pwr);
		break;

	case PDO_TYPE_VAR:
		volt = pdo_max_voltage(pdo);
		volt2 = pdo_min_voltage(pdo);
		curr = pdo_max_current(pdo);
		dev_info(data->dev,
			 "- VAR:%umV:%umV:%umA\n", volt, volt2, curr);
		break;

	default:
		tcpmtest_log_dobj_hex(data, pdo);
		break;
	}
}

static void tcpmtest_log_request_pdo(struct tcpmtest_data *data, u32 rdo)
{
	unsigned int index = rdo_index(rdo) - 1;
	enum pd_pdo_type type = data->src_cap_pdo_type[index];
	unsigned int gb, cm, cc, ns, curr1, curr2, pow1, pow2;

	switch (type) {
	case PDO_TYPE_FIXED:
	case PDO_TYPE_VAR:
		gb = (rdo & RDO_GIVE_BACK) ? 1 : 0;
		cm = (rdo & RDO_CAP_MISMATCH) ? 1 : 0;
		cc = (rdo & RDO_USB_COMM) ? 1 : 0;
		ns = (rdo & RDO_NO_SUSPEND) ? 1 : 0;
		curr1 = rdo_op_current(rdo);
		curr2 = rdo_max_current(rdo);
		dev_info(data->dev, "- %s:GB=%u:CM=%u:CC=%u:NS=%u:%umA:%umA\n",
			(type == PDO_TYPE_FIXED) ? "FIX" : "VAR",
			gb, cm, cc, ns, curr1, curr2);
		break;

	case PDO_TYPE_BATT:
		gb = (rdo & RDO_GIVE_BACK) ? 1 : 0;
		cm = (rdo & RDO_CAP_MISMATCH) ? 1 : 0;
		cc = (rdo & RDO_USB_COMM) ? 1 : 0;
		ns = (rdo & RDO_NO_SUSPEND) ? 1 : 0;
		pow1 = rdo_op_power(rdo);
		pow2 = rdo_max_power(rdo);
		dev_info(data->dev, "- BAT:GB=%u:CM=%u:CC=%u:NS=%u:%umW:%umW\n",
			gb, cm, cc, ns, pow1, pow2);
		break;

	default:
		tcpmtest_log_dobj_hex(data, rdo);
		break;
	}
}

static void tcpmtest_log_vdo_header(struct tcpmtest_data *data, u32 header)
{
	unsigned int svid, ver, obj_pos;
	const char *cmdt, *cmd;

	svid = PD_VDO_VID(header);
	ver = PD_VDO_VER(header);
	obj_pos = PD_VDO_OPOS(header);

	switch (PD_VDO_CMDT(header)) {
	case CMDT_INIT:     cmdt = "INIT"; break;
	case CMDT_RSP_ACK:  cmdt = "ACK";  break;
	case CMDT_RSP_NAK:  cmdt = "NAK";  break;
	case CMDT_RSP_BUSY: cmdt = "BUSY"; break;
	}

	switch (PD_VDO_CMD(header)) {
	case CMD_DISCOVER_IDENT: cmd = "Disc.Ident";  break;
	case CMD_DISCOVER_SVID:  cmd = "Disc.SVIDs";  break;
	case CMD_DISCOVER_MODES: cmd = "Disc.Modes";  break;
	case CMD_ENTER_MODE:     cmd = "Enter.Mode";  break;
	case CMD_EXIT_MODE:      cmd = "Exit.Mode";   break;
	case CMD_ATTENTION:      cmd = "Attention";   break;
	case CMD_DP_STATUS:      cmd = "DP.Status";   break;
	case CMD_DP_CONFIG:      cmd = "DP.Config";   break;
	default:                 cmd = "<undefined>"; break;
	}

	dev_info(data->dev, "- SVID=0x%04x:S=1:V=%u:OP=%u:CT=%s:C=%s\n",
		 svid, ver, obj_pos, cmdt, cmd);
}

static void tcpmtest_log_vdo(struct tcpmtest_data *data,
			     u32 vdo, unsigned int index, u32 *header)
{
	u32 vdo_hdr;
	u16 svid;
	const char *ptype;
	unsigned int vcpwr;
	const char *sssup;
	const char *pcap;

	if (index == VDO_INDEX_HDR)
		*header = vdo_hdr = vdo;
	else
		vdo_hdr = *header;

	svid = PD_VDO_VID(vdo_hdr);
	if (!(vdo_hdr & VDO_SVDM_TYPE)) {
		if (index == VDO_INDEX_HDR)
			dev_info(data->dev, "- SVID=0x%04x:S=0:CMD=0x%04x\n",
				 svid, (vdo_hdr & 0x7fff));
		else
			dev_info(data->dev, "- 0x%08x\n", vdo);
		return;
	}

	if (index == VDO_INDEX_HDR) {
		tcpmtest_log_vdo_header(data, vdo_hdr);
		return;
	}

	switch (PD_VDO_CMD(vdo_hdr)) {
	case CMD_DISCOVER_IDENT:
		if (index == VDO_INDEX_IDH) {
			switch (PD_IDH_PTYPE(vdo)) {
			case IDH_PTYPE_HUB:    ptype = "PDUSB.Hub";    break;
			case IDH_PTYPE_PERIPH: ptype = "PDUSB.Periph"; break;
			case IDH_PTYPE_PCABLE: ptype = "Pass.Cable";   break;
			case IDH_PTYPE_ACABLE: ptype = "Act.Cable";    break;
			case IDH_PTYPE_AMA:    ptype = "Alt.Md.Adapt"; break;
			default:               ptype = "<undefined>";  break;
			};
			dev_info(data->dev,
				 "- UH=%u:UD=%u:PT=%s:MO=%u:VID=0x%04x\n",
				 PD_IDH_USB_HOST(vdo) ? 1 : 0,
				 PD_IDH_USB_DEV(vdo) ? 1 : 0,
				 ptype,
				 PD_IDH_MODAL_SUPP(vdo) ? 1 : 0,
				 PD_IDH_VID(vdo));
		} else if (index == VDO_INDEX_CSTAT) {
			dev_info(data->dev, "- XID=0x%08x\n", vdo);
		} else if (index == VDO_INDEX_PRODUCT) {
			dev_info(data->dev, "- PID=0x%04x:bcdDev=0x%04x\n",
				 PD_PRODUCT_PID(vdo), PD_PRODUCT_BCDD(vdo));
		} else if (index == VDO_INDEX_AMA) {
			// FIXME: depends on IDH:PTYPE == AMA!
			switch (PD_VDO_AMA_VCONN_PWR(vdo)) {
			case AMA_VCONN_PWR_1W:  vcpwr = 1000; break;
			case AMA_VCONN_PWR_1W5: vcpwr = 1500; break;
			case AMA_VCONN_PWR_2W:  vcpwr = 2000; break;
			case AMA_VCONN_PWR_3W:  vcpwr = 3000; break;
			case AMA_VCONN_PWR_4W:  vcpwr = 4000; break;
			case AMA_VCONN_PWR_5W:  vcpwr = 5000; break;
			case AMA_VCONN_PWR_6W:  vcpwr = 6000; break;
			default:                vcpwr = 0;    break;
			}
			switch (PD_VDO_AMA_SS_SUPP(vdo)) {
			case AMA_USBSS_U2_ONLY:  sssup = "USB2.0";     break;
			case AMA_USBSS_U31_GEN1: sssup = "USB3.1gen1"; break;
			case AMA_USBSS_U31_GEN2: sssup = "USB3.1gen2"; break;
			case AMA_USBSS_BBONLY:   sssup = "USB2.0BB";   break;
			default:                 sssup = "<resvd>";    break;
			}
			dev_info(data->dev,
				 "- HW=%u:FW=%u:SSTX1=%c:SSTX2=%c:SSRX1=%c:"
				 "SSRX2=%c:\n  VCP=%umW:VCR=%u:VBR=%u:SSS=%s\n",
				PD_VDO_AMA_HW_VER(vdo),
				PD_VDO_AMA_FW_VER(vdo),
				PD_VDO_AMA_SSTX2_DIR(vdo) ? 'C' : 'F',
				PD_VDO_AMA_SSTX1_DIR(vdo) ? 'C' : 'F',
				PD_VDO_AMA_SSRX1_DIR(vdo) ? 'C' : 'F',
				PD_VDO_AMA_SSRX2_DIR(vdo) ? 'C' : 'F',
				vcpwr,
				PD_VDO_AMA_VCONN_REQ(vdo),
				PD_VDO_AMA_VBUS_REQ(vdo),
				sssup);
		} else {
			tcpmtest_log_dobj_hex(data, vdo);
		}
		break;
	case CMD_DISCOVER_SVID:
		tcpmtest_log_dobj_hex(data, vdo);
		break;
	case CMD_DISCOVER_MODES:
		if (svid == USB_SID_DISPLAYPORT) {
			switch (PD_DPM_CAP(vdo)) {
			case DPM_PORT_CAP_UFP_D:     pcap = "UFP_D";     break;
			case DPM_PORT_CAP_DFP_D:     pcap = "DFP_D";     break;
			case DPM_PORT_CAP_DFP_UFP_D: pcap = "DFP+UFP_D"; break;
			default:                     pcap = "<resvd>";   break;
			}
			dev_info(data->dev,
				 "- UFP_D=0x%02x:DFP_D=0x%02x:"
				 "N2=%u:R=%u:S=0x%x:CAP=%s\n",
				 PD_DPM_UFP_D_PIN(vdo),
				 PD_DPM_DFP_D_PIN(vdo),
				 (vdo & DPM_USB20_NOT_USED) ? 1 : 0,
				 (vdo & DPM_RECEPTACLE) ? 1 : 0,
				 PD_DPM_SIGN(vdo),
				 pcap);
		} else {
			tcpmtest_log_dobj_hex(data, vdo);
		}
		break;
	case CMD_ENTER_MODE:
	case CMD_EXIT_MODE:
	case CMD_ATTENTION:
	case CMD_DP_STATUS:
	case CMD_DP_CONFIG:
	default:
		tcpmtest_log_dobj_hex(data, vdo);
		break;
	}
}

static void tcpmtest_log_msg(struct tcpmtest_data *data,
				enum tcpm_transmit_type tx_type,
				const struct pd_message *msg)
{
	u16 header = le16_to_cpu(msg->header);
	const char *tx_type_str = tcpmtest_txtype2str(tx_type);
	const char *type_str, *pr_str, *dr_str;
	unsigned int rev, id, cnt, ix;
	u32 dobj, vdo_hdr;

	if (tx_type > TCPC_TX_SOP_PRIME_PRIME) {
		dev_err(data->dev, "%s\n", tx_type_str);
		return;
	}

	rev = (header >> PD_HEADER_REV_SHIFT) & PD_HEADER_REV_MASK;
	if (rev != PD_REV20) {
		dev_err(data->dev, "%s: Trying to decode non rev 2.0 msg\n",
			__FUNCTION__);
		return;
	}

	type_str = tcpmtest_header2typestr(header);
	id = (header >> PD_HEADER_ID_SHIFT) & PD_HEADER_ID_MASK;
	if (tx_type == TCPC_TX_SOP) {
		pr_str = (header & PD_HEADER_PWR_ROLE) ? "SRC" : "SNK";
		dr_str = (header & PD_HEADER_DATA_ROLE) ? "DFP" : "UFP";
	} else {
		pr_str = (header & PD_HEADER_PWR_ROLE) ? "Plug" : "Port";
		dr_str = "-";
	}
		
	dev_info(data->dev, "%s:%s[%u]:%s:%s\n",
		 tx_type_str, type_str, id, pr_str, dr_str);

	cnt = pd_header_cnt(header);
	for (ix=0; ix<cnt; ++ix) {
		dobj = le32_to_cpu(msg->payload[ix]);
		switch (pd_header_type(header)) {
		case PD_DATA_SOURCE_CAP:
			tcpmtest_log_capability_pdo(data, dobj, true, ix);
			break;
		case PD_DATA_REQUEST:
			tcpmtest_log_request_pdo(data, dobj);
			break;
		case PD_DATA_BIST:
			tcpmtest_log_dobj_hex(data, dobj);
			break;
		case PD_DATA_SINK_CAP:
			tcpmtest_log_capability_pdo(data, dobj, false, ix);
			break;
		case PD_DATA_VENDOR_DEF:
			tcpmtest_log_vdo(data, dobj, ix, &vdo_hdr);
			break;
		default:
			tcpmtest_log_dobj_hex(data, dobj);
			break;
		}
	}
}


/*************** SysFs interface *********************************************/

static ssize_t tcpmtest_sysfs_show(struct device* dev,
				   struct device_attribute* attr, char* buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tcpmtest_data *data =
		(struct tcpmtest_data *)i2c_get_clientdata(client);
	const char *mode_str = NULL;

	if (!strcmp(attr->attr.name, "mode")) {
		switch (data->sysfs.mode) {
		case TESTMODE_SNK: mode_str = "snk"; break;
		case TESTMODE_SRC: mode_str = "src"; break;
		default: mode_str = "none"; break;
		}
	} else
		dev_err(dev, "Unknown sysfs file %s\n", attr->attr.name);

	return sprintf(buf, "%s\n", mode_str);
}

static ssize_t tcpmtest_sysfs_store(struct device* dev,
				    struct device_attribute* attr,
				    const char* buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tcpmtest_data *data =
		(struct tcpmtest_data *)i2c_get_clientdata(client);
	ssize_t err = 0;

	if (!strcmp(attr->attr.name, "mode")) {

		mutex_lock(&data->lock);
		if (buf && count) {
			if (sysfs_streq("none", buf))
				data->sysfs.mode = TESTMODE_NONE;
			else if (sysfs_streq("snk", buf))
				data->sysfs.mode = TESTMODE_SNK;
			else if (sysfs_streq("src", buf))
				data->sysfs.mode = TESTMODE_SRC;
			else
				err = -EINVAL;
			if (!err) {
				data->request.mode_set = 1;
				wake_up_process(data->thread);
			}
		} else {
			err = -EINVAL;
		}
		mutex_unlock(&data->lock);

	} else {
		dev_err(dev, "Unknown sysfs file %s\n", attr->attr.name);
		err = -ENODEV;
	}

	return err ? err : count;
}

static DEVICE_ATTR(mode, S_IRUSR|S_IRGRP|S_IROTH|S_IWUSR|S_IWGRP,
				tcpmtest_sysfs_show, tcpmtest_sysfs_store);

static struct attribute *tcpmtest_sysfs_control_attrs[] = {
	&dev_attr_mode.attr,
	NULL
};

static struct attribute_group tcpmtest_sysfs_control_attr_grp = {
    .name = "control",
    .attrs = tcpmtest_sysfs_control_attrs,
};


/****************** TCPM interface functions *********************************/

static int tcpmtest_set_cc(struct tcpc_dev *tcpc, enum typec_cc_status cc)
{
	struct tcpmtest_data *data = tcpc_to_tcpmtest_data(tcpc);
	const char *cc_str = tcpmtest_cc2str(cc);

	dev_info(data->dev, "%s(%s)\n", __FUNCTION__, cc_str);
	return 0;
}

static int tcpmtest_start_drp_toggling(struct tcpc_dev *tcpc,
				    enum typec_cc_status cc)
{
	struct tcpmtest_data *data = tcpc_to_tcpmtest_data(tcpc);
	const char *cc_str = tcpmtest_cc2str(cc);

	dev_info(data->dev, "%s(%s)\n", __FUNCTION__, cc_str);
	return 0;
}

static int tcpmtest_get_cc(struct tcpc_dev *tcpc,
			enum typec_cc_status *cc1, enum typec_cc_status *cc2)
{
	struct tcpmtest_data *data = tcpc_to_tcpmtest_data(tcpc);
	const char *cc1_str = tcpmtest_cc2str(data->cc1_status);
	const char *cc2_str = tcpmtest_cc2str(data->cc2_status);

	dev_info(data->dev, "%s(%s, %s)\n", __FUNCTION__, cc1_str, cc2_str);
	*cc1 = data->cc1_status;
	*cc2 = data->cc2_status;
	return 0;
}

static int tcpmtest_set_polarity(struct tcpc_dev *tcpc,
			      enum typec_cc_polarity polarity)
{
	struct tcpmtest_data *data = tcpc_to_tcpmtest_data(tcpc);

	data->polarity = polarity;
	dev_info(data->dev, "%s(%s)\n", __FUNCTION__,
		 polarity == TYPEC_POLARITY_CC1 ? "CC1" : "CC2");
	return 0;
}

static int tcpmtest_set_vconn(struct tcpc_dev *tcpc, bool enable)
{
	struct tcpmtest_data *data = tcpc_to_tcpmtest_data(tcpc);

	data->vconn_enable = enable;
	dev_info(data->dev, "%s(%s)\n", __FUNCTION__,
		 enable ? "enable" : "disable");
	return 0;
}

static int tcpmtest_set_roles(struct tcpc_dev *tcpc, bool attached,
			   enum typec_role pr, enum typec_data_role dr)
{
	struct tcpmtest_data *data = tcpc_to_tcpmtest_data(tcpc);

	data->data_role = dr;
	data->power_role = pr;
	dev_info(data->dev, "%s(%s, %s)\n", __FUNCTION__,
		 (pr == TYPEC_SINK) ? "SINK" : "SOURCE",
		 (dr == TYPEC_DEVICE) ? "DEVICE" : "HOST");
	return 0;
}

static int tcpmtest_set_pd_rx(struct tcpc_dev *tcpc, bool enable)
{
	struct tcpmtest_data *data = tcpc_to_tcpmtest_data(tcpc);

	data->pd_rx_enable = enable;
	dev_info(data->dev, "%s(%s)\n", __FUNCTION__,
		 enable ? "enable" : "disable");
	return 0;
}

static int tcpmtest_get_vbus(struct tcpc_dev *tcpc)
{
	struct tcpmtest_data *data = tcpc_to_tcpmtest_data(tcpc);

	dev_info(data->dev, "%s() = %s\n", __FUNCTION__,
		 data->vbus_present ? "PRESENT" : "NOT_PRESENT");

	return data->vbus_present;
}

static int tcpmtest_set_vbus(struct tcpc_dev *tcpc, bool source, bool sink)
{
	struct tcpmtest_data *data = tcpc_to_tcpmtest_data(tcpc);

	dev_info(data->dev, "%s(%s, %s)\n", __FUNCTION__,
		 source ? "SOURCE" : "off", sink ? "SINK" : "off");

	mutex_lock(&data->lock);
	data->request.vbus_chng = 1;
	data->vbus_present = (source || sink);
	wake_up_process(data->thread);
	mutex_unlock(&data->lock);

	return 0;
}

static int tcpmtest_pd_transmit(struct tcpc_dev *tcpc,
			     enum tcpm_transmit_type type,
			     const struct pd_message *msg)
{
	struct tcpmtest_data *data = tcpc_to_tcpmtest_data(tcpc);

	dev_info(data->dev, "%s():\n", __FUNCTION__);
	tcpmtest_log_msg(data, type, msg);

	mutex_lock(&data->lock);
	data->tx_type = type;
	data->tx_msg = *msg;
	data->request.msg_tx = 1;
	wake_up_process(data->thread);
	mutex_unlock(&data->lock);

	return 0;
}

static int tcpmtest_init(struct tcpc_dev *tcpc)
{
	struct tcpmtest_data *data = tcpc_to_tcpmtest_data(tcpc);

	dev_info(data->dev, "%s()\n", __FUNCTION__);
	return 0;
}


/****************** Driver internals *****************************************/

static void tcpmtest_hard_reset(struct tcpmtest_data *data, bool from_peer)
{
	data->request.word = 0;
	data->rx_id = 0;
	if (from_peer)
		tcpm_pd_hard_reset(data->port);
}

static void tcpmtest_mk_snk_request(struct tcpmtest_data *data)
{
	struct pd_message *rx_msg;

	rx_msg = &(data->rx_msg);
	rx_msg->header = PD_HEADER_LE(PD_DATA_REQUEST,
				      TYPEC_SINK, TYPEC_DEVICE,
				      data->rx_id++, 1);
	rx_msg->payload[0] = cpu_to_le32(RDO_FIXED(1, 1500, 1500,
						   RDO_USB_COMM));
	data->request.msg_rx = 1;
}

static void tcpmtest_mk_snk_sink_cap(struct tcpmtest_data *data)
{
	struct pd_message *rx_msg;

	rx_msg = &(data->rx_msg);
	rx_msg->header = PD_HEADER_LE(PD_DATA_SINK_CAP,
				      TYPEC_SINK, TYPEC_DEVICE,
				      data->rx_id++, 1);
	rx_msg->payload[0] = cpu_to_le32(PDO_FIXED(5000, 2000,
						   PDO_FIXED_USB_COMM));
	data->request.msg_rx = 1;
}

static void tcpmtest_mk_snk_disc_ident_resp_vdm(struct tcpmtest_data *data)
{
	struct pd_message *rx_msg;

	rx_msg = &(data->rx_msg);
	rx_msg->header = PD_HEADER_LE(PD_DATA_VENDOR_DEF,
				      TYPEC_SINK, TYPEC_DEVICE,
				      data->rx_id++, 5);
	rx_msg->payload[VDO_INDEX_HDR] =
		cpu_to_le32(VDO_STR(USB_SID_PD, 0, 0,
				    CMDT_RSP_ACK, CMD_DISCOVER_IDENT));
	rx_msg->payload[VDO_INDEX_IDH] =
		cpu_to_le32(VDO_IDH(0, 1, IDH_PTYPE_AMA, 1, 0x2109));
	rx_msg->payload[VDO_INDEX_CSTAT] =
		cpu_to_le32(0);
	rx_msg->payload[VDO_INDEX_PRODUCT] =
		cpu_to_le32(VDO_PRODUCT(0x0101, 0x0001));
	rx_msg->payload[VDO_INDEX_AMA] =
		cpu_to_le32(VDO_AMA(0, 0, 0, 0, 0, 0,
				    AMA_VCONN_PWR_1W5, 1, 1, AMA_USBSS_BBONLY));
	data->request.msg_rx = 1;
}

static void tcpmtest_mk_snk_disc_svid_resp_vdm(struct tcpmtest_data *data)
{
	struct pd_message *rx_msg;

	rx_msg = &(data->rx_msg);
	rx_msg->header = PD_HEADER_LE(PD_DATA_VENDOR_DEF,
				      TYPEC_SINK, TYPEC_DEVICE,
				      data->rx_id++, 2);
	rx_msg->payload[VDO_INDEX_HDR] =
		cpu_to_le32(VDO_STR(USB_SID_PD, 0, 0,
				    CMDT_RSP_ACK, CMD_DISCOVER_SVID));
	rx_msg->payload[1] = cpu_to_le32(VDO_SVID(USB_SID_DISPLAYPORT, 0x0000));
	data->request.msg_rx = 1;
}

static void tcpmtest_mk_snk_disc_modes_resp_vdm(struct tcpmtest_data *data,
						u16 svid)
{
	struct pd_message *rx_msg;

	if (svid != USB_SID_DISPLAYPORT)
		return;

	rx_msg = &(data->rx_msg);
	rx_msg->header = PD_HEADER_LE(PD_DATA_VENDOR_DEF,
				      TYPEC_SINK, TYPEC_DEVICE,
				      data->rx_id++, 2);
	rx_msg->payload[VDO_INDEX_HDR] =
		cpu_to_le32(VDO_STR(USB_SID_DISPLAYPORT, 0, 0,
				    CMDT_RSP_ACK, CMD_DISCOVER_MODES));
	rx_msg->payload[1] =
		cpu_to_le32(VDO_DPM(0, DPM_DFP_D_PIN_C, 0, 0,
				    DPM_SIGN_SUPP_DP13, DPM_PORT_CAP_UFP_D));
	data->request.msg_rx = 1;
}

static void tcpmtest_process_tx_vdm_for_snk(struct tcpmtest_data *data,
					    unsigned int cnt)
{
	const struct pd_message *tx_msg;
	u32 vdo_hdr;
	unsigned int cmdt;

	tx_msg = &(data->tx_msg);

	vdo_hdr = tx_msg->payload[0];
	if (!(vdo_hdr & VDO_SVDM_TYPE))
		return;		/* Unstructured VDM, ignore */

	//svid = PD_VDO_VID(vdo_hdr);

	cmdt = PD_VDO_CMDT(vdo_hdr);
	switch (PD_VDO_CMD(vdo_hdr)) {
	case CMD_DISCOVER_IDENT:
		if (cmdt == CMDT_INIT)
			tcpmtest_mk_snk_disc_ident_resp_vdm(data);
		break;
	case CMD_DISCOVER_SVID:
		if (cmdt == CMDT_INIT)
			tcpmtest_mk_snk_disc_svid_resp_vdm(data);
		break;
	case CMD_DISCOVER_MODES:
		if (cmdt == CMDT_INIT)
			tcpmtest_mk_snk_disc_modes_resp_vdm(
				data, PD_VDO_VID(vdo_hdr)
			);
		break;
	case CMD_ENTER_MODE:
	case CMD_EXIT_MODE:
	case CMD_ATTENTION:
	case CMD_DP_STATUS:
	case CMD_DP_CONFIG:
	default:
		break;
	}
}

static void tcpmtest_process_tx_msg_for_snk(struct tcpmtest_data *data)
{
	enum tcpm_transmit_type tx_type = data->tx_type;
	const struct pd_message *tx_msg;
	u16 header;
	unsigned int cnt;
	
	if (tx_type == TCPC_TX_HARD_RESET) {
		tcpmtest_hard_reset(data, false);
		return;
	}
	
	if (tx_type != TCPC_TX_SOP)
		return; // ignore

	tx_msg = &(data->tx_msg);
	header = le16_to_cpu(tx_msg->header);
	cnt = pd_header_cnt(header);
	if (!cnt) {
		/* Control message */
		switch (pd_header_type(header)) {
		case PD_CTRL_GOOD_CRC:
		case PD_CTRL_GOTO_MIN:
		case PD_CTRL_ACCEPT:
			break;
		case PD_CTRL_REJECT:
			tcpmtest_hard_reset(data, true);
			break;
		case PD_CTRL_PING:
		case PD_CTRL_PS_RDY:
			break;
		case PD_CTRL_GET_SOURCE_CAP:
			tcpmtest_hard_reset(data, true);
			break;
		case PD_CTRL_GET_SINK_CAP:
			tcpmtest_mk_snk_sink_cap(data);
			break;
		case PD_CTRL_DR_SWAP:
		case PD_CTRL_PR_SWAP:
		case PD_CTRL_VCONN_SWAP:
			tcpmtest_hard_reset(data, true);
			break;
		case PD_CTRL_WAIT:
			break;
		case PD_CTRL_SOFT_RESET:
			data->rx_id = 0;
			break;
		default:
			break;
		}
	} else {
		/* Data message */
		switch (pd_header_type(header)) {
		case PD_DATA_SOURCE_CAP:
			tcpmtest_mk_snk_request(data);
			break;
		case PD_DATA_REQUEST:
			tcpmtest_hard_reset(data, true);
			break;
		case PD_DATA_BIST:
			break;
		case PD_DATA_SINK_CAP:
			tcpmtest_hard_reset(data, true);
			break;
		case PD_DATA_VENDOR_DEF:
			tcpmtest_process_tx_vdm_for_snk(data, cnt);
			break;
		default:
			break;
		}
	}
}

static void tcpmtest_process_tx_msg_for_src(struct tcpmtest_data *data)
{
	enum tcpm_transmit_type tx_type = data->tx_type;
	const struct pd_message *tx_msg;
	u16 header;
	unsigned int cnt;

	if (tx_type == TCPC_TX_HARD_RESET) {
		tcpmtest_hard_reset(data, false);
		return;
	}

	if (tx_type != TCPC_TX_SOP)
		return; // ignore

	tx_msg = &(data->tx_msg);
	header = le16_to_cpu(tx_msg->header);
	cnt = pd_header_cnt(header);
	if (!cnt) {
		/* Control message */
		switch (pd_header_type(header)) {
		case PD_CTRL_GOOD_CRC:
		case PD_CTRL_GOTO_MIN:
		case PD_CTRL_ACCEPT:
			break;
		case PD_CTRL_REJECT:
			tcpmtest_hard_reset(data, true);
			break;
		case PD_CTRL_PING:
		case PD_CTRL_PS_RDY:
			break;
		case PD_CTRL_GET_SOURCE_CAP:
			tcpmtest_hard_reset(data, true);
			break;
		case PD_CTRL_GET_SINK_CAP:
			tcpmtest_mk_snk_sink_cap(data);
			break;
		case PD_CTRL_DR_SWAP:
		case PD_CTRL_PR_SWAP:
		case PD_CTRL_VCONN_SWAP:
			tcpmtest_hard_reset(data, true);
			break;
		case PD_CTRL_WAIT:
			break;
		case PD_CTRL_SOFT_RESET:
			data->rx_id = 0;
			break;
		default:
			break;
		}
	} else {
		/* Data message */
		switch (pd_header_type(header)) {
		case PD_DATA_SOURCE_CAP:
			tcpmtest_hard_reset(data, true);
			break;
		case PD_DATA_REQUEST:
			tcpmtest_mk_snk_request(data);
			break;
		case PD_DATA_BIST:
			break;
		case PD_DATA_SINK_CAP:
			tcpmtest_hard_reset(data, true);
			break;
		case PD_DATA_VENDOR_DEF:
			tcpmtest_process_tx_vdm_for_snk(data, cnt);
			break;
		default:
			break;
		}
	}
}

static int tcpmtest_thread(void *tdata)
{
	struct tcpmtest_data *data = (struct tcpmtest_data *)tdata;
	bool yield;
	unsigned delay = 0;

	dev_info(&data->client->dev, "thread started\n");

	/* Wait for tcpm registration to complete before calling back */
	while (!data->port) {
		set_current_state(TASK_INTERRUPTIBLE);
		schedule();
	}

	mutex_lock(&data->lock);

	for (;;) {

		yield = false;
		if (data->request.word == 0 && !delay) {
			/* No pending requests so yield */
			yield = true;
			set_current_state(TASK_INTERRUPTIBLE);
		}

		mutex_unlock(&data->lock);

		if (delay) {
			msleep(delay);
			delay = 0;
		} else if (yield) {
			schedule();
		}
		if (kthread_should_stop())
			break;

		mutex_lock(&data->lock);

		//tcpm_tcpc_reset(data->port);
		
		if (data->request.vbus_chng) {
			data->request.vbus_chng = 0;
			tcpm_vbus_change(data->port);
		}

		if (data->request.mode_set) {
			data->request.mode_set = 0;
			if (data->sysfs.mode == TESTMODE_SNK) {
				data->cc1_status = TYPEC_CC_RD;
				data->cc2_status = TYPEC_CC_RA;
			if (data->sysfs.mode == TESTMODE_SRC) {
				data->cc1_status = TYPEC_CC_RP_3_0;
				data->cc2_status = TYPEC_CC_RP_3_0;
			} else {
				data->cc1_status = TYPEC_CC_OPEN;
				data->cc2_status = TYPEC_CC_OPEN;
			}
			data->vbus_present = false;
			tcpm_cc_change(data->port);
		}

		if (data->request.msg_rx) {
			data->request.msg_rx = 0;
			dev_info(&data->client->dev, "Msg received:\n");
			tcpmtest_log_msg(data, TCPC_TX_SOP, &(data->rx_msg));
			tcpm_pd_receive(data->port, &(data->rx_msg));
		}
		
		if (data->request.msg_tx) {
			enum tcpm_transmit_status tx_status = TCPC_TX_FAILED;
			data->request.msg_tx = 0;
			// TCPC_TX_SUCCESS, TCPC_TX_DISCARDED, TCPC_TX_FAILED
			if (data->sysfs.mode == TESTMODE_SNK) {
				tcpmtest_process_tx_msg_for_snk(data);
				tx_status = TCPC_TX_SUCCESS;
			} else if (data->sysfs.mode == TESTMODE_SRC) {
				tcpmtest_process_tx_msg_for_src(data);
				tx_status = TCPC_TX_SUCCESS;
			}
			tcpm_pd_transmit_complete(data->port, tx_status);
		}
	}

	mutex_unlock(&data->lock);
	dev_info(&data->client->dev, "thread ended\n");

	return 0;
}

const u32 tcpmtest_src_pdos[1] = { 0x26019096 };

const struct typec_altmode_desc tcpmtest_alt_modes[3] = {
	{
		.svid = USB_SID_DISPLAYPORT,
		.n_modes = 2,
		.modes = {
			{
				.index = 0,
				.vdo = 0x00000000,
				.desc = "DP alt 1",
				.roles = TYPEC_PORT_DFP,
			},
			{
				.index = 1,
				.vdo = 0x00000001,
				.desc = "DP alt 2",
				.roles = TYPEC_PORT_DFP,
			},
		}
	},
	{
		.svid = USB_VID_GOOGLE,
		.n_modes = 2,
		.modes = {
			{
				.index = 0,
				.vdo = 0x00000123,
				.desc = "Google mode 1",
				.roles = TYPEC_PORT_UFP,
			},
			{
				.index = 1,
				.vdo = 0x00000456,
				.desc = "Google mode 2",
				.roles = TYPEC_PORT_DFP,
			},
		}
	},
	{
		.svid = 0,
	}
};

const struct tcpc_config tcpmtest_tcpc_config = {
	.nr_src_pdo = ARRAY_SIZE(tcpmtest_src_pdos),
	.src_pdo = tcpmtest_src_pdos,

	.type = TYPEC_PORT_DFP,
	.default_role = TYPEC_SOURCE,

	.alt_modes = tcpmtest_alt_modes,
};

static int tcpmtest_parse_config(struct tcpmtest_data *data)
{
	data->controls_vbus = true; /* XXX */

	/* TODO: Populate struct tcpc_config from ACPI/device-tree */
	data->tcpc.config = &tcpmtest_tcpc_config;

	return 0;
}

static int tcpmtest_probe(struct i2c_client *client,
			  const struct i2c_device_id *i2c_id)
{
	struct tcpmtest_data *data;
	int err;

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	data->dev = &client->dev;
	i2c_set_clientdata(client, data);

	data->tcpc.init = tcpmtest_init;
	data->tcpc.get_vbus = tcpmtest_get_vbus;
	data->tcpc.set_vbus = tcpmtest_set_vbus;
	data->tcpc.set_cc = tcpmtest_set_cc;
	data->tcpc.get_cc = tcpmtest_get_cc;
	data->tcpc.set_polarity = tcpmtest_set_polarity;
	data->tcpc.set_vconn = tcpmtest_set_vconn;
	data->tcpc.start_drp_toggling = tcpmtest_start_drp_toggling;
	data->tcpc.set_pd_rx = tcpmtest_set_pd_rx;
	data->tcpc.set_roles = tcpmtest_set_roles;
	data->tcpc.pd_transmit = tcpmtest_pd_transmit;

	err = tcpmtest_parse_config(data);
	if (err)
		goto out;

	mutex_init(&data->lock);

	data->thread = kthread_create(tcpmtest_thread, data, "tcpmtest");
	if (IS_ERR(data->thread)) {
		err = PTR_ERR(data->thread);
		dev_err(data->dev, "Cannot start thread (%d)\n", err);
		data->thread = NULL;
		goto out;
	}

	err = sysfs_create_group(&data->dev->kobj,
				 &tcpmtest_sysfs_control_attr_grp);
	if (err)
		dev_err(data->dev, "Cannot create sysfs group (%d)\n", err);

	data->port = tcpm_register_port(data->dev, &data->tcpc);
	if (IS_ERR(data->port)) {
		err = PTR_ERR(data->port);
		goto out;
	}

	return 0;

    out:
	devm_kfree(&client->dev, data);
	return err;
}

static int tcpmtest_remove(struct i2c_client *client)
{
	struct tcpmtest_data *data = i2c_get_clientdata(client);

	tcpm_unregister_port(data->port);

	return 0;
}

static void tcpmtest_shutdown(struct i2c_client *client)
{
	struct tcpmtest_data *data = i2c_get_clientdata(client);

	if (data->thread) {
		kthread_stop(data->thread);
		data->thread = NULL;
	}

	tcpmtest_remove(client);
}

static const struct i2c_device_id tcpmtest_id[] = {
	{ "tcpmtest", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tcpmtest_id);

#ifdef CONFIG_OF
static const struct of_device_id tcpmtest_of_match[] = {
	{ .compatible = "usb,tcpmtest", },
	{ }
};
MODULE_DEVICE_TABLE(of, tcpmtest_of_match);
#endif

static struct i2c_driver tcpmtest_i2c_driver = {
	.driver = {
		.name = "tcpmtest",
		.of_match_table = of_match_ptr(tcpmtest_of_match),
	},
	.probe = tcpmtest_probe,
	.remove = tcpmtest_remove,
	.shutdown = tcpmtest_shutdown,
	.id_table = tcpmtest_id,
};
module_i2c_driver(tcpmtest_i2c_driver);

MODULE_DESCRIPTION("USB Type-C Port Manager test device");
MODULE_AUTHOR("Mats Karrman <mats@southpole.se>");
MODULE_LICENSE("GPL v2");
