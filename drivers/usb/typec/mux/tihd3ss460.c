// SPDX-License-Identifier: GPL-2.0+
/*
 * TI HD3SS460 USB Type-C Alternate Mode MUX driver
 *
 * Copyright (c) 2018 Mats Karrman <mats.dev.list@gmail.com>
 *
 * NOTE:
 *   The TI HD3sS460 uses 3-state (low, medium, high) inputs for AMSEL and EN
 *   signals. This driver implements the "medium" state by setting the signal
 *   as input and assumes that this is translated to the proper level by
 *   hardware (e.g. using bias resistors).
 */

#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/usb/typec.h>
#include <linux/usb/typec_mux.h>

struct tihd3ss460_platform_data {
	struct typec_switch sw;
	struct typec_mux mux;
	enum typec_mux_mode default_mux_mode; /* Mode for TYPEC_MUX_DEFAULT */
	u8 mode_support; /* Modes supported by hardware as bit flags */
	struct mutex lock;		/* protects gpio update sequence */
	struct gpio_desc *en_gpio;
	struct gpio_desc *amsel_gpio;
	struct gpio_desc *pol_gpio;
};

static int tihd3ss460_sw_set(struct typec_switch *sw,
			     enum typec_orientation orientation)
{
	struct tihd3ss460_platform_data *hdp =
		container_of(sw, struct tihd3ss460_platform_data, sw);
	int ret = -EINVAL;

	mutex_lock(&hdp->lock);

	switch (orientation) {
	case TYPEC_ORIENTATION_NONE:
		/* Mux disabled */
		ret = gpiod_direction_output(hdp->en_gpio, 0);
		break;
	case TYPEC_ORIENTATION_NORMAL:
		/* Polarity normal */
		ret = gpiod_direction_output(hdp->pol_gpio, 0);
		break;
	case TYPEC_ORIENTATION_REVERSE:
		/* Polarity flipped */
		ret = gpiod_direction_output(hdp->pol_gpio, 1);
		break;
	}

	mutex_unlock(&hdp->lock);

	return ret;
}

static int tihd3ss460_mux_set(struct typec_mux *mux, enum typec_mux_mode mode)
{
	struct tihd3ss460_platform_data *hdp =
		container_of(mux, struct tihd3ss460_platform_data, mux);
	int ret = -EINVAL;

	mutex_lock(&hdp->lock);

	if (mode == TYPEC_MUX_DEFAULT)
		mode = hdp->default_mux_mode;

	switch (mode) {
	default:
	case TYPEC_MUX_NONE:
		/* Mux disabled */
		ret = gpiod_direction_output(hdp->en_gpio, 0);
		break;
	case TYPEC_MUX_2CH_USBSS:
		/* CRTX1<=>SSRTX, CSBU HiZ, SBU12 HiZ (Normal)  */
		/* CRTX2<=>SSRTX, CSBU HiZ, SBU12 HiZ (Flipped) */
		if (hdp->mode_support & (0x1 << TYPEC_MUX_2CH_USBSS)) {
			ret = gpiod_direction_input(hdp->amsel_gpio);
			if (!ret)
				ret = gpiod_direction_output(hdp->en_gpio, 1);
		}
		break;
	case TYPEC_MUX_4CH_AM:
		/* CRTX1<=>LnDC, CRTX2<=>LnAB, CSBU12<=>SBU12 (Normal)  */
		/* CRTX1<=>LnAB, CRTX2<=>LnDC, CSBU12<=>SBU21 (Flipped) */
		if (hdp->mode_support & (0x1 << TYPEC_MUX_4CH_AM)) {
			ret = gpiod_direction_output(hdp->amsel_gpio, 1);
			if (!ret)
				ret = gpiod_direction_output(hdp->en_gpio, 1);
		}
		break;
	case TYPEC_MUX_2CH_USBSS_2CH_AM:
		/* CRTX1<=>SSRTX, CRTX2<=>LnAB, CSBU12<=>SBU12 (Normal)  */
		/* CRTX1<=>LnAB, CRTX2<=>SSRTX, CSBU12<=>SBU21 (Flipped) */
		if (hdp->mode_support & (0x1 << TYPEC_MUX_2CH_USBSS_2CH_AM)) {
			ret = gpiod_direction_output(hdp->amsel_gpio, 0);
			if (!ret)
				ret = gpiod_direction_output(hdp->en_gpio, 1);
		}
		break;
	case TYPEC_MUX_2CH_USBSS_2CH_AM_B:
		/* CRTX1<=>SSRTX, CRTX2<=>LnDC, CSBU12<=>SBU12 (Normal)  */
		/* CRTX1<=>LnDC, CRTX2<=>SSRTX, CSBU12<=>SBU21 (Flipped) */
		if (hdp->mode_support & (0x1 << TYPEC_MUX_2CH_USBSS_2CH_AM_B)) {
			ret = gpiod_direction_output(hdp->amsel_gpio, 0);
			if (!ret)
				ret = gpiod_direction_input(hdp->en_gpio);
		}
	}

	mutex_unlock(&hdp->lock);

	return ret;
}

static int tihd3ss460_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tihd3ss460_platform_data *hdp;
	const char *mode_str;
	int ret;

	hdp = devm_kzalloc(dev, sizeof(*hdp), GFP_KERNEL);
	if (!hdp)
		return -ENOMEM;

	hdp->sw.dev = dev;
	hdp->sw.set = tihd3ss460_sw_set;
	hdp->mux.dev = dev;
	hdp->mux.set = tihd3ss460_mux_set;
	mutex_init(&hdp->lock);

	if (!device_property_present(dev, "default-mux-mode")) {
		hdp->default_mux_mode = TYPEC_MUX_2CH_USBSS;
	} else {
		ret = device_property_read_string(dev, "default-mux-mode",
						  &mode_str);
		if (ret)
			return ret;
		ret = typec_find_mux_mode(mode_str);
		if (ret < 0)
			return ret;
		hdp->default_mux_mode = ret;
	}

	if (device_property_present(dev, "have-2ch-usbss"))
		hdp->mode_support |= 0x1 << TYPEC_MUX_2CH_USBSS;
	if (device_property_present(dev, "have-4ch-am"))
		hdp->mode_support |= 0x1 << TYPEC_MUX_4CH_AM;
	if (device_property_present(dev, "have-2ch-usbss-2ch-am"))
		hdp->mode_support |= 0x1 << TYPEC_MUX_2CH_USBSS_2CH_AM;

	if (!hdp->mode_support) {
		dev_warn(dev, "No mode support found, assuming full support\n");
		hdp->mode_support = (u8)-1;
	}

	if (!device_property_present(dev, "ti,control-gpios"))
		return -ENODEV;

	hdp->en_gpio = devm_gpiod_get_index(dev, "ti,control", 0,
					    GPIOD_OUT_LOW);
	if (IS_ERR(hdp->en_gpio)) {
		dev_err(&pdev->dev, "Failed to get EN gpio (#0)\n");
		return PTR_ERR(hdp->en_gpio);
	}

	hdp->amsel_gpio = devm_gpiod_get_index(dev, "ti,control", 1,
					       GPIOD_OUT_LOW);
	if (IS_ERR(hdp->amsel_gpio)) {
		dev_err(&pdev->dev, "Failed to get AMSEL gpio (#0)\n");
		return PTR_ERR(hdp->amsel_gpio);
	}

	hdp->pol_gpio = devm_gpiod_get_index(dev, "ti,control", 2,
					     GPIOD_OUT_LOW);
	if (IS_ERR(hdp->pol_gpio)) {
		dev_err(&pdev->dev, "Failed to get POL gpio (#0)\n");
		return PTR_ERR(hdp->pol_gpio);
	}

	dev->platform_data = hdp;

	ret = typec_switch_register(&hdp->sw);
	if (ret) {
		dev_err(dev, "Error registering typec switch: %d\n", ret);
		return ret;
	}

	ret = typec_mux_register(&hdp->mux);
	if (ret) {
		typec_switch_unregister(&hdp->sw);
		dev_err(dev, "Error registering typec mux: %d\n", ret);
		return ret;
	}

	return 0;
}

static int tihd3ss460_remove(struct platform_device *pdev)
{
	struct tihd3ss460_platform_data *hdp = dev_get_platdata(&pdev->dev);

	typec_mux_unregister(&hdp->mux);
	typec_switch_unregister(&hdp->sw);
	return 0;
}

static const struct of_device_id tihd3ss460_of_match_table[] = {
	{ .compatible = "ti,hd3ss460", },
	{},
};
MODULE_DEVICE_TABLE(of, tihd3ss460_of_match_table);

static const struct platform_device_id tihd3ss460_id_table[] = {
	{ "tihd3ss460" },
	{}
};
MODULE_DEVICE_TABLE(i2c, tihd3ss460_table);

static struct platform_driver tihd3ss460_driver = {
	.driver = {
		.name = "tihd3ss460",
		.of_match_table = tihd3ss460_of_match_table,
	},
	.probe		= tihd3ss460_probe,
	.remove		= tihd3ss460_remove,
	.id_table	= tihd3ss460_id_table,
};

module_platform_driver(tihd3ss460_driver);

MODULE_AUTHOR("Mats Karrman <mats.dev.list@gmail.com>");
MODULE_DESCRIPTION("TI HD3SS460 USB Type-C Alternate Mode MUX driver");
MODULE_LICENSE("GPL");
