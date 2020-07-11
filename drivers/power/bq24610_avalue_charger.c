/*
 * Driver for the TI bq24610 battery charger.
 *
 * Author: Mark A. Greer <mgreer@animalcreek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/power_supply.h>
#include <linux/gpio.h>
#include <linux/i2c.h>

#define	BQ24610_MANUFACTURER	"Texas Instruments"
#define BQ24610_REG_SS_VBUS_STAT_MASK		(BIT(7) | BIT(6))
#define BQ24610_REG_VPRS_PN_24610			0x4

struct bq24610_platform_data {
	unsigned int    gpio_int;   /* GPIO pin that's connected to INT# */
};


/*
 * The FAULT register is latched by the bq24610 (except for NTC_FAULT)
 * so the first read after a fault returns the latched value and subsequent
 * reads return the current value.  In order to return the fault status
 * to the user, have the interrupt handler save the reg's value and retrieve
 * it in the appropriate health/status routine.
 */
struct bq24610_dev_info {
	struct i2c_client		*client;
	struct device			*dev;
	struct power_supply		*charger;
	struct power_supply		*battery;
	char				model_name[I2C_NAME_SIZE];
	kernel_ulong_t			model;
	unsigned int			gpio_int;
	unsigned int			irq;
	struct mutex			f_reg_lock;
	u8				f_reg;
	u8				ss_reg;
	u8				watchdog;
};

/* Basic driver I/O routines */

static int bq24610_read(struct bq24610_dev_info *bdi, u8 reg, u8 *data)
{
	int ret;

	struct i2c_adapter *adap = bdi->client->adapter;
	struct i2c_msg msgs[2];
	char reg_buf = reg;

	msgs[0].addr = bdi->client->addr;
	msgs[0].flags = bdi->client->flags;
	msgs[0].len = 1;
	msgs[0].buf = &reg_buf;

	msgs[1].addr = bdi->client->addr;
	msgs[1].flags = bdi->client->flags | I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = (char *)data;

	ret = i2c_transfer(adap, msgs, 2);
	return ret;
}

static int bq24610_sysfs_create_group(struct bq24610_dev_info *bdi)
{
	return 0;
}

static inline void bq24610_sysfs_remove_group(struct bq24610_dev_info *bdi)
{
}

static int bq24610_charge_online(struct bq24610_dev_info *bdi,
		union power_supply_propval *val)
{
	int ret;
	u8 data;
	ret = bq24610_read(bdi, 0x00, &data);
	if (ret > 0) {
		switch(data)
		{
		case 0x00:
		case 0x01:
			return 1;
		default:
			return 0;
		}
	}
	return 0;
}

static int bq24610_charger_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct bq24610_dev_info *bdi = power_supply_get_drvdata(psy);
	int ret;

	pm_runtime_get_sync(bdi->dev);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = bq24610_charge_online(bdi, val);
		ret = 0;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		ret = 0;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = bdi->model_name;
		ret = 0;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = BQ24610_MANUFACTURER;
		ret = 0;
		break;
	default:
		ret = -ENODATA;
	}

	pm_runtime_put_sync(bdi->dev);
	return ret;
}

static enum power_supply_property bq24610_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static char *bq24610_charger_supplied_to[] = {
	"main-battery",
};

static const struct power_supply_desc bq24610_charger_desc = {
	.name			= "bq24610-AC",
	.type			= POWER_SUPPLY_TYPE_MAINS,
	.properties		= bq24610_charger_properties,
	.num_properties		= ARRAY_SIZE(bq24610_charger_properties),
	.get_property		= bq24610_charger_get_property,
};

/* Battery power supply property routines */
static int bq24610_battery_get_capacity(struct bq24610_dev_info *bdi,
		union power_supply_propval *val)
{
	int ret = 0, i = 0;
	u8 data;

	for (i = 0; i < 3; i++) {
		ret = bq24610_read(bdi, 0x01, &data);
		if (ret <= 0) {
			continue;
		}

		if (data <= 10) {
			continue;
		}
		val->intval = data;
		return 0;
	}

	return ret;
}

static int bq24610_battery_get_status(struct bq24610_dev_info *bdi,
		union power_supply_propval *val)
{
	int ret;
	u8 data;

	// Get status
	ret = bq24610_read(bdi, 0x00, &data);
	if (ret > 0) {
		switch(data)
		{
		case 0x00:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		case 0x01:
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case 0x02:
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
			ret = -EIO;
			break;
		};
	}

	return ret;
}

static int bq24610_battery_online(struct bq24610_dev_info *bdi,
		union power_supply_propval *val)
{
	int ret;
	u8 data;

	ret = bq24610_read(bdi, 0x00, &data);
	if (ret <= 0) {
		return 0;
	}
	if (data == 0x01) {
		data = 0;
		ret = bq24610_read(bdi, 0x01, &data);
		if (ret > 0) {
			if (data != 0xff) {
				return 1;
			}
		}
	}
	return 0;
}

static int bq24610_battery_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct bq24610_dev_info *bdi = power_supply_get_drvdata(psy);
	int ret;

	pm_runtime_get_sync(bdi->dev);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = bq24610_battery_online(bdi, val);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = bq24610_battery_get_capacity(bdi, val);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = bq24610_battery_get_status(bdi, val);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		ret = 0;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_SYSTEM;
		ret = 0;
		break;
	default:
		ret = -ENODATA;
	}

	pm_runtime_put_sync(bdi->dev);
	return ret;
}

static enum power_supply_property bq24610_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_SCOPE,
	POWER_SUPPLY_PROP_CAPACITY,
};

static const struct power_supply_desc bq24610_battery_desc = {
	.name			= "bq24610-battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.properties		= bq24610_battery_properties,
	.num_properties		= ARRAY_SIZE(bq24610_battery_properties),
	.get_property		= bq24610_battery_get_property,
};

static irqreturn_t bq24610_irq_handler_thread(int irq, void *data)
{
	struct bq24610_dev_info *bdi = data;

	pm_runtime_get_sync(bdi->dev);
	power_supply_changed(bdi->charger);
	power_supply_changed(bdi->battery);
	pm_runtime_put_sync(bdi->dev);

	return IRQ_HANDLED;
}

#ifdef CONFIG_OF
static int bq24610_setup_dt(struct bq24610_dev_info *bdi)
{
	bdi->irq = irq_of_parse_and_map(bdi->dev->of_node, 0);
	if (bdi->irq <= 0)
		return -1;

	return 0;
}
#else
static int bq24610_setup_dt(struct bq24610_dev_info *bdi)
{
	return -1;
}
#endif

static int bq24610_setup_pdata(struct bq24610_dev_info *bdi,
		struct bq24610_platform_data *pdata)
{
	int ret;

	if (!gpio_is_valid(pdata->gpio_int))
		return -1;

	ret = gpio_request(pdata->gpio_int, dev_name(bdi->dev));
	if (ret < 0)
		return -1;

	ret = gpio_direction_input(pdata->gpio_int);
	if (ret < 0)
		goto out;

	bdi->irq = gpio_to_irq(pdata->gpio_int);
	if (!bdi->irq)
		goto out;

	bdi->gpio_int = pdata->gpio_int;
	return 0;

out:
	gpio_free(pdata->gpio_int);
	return -1;
}

static int bq24610_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct device *dev = &client->dev;
	struct bq24610_platform_data *pdata = client->dev.platform_data;
	struct power_supply_config charger_cfg = {}, battery_cfg = {};
	struct bq24610_dev_info *bdi;
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "No support for SMBUS_BYTE_DATA\n");
		return -ENODEV;
	}

	bdi = devm_kzalloc(dev, sizeof(*bdi), GFP_KERNEL);
	if (!bdi) {
		dev_err(dev, "Can't alloc bdi struct\n");
		return -ENOMEM;
	}

	bdi->client = client;
	bdi->dev = dev;
	bdi->model = id->driver_data;
	strncpy(bdi->model_name, id->name, I2C_NAME_SIZE);
	mutex_init(&bdi->f_reg_lock);
	bdi->f_reg = 0;
	bdi->ss_reg = BQ24610_REG_SS_VBUS_STAT_MASK; /* impossible state */

	i2c_set_clientdata(client, bdi);

	if (dev->of_node)
		ret = bq24610_setup_dt(bdi);
	else
		ret = bq24610_setup_pdata(bdi, pdata);

	/*
	if (ret) {
		dev_err(dev, "Can't get irq info\n");
		return -EINVAL;
	}
	*/

	pm_runtime_enable(dev);
	pm_runtime_resume(dev);

	charger_cfg.drv_data = bdi;
	charger_cfg.supplied_to = bq24610_charger_supplied_to;
	charger_cfg.num_supplicants = ARRAY_SIZE(bq24610_charger_supplied_to),
	bdi->charger = power_supply_register(dev, &bq24610_charger_desc,
						&charger_cfg);
	if (IS_ERR(bdi->charger)) {
		dev_err(dev, "Can't register charger\n");
		ret = PTR_ERR(bdi->charger);
		goto out1;
	}

	battery_cfg.drv_data = bdi;
	bdi->battery = power_supply_register(dev, &bq24610_battery_desc,
						&battery_cfg);
	if (IS_ERR(bdi->battery)) {
		dev_err(dev, "Can't register battery\n");
		ret = PTR_ERR(bdi->battery);
		goto out2;
	}

	ret = bq24610_sysfs_create_group(bdi);
	if (ret) {
		dev_err(dev, "Can't create sysfs entries\n");
		goto out3;
	}

	ret = devm_request_threaded_irq(dev, bdi->irq, NULL,
			bq24610_irq_handler_thread,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"bq24610-charger", bdi);

	/*
	if (ret < 0) {
		dev_err(dev, "Can't set up irq handler\n");
		goto out4;
	}
	*/

	return 0;

/*
out4:
	bq24610_sysfs_remove_group(bdi);
*/
out3:
	power_supply_unregister(bdi->battery);
out2:
	power_supply_unregister(bdi->charger);
out1:
	pm_runtime_disable(dev);
	if (bdi->gpio_int)
		gpio_free(bdi->gpio_int);

	return ret;
}

static int bq24610_remove(struct i2c_client *client)
{
	struct bq24610_dev_info *bdi = i2c_get_clientdata(client);

	bq24610_sysfs_remove_group(bdi);
	power_supply_unregister(bdi->battery);
	power_supply_unregister(bdi->charger);
	pm_runtime_disable(bdi->dev);

	if (bdi->gpio_int)
		gpio_free(bdi->gpio_int);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int bq24610_pm_suspend(struct device *dev)
{
	return 0;
}

static int bq24610_pm_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bq24610_dev_info *bdi = i2c_get_clientdata(client);

	power_supply_changed(bdi->charger);
	power_supply_changed(bdi->battery);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(bq24610_pm_ops, bq24610_pm_suspend, bq24610_pm_resume);

static const struct i2c_device_id bq24610_i2c_ids[] = {
	{ "bq24610", BQ24610_REG_VPRS_PN_24610 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, bq24610_i2c_ids);

#ifdef CONFIG_OF
static const struct of_device_id bq24610_of_match[] = {
	{ .compatible = "ti,bq24610", },
	{ },
};
MODULE_DEVICE_TABLE(of, bq24610_of_match);
#else
static const struct of_device_id bq24610_of_match[] = {
	{ },
};
#endif

static struct i2c_driver bq24610_driver = {
	.probe		= bq24610_probe,
	.remove		= bq24610_remove,
	.id_table	= bq24610_i2c_ids,
	.driver = {
		.name		= "bq24610-charger",
		.pm		= &bq24610_pm_ops,
		.of_match_table	= of_match_ptr(bq24610_of_match),
	},
};
module_i2c_driver(bq24610_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Avalue");
MODULE_DESCRIPTION("TI BQ24610 Charger Driver");
