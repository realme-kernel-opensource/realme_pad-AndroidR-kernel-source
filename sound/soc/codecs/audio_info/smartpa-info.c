#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif
#include "smartpa-info.h"

struct platform_driver smartpainfo_driver_func(void);

uint32_t smartpainfo_num = 0;
// char *smartpainfo_vendor = NULL;
const char *smartpa_name[SMARTPA_TYPE_MAX] = {"none", "fs1894su", "aw88194a","aw88394"};
const char *smartpa_num[SMARTPA_NUM_MAX] = {"0", "1", "2", "3", "4", "5", "6", "7", "8"};
static struct lc_audio_info g_audio_priv;

static struct lc_audio_info *get_audio_info_priv(void)
{
	return &g_audio_priv;
}

void set_smartpa_type(const char *buf, int len)
{
	struct lc_audio_info *priv = get_audio_info_priv();

	if (buf == NULL) {
		pr_err("[%s] buf is NULL set smartpa type failed!\n",  __func__);
		return;
	}
	pr_err("[%s] set smartpa type %s!\n", __func__, buf);
	memset(priv->smartpa_type_str, 0 ,sizeof(priv->smartpa_type_str));
	memcpy(priv->smartpa_type_str, buf, len);
	return;
}
EXPORT_SYMBOL(set_smartpa_type);

enum smartpa_type get_smartpa_type(void)
{
	int i;
	struct lc_audio_info *priv = get_audio_info_priv();

	for (i = 0; i< SMARTPA_TYPE_MAX; i++) {
		if (!strncmp(priv->smartpa_type_str, smartpa_name[i], strlen(smartpa_name[i]) + 1)) {
			pr_info("[%s] get smartpa type sucess, smartpa index %d\n", __func__, i);
			return (enum smartpa_type)i;
		}
	}
	return INVALID;
}
EXPORT_SYMBOL(get_smartpa_type);

static ssize_t chip_vendor_show(struct device_driver *ddri, char *buf)
{
	int ret = 0;
	struct lc_audio_info *priv = get_audio_info_priv();

	if (buf == NULL) {
		pr_err("[%s] *buf is NULL!\n",  __func__);
		return -EINVAL;
	}
	ret = snprintf(buf, 15, "%s", priv->smartpa_type_str);
	if (ret < 0)
		pr_err("snprintf failed\n");

	return strlen(buf);
}

static ssize_t pa_num_show(struct device_driver *ddri, char *buf)
{
	unsigned int pa_num = smartpainfo_num;
	int ret = 0;

	if (buf == NULL) {
		pr_err("[%s] *buf is NULL!\n",  __func__);
		return -EINVAL;
	}
	ret = snprintf(buf, 3, "%d", pa_num);
	if (ret < 0)
		pr_err("snprintf failed\n");

	return strlen(buf);
}

static DRIVER_ATTR_RO(chip_vendor);
static DRIVER_ATTR_RO(pa_num);

static struct driver_attribute *smartpainfo_attr_list[] = {
	&driver_attr_chip_vendor,
	&driver_attr_pa_num,
};


static int smartpainfo_create_attr(struct device_driver *driver)
{
	int idx, err;
	int num = ARRAY_SIZE(smartpainfo_attr_list);

	if (driver == NULL)
		return -EINVAL;
	for (idx = 0; idx < num; idx++) {
		err = driver_create_file(driver, smartpainfo_attr_list[idx]);
		if (err) {
			pr_notice("%s() driver_create_file %s err:%d\n",
			__func__, smartpainfo_attr_list[idx]->attr.name, err);
			break;
		}
	}
	return err;
}

static int smartpainfo_probe(struct platform_device *dev)
{
	struct platform_driver smartpainfo_driver_hal = smartpainfo_driver_func();
	int ret = 0;
	struct device_node *audio_info_node = NULL;
	uint32_t val = 0;

	pr_info("%s() begin!\n", __func__);

	ret = smartpainfo_create_attr(&smartpainfo_driver_hal.driver);
	if (ret) {
		pr_notice("%s create_attr fail, ret = %d\n", __func__, ret);
		return -1;
	}

	audio_info_node = of_find_compatible_node(NULL, NULL, "mediatek,smartpainfo");
	if (audio_info_node == NULL) {
		pr_err("%s: Cannot find smartpainfo from dts\n", __func__);
	} else {
		ret = of_property_read_u32(audio_info_node, "smartpa_num", &val);
		if (!ret) {
			smartpainfo_num = val;
			pr_err("%s: smartpa_num =%d\n", __func__, smartpainfo_num);
		} else {
			smartpainfo_num = SMARTPA_NUM_ONE;
			pr_err("%s: get fail smartpa_num =%d\n", __func__, smartpainfo_num);
		}
	}

	pr_info("%s done!\n", __func__);
	return 0;
}

static int smartpainfo_remove(struct platform_device *dev)
{
	pr_debug("%s done!\n", __func__);
	return 0;
}

const struct of_device_id smartpainfo_of_match[] = {
	{ .compatible = "mediatek,smartpainfo", },
	{},
};

static struct platform_driver smartpainfo_driver = {
	.probe = smartpainfo_probe,
	.remove = smartpainfo_remove,
	.driver = {
		.name = "smartpainfo",
		.of_match_table = smartpainfo_of_match,
	},
};

struct platform_driver smartpainfo_driver_func(void)
{
	return smartpainfo_driver;
}

static int smartpainfo_mod_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&smartpainfo_driver);
	if (ret)
		pr_err("smartpainfo platform_driver_register error:(%d)\n", ret);

	pr_info("%s() done!\n", __func__);
	return ret;
}

static void smartpainfo_mod_exit(void)
{
	pr_info("%s()\n", __func__);
	platform_driver_unregister(&smartpainfo_driver);
}

module_init(smartpainfo_mod_init);
module_exit(smartpainfo_mod_exit);

MODULE_DESCRIPTION("smartpainfo driver");
MODULE_AUTHOR("wangyongfu");
MODULE_LICENSE("GPL v2");
