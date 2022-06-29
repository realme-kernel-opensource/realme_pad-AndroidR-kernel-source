ifeq ($(CONFIG_OPLUS_CHARGER_MTK),y)

	ifeq ($(CONFIG_MTK_HAFG_20),y)
		obj-y	+= oplus_battery_mtk_fg_20.o
	else ifeq ($(CONFIG_OPLUS_CHARGER_MTK6769),y)
                MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
                subdir-ccflags-y += -Werror
                subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
                subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
                subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/base/power/include
                subdir-ccflags-$(CONFIG_USB_MU3D_DRV) += -I$(srctree)/drivers/misc/mediatek/mu3d/drv
                subdir-ccflags-$(CONFIG_USB_MTK_HDRC) += -I$(srctree)/drivers/misc/mediatek/usb20/
                subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6355) += -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6355
                subdir-ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6356) += -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6356
                subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/include
                subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/typec/tcpc/inc
                subdir-ccflags-y += -I$(srctree)/drivers/power/supply/mediatek/misc/
                subdir-ccflags-y += -I$(srctree)/drivers/power/supply/mediatek/battery/
                obj-$(CONFIG_CHARGER_BQ2589X) += oplus_bq2589x_charger.o
                obj-$(CONFIG_CHARGER_BQ2591X) += oplus_bq2591x_charger.o
                obj-$(CONFIG_CHARGER_RT9471) += oplus_rt9471_charger.o
                obj-$(CONFIG_CHARGER_RT9467) += oplus_rt9467_charger.o
                obj-$(CONFIG_CHARGER_SY6970) += oplus_sy6970_charger.o
		obj-$(CONFIG_CHARGER_Z350) += oplus_z350_charger.o
		obj-$(CONFIG_CHARGER_WT6670F) += oplus_wt6670f_charger.o
		obj-$(CONFIG_CHARGER_SGM41510) += oplus_sgm41510_charger.o
                obj-y   += oplus_battery_mtk6769R.o
                obj-$(CONFIG_CHARGER_SY6974) += oplus_sy6974.o
	else
		//obj-y	+= oplus_battery_mtk.o
	endif
	//obj-y	+= oplus_bq24196.o

endif

	obj-y   += oplus_short_ic.o
