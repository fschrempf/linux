#include <linux/io.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#define SRAM_ETZPC_BASE 0x1003F000
#define SRAM_ETZPC_SIZE 0x00001000

#define ETZPC_TZMA0_SIZE        0x00
#define ETZPC_DECPROT0          0x10
#define ETZPC_DECPROT_LOCK0     0x30
#define ETZPC_IP_VER            0x3F4

#define TZMA0_SIZE_CUT1         0x80000009
#define IP_VER_CUT2             0x00000020

#define DECPROT_MPU_SEC         0x00
#define DECPROT_NO_SEC_MCU_ISOL 0x02
#define DECPROT_NO_SEC          0x03
#define DECPROT_MASK            0x03
#define DECPROT_NB_BITS         0x02
#define DECPROT_NB_ENTRIES      0x10

#define DECPROT_UNLOCKED        0x00
#define DECPROT_LOCKED          0x01
#define DECPROT_LOCK_MASK       0x01
#define DECPROT_LOCK_NB_BITS    0x01
#define DECPROT_LOCK_NB_ENTRIES 0x20

static u32 *etzpc, *etzpc_decprot, *etzpc_decprot_lock;

static const u32 ip_addr[] = {
	0x5c008000,	/* 00 stgenc */
	0x54000000,	/* 01 bkpsram */
	0x5c003000,	/* 02 iwdg1 */
	0x5c000000,	/* 03 usart1 */
	0x5c001000,	/* 04 spi6 */
	0x5c002000,	/* 05 i2c4 */
	0xffffffff,	/* 06 reserved */
	0x54003000,	/* 07 rng1 */
	0x54002000,	/* 08 hash1 */
	0x54001000,	/* 09 cryp1 */
	0x5a003000,	/* 0A ddrctrl */
	0x5a004000,	/* 0B ddrphyc */
	0x5c009000,	/* 0C i2c6 */
	0xffffffff,	/* 0D reserved */
	0xffffffff,	/* 0E reserved */
	0xffffffff,	/* 0F reserved */
	0x40000000,	/* 10 tim2 */
	0x40001000,	/* 11 tim3 */
	0x40002000,	/* 12 tim4 */
	0x40003000,	/* 13 tim5 */
	0x40004000,	/* 14 tim6 */
	0x40005000,	/* 15 tim7 */
	0x40006000,	/* 16 tim12 */
	0x40007000,	/* 17 tim13 */
	0x40008000,	/* 18 tim14 */
	0x40009000,	/* 19 lptim1 */
	0x4000a000,	/* 1A wwdg1 */
	0x4000b000,	/* 1B spi2 */
	0x4000c000,	/* 1C spi3 */
	0x4000d000,	/* 1D spdifrx */
	0x4000e000,	/* 1E usart2 */
	0x4000f000,	/* 1F usart3 */
	0x40010000,	/* 20 uart4 */
	0x40011000,	/* 21 uart5 */
	0x40012000,	/* 22 i2c1 */
	0x40013000,	/* 23 i2c2 */
	0x40014000,	/* 24 i2c3 */
	0x40015000,	/* 25 i2c5 */
	0x40016000,	/* 26 cec */
	0x40017000,	/* 27 dac */
	0x40018000,	/* 28 uart7 */
	0x40019000,	/* 29 uart8 */
	0xffffffff,	/* 2A reserved */
	0xffffffff,	/* 2B reserved */
	0x4001c000,	/* 2C mdios */
	0xffffffff,	/* 2D reserved */
	0xffffffff,	/* 2E reserved */
	0xffffffff,	/* 2F reserved */
	0x44000000,	/* 30 tim1 */
	0x44001000,	/* 31 tim8 */
	0xffffffff,	/* 32 reserved */
	0x44003000,	/* 33 usart6 */
	0x44004000,	/* 34 spi1 */
	0x44005000,	/* 35 spi4 */
	0x44006000,	/* 36 tim15 */
	0x44007000,	/* 37 tim16 */
	0x44008000,	/* 38 tim17 */
	0x44009000,	/* 39 spi5 */
	0x4400a000,	/* 3A sai1 */
	0x4400b000,	/* 3B sai2 */
	0x4400c000,	/* 3C sai3 */
	0x4400d000,	/* 3D dfsdm */
	0x4400e000,	/* 3E tt_fdcan */
	0xffffffff,	/* 3F reserved */
	0x50021000,	/* 40 lptim2 */
	0x50022000,	/* 41 lptim3 */
	0x50023000,	/* 42 lptim4 */
	0x50024000,	/* 43 lptim5 */
	0x50027000,	/* 44 sai4 */
	0x50025000,	/* 45 vrefbuf */
	0x4c006000,	/* 46 dcmi */
	0x4c004000,	/* 47 crc2 */
	0x48003000,	/* 48 adc */
	0x4c002000,	/* 49 hash2 */
	0x4c003000,	/* 4A rng2 */
	0x4c005000,	/* 4B cryp2 */
	0xffffffff,	/* 4C reserved */
	0xffffffff,	/* 4D reserved */
	0xffffffff,	/* 4E reserved */
	0xffffffff,	/* 4F reserved */
	0xffffffff,	/* 50 sram1 */
	0xffffffff,	/* 51 sram2 */
	0xffffffff,	/* 52 sram3 */
	0xffffffff,	/* 53 sram4 */
	0xffffffff,	/* 54 retram */
	0x49000000,	/* 55 otg */
	0x48004000,	/* 56 sdmmc3 */
	0x48005000,	/* 57 dlybsd3 */
	0x48000000,	/* 58 dma1 */
	0x48001000,	/* 59 dma2 */
	0x48002000,	/* 5A dmamux */
	0x58002000,	/* 5B fmc */
	0x58003000,	/* 5C qspi */
	0x58004000,	/* 5D dlybq */
	0x5800a000,	/* 5E eth */
	0xffffffff,	/* 5F reserved */
};

static const u32 decprot_dflt[] = {
	0x000FFC00, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
};

static const u32 lock_dflt[] = {
	0x00000000, 0x00000000, 0x00000000,
};

static bool is_etzpc_granted(struct device *dev)
{
	int idx, status;
	struct resource res;

	if (of_address_to_resource(dev->of_node, 0, &res))
		return true;

	for (idx = 0; idx < ARRAY_SIZE(ip_addr); idx++) {
		if (ip_addr[idx] == res.start) {
			status = etzpc_decprot[idx / DECPROT_NB_ENTRIES];
			status >>= DECPROT_NB_BITS * (idx % DECPROT_NB_ENTRIES);
			status &= DECPROT_MASK;

			return (status == DECPROT_NO_SEC);
		}
	}

	return true;
}

static int etzpc_bus_set_virt_etzpc(void)
{
	u32 *tzma0_size, *ip_ver;

	etzpc = (u32 *)ioremap(SRAM_ETZPC_BASE, SRAM_ETZPC_SIZE);
	if (!etzpc) {
		pr_err("Cannot map\n");
		return -ENXIO;
	}

	etzpc_decprot = etzpc + (ETZPC_DECPROT0 / sizeof(u32));
	etzpc_decprot_lock = etzpc + (ETZPC_DECPROT_LOCK0 / sizeof(u32));

	/* Set default values if cut1 and not set by ATF */
	ip_ver = etzpc + (ETZPC_IP_VER / sizeof(u32));
	tzma0_size = etzpc + (ETZPC_TZMA0_SIZE / sizeof(u32));

	if (*ip_ver != IP_VER_CUT2 && *tzma0_size != TZMA0_SIZE_CUT1) {
		memcpy(etzpc_decprot, decprot_dflt, sizeof(decprot_dflt));
		memcpy(etzpc_decprot_lock, lock_dflt, sizeof(lock_dflt));
		*tzma0_size = TZMA0_SIZE_CUT1;
	}

	return 0;
}

static int etzpc_bus_notifier_call(struct notifier_block *nb,
				   unsigned long event, void *dev)
{
	if (event == BUS_NOTIFY_BIND_DRIVER &&
	    !is_etzpc_granted((struct device *)dev)) {
		dev_err(dev, "Access for %s not granted\n", dev_name(dev));
		return NOTIFY_BAD;
	}

	return NOTIFY_DONE;
}

static struct notifier_block platform_nb = {
	.notifier_call = etzpc_bus_notifier_call,
};

static int __init etzpc_bus_init(void)
{
	int ret;

	ret = etzpc_bus_set_virt_etzpc();
	if (ret)
		return ret;

	return bus_register_notifier(&platform_bus_type, &platform_nb);
}

/* To be called before DT (which is 'arch_initcall') is parsed */
postcore_initcall(etzpc_bus_init);
