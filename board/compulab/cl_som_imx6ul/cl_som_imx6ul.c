/*
 * Copyright (C) 2015 CompuLab LTD.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <fsl_esdhc.h>
#include <i2c.h>
#include <miiphy.h>
#include <linux/sizes.h>
#include <mmc.h>
#include <netdev.h>
#include <usb.h>
#include <asm/arch/clock.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/mx6ul_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/io.h>
#include <usb/ehci-fsl.h>
#include "../common/eeprom.h"
#include "../common/common.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_DAT3_CD_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |	\
	PAD_CTL_PUS_100K_DOWN  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define I2C_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |               \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_ODE)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP | PAD_CTL_PUE |     \
	PAD_CTL_SPEED_HIGH   |                                  \
	PAD_CTL_DSE_48ohm   | PAD_CTL_SRE_FAST)

#define MDIO_PAD_CTRL  (PAD_CTL_PUS_100K_UP | PAD_CTL_PUE |     \
	PAD_CTL_DSE_48ohm   | PAD_CTL_SRE_FAST | PAD_CTL_ODE)

#define ENET_CLK_PAD_CTRL  (PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST)

#define ENET_RX_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |          \
	PAD_CTL_SPEED_HIGH   | PAD_CTL_SRE_FAST)

#define SPI_PAD_CTRL (PAD_CTL_HYS |				\
	PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#ifndef CONFIG_SPL_BUILD
static int nand_enabled = 0;
#endif

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

	return 0;
}

static iomux_v3_cfg_t const uart3_pads[] = {
	MX6_PAD_UART3_TX_DATA__UART3_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART3_RX_DATA__UART3_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc1_pads[] = {
	MX6_PAD_SD1_CLK__USDHC1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_CMD__USDHC1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA0__USDHC1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA1__USDHC1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA2__USDHC1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DATA3__USDHC1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	/* CD */
	MX6_PAD_UART1_RTS_B__GPIO1_IO19 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	/* WP */
	MX6_PAD_UART1_CTS_B__GPIO1_IO18 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc2_emmc_pads[] = {
	MX6_PAD_NAND_RE_B__USDHC2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_WE_B__USDHC2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA00__USDHC2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA01__USDHC2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA02__USDHC2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA03__USDHC2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA04__USDHC2_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA05__USDHC2_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA06__USDHC2_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA07__USDHC2_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart3_pads, ARRAY_SIZE(uart3_pads));
#ifdef CONFIG_SPL_BUILD
	enable_uart_clk(1);
#endif
}

#ifdef CONFIG_SPI
static iomux_v3_cfg_t const spi_pads[] = {
	MX6_PAD_ENET2_TX_DATA1__ECSPI4_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_ENET2_TX_EN__ECSPI4_MOSI  | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_ENET2_TX_CLK__ECSPI4_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_ENET2_RX_ER__GPIO2_IO15   | MUX_PAD_CTRL(SPI_PAD_CTRL),
};

static void board_spi_init(void)
{
	imx_iomux_v3_setup_multiple_pads(spi_pads, ARRAY_SIZE(spi_pads));
#ifdef CONFIG_SPL_BUILD
	enable_spi_clk(1, 3);
#endif
}

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	if (bus == 3 && cs == 0)
		return IMX_GPIO_NR(2, 15);

	return -1;
}
#else
static void board_spi_init(void) { return; }
#endif

#ifdef CONFIG_FSL_ESDHC
static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC1_BASE_ADDR, 0, 4},
	{USDHC2_BASE_ADDR, 0, 8},
};

#define USDHC1_CD_GPIO	IMX_GPIO_NR(1, 19)
#define USDHC1_WP_GPIO	IMX_GPIO_NR(1, 18)

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC1_BASE_ADDR:
		ret = !gpio_get_value(USDHC1_CD_GPIO);
		break;
	case USDHC2_BASE_ADDR:
		/* eMMC device is always on */
		ret = 1;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
#ifdef CONFIG_SPL_BUILD
	imx_iomux_v3_setup_multiple_pads(usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
	return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
#else
	int i, ret;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    USDHC1
	 * mmc1                    USDHC2
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
			gpio_direction_input(USDHC1_CD_GPIO);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
			break;
		case 1:
			if (nand_enabled)
				return -EINVAL;
			/* emmc enabled configuration */
			imx_iomux_v3_setup_multiple_pads(
				usdhc2_emmc_pads, ARRAY_SIZE(usdhc2_emmc_pads));
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers (%d) than supported by the board\n", i + 1);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret) {
			printf("Warning: failed to initialize mmc dev %d\n", i);
			return ret;
		}
	}
#endif
	return 0;
}
#endif

#ifdef CONFIG_USB_EHCI_MX6
#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)
#define USB_OTG1_VBUS		IMX_GPIO_NR(1, 4)

static iomux_v3_cfg_t const usb_otg_pads[] = {
	MX6_PAD_GPIO1_IO04__GPIO1_IO04	| MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_GPIO1_IO00__ANATOP_OTG1_ID | MUX_PAD_CTRL(NO_PAD_CTRL),
};

/* At default the 3v3 enables the MIC2026 for VBUS power */
static void setup_usb(void)
{
	imx_iomux_v3_setup_multiple_pads(usb_otg_pads,
					 ARRAY_SIZE(usb_otg_pads));

	if (gpio_request(USB_OTG1_VBUS, "usb_otg1_vbus")) {
		printf("USB OTG1 vbus gpio request failed\n");
		return;
	}

	gpio_direction_output(USB_OTG1_VBUS, 0);
}

int board_usb_phy_mode(int port)
{
	if (port == 1)
		return USB_INIT_HOST;
	else
		return usb_phy_mode(port);
}

int board_ehci_hcd_init(int port)
{
	int ret;
	u32 *usbnc_usb_ctrl;

	if (port > 1)
		return -EINVAL;

	usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET +
				 port * 4);

	/* Set Power polarity */
	setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);

	if (port == 1)
		return 0;

	ret = gpio_direction_output(USB_OTG1_VBUS, 1);
	if (ret)
		return ret;

	mdelay(1);

	return 0;
}
#else
static void setup_usb(void) { return; }
#endif

#ifdef CONFIG_FEC_MXC
static iomux_v3_cfg_t const fec1_pads[] = {
	MX6_PAD_GPIO1_IO06__ENET1_MDIO | MUX_PAD_CTRL(MDIO_PAD_CTRL),
	MX6_PAD_GPIO1_IO07__ENET1_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_DATA0__ENET1_TDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_DATA1__ENET1_TDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_EN__ENET1_TX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_TX_CLK__ENET1_REF_CLK1 | MUX_PAD_CTRL(ENET_CLK_PAD_CTRL),
	MX6_PAD_ENET1_RX_DATA0__ENET1_RDATA00 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_DATA1__ENET1_RDATA01 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_ER__ENET1_RX_ER | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET1_RX_EN__ENET1_RX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_SNVS_TAMPER8__GPIO5_IO08 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_iomux_fec(int fec_id)
{
	if (fec_id == 0)
		imx_iomux_v3_setup_multiple_pads(fec1_pads,
						 ARRAY_SIZE(fec1_pads));
}

static int handle_mac_address(char *env_var, uint eeprom_bus)
{
	unsigned char enetaddr[6];
	int rc;

	rc = eth_getenv_enetaddr(env_var, enetaddr);
	if (rc)
		return 0;

	rc = cl_eeprom_read_mac_addr(enetaddr, eeprom_bus);
	if (rc)
		return rc;

	if (!is_valid_ethaddr(enetaddr))
		return -1;

	return eth_setenv_enetaddr(env_var, enetaddr);
}

#define ENET1_PHY_nRST IMX_GPIO_NR(5, 8)
#define NO_MAC_ADDR	"No MAC address found for %s\n"

int board_eth_init(bd_t *bis)
{
	if (handle_mac_address("ethaddr", CONFIG_SYS_I2C_EEPROM_BUS))
		printf(NO_MAC_ADDR, "primary NIC");

	setup_iomux_fec(CONFIG_FEC_ENET_DEV);

	gpio_request(ENET1_PHY_nRST, "ENET1_PHY_nRST");
	gpio_direction_output(ENET1_PHY_nRST, 0);
	mdelay(10);
	gpio_direction_output(ENET1_PHY_nRST, 1);


	return fecmxc_initialize_multi(bis, CONFIG_FEC_ENET_DEV,
				       CONFIG_FEC_MXC_PHYADDR, IMX_FEC_BASE);
}

static int setup_fec(int fec_id)
{
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int ret;

	if (fec_id > 0)
		return -ENODEV;

	/*
	 * Use 50M ksz8041 REF_CLK for ENET1,
	 * set gpr1[13], clear gpr1[17].
	 */

	clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC1_MASK,
				IOMUX_GPR1_FEC1_CLOCK_MUX2_SEL_MASK);

	ret = enable_fec_anatop_clock(fec_id, ENET_50MHZ);
	if (ret)
		return ret;

	enable_enet_clk(1);
	return 0;
}
#else
static int setup_fec(int fec_id) { return 0; }
#endif

#ifdef CONFIG_SYS_I2C_MXC
/* I2C1 */
static struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_GPIO1_IO02__I2C1_SCL |
			MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_GPIO1_IO02__GPIO1_IO02 |
			MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 2),
	},
	.sda = {
		.i2c_mode = MX6_PAD_GPIO1_IO03__I2C1_SDA |
			MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_GPIO1_IO03__GPIO1_IO03 |
			MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 3),
	},
};

/* I2C3 */
static struct i2c_pads_info i2c_pad_info3 = {
	.scl = {
		.i2c_mode =  MX6_PAD_UART1_TX_DATA__I2C3_SCL |
			MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_UART1_TX_DATA__GPIO1_IO16 |
			MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 16),
	},
	.sda = {
		.i2c_mode = MX6_PAD_UART1_RX_DATA__I2C3_SDA |
			MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_UART1_RX_DATA__GPIO1_IO17 |
			MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 17),
	},
};

static int setup_one_i2c(int busnum, struct i2c_pads_info *pads)
{
	int ret;

	ret = setup_i2c(busnum, CONFIG_SYS_I2C_SPEED, 0x7f, pads);
	if (ret)
		printf("Warning: I2C%d setup failed: %d\n", busnum, ret);

	return ret;
}

static int board_setup_i2c(void)
{
	int ret = 0, err;

	/* i2c<x>_pads are wierd macro variables; we can't use an array */
	/* configures the logic bus #0 */
	err = setup_one_i2c(1, &i2c_pad_info1);
	if (err)
		ret = err;

	/* configures the logic bus #1 */
	/* this is the SOC eeprom bus */
	err = setup_one_i2c(3, &i2c_pad_info3);
	if (err)
		ret = err;

	return ret;
}
#else
static int board_setup_i2c(void) { return 0; }
#endif

#ifdef CONFIG_NAND_MXS
static iomux_v3_cfg_t const usdhc2_nand_pads[] = {
	MX6_PAD_NAND_CLE__RAWNAND_CLE | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_ALE__RAWNAND_ALE | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_WP_B__RAWNAND_WP_B | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_READY_B__RAWNAND_READY_B | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_CE0_B__RAWNAND_CE0_B | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_CE1_B__RAWNAND_CE1_B | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_RE_B__RAWNAND_RE_B | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_WE_B__RAWNAND_WE_B | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA00__RAWNAND_DATA00 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA01__RAWNAND_DATA01 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA02__RAWNAND_DATA02 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA03__RAWNAND_DATA03 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA04__RAWNAND_DATA04 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA05__RAWNAND_DATA05 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA06__RAWNAND_DATA06 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_NAND_DATA07__RAWNAND_DATA07 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

#define NAND_ENABLE	IMX_GPIO_NR(4, 13)
static iomux_v3_cfg_t const nand_enable_pads[] = {
	MX6_PAD_NAND_CE0_B__GPIO4_IO13 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static void get_nand_enable_state(void) {
	imx_iomux_v3_setup_multiple_pads(
		nand_enable_pads, ARRAY_SIZE(nand_enable_pads));
	gpio_direction_input(NAND_ENABLE);
	mdelay(1);
	nand_enabled = gpio_get_value(NAND_ENABLE);
}

static void setup_gpmi_nand(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	get_nand_enable_state();

	/* nand enabled configuration */
	imx_iomux_v3_setup_multiple_pads(
		usdhc2_nand_pads, ARRAY_SIZE(usdhc2_nand_pads));

	setup_gpmi_io_clk((MXC_CCM_CS2CDR_ENFC_CLK_PODF(0xf) |
		MXC_CCM_CS2CDR_ENFC_CLK_PRED(1) |
		MXC_CCM_CS2CDR_ENFC_CLK_SEL(0)));

	/* enable apbh clock gating */
	setbits_le32(&mxc_ccm->CCGR0, MXC_CCM_CCGR0_APBHDMA_MASK);
}
#else
static void setup_gpmi_nand(void) {}
#endif

int board_early_init_f(void)
{
	setup_iomux_uart();

	return 0;
}

int board_init(void)
{
	/* Address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	setup_gpmi_nand();

	setup_fec(CONFIG_FEC_ENET_DEV);

	setup_usb();

	board_spi_init();

	board_setup_i2c();

	return 0;
}

int board_late_init(void)
{
	setenv("board_name", "CL-SOM-iMX6UL");
	return 0;
}

int checkboard(void)
{
	puts("Board: CL-SOM-iMX6UL\n");
	return 0;
}

u32 get_board_rev(void)
{
	return cl_eeprom_get_board_rev(CONFIG_SYS_I2C_EEPROM_BUS);
}

int misc_init_r(void)
{
	cl_print_pcb_info();

	return 0;
}

#ifdef CONFIG_SPL_BUILD
#include <libfdt.h>
#include <spl.h>
#include <asm/arch/mx6-ddr.h>

static struct mx6ul_iomux_grp_regs mx6_grp_ioregs = {
	.grp_addds = 0x00000030,
	.grp_ddrmode_ctl = 0x00020000,
	.grp_b0ds = 0x00000030,
	.grp_ctlds = 0x00000030,
	.grp_b1ds = 0x00000030,
	.grp_ddrpke = 0x00000000,
	.grp_ddrmode = 0x00020000,
	.grp_ddr_type = 0x000c0000,
};

static struct mx6ul_iomux_ddr_regs mx6_ddr_ioregs = {
	.dram_dqm0 = 0x00000030,
	.dram_dqm1 = 0x00000030,
	.dram_ras = 0x00000030,
	.dram_cas = 0x00000030,
	.dram_odt0 = 0x00000030,
	.dram_odt1 = 0x00000030,
	.dram_sdba2 = 0x00000000,
	.dram_sdclk_0 = 0x00000008,
	.dram_sdqs0 = 0x00000038,
	.dram_sdqs1 = 0x00000030,
	.dram_reset = 0x00000030,
};

static struct mx6_mmdc_calibration mx6_mmcd_calib = {
	.p0_mpwldectrl0 = 0x001C0013,
	.p0_mpdgctrl0 = 0x415C015C,
	.p0_mprddlctl = 0x4040484C,
	.p0_mpwrdlctl = 0x40404C42,
};

struct mx6_ddr_sysinfo ddr_sysinfo = {
	.dsize = 0,
	.cs_density = 20,
	.ncs = 1,
	.cs1_mirror = 0,
	.rtt_wr = 2,
	.rtt_nom = 1,		/* RTT_Nom = RZQ/2 */
	.walat = 1,		/* Write additional latency */
	.ralat = 5,		/* Read additional latency */
	.mif3_mode = 3,		/* Command prediction working mode */
	.bi_on = 1,		/* Bank interleaving enabled */
	.sde_to_rst = 0x10,	/* 14 cycles, 200us (JEDEC default) */
	.rst_to_cke = 0x23,	/* 33 cycles, 500us (JEDEC default) */
	.ddr_type = DDR_TYPE_DDR3,
};

static struct mx6_ddr3_cfg mem_ddr = {
	.mem_speed = 800,
	.density = 4,
	.width = 16,
	.banks = 8,
	.rowaddr = 15,
	.coladdr = 10,
	.pagesz = 2,
	.trcd = 1375,
	.trcmin = 4875,
	.trasmin = 3500,
};

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0xFFFFFFFF, &ccm->CCGR0);
	writel(0xFFFFFFFF, &ccm->CCGR1);
	writel(0xFFFFFFFF, &ccm->CCGR2);
	writel(0xFFFFFFFF, &ccm->CCGR3);
	writel(0xFFFFFFFF, &ccm->CCGR4);
	writel(0xFFFFFFFF, &ccm->CCGR5);
	writel(0xFFFFFFFF, &ccm->CCGR6);
	writel(0xFFFFFFFF, &ccm->CCGR7);
}

static void spl_dram_init(void)
{
	mx6ul_dram_iocfg(mem_ddr.width, &mx6_ddr_ioregs, &mx6_grp_ioregs);
	mx6_dram_cfg(&ddr_sysinfo, &mx6_mmcd_calib, &mem_ddr);
}

void board_init_f(ulong dummy)
{
	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	ccgr_init();

	/* setup GP timer */
	timer_init();

#ifdef CONFIG_SPL_SPI_SUPPORT
	board_spi_init();
#endif

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

	/* DDR initialization */
	spl_dram_init();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}

void spl_board_init(void)
{
	u32 boot_device = spl_boot_device();

	if (boot_device == BOOT_DEVICE_SPI)
		puts("Booting from SPI flash\n");
	else if (boot_device == BOOT_DEVICE_MMC1)
		puts("Booting from MMC\n");
	else
		puts("Unknown boot device\n");
}
#endif
