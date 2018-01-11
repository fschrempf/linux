/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) STMicroelectronics 2017 - All Rights Reserved
 * Author: Olivier Bideau <olivier.bideau@st.com> for STMicroelectronics.
 */

#define RCC_OCENSETR		0x0C
#define RCC_OCENCLRR		0x10
#define RCC_OCRDYR		0x808
#define RCC_HSICFGR		0x18
#define RCC_RDLSICR		0x144
#define RCC_PLL1CR		0x80
#define RCC_PLL1CFGR1		0x84
#define RCC_PLL1CFGR2		0x88
#define RCC_PLL2CR		0x94
#define RCC_PLL2CFGR1		0x98
#define RCC_PLL2CFGR2		0x9C
#define RCC_PLL3CR		0x880
#define RCC_PLL3CFGR1		0x884
#define RCC_PLL3CFGR2		0x888
#define RCC_PLL4CR		0x894
#define RCC_PLL4CFGR1		0x898
#define RCC_PLL4CFGR2		0x89C
#define RCC_APB1ENSETR		0xA00
#define RCC_APB2ENSETR		0xA08
#define RCC_APB3ENSETR		0xA10
#define RCC_APB4ENSETR		0x200
#define RCC_APB5ENSETR		0x208
#define RCC_AHB2ENSETR		0xA18
#define RCC_AHB3ENSETR		0xA20
#define RCC_AHB4ENSETR		0xA28
#define RCC_AHB5ENSETR		0x210
#define RCC_AHB6ENSETR		0x218
#define RCC_AHB6LPENSETR	0x318
#define RCC_RCK12SELR		0x28
#define RCC_RCK3SELR		0x820
#define RCC_RCK4SELR		0x824
#define RCC_MPCKSELR		0x20
#define RCC_ASSCKSELR		0x24
#define RCC_MSSCKSELR		0x48
#define RCC_SPI6CKSELR		0xC4
#define RCC_SDMMC12CKSELR	0x8F4
#define RCC_SDMMC3CKSELR	0x8F8
#define RCC_FMCCKSELR		0x904
#define RCC_I2C4CKSELR		0xC0
#define RCC_I2C12CKSELR		0x8C0
#define RCC_I2C35CKSELR		0x8C4
#define RCC_UART1CKSELR		0xC8
#define RCC_QSPICKSELR		0x900
#define RCC_ETHCKSELR		0x8FC
#define RCC_RNG1CKSELR		0xCC
#define RCC_RNG2CKSELR		0x920
#define RCC_GPUCKSELR		0x938
#define RCC_USBCKSELR		0x91C
#define RCC_STGENCKSELR		0xD4
#define RCC_SPDIFCKSELR		0x914
#define RCC_SPI2S1CKSELR	0x8D8
#define RCC_SPI2S23CKSELR	0x8DC
#define RCC_SPI2S45CKSELR	0x8E0
#define RCC_CECCKSELR		0x918
#define RCC_LPTIM1CKSELR	0x934
#define RCC_LPTIM23CKSELR	0x930
#define RCC_LPTIM45CKSELR	0x92C
#define RCC_UART24CKSELR	0x8E8
#define RCC_UART35CKSELR	0x8EC
#define RCC_UART6CKSELR		0x8E4
#define RCC_UART78CKSELR	0x8F0
#define RCC_DFSDMCKSELR		0x910
#define RCC_FDCANCKSELR		0x90C
#define RCC_SAI1CKSELR		0x8C8
#define RCC_SAI2CKSELR		0x8CC
#define RCC_SAI3CKSELR		0x8D0
#define RCC_SAI4CKSELR		0x8D4
#define RCC_ADCCKSELR		0x928
#define RCC_MPCKDIVR		0x2C
#define RCC_DSICKSELR		0x924
#define RCC_CPERCKSELR		0xD0
#define RCC_GRSTCSETR		0x404
#define RCC_MCO1CFGR		0x800
#define RCC_MCO2CFGR		0x804
#define RCC_BDCR		0x140
#define RCC_AXIDIVR		0x30
#define RCC_MCUDIVR		0x830
#define RCC_APB1DIVR		0x834
#define RCC_APB2DIVR		0x838
#define RCC_APB3DIVR		0x83C
#define RCC_APB4DIVR		0x3C
#define RCC_APB5DIVR		0x40
#define RCC_TIMG1PRER		0x828
#define RCC_TIMG2PRER		0x82C
#define RCC_RTCDIVR		0x44
#define RCC_DBGCFGR		0x80C

#define PLL800			0x1
#define NO_DIV			0x1
#define LINEAR_DIV		0x2
#define REFCLK			16000000
#define DIVN_MASK		0x1FF
#define DIVM_MASK		0x3F
#define DIVM_SHIFT		16
#define DIVN_SHIFT		0
#define FRAC_OFFSET		0xC
#define FRAC_MASK		0x1FFF
#define FRAC_SHIFT		3
#define DIVP_MASK		0x7F
#define ODF_MASK		0x7
#define ODF_SHIFT		4

#define RCC_CLR			0x4
#define RCC_LPEN		0x100
#define RCC_LPEN_CLR		0x104

#define BCLK_BANK		5
#define TIM_BANK		200
#define KCLK_BANK		121
#define PLL_BANK		170
#define ODF_BANK		175
#define ACLK_BANK		190
#define MCLK_BANK		195
#define CKREF_BANK		215
#define MCO_BANK		220
#define DBG_BANK		222
#define OSC_BANK		225

static const char * const ref12_parents[] = {
	"ck_hsi", "ck_hse"
};

static const char * const ref3_parents[] = {
	"ck_hsi", "ck_hse", "ck_csi"
};

static const char * const ref4_parents[] = {
	"ck_hsi", "ck_hse", "ck_csi"
};

static const char * const cpu_src[] = {
	"ck_hsi", "ck_hse", "pll1_p"
};

static const char * const axi_src[] = {
	"ck_hsi", "ck_hse", "pll2_p", "pll3_p"
};

static const char * const per_src[] = {
	"ck_hsi", "ck_csi", "ck_hse"
};

static const char * const mcu_src[] = {
	"ck_hsi", "ck_hse", "ck_csi", "pll3_p"
};

static const char * const sdmmc1_src[] = {
	"ck_axi", "pll3_r", "pll4_p", "ck_hsi"
};

static const char * const sdmmc2_src[] = {
	"ck_axi", "pll3_r", "pll4_p", "ck_hsi"
};

static const char * const sdmmc3_src[] = {
	"ck_axi", "pll3_r", "pll4_p", "ck_hsi"
};

static const char * const fmc_src[] = {
	"ck_axi", "pll3_r", "pll4_p", "ck_per"
};

static const char * const qspi_src[] = {
	"ck_axi", "pll3_r", "pll4_p", "ck_per"
};

static const char * const eth_src[] = {
	"pll4_p", "pll3_q"
};

static const char * const rng_src[] = {
	"ck_csi", "pll4_r", "ck_lse", "ck_lsi"
};

static const char * const gpu_src[] = {
	"pll2_q"
};

static const char * const usbphy_src[] = {
	"ck_hse", "pll4_r", "clk-hse-div2"
};

static const char * const usbo_src[] = {
	"pll4_r", "ck_usbo_48m"
};

static const char * const stgen_src[] = {
	"ck_hsi", "ck_hse"
};

static const char * const spdif_src[] = {
	"pll4_p", "pll3_q", "ck_hsi"
};

static const char * const spi1_src[] = {
	"pll4_p", "pll3_q", "i2s_ckin", "ck_per"
};

static const char * const spi2_src[] = {
	"pll4_p", "pll3_q", "i2s_ckin", "ck_per"
};

static const char * const spi3_src[] = {
	"pll4_p", "pll3_q", "i2s_ckin", "ck_per"
};

static const char * const spi4_src[] = {
	"pclk2", "pll4_q", "ck_hsi", "ck_csi", "ck_hse"
};

const char * const spi5_src[] = {
	"pclk2", "pll4_q", "ck_hsi", "ck_csi", "ck_hse"
};

static const char * const spi6_src[] = {
	"pclk5", "pll4_q", "ck_hsi", "ck_csi", "ck_hse", "pll3_q"
};

static const char * const cec_src[] = {
	"ck_lse", "ck_lsi", "ck_csi"
};

static const char * const i2c1_src[] = {
	"pclk1", "pll4_r", "ck_hsi", "ck_csi"
};

static const char * const i2c2_src[] = {
	"pclk1", "pll4_r", "ck_hsi", "ck_csi"
};

static const char * const i2c3_src[] = {
	"pclk1", "pll4_r", "ck_hsi", "ck_csi"
};

static const char * const i2c4_src[] = {
	"pclk5", "pll3_q", "ck_hsi", "ck_csi"
};

static const char * const i2c5_src[] = {
	"pclk1", "pll4_r", "ck_hsi", "ck_csi"
};

static const char * const i2c6_src[] = {
	"pclk5", "pll3_q", "ck_hsi", "ck_csi"
};

static const char * const lptim1_src[] = {
	"pclk1", "pll4_p", "pll3_q", "ck_lse", "ck_lsi", "ck_per"
};

static const char * const lptim2_src[] = {
	"pclk3", "pll4_q", "ck_per", "ck_lse", "ck_lsi"
};

static const char * const lptim3_src[] = {
	"pclk3", "pll4_q", "ck_per", "ck_lse", "ck_lsi"
};

static const char * const lptim4_src[] = {
	"pclk3", "pll4_p", "pll3_q", "ck_lse", "ck_lsi", "ck_per"
};

static const char * const lptim5_src[] = {
	"pclk3", "pll4_p", "pll3_q", "ck_lse", "ck_lsi", "ck_per"
};

static const char * const usart1_src[] = {
	"pclk5", "pll3_q", "ck_hsi", "ck_csi", "pll4_q", "ck_hse"
};

const char * const usart2_src[] = {
	"pclk1", "pll4_q", "ck_hsi", "ck_csi", "ck_hse"
};

static const char * const usart3_src[] = {
	"pclk1", "pll4_q", "ck_hsi", "ck_csi", "ck_hse"
};

static const char * const uart4_src[] = {
	"pclk1", "pll4_q", "ck_hsi", "ck_csi", "ck_hse"
};

static const char * const uart5_src[] = {
	"pclk1", "pll4_q", "ck_hsi", "ck_csi", "ck_hse"
};

static const char * const usart6_src[] = {
	"pclk2", "pll4_q", "ck_hsi", "ck_csi", "ck_hse"
};

static const char * const uart7_src[] = {
	"pclk1", "pll4_q", "ck_hsi", "ck_csi", "ck_hse"
};

static const char * const uart8_src[] = {
	"pclk1", "pll4_q", "ck_hsi", "ck_csi", "ck_hse"
};

static const char * const adfsdm_src[] = {
	"pll4_q", "pll3_q", "i2s_ckin", "ck_per"
};

static const char * const dfsdm_src[] = {
	"pclk2", "ck_mcu"
};

static const char * const fdcan_src[] = {
	"ck_hse", "pll3_q", "pll4_q"
};

static const char * const sai_src[] = {
	"pll4_q", "pll3_q", "i2s_ckin", "ck_per"
};

static const char * const sai2_src[] = {
	"pll4_q", "pll3_q", "i2s_ckin", "ck_per", "spdif_ck_symb"
};

static const char * const adc12_src[] = {
	"pll4_q", "ck_per"
};

static const char * const dsi_src[] = {
	"ck_dsi_phy", "pll4_p"
};

static const char * const rtc_src[] = {
	"off", "ck_lse", "ck_lsi", "ck_hse_rtc"
};

static const char * const ltdc_src[] = {
	"pll4_q"
};

static const char * const ck_trace_src[] = {
	"ck_axi"
};

static const char * const mco1_src[] = {
	"ck_hsi", "ck_hse", "ck_csi", "ck_lsi", "ck_lse"
};

static const char * const mco2_src[] = {
	"ck_mpu", "ck_axi", "ck_mcu", "pll4_p", "ck_hse", "ck_hsi"
};

static const struct clk_div_table axi_div_table[] = {
	{ 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 4 },
	{ 4, 4 }, { 5, 4 }, { 6, 4 }, { 7, 4 },
	{ 0 },
};

static const struct clk_div_table mcu_div_table[] = {
	{ 0, 1 }, { 1, 2 }, { 2, 4 }, { 3, 8 },
	{ 4, 16 }, { 5, 32 }, { 6, 64 }, { 7, 128 },
	{ 8, 512 }, { 9, 512 }, { 10, 512}, { 11, 512 },
	{ 12, 512 }, { 13, 512 }, { 14, 512}, { 15, 512 },
	{ 0 },
};

static const struct clk_div_table ck_trace_div_table[] = {
	{ 0, 1 }, { 1, 2 }, { 2, 4 }, { 3, 8 },
	{ 4, 16 }, { 5, 16 }, { 6, 16 }, { 7, 16 },
	{ 0 },
};

static const struct clk_div_table apb_div_table[] = {
	{ 0, 1 }, { 1, 2 }, { 2, 4 }, { 3, 8 },
	{ 4, 16 }, { 5, 16 }, { 6, 16 }, { 7, 16 },
	{ 0 },
};

struct stm32_pll_cfg {
	u32	offset;
};

struct stm32_cktim_cfg {
	u32 offset_apbdiv;
	u32 offset_timpre;
};

struct clk_hw *_clk_register_pll(struct device *dev,
				 struct clk_hw_onecell_data *clk_data,
				 void __iomem *base,
				 spinlock_t *lock,
				 const struct clock_config *cfg);

struct clk_hw *_clk_register_cktim(struct device *dev,
				   struct clk_hw_onecell_data *clk_data,
				   void __iomem *base,
				   spinlock_t *lock,
				   const struct clock_config *cfg);

#define OSC(_id, _name, _parent, _offset_set, _offset_clr, _bit_idx)\
	    STM32_GATE(_id, _name, _parent, CLK_IGNORE_UNUSED,\
		       _offset_set, _offset_clr, _bit_idx)

#define PLL(_id, _name, _parent, _flags, _offset)\
{\
	.id	= _id,\
	.name		= _name,\
	.parent_name	= _parent,\
	.flags		= _flags,\
	.cfg =  &(struct stm32_pll_cfg) {\
		.offset		= _offset,\
	},\
	.func = _clk_register_pll,\
}

#define STM32_COMPOSITE_NO_MUX(_id, _name, _parent, _flags, _cfg)\
{\
	.id	= _id,\
	.name		= _name,\
	.parent_name	= _parent,\
	.flags		= _flags,\
	.cfg = &(struct stm32_composite_cfg)_cfg,\
}

#define STM32_M(_mux_offset,\
		_div_offset,\
		_div_width,\
		_div_table) \
{\
	_NO_GATE,\
	_MUX(_mux_offset, 0, 2, CLK_MUX_READ_ONLY),\
	_DIV_TABLE(_div_offset, 0, _div_width, _div_table,\
		   CLK_DIVIDER_READ_ONLY),\
}

#define STM32_MUX(_id, _name, _parents, _flags,	_mux_offset, _div_offset,\
		  _div_width, _div_table)\
		  STM32_COMPOSITE(_id, _name, _parents, _flags,\
				  STM32_M(_mux_offset, _div_offset,\
					  _div_width, _div_table))

#define CKTIM(_name, _parent, _flags, _offset_apbdiv, _offset_timpre)\
{\
	.id	= NO_ID,\
	.name		= _name,\
	.parent_name	= _parent,\
	.flags		= _flags,\
	.cfg =  &(struct stm32_cktim_cfg) {\
		.offset_apbdiv	= _offset_apbdiv,\
		.offset_timpre	= _offset_timpre,\
	},\
	.func = _clk_register_cktim,\
}

#define TIM(_id, _name, _parent, _offset_set, _bit_idx)\
	    STM32_GATE(_id, _name, _parent, CLK_SET_RATE_PARENT,\
		       _offset_set, _offset_set + RCC_CLR, _bit_idx)\

#define APB1_F(_id, _name, _gate_idx, _flags)\
	       STM32_GATE(_id, _name, "pclk1", _flags,\
			  RCC_APB1ENSETR, RCC_APB1ENSETR + RCC_CLR, _gate_idx)

#define APB2_F(_id, _name, _gate_idx, _flags)\
	       STM32_GATE(_id, _name, "pclk2", _flags,\
			  RCC_APB2ENSETR, RCC_APB2ENSETR + RCC_CLR, _gate_idx)

#define APB3_F(_id, _name, _gate_idx, _flags)\
	       STM32_GATE(_id, _name, "pclk3", _flags,\
			  RCC_APB3ENSETR, RCC_APB3ENSETR + RCC_CLR, _gate_idx)

#define APB4_F(_id, _name, _gate_idx, _flags)\
	       STM32_GATE(_id, _name, "pclk4", _flags,\
			  RCC_APB4ENSETR, RCC_APB4ENSETR + RCC_CLR, _gate_idx)

#define APB5_F(_id, _name, _gate_idx, _flags)\
	       STM32_GATE_S(_id, _name, "pclk5", _flags,\
			    RCC_APB5ENSETR, RCC_APB5ENSETR + RCC_CLR, _gate_idx)

#define AHB2_F(_id, _name, _gate_idx, _flags)\
	       STM32_GATE(_id, _name, "ck_mcu", _flags,\
			  RCC_AHB2ENSETR, RCC_AHB2ENSETR + RCC_CLR, _gate_idx)

#define AHB3_F(_id, _name, _gate_idx, _flags)\
	       STM32_GATE(_id, _name, "ck_mcu", _flags,\
			  RCC_AHB3ENSETR, RCC_AHB3ENSETR + RCC_CLR, _gate_idx)

#define AHB4_F(_id, _name, _gate_idx, _flags)\
	       STM32_GATE(_id, _name, "ck_mcu", _flags,\
			  RCC_AHB4ENSETR, RCC_AHB4ENSETR + RCC_CLR, _gate_idx)

#define AHB5_F(_id, _name, _gate_idx, _flags)\
	       STM32_GATE_S(_id, _name, "ck_axi", _flags,\
			    RCC_AHB5ENSETR, RCC_AHB5ENSETR + RCC_CLR, _gate_idx)

#define AHB6_F(_id, _name, _gate_idx, _flags)\
	       STM32_GATE(_id, _name, "ck_axi", _flags,\
			  RCC_AHB6ENSETR, RCC_AHB6ENSETR + RCC_CLR, _gate_idx)

#define AHB6LP_F(_id, _name, _gate_idx, _flags)\
		 STM32_GATE(_id, _name, "ck_axi", _flags,\
			    RCC_AHB6LPENSETR, RCC_AHB6LPENSETR + RCC_CLR,\
			    _gate_idx)

#define K_GATEMUX(_gate_offset, _bit_idx, _mux_offset) \
{\
	_NO_DIV,\
	_GATE(_gate_offset, _gate_offset + RCC_CLR, _bit_idx, 0),\
	_MUX(_mux_offset, 0, 3, 0),\
}

#define KER_F(_id, _gate_offset,\
		_bit_idx,\
		_mux_offset,\
		_name, _parents, _flags)\
	      STM32_COMPOSITE(_id, _name, _parents, _flags,\
			      K_GATEMUX(_gate_offset, _bit_idx, _mux_offset))

#define K_AHB6(_id, _bit_idx, _mux_offset, _name, _parents)\
	       KER_F(_id, RCC_AHB6ENSETR, _bit_idx, _mux_offset,\
		     _name, _parents, 0)

#define K_AHB2_F(_id, _bit_idx, _mux_offset, _name, _parents, _flags)\
	       KER_F(_id, RCC_AHB2ENSETR, _bit_idx, _mux_offset,\
		     _name, _parents, _flags)

#define K_AHB2(_id, _bit_idx, _mux_offset, _name, _parents)\
	       K_AHB2_F(_id, _bit_idx, _mux_offset, _name, _parents, 0)

#define K_AHB3(_id, _bit_idx, _mux_offset, _name, _parents)\
	       KER_F(_id, RCC_AHB3ENSETR, _bit_idx, _mux_offset,\
		     _name, _parents, 0)

#define K_APB1(_id, _bit_idx, _mux_offset, _name, _parents)\
	       KER_F(_id, RCC_APB1ENSETR, _bit_idx, _mux_offset,\
		     _name, _parents, 0)

#define K_APB2(_id, _bit_idx, _mux_offset, _name, _parents)\
	       KER_F(_id, RCC_APB2ENSETR, _bit_idx, _mux_offset,\
		     _name, _parents, 0)

#define K_APB3(_id, _bit_idx, _mux_offset, _name, _parents)\
	       KER_F(_id, RCC_APB3ENSETR, _bit_idx, _mux_offset,\
		     _name, _parents, 0)

#define K_APB4(_id, _bit_idx, _mux_offset, _name, _parents)\
	       KER_F(_id, RCC_APB4ENSETR, _bit_idx, _mux_offset,\
		     _name, _parents, 0)

/* SECURE REGISTERS */
#define K_AHB5(_id, _bit_idx, _mux_offset, _name, _parents)\
	       STM32_COMPOSITE_S(_id, _name, _parents, 0,\
				 K_GATEMUX(RCC_AHB5ENSETR, _bit_idx,\
					   _mux_offset))

#define K_APB5_F(_id, _bit_idx, _mux_offset, _name, _parents, _flags)\
		 STM32_COMPOSITE_S(_id, _name, _parents, _flags,\
				   K_GATEMUX(RCC_APB5ENSETR, _bit_idx,\
					     _mux_offset))

#define K_APB5(_id, _bit_idx, _mux_offset, _name, _parents)\
	       STM32_COMPOSITE_S(_id, _name, _parents, 0,\
				 K_GATEMUX(RCC_APB5ENSETR, _bit_idx,\
					   _mux_offset))

#define K_USB(_id, _gate_offset,\
		_bit_idx,\
		_mux_offset,\
		_mux_bit,\
		_mux_width,\
		_name, _parents, _flags)\
	      STM32_COMPOSITE(_id, _name, _parents, _flags,\
			      STM32_GATEMUX(_gate_offset, _bit_idx,\
					    _mux_offset, _mux_bit, _mux_width))

#define RTC_S(_id, _name, _parents, _flags,\
	      _gate_offset, _bit_idx,\
	      _mux_offset, _mux_shift, _mux_width)\
	      STM32_COMPOSITE_S(_id, _name, _parents, _flags,\
				GATEMUX(_gate_offset, _bit_idx,\
					_mux_offset, _mux_shift, _mux_width))

#define MCO(_id, _name, _parents, _flags, _offset)\
	    STM32_COMPOSITE(_id, _name, _parents, _flags,\
			    GATEMUXDIV(_offset, 12, _offset, 0, 3,\
				       _offset, 4, 4))

#define DBG_F(_id, _name, _parents, _flags,\
	      _gate_offset, _gate_bit_idx,\
	      _div_offset, _div_shift, _div_width, _div_table)\
	      STM32_COMPOSITE(_id, _name, _parents, _flags,\
			      GATEDIV(_gate_offset, _gate_bit_idx,\
				      _div_offset, _div_shift, _div_width,\
				      _div_table))

static const struct clock_config stm32mp1_clock_cfg[] = {
	/* Oscillator divider */
	FIXED_FACTOR(CK_HSE_DIV2, "clk-hse-div2", "ck_hse", 0, 1, 2),
	DIV(NO_ID, "clk-hsi-div", "clk-hsi", 0, RCC_HSICFGR, 0, 2,
	    CLK_DIVIDER_READ_ONLY),

	/*  external / internal Oscillators */
	OSC_S(CK_HSE, "ck_hse", "clk-hse", RCC_OCENSETR, RCC_OCENCLRR, 8),
	OSC_S(CK_CSI, "ck_csi", "clk-csi", RCC_OCENSETR, RCC_OCENCLRR, 4),
	OSC_S(CK_HSI, "ck_hsi", "clk-hsi-div", RCC_OCENSETR, RCC_OCENCLRR, 0),
	OSC_S(CK_LSI, "ck_lsi", "clk-lsi", RCC_RDLSICR, RCC_RDLSICR, 0),
	OSC_S(CK_LSE, "ck_lse", "clk-lse", RCC_BDCR, RCC_BDCR, 0),

	/* ref clock pll */
	MUX(NO_ID, "ref1", ref12_parents, CLK_OPS_PARENT_ENABLE, RCC_RCK12SELR,
	    0, 2, CLK_MUX_READ_ONLY),

	MUX(NO_ID, "ref3", ref3_parents, CLK_OPS_PARENT_ENABLE, RCC_RCK3SELR,
	    0, 2, CLK_MUX_READ_ONLY),

	MUX(NO_ID, "ref4", ref4_parents, CLK_OPS_PARENT_ENABLE, RCC_RCK4SELR,
	    0, 2, CLK_MUX_READ_ONLY),

	/* PLLs */
	PLL(PLL1, "pll1", "ref1", CLK_IGNORE_UNUSED, RCC_PLL1CR),
	PLL(PLL2, "pll2", "ref1", CLK_IGNORE_UNUSED, RCC_PLL2CR),
	PLL(PLL3, "pll3", "ref3", CLK_IGNORE_UNUSED, RCC_PLL3CR),
	PLL(PLL4, "pll4", "ref4", CLK_IGNORE_UNUSED, RCC_PLL4CR),

	/* ODF */
	DIV(PLL1_P, "pll1_p", "pll1", CLK_IGNORE_UNUSED, RCC_PLL1CFGR2, 0, 7,
	    CLK_DIVIDER_READ_ONLY),
	DIV(PLL1_Q, "pll1_q", "pll1", CLK_IGNORE_UNUSED, RCC_PLL1CFGR2, 8, 7,
	    CLK_DIVIDER_READ_ONLY),
	DIV(PLL1_R, "pll1_r", "pll1", CLK_IGNORE_UNUSED, RCC_PLL1CFGR2, 16, 7,
	    CLK_DIVIDER_READ_ONLY),

	DIV(PLL2_P, "pll2_p", "pll2", CLK_IGNORE_UNUSED, RCC_PLL2CFGR2, 0, 7,
	    CLK_DIVIDER_READ_ONLY),
	DIV(PLL2_Q, "pll2_q", "pll2", CLK_IGNORE_UNUSED, RCC_PLL2CFGR2, 8, 7,
	    CLK_DIVIDER_READ_ONLY),
	DIV(PLL2_R, "pll2_r", "pll2", CLK_IGNORE_UNUSED, RCC_PLL2CFGR2, 16, 7,
	    CLK_DIVIDER_READ_ONLY),

	DIV_S(PLL3_P, "pll3_p", "pll3", CLK_IGNORE_UNUSED, RCC_PLL3CFGR2, 0, 7,
	      0),
	DIV_S(PLL3_Q, "pll3_q", "pll3", CLK_IGNORE_UNUSED, RCC_PLL3CFGR2, 8, 7,
	      0),
	DIV_S(PLL3_R, "pll3_r", "pll3", CLK_IGNORE_UNUSED, RCC_PLL3CFGR2, 16, 7,
	      0),

	DIV(PLL4_P, "pll4_p", "pll4", CLK_IGNORE_UNUSED, RCC_PLL4CFGR2, 0, 7,
	    0),
	DIV(PLL4_Q, "pll4_q", "pll4", CLK_IGNORE_UNUSED, RCC_PLL4CFGR2, 8, 7,
	    0),
	DIV(PLL4_R, "pll4_r", "pll4", CLK_IGNORE_UNUSED, RCC_PLL4CFGR2, 16, 7,
	    0),

	/* MUX system clocks */
	MUX(CK_PER, "ck_per", per_src, CLK_OPS_PARENT_ENABLE,
	    RCC_CPERCKSELR, 0, 2, 0),

	MUX(CK_MPU, "ck_mpu", cpu_src, CLK_OPS_PARENT_ENABLE,
	    RCC_MPCKSELR, 0, 2, CLK_MUX_READ_ONLY),

	STM32_MUX(CK_AXI, "ck_axi", axi_src, CLK_IGNORE_UNUSED |
		  CLK_OPS_PARENT_ENABLE, RCC_ASSCKSELR,
		  RCC_AXIDIVR, 3, axi_div_table),

	STM32_MUX(CK_MCU, "ck_mcu", mcu_src, CLK_IGNORE_UNUSED |
		  CLK_OPS_PARENT_ENABLE, RCC_MSSCKSELR,
		  RCC_MCUDIVR, 4, mcu_div_table),

	DIV_TABLE(NO_ID, "pclk1", "ck_mcu", CLK_IGNORE_UNUSED, RCC_APB1DIVR, 0,
		  3, CLK_DIVIDER_READ_ONLY, apb_div_table),
	DIV_TABLE(NO_ID, "pclk2", "ck_mcu", CLK_IGNORE_UNUSED, RCC_APB2DIVR, 0,
		  3, CLK_DIVIDER_READ_ONLY, apb_div_table),
	DIV_TABLE(NO_ID, "pclk3", "ck_mcu", CLK_IGNORE_UNUSED, RCC_APB3DIVR, 0,
		  3, CLK_DIVIDER_READ_ONLY, apb_div_table),
	DIV_TABLE(NO_ID, "pclk4", "ck_axi", CLK_IGNORE_UNUSED, RCC_APB4DIVR, 0,
		  3, CLK_DIVIDER_READ_ONLY, apb_div_table),
	DIV_TABLE(NO_ID, "pclk5", "ck_axi", CLK_IGNORE_UNUSED, RCC_APB5DIVR, 0,
		  3, CLK_DIVIDER_READ_ONLY, apb_div_table),

	/* Kernel Timers */
	CKTIM("ck1_tim", "pclk1", CLK_IGNORE_UNUSED, RCC_APB1DIVR,
	      RCC_TIMG1PRER),
	CKTIM("ck2_tim", "pclk2", CLK_IGNORE_UNUSED, RCC_APB2DIVR,
	      RCC_TIMG2PRER),

	/* Timers */
	TIM(TIM2_K,	"tim2_k",  "ck1_tim", RCC_APB1ENSETR, 0),
	TIM(TIM3_K,	"tim3_k",  "ck1_tim", RCC_APB1ENSETR, 1),
	TIM(TIM4_K,	"tim4_k",  "ck1_tim", RCC_APB1ENSETR, 2),
	TIM(TIM5_K,	"tim5_k",  "ck1_tim", RCC_APB1ENSETR, 3),
	TIM(TIM6_K,	"tim6_k",  "ck1_tim", RCC_APB1ENSETR, 4),
	TIM(TIM7_K,	"tim7_k",  "ck1_tim", RCC_APB1ENSETR, 5),
	TIM(TIM12_K,	"tim12_k", "ck1_tim", RCC_APB1ENSETR, 6),
	TIM(TIM13_K,	"tim13_k", "ck1_tim", RCC_APB1ENSETR, 7),
	TIM(TIM14_K,	"tim14_k", "ck1_tim", RCC_APB1ENSETR, 8),
	TIM(TIM1_K,	"tim1_k",  "ck2_tim", RCC_APB2ENSETR, 0),
	TIM(TIM8_K,	"tim8_k",  "ck2_tim", RCC_APB2ENSETR, 1),
	TIM(TIM15_K,	"tim15_k", "ck2_tim", RCC_APB2ENSETR, 2),
	TIM(TIM16_K,	"tim16_k", "ck2_tim", RCC_APB2ENSETR, 3),
	TIM(TIM17_K,	"tim17_k", "ck2_tim", RCC_APB2ENSETR, 4),

	/* Perif clocks */
	APB1_F(TIM2,	"tim2",		 0, CLK_IGNORE_UNUSED),
	APB1_F(TIM3,	"tim3",		 1, CLK_IGNORE_UNUSED),
	APB1_F(TIM4,	"tim4",		 2, CLK_IGNORE_UNUSED),
	APB1_F(TIM5,	"tim5",		 3, CLK_IGNORE_UNUSED),
	APB1_F(TIM6,	"tim6",		 4, CLK_IGNORE_UNUSED),
	APB1_F(TIM7,	"tim7",		 5, CLK_IGNORE_UNUSED),
	APB1_F(TIM12,	"tim12",	 6, CLK_IGNORE_UNUSED),
	APB1_F(TIM13,	"tim13",	 7, CLK_IGNORE_UNUSED),
	APB1_F(TIM14,	"tim14",	 8, CLK_IGNORE_UNUSED),
	APB1_F(LPTIM1,	"lptim1",	 9, CLK_IGNORE_UNUSED),
	APB1_F(SPI2,	"spi2",		11, CLK_IGNORE_UNUSED),
	APB1_F(SPI3,	"spi3",		12, CLK_IGNORE_UNUSED),
	APB1_F(USART2,	"usart2",	14, CLK_IGNORE_UNUSED),
	APB1_F(USART3,	"usart3",	15, CLK_IGNORE_UNUSED),
	APB1_F(UART4,	"uart4",	16, CLK_IGNORE_UNUSED),
	APB1_F(UART5,	"uart5",	17, CLK_IGNORE_UNUSED),
	APB1_F(UART7,	"uart7",	18, CLK_IGNORE_UNUSED),
	APB1_F(UART8,	"uart8",	19, CLK_IGNORE_UNUSED),
	APB1_F(I2C1,	"i2c1",		21, CLK_IGNORE_UNUSED),
	APB1_F(I2C2,	"i2c2",		22, CLK_IGNORE_UNUSED),
	APB1_F(I2C3,	"i2c3",		23, CLK_IGNORE_UNUSED),
	APB1_F(I2C5,	"i2c5",		24, CLK_IGNORE_UNUSED),
	APB1_F(SPDIF,	"spdif",	26, CLK_IGNORE_UNUSED),
	APB1_F(CEC,	"cec",		27, CLK_IGNORE_UNUSED),
	APB1_F(DAC12,	"dac12",	29, 0),
	APB1_F(MDIO,	"mdio",		31, 0),

	APB2_F(TIM1,	"tim1",		 0, CLK_IGNORE_UNUSED),
	APB2_F(TIM8,	"tim8",		 1, CLK_IGNORE_UNUSED),
	APB2_F(TIM15,	"tim15",	 2, CLK_IGNORE_UNUSED),
	APB2_F(TIM16,	"tim16",	 3, CLK_IGNORE_UNUSED),
	APB2_F(TIM17,	"tim17",	 4, CLK_IGNORE_UNUSED),
	APB2_F(SPI1,	"spi1",		 8, CLK_IGNORE_UNUSED),
	APB2_F(SPI4,	"spi4",		 9, CLK_IGNORE_UNUSED),
	APB2_F(SPI5,	"spi5",		10, CLK_IGNORE_UNUSED),
	APB2_F(USART6,	"usart6",	13, CLK_IGNORE_UNUSED),
	APB2_F(SAI1,	"sai1",		16, CLK_IGNORE_UNUSED),
	APB2_F(SAI2,	"sai2",		17, CLK_IGNORE_UNUSED),
	APB2_F(SAI3,	"sai3",		18, CLK_IGNORE_UNUSED),
	APB2_F(DFSDM,	"dfsdm",	20, CLK_IGNORE_UNUSED),
	APB2_F(FDCAN,	"fdcan",	24, CLK_IGNORE_UNUSED),

	APB3_F(LPTIM2,	"lptim2",	 0, CLK_IGNORE_UNUSED),
	APB3_F(LPTIM3,	"lptim3",	 1, CLK_IGNORE_UNUSED),
	APB3_F(LPTIM4,	"lptim4",	 2, CLK_IGNORE_UNUSED),
	APB3_F(LPTIM5,	"lptim5",	 3, CLK_IGNORE_UNUSED),
	APB3_F(SAI4,	"sai4",		 8, CLK_IGNORE_UNUSED),
	APB3_F(SYSCFG,	"syscfg",	11, 0),
	APB3_F(VREF,	"vref",		13, 0),
	APB3_F(TMPSENS,	"tmpsens",	16, 0),
	APB3_F(PMBCTRL,	"pmbctrl",	17, 0),
	APB3_F(HDP,	"hdp",		20, 0),

	APB4_F(LTDC,	"ltdc",		 0, CLK_IGNORE_UNUSED),
	APB4_F(DSI,	"dsi",		 4, CLK_IGNORE_UNUSED),
	APB4_F(IWDG2,	"iwdg2",	15, 0),
	APB4_F(USBPHY,	"usbphy",	16, CLK_IGNORE_UNUSED),
	APB4_F(STGENRO,	"stgenro",	20, 0),

	APB5_F(SPI6,	"spi6",		 0, CLK_IGNORE_UNUSED),
	APB5_F(I2C4,	"i2c4",		 2, CLK_IGNORE_UNUSED),
	APB5_F(I2C6,	"i2c6",		 3, CLK_IGNORE_UNUSED),
	APB5_F(USART1,	"usart1",	 4, CLK_IGNORE_UNUSED),
	APB5_F(RTCAPB,	"rtcapb",	 8, CLK_IGNORE_UNUSED |
	       CLK_IS_CRITICAL),
	APB5_F(TZC,	"tzc",		12, CLK_IGNORE_UNUSED),
	APB5_F(TZPC,	"tzpc",		13, CLK_IGNORE_UNUSED),
	APB5_F(BSEC,	"bsec",		16, CLK_IGNORE_UNUSED),
	APB5_F(STGEN,	"stgen",	20, CLK_IGNORE_UNUSED),

	AHB2_F(DMA1,	"dma1",		 0, 0),
	AHB2_F(DMA2,	"dma2",		 1, 0),
	AHB2_F(DMAMUX,	"dmamux",	 2, 0),
	AHB2_F(ADC12,	"adc12",	 5, CLK_IGNORE_UNUSED),
	AHB2_F(USBO,	"usbo",		 8, CLK_IGNORE_UNUSED),
	AHB2_F(SDMMC3,	"sdmmc3",	16, CLK_IGNORE_UNUSED),

	AHB3_F(DCMI,	"dcmi",		 0, 0),
	AHB3_F(CRYP2,	"cryp2",	 4, 0),
	AHB3_F(HASH2,	"hash2",	 5, 0),
	AHB3_F(RNG2,	"rng2",		 6, CLK_IGNORE_UNUSED),
	AHB3_F(CRC2,	"crc2",		 7, 0),
	AHB3_F(HSEM,	"hsem",		11, 0),
	AHB3_F(IPCC,	"ipcc",		12, 0),

	AHB4_F(GPIOA,	"gpioa",	 0, 0),
	AHB4_F(GPIOB,	"gpiob",	 1, 0),
	AHB4_F(GPIOC,	"gpioc",	 2, 0),
	AHB4_F(GPIOD,	"gpiod",	 3, 0),
	AHB4_F(GPIOE,	"gpioe",	 4, 0),
	AHB4_F(GPIOF,	"gpiof",	 5, 0),
	AHB4_F(GPIOG,	"gpiog",	 6, 0),
	AHB4_F(GPIOH,	"gpioh",	 7, 0),
	AHB4_F(GPIOI,	"gpioi",	 8, 0),
	AHB4_F(GPIOJ,	"gpioj",	 9, 0),
	AHB4_F(GPIOK,	"gpiok",	10, 0),

	AHB5_F(GPIOZ,	"gpioz",	 0, CLK_IGNORE_UNUSED),
	AHB5_F(CRYP1,	"cryp1",	 4, CLK_IGNORE_UNUSED),
	AHB5_F(HASH1,	"hash1",	 5, CLK_IGNORE_UNUSED),
	AHB5_F(RNG1,	"rng1",		 6, CLK_IGNORE_UNUSED),
	AHB5_F(BKPSRAM,	"bkpsram",	 8, CLK_IGNORE_UNUSED),

	AHB6_F(MDMA,	"mdma",		 0, 0),
	AHB6_F(DMA2D,	"dma2d",	 4, 0),
	AHB6_F(GPU,	"gpu",		 5, CLK_IGNORE_UNUSED),
	AHB6_F(ETHCK,	"ethck",	 7, 0),
	AHB6_F(ETHTX,	"ethtx",	 8, 0),
	AHB6_F(ETHRX,	"ethrx",	 9, 0),
	AHB6_F(ETHMAC,	"ethmac",	10, CLK_IGNORE_UNUSED),
	AHB6_F(FMC,	"fmc",		12, CLK_IGNORE_UNUSED),
	AHB6_F(QSPI,	"qspi",		14, CLK_IGNORE_UNUSED),
	AHB6_F(SDMMC1,	"sdmmc1",	16, CLK_IGNORE_UNUSED),
	AHB6_F(SDMMC2,	"sdmmc2",	17, CLK_IGNORE_UNUSED),
	AHB6_F(CRC1,	"crc1",		20, 0),
	AHB6_F(USBH,	"usbh",		24, 0),

	AHB6LP_F(ETHSTP, "ethstp",	11, 0),

	/* Kernel clocks */
	K_AHB6(SDMMC1_K, 16, RCC_SDMMC12CKSELR, "sdmmc1_k", sdmmc1_src),
	K_AHB6(SDMMC2_K, 17, RCC_SDMMC12CKSELR,	"sdmmc2_k", sdmmc2_src),
	K_AHB2(SDMMC3_K, 16, RCC_SDMMC3CKSELR,	"sdmmc3_k", sdmmc3_src),
	K_AHB6(FMC_K, 12, RCC_FMCCKSELR, "fmc_k", fmc_src),
	K_AHB6(QSPI_K, 14, RCC_QSPICKSELR, "qspi_k", qspi_src),
	K_AHB6(ETHMAC_K, 10, RCC_ETHCKSELR, "ethmac_k", eth_src),
	K_AHB5(RNG1_K, 6, RCC_RNG1CKSELR, "rng1_k", rng_src),
	K_AHB3(RNG2_K, 6, RCC_RNG2CKSELR, "rng2_k", rng_src),
	K_AHB6(GPU_K, 5, RCC_GPUCKSELR,	"gpu_k", gpu_src),
	K_APB4(USBPHY_K, 16, RCC_USBCKSELR, "usbphy_k", usbphy_src),
	K_APB5_F(STGEN_K, 20, RCC_STGENCKSELR,	"stgen_k",  stgen_src,
		 CLK_IGNORE_UNUSED),
	K_APB1(SPDIF_K, 26, RCC_SPDIFCKSELR,	"spdif_k",  spdif_src),
	K_APB2(SPI1_K, 8, RCC_SPI2S1CKSELR,	"spi1_k",   spi1_src),
	K_APB1(SPI2_K, 11, RCC_SPI2S23CKSELR,	"spi2_k",   spi2_src),
	K_APB1(SPI3_K, 12, RCC_SPI2S23CKSELR,	"spi3_k",   spi3_src),
	K_APB2(SPI4_K, 9, RCC_SPI2S45CKSELR,	"spi4_k",   spi4_src),
	K_APB2(SPI5_K, 10, RCC_SPI2S45CKSELR,	"spi5_k",   spi5_src),
	K_APB5(SPI6_K, 0, RCC_SPI6CKSELR, "spi6_k", spi6_src),
	K_APB1(CEC_K, 27, RCC_CECCKSELR,	"cec_k",    cec_src),
	K_APB1(I2C1_K, 21, RCC_I2C12CKSELR,	"i2c1_k",   i2c1_src),
	K_APB1(I2C2_K,   22, RCC_I2C12CKSELR,	"i2c2_k",   i2c2_src),
	K_APB1(I2C3_K,   23, RCC_I2C35CKSELR,	"i2c3_k",   i2c3_src),
	K_APB5(I2C4_K,    2, RCC_I2C4CKSELR,	"i2c4_k",   i2c4_src),
	K_APB1(I2C5_K,   24, RCC_I2C35CKSELR,	"i2c5_k",   i2c5_src),
	K_APB5(I2C6_K,	  3, RCC_I2C4CKSELR,	"i2c6_k",   i2c6_src),
	K_APB1(LPTIM1_K,  9, RCC_LPTIM1CKSELR,	"lptim1_k", lptim1_src),
	K_APB3(LPTIM2_K,  0, RCC_LPTIM23CKSELR,	"lptim2_k", lptim2_src),
	K_APB3(LPTIM3_K,  1, RCC_LPTIM23CKSELR,	"lptim3_k", lptim3_src),
	K_APB3(LPTIM4_K,  2, RCC_LPTIM45CKSELR,	"lptim4_k", lptim4_src),
	K_APB3(LPTIM5_K,  3, RCC_LPTIM45CKSELR,	"lptim5_k", lptim5_src),
	K_APB5(USART1_K,  4, RCC_UART1CKSELR,	"usart1_k", usart1_src),
	K_APB1(USART2_K, 14, RCC_UART24CKSELR,	"usart2_k", usart2_src),
	K_APB1(USART3_K, 15, RCC_UART35CKSELR,	"usart3_k", usart3_src),
	K_APB1(UART4_K,  16, RCC_UART24CKSELR,	"uart4_k",  uart4_src),
	K_APB1(UART5_K,	 17, RCC_UART35CKSELR,	"uart5_k",  uart5_src),
	K_APB2(USART6_K, 13, RCC_UART6CKSELR,	"uart6_k",  usart6_src),
	K_APB1(UART7_K,  18, RCC_UART78CKSELR,	"uart7_k",  uart7_src),
	K_APB1(UART8_K,  19, RCC_UART78CKSELR,	"uart8_k",  uart8_src),
	K_APB2(DFSDM_K,  20, RCC_DFSDMCKSELR,	"dfsdm_k",  dfsdm_src),
	K_APB2(FDCAN_K,  24, RCC_FDCANCKSELR,	"fdcan_k",  fdcan_src),
	K_APB2(SAI1_K,   16, RCC_SAI1CKSELR,	"sai1_k",   sai_src),
	K_APB2(SAI2_K,   17, RCC_SAI2CKSELR,	"sai2_k",   sai2_src),
	K_APB2(SAI3_K,   18, RCC_SAI3CKSELR,	"sai3_k",   sai_src),
	K_APB3(SAI4_K,    8, RCC_SAI4CKSELR,	"sai4_k",   sai_src),

	K_AHB2_F(ADC12_K, 5, RCC_ADCCKSELR, "adc12_k", adc12_src,
		 CLK_IGNORE_UNUSED),

	K_APB4(DSI_K, 4, RCC_DSICKSELR,	"dsi_k", dsi_src),
	K_APB2(ADFSDM_K, 21, RCC_SAI1CKSELR,	"adfsdm_k", adfsdm_src),

	K_USB(USBO_K, RCC_AHB2ENSETR, 8, RCC_USBCKSELR, 4, 1, "usbo_k",
	      usbo_src, 0),

	STM32_GATE(LTDC_K, "ltdc_k", "pll4_q", CLK_SET_RATE_PARENT,
		   RCC_APB4ENSETR, RCC_APB4ENSETR + RCC_CLR, 0),

	/* RTC clock */
	DIV_S(NO_ID, "ck_hse_rtc", "ck_hse", 0, RCC_RTCDIVR, 0, 7,
	      CLK_DIVIDER_ALLOW_ZERO),

	RTC_S(RTC, "ck_rtc", rtc_src, CLK_SET_RATE_PARENT,
	      RCC_BDCR, 20, RCC_BDCR, 16, 2),

	MCO(CK_MCO1, "ck_mco1", mco1_src, CLK_SET_RATE_NO_REPARENT,
	    RCC_MCO1CFGR),

	MCO(CK_MCO2, "ck_mco2", mco2_src, CLK_SET_RATE_NO_REPARENT,
	    RCC_MCO2CFGR),

	FIXED_FACTOR(NO_ID, "ck_axi_div2", "ck_axi", 0, 1, 2),

	GATE(DBG, "ck_apb_dbg", "ck_axi_div2", 0, RCC_DBGCFGR, 8, 0),

	GATE(CK_DBG, "ck_sys_dbg", "ck_axi", 0, RCC_DBGCFGR, 8, 0),

	DBG_F(CK_TRACE, "ck_trace", ck_trace_src, 0, RCC_DBGCFGR, 9,
	      RCC_DBGCFGR, 0, 3, ck_trace_div_table),
};
