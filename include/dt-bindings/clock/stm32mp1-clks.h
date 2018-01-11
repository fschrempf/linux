/* SPDX-License-Identifier: GPL-2.0 or BSD-3-Clause */
/*
 * Copyright (C) STMicroelectronics 2017 - All Rights Reserved
 * Author: Gabriel Fernandez <gabriel.fernandez@st.com> for STMicroelectronics.
 */

/* OSCILLATOR clocks */
#define CK_HSE 0
#define CK_CSI 1
#define CK_LSI 2
#define CK_LSE 3
#define CK_HSI 4
#define CK_HSE_DIV2 5

/* Bus clocks */
#define TIM2 6
#define TIM3 7
#define TIM4 8
#define TIM5 9
#define TIM6 10
#define TIM7 11
#define TIM12 12
#define TIM13 13
#define TIM14 14
#define LPTIM1 15
#define SPI2 16
#define SPI3 17
#define USART2 18
#define USART3 19
#define UART4 20
#define UART5 21
#define UART7 22
#define UART8 23
#define I2C1 24
#define I2C2 25
#define I2C3 26
#define I2C5 27
#define SPDIF 28
#define CEC 29
#define DAC12 30
#define MDIO 31
#define TIM1 32
#define TIM8 33
#define TIM15 34
#define TIM16 35
#define TIM17 36
#define SPI1 37
#define SPI4 38
#define SPI5 39
#define USART6 40
#define SAI1 41
#define SAI2 42
#define SAI3 43
#define DFSDM 44
#define FDCAN 45
#define LPTIM2 46
#define LPTIM3 47
#define LPTIM4 48
#define LPTIM5 49
#define SAI4 50
#define SYSCFG 51
#define VREF 52
#define TMPSENS 53
#define PMBCTRL 54
#define HDP 55
#define LTDC 56
#define DSI 57
#define IWDG2 58
#define USBPHY 59
#define STGENRO 60
#define SPI6 61
#define I2C4 62
#define I2C6 63
#define USART1 64
#define RTCAPB 65
#define TZC 66
#define TZPC 67
#define BSEC 68
#define STGEN 69
#define DMA1 70
#define DMA2 71
#define DMAMUX 72
#define ADC12 73
#define USBO 74
#define SDMMC3 75
#define DCMI 76
#define CRYP2 77
#define HASH2 78
#define RNG2 79
#define CRC2 80
#define HSEM 81
#define IPCC 82
#define GPIOA 83
#define GPIOB 84
#define GPIOC 85
#define GPIOD 86
#define GPIOE 87
#define GPIOF 88
#define GPIOG 89
#define GPIOH 90
#define GPIOI 91
#define GPIOJ 92
#define GPIOK 93
#define GPIOZ 94
#define CRYP1 95
#define HASH1 96
#define RNG1 97
#define BKPSRAM 98
#define MDMA 99
#define DMA2D 100
#define GPU 101
#define ETHCK 102
#define ETHTX 103
#define ETHRX 104
#define ETHMAC 105
#define FMC 106
#define QSPI 107
#define SDMMC1 108
#define SDMMC2 109
#define CRC1 110
#define USBH 111
#define ETHSTP 112

/* Kernel clocks */
#define SDMMC1_K 113
#define SDMMC2_K 114
#define SDMMC3_K 115
#define FMC_K 116
#define QSPI_K 117
#define ETHMAC_K 118
#define RNG1_K 119
#define RNG2_K 120
#define GPU_K 121
#define USBPHY_K 122
#define STGEN_K 123
#define SPDIF_K 124
#define SPI1_K 125
#define SPI2_K 126
#define SPI3_K 127
#define SPI4_K 128
#define SPI5_K 129
#define SPI6_K 130
#define CEC_K 131
#define I2C1_K 132
#define I2C2_K 133
#define I2C3_K 134
#define I2C4_K 135
#define I2C5_K 136
#define I2C6_K 137
#define LPTIM1_K 138
#define LPTIM2_K 139
#define LPTIM3_K 140
#define LPTIM4_K 141
#define LPTIM5_K 142
#define USART1_K 143
#define USART2_K 144
#define USART3_K 145
#define UART4_K 146
#define UART5_K 147
#define USART6_K 148
#define UART7_K 149
#define UART8_K 150
#define DFSDM_K 151
#define FDCAN_K 152
#define SAI1_K 153
#define SAI2_K 154
#define SAI3_K 155
#define SAI4_K 156
#define ADC12_K 157
#define DSI_K 158
#define ADFSDM_K 159
#define USBO_K 160
#define LTDC_K 161

/* PLL */
#define PLL1 162
#define PLL2 163
#define PLL3 164
#define PLL4 165

/* ODF */
#define PLL1_P 166
#define PLL1_Q 167
#define PLL1_R 168
#define PLL2_P 169
#define PLL2_Q 170
#define PLL2_R 171
#define PLL3_P 172
#define PLL3_Q 173
#define PLL3_R 174
#define PLL4_P 175
#define PLL4_Q 176
#define PLL4_R 177

/* AUX */
#define RTC 178

/* MCLK */
#define CK_PER 179
#define CK_MPU 180
#define CK_AXI 181
#define CK_MCU 182

/* Time base */
#define TIM2_K 183
#define TIM3_K 184
#define TIM4_K 185
#define TIM5_K 186
#define TIM6_K 187
#define TIM7_K 188
#define TIM12_K 189
#define TIM13_K 190
#define TIM14_K 191
#define TIM1_K 192
#define TIM8_K 193
#define TIM15_K 194
#define TIM16_K 195
#define TIM17_K 196

/* MCO clocks */
#define CK_MCO1 197
#define CK_MCO2 198

/* TRACE & DEBUG clocks */
#define DBG 199
#define CK_DBG 200
#define CK_TRACE 201


/* DDR */
#define DDRC1 202
#define DDRC1LP 203
#define DDRC2 204
#define DDRC2LP 205
#define DDRPHYC 206
#define DDRPHYCLP 207
#define DDRCAPB 208
#define DDRCAPBLP 209
#define AXIDCG 210
#define DDRPHYCAPB 211
#define DDRPHYCAPBLP 212
#define DDRPERFM 213

#define STM32MP1_LAST_CLK 214
