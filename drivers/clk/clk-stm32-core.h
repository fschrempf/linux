/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) STMicroelectronics 2017 - All Rights Reserved
 * Author: Gabriel Fernandez <gabriel.fernandez@st.com> for STMicroelectronics.
 */

void stm32_rcc_init(struct device_node *np,
		    const struct of_device_id *match_data);

struct stm32_clock_match_data {
	const struct clock_config *cfg;
	unsigned int num;
	unsigned int maxbinding;
};

/* STM32 Gate clock with ready bit and backup domain management */
struct stm32_gate {
	struct clk_hw hw;
	void __iomem *reg_set;
	void __iomem *reg_clr;
	u8 bit_idx;
	/* lock gate enable/disable registers */
	spinlock_t *lock;
};

#define to_stm32_gate_clk(_hw) container_of(_hw, struct stm32_gate, hw)


struct stm32_gate_cfg {
	u32 offset_set;
	u32 offset_clr;
	u8 bit_idx;
};

struct muxdiv_cfg {
	u32 offset;
	u8 shift;
	u8 width;
	const struct clk_div_table *table;
};

struct composite_clk_gcfg_t {
	u8 flags;
	const struct clk_ops *ops;
};

struct stm32_composite_cfg {
	struct stm32_gate_cfg		*gate;
	struct muxdiv_cfg		*mux;
	struct muxdiv_cfg		*div;
	struct composite_clk_gcfg_t	*mux_c;
	struct composite_clk_gcfg_t	*div_c;
	struct composite_clk_gcfg_t	*gate_c;
};

struct clk_hw *clk_register_stm32_gate(struct device *dev,
				       const char *name,
				       const char *parent_name,
				       void __iomem *base,
				       const struct stm32_gate_cfg *gate,
				       unsigned long flags, spinlock_t *lock);

/*
 * General config definition of a composite clock (only clock diviser for rate)
 */

struct clk_mux *_get_cmux(void __iomem *reg, u8 shift, u8 width,
			  u32 flags, spinlock_t *lock);

struct clk_divider *_get_cdiv(void __iomem *reg, u8 shift, u8 width,
			      const struct clk_div_table *table,
			      u32 flags, spinlock_t *lock);

struct stm32_gate *_get_stm32_cgate(void __iomem *reg_set,
				    void __iomem *reg_clr,
				    u8 bit_idx, u32 flags,
				    spinlock_t *lock);

#define _NO_MUX .mux = NULL, .mux_c = NULL
#define _NO_DIV .div = NULL, .div_c = NULL
#define _NO_GATE .gate = NULL, .gate_c = NULL

#define _GATE(_offset_set, _offset_clr, _bit_idx, _gate_flags) \
		.gate = &(struct stm32_gate_cfg) {\
			.offset_set = _offset_set,\
			.offset_clr = _offset_clr,\
			.bit_idx = _bit_idx,\
		},\
		.gate_c = &(struct composite_clk_gcfg_t) {\
			.flags = _gate_flags,\
			.ops = NULL,\
		}

#define _DIV_TABLE(_offset, _shift, _width, _table, _div_flags)\
	.div = &(struct muxdiv_cfg) {\
		_offset,\
		_shift,\
		_width,\
		_table\
	},\
	.div_c = &(struct composite_clk_gcfg_t) {\
		.flags = _div_flags,\
		.ops = NULL,\
	}

#define _DIV(_offset, _shift, _width, _div_flags)\
	_DIV_TABLE(_offset, _shift, _width, NULL, _div_flags)

#define _MUX(_offset, _shift, _width, _mux_flags)\
	.mux = &(struct muxdiv_cfg) {\
		_offset,\
		_shift,\
		_width,\
		NULL,\
	},\
	.mux_c = &(struct composite_clk_gcfg_t) {\
		.flags = _mux_flags,\
		.ops = NULL,\
	}

struct clk_hw *
clk_stm32_register_composite(struct device *dev, const char *name,
			     const char * const *parent_names,
			     int num_parents,
			     void __iomem *base,
			     const struct stm32_composite_cfg *cfg,
			     unsigned long flags,
			     spinlock_t *lock);

struct clock_config {
	u32 id;
	const char *name;
	union {
		const char *parent_name;
		const char * const *parent_names;
	};
	int num_parents;
	unsigned long flags;
	void *cfg;
	struct clk_hw * (*func)(struct device *dev,
				struct clk_hw_onecell_data *clk_data,
				void __iomem *base, spinlock_t *lock,
				const struct clock_config *cfg);
};

#define CLOCK_CONFIG (struct clock_config)

#define NO_ID ~0

struct fixed_factor_cfg {
	unsigned int mult;
	unsigned int div;
};

struct gate_cfg {
	u32 reg_off;
	u8 bit_idx;
	u8 gate_flags;
};

struct divider_cfg {
	u32 reg_off;
	u8 shift;
	u8 width;
	u8 div_flags;
	const struct clk_div_table *table;
};

struct mux_cfg {
	u32 reg_off;
	u8 shift;
	u8 width;
	u8 mux_flags;
};

int  register_hw_clk(struct device *dev,
		     struct clk_hw_onecell_data *clk_data,
		     void __iomem *base,
		     spinlock_t *lock,
		     const struct clock_config *cfg);

struct clk_hw *
_clk_hw_register_fixed_factor(struct device *dev,
			      struct clk_hw_onecell_data *clk_data,
			      void __iomem *base, spinlock_t *lock,
			      const struct clock_config *cfg);

struct clk_hw *
_clk_hw_register_gate(struct device *dev,
		      struct clk_hw_onecell_data *clk_data,
		      void __iomem *base, spinlock_t *lock,
		      const struct clock_config *cfg);

struct clk_hw *
_clk_hw_register_divider_table(struct device *dev,
			       struct clk_hw_onecell_data *clk_data,
			       void __iomem *base, spinlock_t *lock,
			       const struct clock_config *cfg);

struct clk_hw *
_clk_hw_register_mux(struct device *dev,
		     struct clk_hw_onecell_data *clk_data,
		     void __iomem *base, spinlock_t *lock,
		     const struct clock_config *cfg);

struct clk_hw *
_clk_register_stm32_gate(struct device *dev,
			 struct clk_hw_onecell_data *clk_data,
			 void __iomem *base, spinlock_t *lock,
			 const struct clock_config *cfg);

struct clk_hw *
_clk_stm32_register_composite(struct device *dev,
			      struct clk_hw_onecell_data *clk_data,
			      void __iomem *base, spinlock_t *lock,
			      const struct clock_config *cfg);

#define FIXED_FACTOR(_id, _name, _parent, _flags, _mult, _div)\
{\
	.id	= _id,\
	.name		= _name,\
	.parent_name	= _parent,\
	.flags		= _flags,\
	.cfg =  &(struct fixed_factor_cfg) {\
		.mult		= _mult,\
		.div		= _div,\
	},\
	.func = _clk_hw_register_fixed_factor,\
}

#define  GATE(_id, _name, _parent, _flags, _offset, _bit_idx, _gate_flags)\
{\
	.id	= _id,\
	.name		= _name,\
	.parent_name	= _parent,\
	.flags		= _flags,\
	.cfg =  &(struct gate_cfg) {\
		.reg_off	= _offset,\
		.bit_idx	= _bit_idx,\
		.gate_flags	= _gate_flags,\
	},\
	.func = _clk_hw_register_gate,\
}

#define DIV_TABLE(_id, _name, _parent, _flags, _offset, _shift, _width,\
		  _div_flags, _div_table)\
{\
	.id	= _id,\
	.name		= _name,\
	.parent_name	= _parent,\
	.flags		= _flags,\
	.cfg =  &(struct divider_cfg) {\
		.reg_off	= _offset,\
		.shift		= _shift,\
		.width		= _width,\
		.div_flags	= _div_flags,\
		.table		= _div_table,\
	},\
	.func = _clk_hw_register_divider_table,\
}

#define DIV(_id, _name, _parent, _flags, _offset, _shift, _width, _div_flags)\
	DIV_TABLE(_id, _name, _parent, _flags, _offset, _shift, _width,\
		  _div_flags, NULL)

#define MUX(_id, _name, _parents, _flags, _offset, _shift, _width, _mux_flags)\
{\
	.id	= _id,\
	.name		= _name,\
	.parent_names	= _parents,\
	.num_parents	= ARRAY_SIZE(_parents),\
	.flags		= _flags,\
	.cfg =  &(struct mux_cfg) {\
		.reg_off	= _offset,\
		.shift		= _shift,\
		.width		= _width,\
		.mux_flags	= _mux_flags,\
	},\
	.func = _clk_hw_register_mux,\
}

#define STM32_GATE(_id, _name, _parent, _flags, _offset_set, _offset_clr,\
		   _bit_idx)\
{\
	.id	= _id,\
	.name		= _name,\
	.parent_name	= _parent,\
	.flags		= _flags,\
	.cfg =  &(struct stm32_gate_cfg) {\
		.offset_set	= _offset_set,\
		.offset_clr	= _offset_clr,\
		.bit_idx	= _bit_idx,\
	},\
	.func = _clk_register_stm32_gate,\
}

#define STM32_COMPOSITE(_id, _name, _parents, _flags, _cfg)\
{\
	.id	= _id,\
	.name		= _name,\
	.parent_names	= _parents,\
	.num_parents	= ARRAY_SIZE(_parents),\
	.flags		= CLK_OPS_PARENT_ENABLE | _flags,\
	.cfg = &(struct stm32_composite_cfg)_cfg,\
	.func = _clk_stm32_register_composite,\
}

#define STM32_GATEDIV(_gate_offset,\
		      _bit_idx,\
		      _div_offset,\
		      _div_shift,\
		      _div_width,\
		      _div_table)\
{\
	_DIV_TABLE(_div_offset, _div_shift, _div_width, _div_table, 0),\
	_GATE(_gate_offset, _gate_offset + RCC_CLR, _bit_idx, 0),\
	_NO_MUX,\
}

#define GATEDIV(_gate_offset,\
		_bit_idx,\
		_div_offset,\
		_div_shift,\
		_div_width,\
		_div_table)\
{\
	_DIV_TABLE(_div_offset, _div_shift, _div_width, _div_table, 0),\
	_GATE(_gate_offset, _gate_offset, _bit_idx, 0),\
	_NO_MUX,\
}

#define STM32_GATEMUX(_gate_offset, _bit_idx,\
		      _mux_offset, _mux_bit, _mux_width)\
{\
	_NO_DIV,\
	_GATE(_gate_offset, _gate_offset + RCC_CLR, _bit_idx, 0),\
	_MUX(_mux_offset, _mux_bit, _mux_width, 0),\
}

#define GATEMUX(_gate_offset, _bit_idx, _mux_offset, _mux_bit, _mux_width)\
{\
	_NO_DIV,\
	_GATE(_gate_offset, _gate_offset, _bit_idx, 0),\
	_MUX(_mux_offset, _mux_bit, _mux_width, 0),\
}

#define GATEMUXDIV(_gate_offset, _bit_idx,\
		   _mux_offset, _mux_bit, _mux_width,\
		   _div_offset, _div_bit, _div_width\
		)\
{\
	_DIV(_div_offset, _div_bit, _div_width, 0),\
	_GATE(_gate_offset, _gate_offset, _bit_idx, 0),\
	_MUX(_mux_offset, _mux_bit, _mux_width, 0),\
}

struct clk_hw *
__clk_hw_register_divider_table(struct device *dev,
				struct clk_hw_onecell_data *clk_data,
				void __iomem *base, spinlock_t *lock,
				const struct clock_config *cfg);

struct clk_hw *
__clk_register_stm32_gate(struct device *dev,
			  struct clk_hw_onecell_data *clk_data,
			  void __iomem *base,
			  spinlock_t *lock,
			  const struct clock_config *cfg);

struct clk_hw *
__clk_stm32_register_composite(struct device *dev,
			       struct clk_hw_onecell_data *clk_data,
			       void __iomem *base, spinlock_t *lock,
			       const struct clock_config *cfg);

#define DIV_TABLE_S(_id, _name, _parent, _flags, _offset, _shift, _width,\
		    _div_flags, _div_table)\
{\
	.id	= _id,\
	.name		= _name,\
	.parent_name	= _parent,\
	.flags		= _flags,\
	.cfg =  &(struct divider_cfg) {\
		.reg_off	= _offset,\
		.shift		= _shift,\
		.width		= _width,\
		.div_flags	= _div_flags,\
		.table		= _div_table,\
	},\
	.func = __clk_hw_register_divider_table,\
}

#define DIV_S(_id, _name, _parent, _flags, _offset, _shift, _width, _div_flags)\
	DIV_TABLE_S(_id, _name, _parent, _flags, _offset, _shift, _width,\
		_div_flags, NULL)

#define STM32_COMPOSITE_S(_id, _name, _parents, _flags, _cfg)\
{\
	.id	= _id,\
	.name		= _name,\
	.parent_names	= _parents,\
	.num_parents	= ARRAY_SIZE(_parents),\
	.flags		= CLK_OPS_PARENT_ENABLE | _flags,\
	.cfg = &(struct stm32_composite_cfg)_cfg,\
	.func = __clk_stm32_register_composite,\
}

#define STM32_GATE_S(_id, _name, _parent, _flags, _offset_set, _offset_clr,\
		     _bit_idx)\
{\
	.id	= _id,\
	.name		= _name,\
	.parent_name	= _parent,\
	.flags		= _flags,\
	.cfg =  &(struct stm32_gate_cfg) {\
		.offset_set	= _offset_set,\
		.offset_clr	= _offset_clr,\
		.bit_idx	= _bit_idx,\
	},\
	.func = __clk_register_stm32_gate,\
}

#define OSC_S(_id, _name, _parent, _offset_set, _offset_clr, _bit_idx)\
	      STM32_GATE_S(_id, _name, _parent, CLK_IGNORE_UNUSED, _offset_set,\
			   _offset_clr, _bit_idx)
