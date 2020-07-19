/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 *
 * drivers/video/msm/mdss/disp_ext_spi.h
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
*/
#ifndef DISP_EXT_SPI_H
#define DISP_EXT_SPI_H

struct disp_ext_spi_pinctrl_res {
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
};

struct disp_ext_spi_data {
	int  cs_gpio;
	int  clk_gpio;
	int  data_gpio;
	int  rs_gpio;
	struct disp_ext_spi_pinctrl_res pin_res;
};

int disp_ext_sub_get_spi_dt(struct device_node * np, struct device *dev, struct disp_ext_spi_data *sdata);
void disp_ext_spi_ctrl_cs(struct disp_ext_spi_data *sdata, uint8_t level);
void disp_ext_spi_ctrl_rs(struct disp_ext_spi_data *sdata, uint8_t level);
int32_t disp_ext_spi_write(struct disp_ext_spi_data *sdata, uint8_t *data_p, uint32_t size);
int disp_ext_spi_init(struct disp_ext_spi_data *sdata);
int disp_ext_spi_pinctrl_init(struct device *dev, struct disp_ext_spi_data *sdata);
int disp_ext_spi_pinctrl_set_state(struct disp_ext_spi_data *sdata, bool active);

#endif /* DISP_EXT_SPI_H */
