/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2019 KYOCERA Corporation
 */

/* QLSPI APIs definitions */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>

#include <linux/ql/qlspi.h>
#include <linux/ql/qlspi_linux_plat.h>
#include <linux/ql/qlspi_linux.h>


QL_Status ql_spi_init (struct QLSPI_Platform *spi_plat)
{

	int error=0;

	struct SPIDEV_DATA *spi_priv;

	QLSPI_DBG(" Start \n");

	spi_priv = spi_get_drvdata(spi_slave_dev);

	spi_slave_dev->bits_per_word = 8;

	QLSPI_DBG(" SPI speed %d \n",spi_priv->speed_hz);

	spi_slave_dev->max_speed_hz = spi_priv->speed_hz;

	spi_slave_dev->mode =SPI_MODE_0;

	QLSPI_LOG(" Configuring SPI controller \n");

	error=spi_setup(spi_slave_dev);
	if(error<0)
	{
		QLSPI_ERR(" Error setting up SPI controller QLSPI , ret %d \n",error);
		goto err;
	}

	QLSPI_DBG(" Configuring SPI controller DONE \n");


#ifdef EN_CS_CONTROL

	error = gpio_request(spi_plat->cs_gpio , "SPI_CS");
	if(error < 0) {
		QLSPI_ERR("gpio request failed for SPI CS,  ret %d \n",error );
		goto err;
	}

	QLSPI_DBG("  CS GPIO request DONE \n");

	error = gpio_direction_output(spi_plat->cs_gpio , 1);
	if(error){
		QLSPI_ERR("gpio request output failed for SPI CS, ret %d \n",error);
		goto err;
	}

	QLSPI_LOG("  CS GPIO cfg as OUT DONE \n");

#endif

#ifdef SW_INTR1_GPIO


QLSPI_DBG(" Configuring irq_out_gpio  \n");

error = gpio_request(spi_plat->irq_out_gpio, "SPI_MICON_INT_OUT_GPIO");
if(error < 0) {
	QLSPI_ERR("gpio request failed for irq_out_gpio,	ret %d \n",error );
	goto err;
}

QLSPI_DBG("  irq_out_gpio request DONE \n");

error = gpio_direction_output(spi_plat->irq_out_gpio , 0);
if(error){
	QLSPI_ERR("gpio request output failed for irq_out_gpio, ret %d \n",error);
	goto err;
}

QLSPI_LOG("  irq_out_gpio cfg as OUT DONE \n");


#endif


	QLSPI_LOG("  Configuring  irq gpio \n");

	//irq gpio
	error = gpio_request(spi_plat->irq_gpio, "qlspi,irq-gpio");
	if(error) {
		QLSPI_ERR("GPIO[%d] Request Fail[%d]",spi_plat->irq_gpio,error);
		goto irq_gpio_fail;
	}

		QLSPI_DBG("  IRQ GPIO req done \n");

	error = gpio_direction_input(spi_plat->irq_gpio);
	if(error){
		QLSPI_ERR("GPIO[%d] Set Direction Fail[%d]",spi_plat->irq_gpio,error);
		goto irq_gpio_dir_fail;

	}
	QLSPI_LOG("  IRQ GPIO req INPUT done \n");


	spi_priv->spi->irq	 = gpio_to_irq(spi_plat->irq_gpio);
	if(spi_priv->spi->irq <= 0) {
		QLSPI_ERR("gpio_to_irq returns invalid value %d\n", spi_priv->spi->irq);
		goto irq_gpio_dir_fail;
	}

	QLSPI_DBG("  IRQ_GPIO,  irq no %d \n",spi_priv->spi->irq);

	spi_priv->irq_no=spi_priv->spi->irq;

	QLSPI_DBG(" QLSPI platform init DONE \n");


	return QL_STATUS_OK;



irq_gpio_dir_fail:

	gpio_free(spi_plat->irq_gpio) ;

irq_gpio_fail:

	gpio_free(spi_plat->cs_gpio) ;

err:


	return QL_STATUS_ERROR;
}



