/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2015 KYOCERA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/msm-bus.h>
#include "kaudio_bus_scale.h"

static uint32_t 	g_bus_client;
static struct mutex	g_lock;

/*===========================================================================

FUNCTION : kaudio_set_bus_scale

===========================================================================*/
int kaudio_set_bus_scale( int in_prm )
{
	int							w_rc;
/**/
	mutex_lock(&g_lock);
	if ( g_bus_client ){
		w_rc = msm_bus_scale_client_update_request( g_bus_client, in_prm );
	} else {
		w_rc = (-EFAULT);
	}
	mutex_unlock(&g_lock);

	pr_info("%s: g_bus_client=%x in_prm=%x w_rc=%x\n", __func__, g_bus_client, in_prm, w_rc);
	return w_rc;
}

/*===========================================================================

FUNCTION : kaudio_bus_scale_probe

===========================================================================*/
static int kaudio_bus_scale_probe( struct platform_device* pdev )
{
	int							w_rc;
	struct msm_bus_scale_pdata*	w_pdata = NULL;
/**/
	pr_debug("%s\n", __func__);

	w_rc = 0 ;

	mutex_init(&g_lock);

	if (!pdev->dev.of_node) {
		pr_err("%s: device tree information missing\n", __func__);
		w_rc = (-ENODEV);
		goto end;
	}

	w_pdata = msm_bus_cl_get_pdata(pdev);
	if (!w_pdata) {
		pr_err("%s: bus device tree allocation failed\n", __func__);
		w_rc = (-EINVAL);
		goto end;
	}

	dev_set_drvdata(&pdev->dev, w_pdata);
	g_bus_client = msm_bus_scale_register_client(w_pdata);
	if (!g_bus_client) {
		pr_err("%s: msm_bus_scale_register_client() failed\n", __func__);
		w_rc = (-EFAULT);
		goto end;
	}

end:
	pr_info("%s g_bus_client=%x w_rc=%x\n", __func__, (int)g_bus_client, w_rc);
	return w_rc;
}

/*===========================================================================

FUNCTION : kaudio_bus_scale_remove

===========================================================================*/
static int kaudio_bus_scale_remove( struct platform_device* pdev )
{
	struct msm_bus_scale_pdata*	w_pdata = NULL;
/**/
	pr_debug("%s\n", __func__);

	w_pdata = (struct msm_bus_scale_pdata *)dev_get_drvdata( &pdev->dev );
	msm_bus_cl_clear_pdata( w_pdata );

	return 0;
}

/*===========================================================================

FUNCTION : kaudio_bus_scale_init

===========================================================================*/
static const struct of_device_id msm_kaudio_bus_scale_dt_match[] = 
{
	{.compatible = "kc,kaudio-bus-scale",},
	{}
};
MODULE_DEVICE_TABLE(of, msm_kaudio_bus_scale_dt_match);

static struct platform_driver kaudio_bus_scale_driver = {
	.driver = {
		.name = "kaudio-bus-scale",
		.owner = THIS_MODULE,
		.of_match_table = msm_kaudio_bus_scale_dt_match,
	},
	.probe = kaudio_bus_scale_probe,
	.remove = kaudio_bus_scale_remove,
};

static int __init kaudio_bus_scale_init( void )
{
	int							w_rc;
/**/
	w_rc = platform_driver_register( &kaudio_bus_scale_driver );
	if (w_rc){
		pr_err("%s: Failed to register kaudio bus scale driver\n", __func__);
	}
	return w_rc;
}
module_init( kaudio_bus_scale_init );

/*===========================================================================

FUNCTION : kaudio_bus_scale_exit

===========================================================================*/
static void __exit kaudio_bus_scale_exit( void )
{
	platform_driver_unregister( &kaudio_bus_scale_driver );
}

module_exit( kaudio_bus_scale_exit );
