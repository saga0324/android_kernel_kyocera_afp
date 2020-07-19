/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <linux/fb.h>

#include "mdss_fb.h"
#include "disp_ext_sub_ctrl.h"

#define DISP_EXT_SUB_FB_NUM    2

static uint8_t* disp_ext_sub_get_img(
	struct fb_var_screeninfo *var,
	struct fb_info *info );

static int fb_sub_dm_flag = 0;

void disp_ext_sub_set_dmflag(int dmflag)
{
	fb_sub_dm_flag = dmflag;
}

static int disp_ext_sub_is_invalid(void)
{
	if (fb_sub_dm_flag != 0 && strcmp(current->comm, "ksubdispdiag") != 0) {
		return 1;
	} else {
		return 0;
	}
}

int disp_ext_sub_blank_ctrl(int blank_mode, struct fb_info *info)
{
	int ret = 0;
	struct disp_ext_sub_info *sub_info_p;
	struct disp_ext_sub_pdata *pdata;
	disp_ext_sub_state_type next_status;

	pr_debug("%s start\n",__func__);

	if(!info){
		pr_err("%s end - null info\n",__func__);
		return -ENODEV;
	}

	if (disp_ext_sub_is_invalid()) {
		pr_debug("%s invalid end\n",__func__);
		return ret;
	}

	sub_info_p = (struct disp_ext_sub_info*)(info->par);
	pdata = &(sub_info_p->pdata);

	switch (blank_mode) {
	case FB_BLANK_UNBLANK:
		next_status = DISP_SUB_STATE_ON;
		break;
	case FB_BLANK_NORMAL:
		next_status = DISP_SUB_STATE_OFF;
		break;
	default:
		pr_err("%s : invalid request - blank_mode[%d]\n",
					__func__, blank_mode );
		ret = -1;
		break;
	}

	if(!ret) {
		ret = disp_ext_sub_panel_set_status( next_status, pdata );
	}

	pr_debug("%s end\n",__func__);
	return ret;
}

int disp_ext_sub_pan_display_ctrl (
	struct fb_var_screeninfo *var,
	struct fb_info *info )
{
	int ret = 0;
	uint8_t* apps_img_p = NULL;

	pr_debug("%s start\n",__func__);

	if( (!var) || (!info) ){
		pr_err("%s end - bad parm var[%x] info[%x]\n",
				__func__,(unsigned int)var, (unsigned int)info);
		return -ENODEV;
	}

	if (disp_ext_sub_is_invalid()) {
		pr_debug("%s invalid end\n",__func__);
		return ret;
	}

	apps_img_p = disp_ext_sub_get_img( var, info );
	if( !apps_img_p ){
		pr_err("%s: miss get img\n",__func__);
		ret = ENOMEM;
		goto exit;
	}

	ret  = disp_ext_sub_panel_update( var, info, apps_img_p);

exit:
	pr_debug("%s end\n",__func__);
	return ret;
}

static uint8_t* disp_ext_sub_get_img(
	struct fb_var_screeninfo *var,
	struct fb_info *info )
{
	void* img_start_addr;

	uint8_t* fb_start_addr;
	uint32_t yoffset;
	uint32_t stride;

	pr_debug("%s start\n",__func__);

	if( (!var) || (!info) ){
		pr_err("%s end - bad parm var[%x] info[%x]\n",
				__func__,(unsigned int)var, (unsigned int)info);
		return NULL;
	}

	fb_start_addr = (uint8_t*)info->screen_base;
	yoffset = (uint32_t)var->yoffset;
	stride = info->fix.line_length;

	img_start_addr = (void*)(fb_start_addr + (yoffset * stride));

	pr_debug("%s end\n",__func__);

	return img_start_addr;
}

int disp_ext_sub_drvdata_init( struct platform_device *pdev )
{
	int ret = 0;
	struct fb_info *fbi;
	struct disp_ext_sub_info *drvdata;

	pr_debug("%s start\n",__func__);

	fbi = framebuffer_alloc(sizeof(struct disp_ext_sub_info), NULL);
	if (fbi == NULL) {
		pr_err("%s can't allocate framebuffer info data\n",__func__);
		return -ENOMEM;
	}

	drvdata = (struct disp_ext_sub_info *)fbi->par;
	drvdata->fbi = fbi;

	platform_set_drvdata(pdev, drvdata);

	pr_debug("%s end\n",__func__);
	return ret;
}

int disp_ext_sub_device_init( struct platform_device *pdev )
{
	int ret = 0;
	struct disp_ext_sub_info *drvdata;
	struct disp_ext_sub_pdata *pdata;
	pr_debug("%s start\n",__func__);

	drvdata = (struct disp_ext_sub_info *)platform_get_drvdata(pdev);
	if(!drvdata) {
		pr_err("%s end - null drvdata\n",__func__);
		return -ENODEV;
	}
	pdata = &(drvdata->pdata);

	ret = disp_ext_sub_panel_bus_init(pdata);
	if(ret){
		pr_err("%s end - fail bus_init\n",__func__);
		return ret;
	}

	ret = disp_ext_sub_panel_signal_init(pdata);
	if(ret){
		pr_err("%s end - fail signal_init\n",__func__);
		return ret;
	}

	pr_debug("%s end\n",__func__);
	return ret;
}

int disp_ext_sub_device_shutdown( struct platform_device *pdev )
{
	int ret = 0;
	struct disp_ext_sub_info *drvdata;
	struct disp_ext_sub_pdata *pdata;
	pr_debug("%s start\n",__func__);

	drvdata = (struct disp_ext_sub_info *)platform_get_drvdata(pdev);
	if(!drvdata) {
		pr_err("%s end - null drvdata\n",__func__);
		return -ENODEV;
	}
	pdata = &(drvdata->pdata);

	ret = disp_ext_sub_panel_set_status( DISP_SUB_STATE_PWR_OFF, pdata );

	pr_debug("%s end\n",__func__);
	return ret;
}

int disp_ext_sub_get_dt( struct platform_device *pdev )
{
	int ret = 0;
	struct disp_ext_sub_info *drvdata;
	struct disp_ext_sub_pdata *pdata;
	struct device_node *np;

	pr_debug("%s start\n",__func__);

	if( !pdev ) {
		pr_err("%s end - null pdev\n",__func__);
		return -ENODEV;
	}

	drvdata = (struct disp_ext_sub_info *)platform_get_drvdata(pdev);
	if(!drvdata) {
		pr_err("%s end - null drvdata\n",__func__);
		return -ENODEV;
	}
	pdata = &(drvdata->pdata);

	np = pdev->dev.of_node;

	ret = disp_ext_sub_get_panel_dt( pdev->dev.of_node, pdata );
	if(ret){
		pr_err("%s end - fail get panel dt\n",__func__);
		disp_ext_sub_pdata_dump( pdata );
		return ret;
	}
	ret = disp_ext_sub_get_signal_dt( pdev->dev.of_node, pdata );
	if(ret){
		pr_err("%s end - fail signal panel dt\n",__func__);
		disp_ext_sub_pdata_dump( pdata );
		return ret;
	}
	ret = disp_ext_sub_get_seq_dt( pdev->dev.of_node, pdata );
	if(ret){
		pr_err("%s end - fail get seq dt\n",__func__);
		disp_ext_sub_pdata_dump( pdata );
		return ret;
	}
	ret = disp_ext_sub_get_vreg_dt(&pdev->dev, pdata);
	if(ret){
		pr_err("%s end - fail get vreg dt\n",__func__);
		disp_ext_sub_pdata_dump( pdata );
		return ret;
	}

	disp_ext_sub_pdata_dump( pdata );

	pr_debug("%s end\n",__func__);
	return ret;
}

int disp_ext_sub_register_framebuffer(
	struct platform_device *pdev,
	struct fb_ops *fbops )
{
	int ret = 0;
	struct disp_ext_sub_info *drvdata;
	struct disp_ext_sub_pdata *pdata;
	struct fb_info *fbi;
	void *virt = NULL;
	dma_addr_t phys = 0;
	size_t size = 0;

	pr_debug("%s start\n",__func__);

	if( !pdev ) {
		pr_err("%s end - null pdev\n",__func__);
		return -ENODEV;
	}

	drvdata = (struct disp_ext_sub_info *)platform_get_drvdata(pdev);
	if(!drvdata) {
		pr_err("%s end - null drvdata\n",__func__);
		return -ENODEV;
	}
	pdata = &(drvdata->pdata);

	fbi = drvdata->fbi;

	/* allocate buffer */
	size = (pdata->xres) * (pdata->yres) * (pdata->bpp / 8 ) * DISP_EXT_SUB_FB_NUM;
	virt = dmam_alloc_coherent(&pdev->dev, size, &phys, GFP_KERNEL);
	pr_debug("%s size=%d vir_addr=%p phys_addr=%08x\n",__func__, size, virt, phys);

	/* Set fix info */
	fbi->screen_base = virt;
	fbi->fix.smem_start = phys;
	fbi->fix.smem_len = size;
	fbi->fix.line_length = (pdata->xres) * (pdata->bpp / 8);
	fbi->fix.xpanstep = 1;
	fbi->fix.ypanstep = 1;

	/* Set var info */
	fbi->var.xres = pdata->xres;
	fbi->var.yres = pdata->yres;
	fbi->var.bits_per_pixel = pdata->bpp;
	fbi->var.xres_virtual = pdata->xres;
	fbi->var.yres_virtual = pdata->yres * DISP_EXT_SUB_FB_NUM;

	/* Set fop */
	fbi->fbops = fbops;

	/* register */
	ret = register_framebuffer(fbi);

	pr_debug("%s end\n",__func__);
	return ret;
}
