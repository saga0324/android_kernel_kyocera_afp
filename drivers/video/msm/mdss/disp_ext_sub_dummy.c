/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
 * (C) 2017 KYOCERA Corporation
 * (C) 2018 KYOCERA Corporation
 */
/*
 * Copyright (C) 2007 Google Incorporated
 * Copyright (c) 2008-2013, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <linux/fb.h>
#include <linux/msm_mdp.h>
#include <linux/dma-mapping.h>

#include "disp_ext_sub_dummy.h"

#define SUB_PANEL_WIDTH			128
#define SUB_PANEL_HEIGHT		36
#define SUB_PANEL_BPP			16
#define DISP_EXT_SUB_FB_NUM		2
#define SUBDISP_DEVICE_ID "subdisplay"

static int disp_ext_sub_ioctl(
	struct fb_info *info,
	unsigned int cmd,
	unsigned long arg );
static int disp_ext_sub_blank(int blank_mode, struct fb_info *info);
static int disp_ext_sub_pan_display(
	struct fb_var_screeninfo *var,
	struct fb_info *info );

static struct fb_ops disp_ext_sub_ops = {
	.owner = THIS_MODULE,
	.fb_open = disp_ext_sub_open,
	.fb_release = disp_ext_sub_release,
	.fb_mmap = disp_ext_sub_mmap,
	.fb_ioctl = disp_ext_sub_ioctl,
	.fb_blank = disp_ext_sub_blank,
	.fb_pan_display = disp_ext_sub_pan_display,
};

static const struct of_device_id disp_ext_sub_dt_match[] = {
	{.compatible = "kc,disp_ext_sub"},
	{}
};

static struct platform_driver disp_ext_sub_driver = {
	.probe = disp_ext_sub_probe,
	.remove = disp_ext_sub_remove,
	.shutdown = disp_ext_sub_shutdown,
	.driver = {
		.name = "disp_ext_sub",
		.of_match_table = disp_ext_sub_dt_match,
	},
};

extern struct fb_info *oem_fb_get_fb_info(unsigned int idx);

static int disp_ext_sub_open(struct fb_info *info, int user)
{
	pr_err("[dummy]%s \n",__func__);
	return 0;
}

static int disp_ext_sub_release(struct fb_info *info, int user)
{
	pr_err("[dummy]%s \n",__func__);
	return 0;
}

static int disp_ext_sub_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	unsigned long start = info->fix.smem_start;
	u32 len = PAGE_ALIGN((start & ~PAGE_MASK) + info->fix.smem_len);
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;

	pr_err("[dummy]%s start\n",__func__);

	if (!start) {
		pr_err("No framebuffer memory is allocated.\n");
		return -ENOMEM;
	}

	start &= PAGE_MASK;
	if ((vma->vm_end <= vma->vm_start) ||
	    (off >= len) ||
	    ((vma->vm_end - vma->vm_start) > (len - off)))
		return -EINVAL;
	off += start;
	if (off < start)
		return -EINVAL;
	vma->vm_pgoff = off >> PAGE_SHIFT;
	vma->vm_flags |= VM_IO | VM_DONTEXPAND | VM_DONTDUMP;

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start,
			       vma->vm_page_prot))
		return -EAGAIN;

	pr_err("[dummy]%s end\n",__func__);
	return 0;
}

static int disp_ext_sub_ioctl(
	struct fb_info *info,
	unsigned int cmd,
	unsigned long arg )
{
	pr_err("[dummy]%s cmd=%d\n",__func__,cmd);

	switch (cmd) {
	case KC_DISP_EXT_SUB_WRITE_CMD:
		break;
	case KC_DISP_EXT_SUB_STATUS_CMD:
		break;
	default:
		break;
	};

	return 0;
}

static int disp_ext_sub_blank(int blank_mode, struct fb_info *info)
{
	pr_err("[dummy]%s mode=%d\n",__func__,blank_mode);
	return 0;
}

static int disp_ext_sub_pan_display(
	struct fb_var_screeninfo *var,
	struct fb_info *info )
{
	pr_err("[dummy]%s \n",__func__);
	return 0;
}

static int disp_ext_sub_probe(struct platform_device *pdev)
{
	int ret = 0;
	pr_debug("[dummy]%s \n",__func__);

	ret = disp_ext_sub_drvdata_init(pdev);

	ret = disp_ext_sub_register_framebuffer(pdev, &disp_ext_sub_ops);

	return ret;
}


static int disp_ext_sub_remove(struct platform_device *pdev)
{
	struct disp_ext_sub_info *drvdata;
	struct fb_info *fbi;

	pr_debug("[dummy]%s \n",__func__);

	drvdata = (struct disp_ext_sub_info *)platform_get_drvdata(pdev);
	if(!drvdata) {
		pr_err("%s end - null drvdata\n",__func__);
		return -ENODEV;
	}
	fbi = drvdata->fbi;

	/* release */
	unregister_framebuffer(fbi);

	return 0;
}

static void disp_ext_sub_shutdown(struct platform_device *pdev)
{
	pr_err("[dummy]%s \n",__func__);
}

static int __init disp_ext_sub_driver_init(void)
{
	int ret;

	pr_debug("[dummy]%s \n",__func__);

	ret = platform_driver_register(&disp_ext_sub_driver);
	if (ret)
		pr_err("%s failed!\n",__func__);

	return ret;
}
module_init(disp_ext_sub_driver_init);

int disp_ext_sub_drvdata_init( struct platform_device *pdev )
{
	int ret = 0;
	struct fb_info *fbi;
	struct disp_ext_sub_info *drvdata;
	struct fb_fix_screeninfo *fix;

	pr_debug("[dummy]%s \n",__func__);

	fbi = framebuffer_alloc(sizeof(struct disp_ext_sub_info), NULL);
	if (fbi == NULL) {
		pr_err("%s can't allocate framebuffer info data\n",__func__);
		return -ENOMEM;
	}

	fix = &fbi->fix;
	snprintf(fix->id, sizeof(fix->id), SUBDISP_DEVICE_ID);

	drvdata = (struct disp_ext_sub_info *)fbi->par;
	drvdata->fbi = fbi;

	/* panel data set */
	drvdata->pdata.xres = SUB_PANEL_WIDTH;
	drvdata->pdata.yres = SUB_PANEL_HEIGHT;
	drvdata->pdata.bpp = SUB_PANEL_BPP;

	platform_set_drvdata(pdev, drvdata);

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

	pr_debug("[dummy]%s \n",__func__);

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
	pr_debug("[dummy]%s x=%d y=%d bpp=%d\n",__func__, pdata->xres, pdata->yres, pdata->bpp);
	virt = dmam_alloc_coherent(&pdev->dev, size, &phys, GFP_KERNEL);
	pr_debug("[dummy]%s size=%d vir_addr=%p phys_addr=%08x\n",__func__, size, virt, phys);

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

	pr_debug("[dummy]%s end\n",__func__);
	return ret;
}

void disp_ext_sub_set_dmflag(int dmflag)
{
	pr_err("[dummy]%s fb_sub_dm_flag no-set.\n",__func__);
}
