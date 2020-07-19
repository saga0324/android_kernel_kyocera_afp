/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2016 KYOCERA Corporation
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

#include "mdss_fb.h"
#include "disp_ext_sub_ctrl.h"

static int disp_ext_sub_probe(struct platform_device *pdev);
static int disp_ext_sub_remove(struct platform_device *pdev);
static int disp_ext_sub_open(struct fb_info *info, int user);
static int disp_ext_sub_release(struct fb_info *info, int user);
static int disp_ext_sub_mmap(struct fb_info *info, struct vm_area_struct *vma);
static void disp_ext_sub_shutdown(struct platform_device *pdev);
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

static int disp_ext_sub_open(struct fb_info *info, int user)
{
	int ret = 0;
	struct disp_ext_sub_info *sub_info_p;
	struct disp_ext_sub_pdata *pdata;

	pr_debug("%s start\n",__func__);

	if(!info){
		pr_err("%s end - null info\n",__func__);
		return -ENODEV;
	}

	sub_info_p = (struct disp_ext_sub_info*)(info->par);
	pdata = &(sub_info_p->pdata);

	if (DISP_SUB_STATE_PWR_OFF == pdata->state) {
		disp_ext_sub_panel_set_status(DISP_SUB_STATE_PWR_ON, pdata);
	}

	pr_debug("%s end\n",__func__);
	return ret;
}

static int disp_ext_sub_release(struct fb_info *info, int user)
{
	int ret = 0;
	pr_debug("%s start\n",__func__);
	pr_debug("%s end\n",__func__);
	return ret;
}

static int disp_ext_sub_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	unsigned long start = info->fix.smem_start;
	u32 len = PAGE_ALIGN((start & ~PAGE_MASK) + info->fix.smem_len);
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;

	pr_debug("%s start\n",__func__);

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

	pr_debug("%s end\n",__func__);
	return 0;
}

static int disp_ext_sub_write_cmd(struct fb_info *info, void __user *p)
{
	struct disp_ext_sub_info *sub_info_p = (struct disp_ext_sub_info*)(info->par);
	struct disp_ext_sub_pdata *pdata = &(sub_info_p->pdata);
	struct disp_ext_sub_write_cmd write_cmd;
	int ret;

	ret = copy_from_user(&write_cmd, p, sizeof(write_cmd));
	if (ret)
		return ret;

	return disp_ext_sub_set_cmd(write_cmd.data, write_cmd.len, pdata);
}

static int disp_ext_sub_status_cmd(struct fb_info *info, void __user *p)
{
	struct disp_ext_sub_info *sub_info_p = (struct disp_ext_sub_info*)(info->par);
	struct disp_ext_sub_pdata *pdata = &(sub_info_p->pdata);
	int ret;
	int state;

	switch(pdata->state) {
	case DISP_SUB_STATE_ON:
		state = 1;
		break;
	case DISP_SUB_STATE_OFF:
	case DISP_SUB_STATE_PWR_ON:
		state = 0;
		break;
	case DISP_SUB_STATE_PWR_OFF:
	default:
		state = -1;
		break;
	}

	ret = copy_to_user(p, (void*)&state, sizeof(state));

	return ret;
}

static int disp_ext_sub_ioctl(
	struct fb_info *info,
	unsigned int cmd,
	unsigned long arg )
{
	int ret = 0;
	void __user *argp = (void __user *)arg;

	if (!info || !info->par)
		return -EINVAL;

	pr_debug("%s start\n",__func__);
	switch (cmd) {
	case KC_DISP_EXT_SUB_WRITE_CMD:
		ret = disp_ext_sub_write_cmd(info, argp);
		break;
	case KC_DISP_EXT_SUB_STATUS_CMD:
		ret = disp_ext_sub_status_cmd(info, argp);
		break;
	default:
		break;
	};
	pr_debug("%s end\n",__func__);
	return ret;
}

static int disp_ext_sub_blank(int blank_mode, struct fb_info *info)
{
	int ret = 0;
	pr_debug("%s start\n",__func__);

	ret = disp_ext_sub_blank_ctrl( blank_mode, info );

	pr_debug("%s end\n",__func__);
	return ret;
}

static int disp_ext_sub_pan_display(
	struct fb_var_screeninfo *var,
	struct fb_info *info )
{
	int ret = 0;
	pr_debug("%s start\n",__func__);

	ret = disp_ext_sub_pan_display_ctrl(var,info);

	pr_debug("%s end\n",__func__);
	return ret;
}

static int disp_ext_sub_probe(struct platform_device *pdev)
{
	int ret = 0;
	pr_debug("%s start\n",__func__);

	ret = disp_ext_sub_drvdata_init(pdev);
	ret = disp_ext_sub_get_dt(pdev);

	ret = disp_ext_sub_device_init(pdev);

	ret = disp_ext_sub_register_framebuffer(pdev, &disp_ext_sub_ops);

	pr_debug("%s end:ret[%d]\n",__func__,ret);
	return ret;
}


static int disp_ext_sub_remove(struct platform_device *pdev)
{
	pr_debug("%s start\n",__func__);
	pr_debug("%s end\n",__func__);
	return 0;
}

static void disp_ext_sub_shutdown(struct platform_device *pdev)
{
	pr_debug("%s start\n",__func__);
	disp_ext_sub_device_shutdown(pdev);
	pr_debug("%s end\n",__func__);
}

static int __init disp_ext_sub_driver_init(void)
{
	int ret;

	pr_debug("%s start\n",__func__);

	ret = platform_driver_register(&disp_ext_sub_driver);
	if (ret)
		pr_err("%s failed!\n",__func__);

	pr_debug("%s end\n",__func__);

	return ret;
}
module_init(disp_ext_sub_driver_init);
