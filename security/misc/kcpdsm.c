/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 */
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>

#define KCPDSM_SYSFS_CREATE(ifname, name) \
	static int name; \
	static ssize_t name##_store(struct kobject *kobj, \
				       struct kobj_attribute *attr, \
				       const char *buf, size_t count ) \
	{ \
		if (current->pid != 1) \
			return count; \
		sscanf(buf, "%du", &name); \
		return count; \
	} \
	static ssize_t name##_show(struct kobject *kobj, \
				    struct kobj_attribute *attr, char *buf) \
	{ \
		return sprintf(buf, "%d\n", name); \
	} \
	static struct kobj_attribute kcpdsm_##name##_attr = \
		__ATTR(ifname, S_IWUSR | S_IRUSR, name##_show, name##_store); \
	int name##_pid(void) \
	{ \
		return name; \
	} \
	static int dump_##name##_info(char *buf, int need_pid) \
	{ \
		if (need_pid) \
			return sprintf(buf, "%s:%d\n", #ifname, name); \
		else \
			return sprintf(buf, "%s\n", #ifname); \
	}

KCPDSM_SYSFS_CREATE(lkspad, lkspad);
KCPDSM_SYSFS_CREATE(debuggerd, debuggerd);
KCPDSM_SYSFS_CREATE(debuggerd64, debuggerd64);
KCPDSM_SYSFS_CREATE(fs_mgr, fs_mgr);

KCPDSM_SYSFS_CREATE(kdiag_common, kdiag_common);
KCPDSM_SYSFS_CREATE(bfss_diag, bfss_diag);
KCPDSM_SYSFS_CREATE(disp_ctrl, disp_ctrl);
KCPDSM_SYSFS_CREATE(usb_init, usb_init);
KCPDSM_SYSFS_CREATE(kdmd, kdmd);
KCPDSM_SYSFS_CREATE(kflcd, kflcd);
KCPDSM_SYSFS_CREATE(kflcdiag, kflcdiag);

static ssize_t dump_list_info(char *buf, int need_pid)
{
	char *needle = buf;

	if (!need_pid) {
		needle += dump_lkspad_info(needle, need_pid);
		needle += dump_debuggerd_info(needle, need_pid);
		needle += dump_debuggerd64_info(needle, need_pid);
		needle += dump_fs_mgr_info(needle, need_pid);
	}
	needle += dump_kdiag_common_info(needle, need_pid);
	needle += dump_bfss_diag_info(needle, need_pid);
	needle += dump_disp_ctrl_info(needle, need_pid);
	needle += dump_usb_init_info(needle, need_pid);
	needle += dump_kdmd_info(needle, need_pid);
	needle += dump_kflcd_info(needle, need_pid);
	needle += dump_kflcdiag_info(needle, need_pid);

	return (needle - buf);
}
static ssize_t process_list_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	if (current->pid != 1)
		return 0;

	return dump_list_info(buf, 0);
}
static ssize_t lkspad_client_show(struct kobject *kobj,
				  struct kobj_attribute *attr, char *buf)
{
	if (current->pid != lkspad
	    && current->group_leader->pid != lkspad)
		return 0;

	return dump_list_info(buf, 1);
}

static struct kobj_attribute kcpdsm_process_list_attr =
	__ATTR(process_list, S_IRUSR, process_list_show, NULL);
static struct kobj_attribute kcpdsm_lkspad_client_attr =
	__ATTR(lkspad_client, S_IRUSR, lkspad_client_show, NULL);

static struct attribute *kcpdsm_attrs[] = {
	/* attributes for processes start */
	&kcpdsm_lkspad_attr.attr,
	&kcpdsm_debuggerd_attr.attr,
	&kcpdsm_debuggerd64_attr.attr,
	&kcpdsm_fs_mgr_attr.attr,
	&kcpdsm_kdiag_common_attr.attr,
	&kcpdsm_bfss_diag_attr.attr,
	&kcpdsm_disp_ctrl_attr.attr,
	&kcpdsm_usb_init_attr.attr,
	&kcpdsm_kdmd_attr.attr,
	&kcpdsm_kflcd_attr.attr,
	&kcpdsm_kflcdiag_attr.attr,
	/* attributes for special I/F start */
	&kcpdsm_process_list_attr.attr,
	&kcpdsm_lkspad_client_attr.attr,
	NULL,
};

static struct attribute_group kcpdsm_attr_group = {
	.attrs = kcpdsm_attrs,
};

static struct kobject *kcpdsm_kobj;
static int __init kcpdsm_sysfs_init(void)
{
	int retval;

	kcpdsm_kobj = kobject_create_and_add("kcpdsm", kernel_kobj);
	if (!kcpdsm_kobj)
		goto out_failed;

	retval = sysfs_create_group(kcpdsm_kobj, &kcpdsm_attr_group);
	if (retval != 0)
		goto out_kcpdsm_kobj;

	pr_info("kcpdsm_sysfs initialized\n");
	return retval;

out_kcpdsm_kobj:
	kobject_put(kcpdsm_kobj);
out_failed:
	pr_info("kcpdsm_sysfs init failed\n");
	return -ENOMEM;
}

static void __exit kcpdsm_sysfs_exit(void)
{
	kobject_put(kcpdsm_kobj);
}
module_init(kcpdsm_sysfs_init);
module_exit(kcpdsm_sysfs_exit);
MODULE_LICENSE("GPL");
