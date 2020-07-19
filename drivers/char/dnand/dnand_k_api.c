/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 and
 *  only version 2 as published by the Free Software Foundation.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#define pr_fmt(fmt) "DNANDAPI: " fmt

#include <linux/module.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/dnand_status.h>
#include <linux/dnand_clid.h>
#include "dnand_fs.h"
#include "dnand_k_api_internal.h"

static DEFINE_MUTEX(dnand_api_lock);

static int32_t kdnand_id_param_check(uint32_t cid, uint32_t offset,
				     uint8_t *pbuf, uint32_t size)
{
	if (cid >= DNAND_ID_ENUM_MAX) {
		pr_warn("DNAND_PARAM_ERROR cid:%u", cid);
		return DNAND_PARAM_ERROR;
	}
	if (!pbuf) {
		pr_warn("DNAND_PARAM_ERROR cid:%u pbuf is NULL", cid);
		return DNAND_PARAM_ERROR;
	}
	if (size == 0) {
		pr_warn("DNAND_PARAM_ERROR cid:%u size is 0", cid);
		return DNAND_PARAM_ERROR;
	}
	return DNAND_NO_ERROR;
}

int32_t kdnand_id_read(uint32_t cid, uint32_t offset,
		       uint8_t *pbuf, uint32_t size)
{
	int32_t ret;

	ret = kdnand_id_param_check(cid, offset, pbuf, size);

	if (ret != DNAND_NO_ERROR) {
		pr_warn("param error\n");
		return ret;
	}
	mutex_lock(&dnand_api_lock);
	ret = dnand_fs_read(cid, offset, pbuf, size);
	mutex_unlock(&dnand_api_lock);

	return ret;
}

int32_t kdnand_id_write(uint32_t cid, uint32_t offset,
			uint8_t *pbuf, uint32_t size)
{
	int32_t ret;

	ret = kdnand_id_param_check(cid, offset, pbuf, size);
	if (ret != DNAND_NO_ERROR)
		return ret;

	mutex_lock(&dnand_api_lock);
	ret = dnand_fs_write(cid, offset, pbuf, size);
	mutex_unlock(&dnand_api_lock);

	return ret;
}

int __init kdnand_k_api_init(void)
{
	return dnand_middle_init();
}
