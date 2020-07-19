/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
 * (C) 2014 KYOCERA Corporation
 * (C) 2015 KYOCERA Corporation
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
#define pr_fmt(fmt) "DNANDMIDDLE: " fmt

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/dnand_status.h>
#include <linux/dnand_clid.h>
#include "dnand_fs.h"
#include "dnand_drv.h"

#define DNAND_FS_PARTITION_SIZE           (1024*1024*8)
#define DNAND_FS_CLID_NUM                 (100)

#define DNAND_FS_BLK_SECT_NUM             (4)
#define DNAND_FS_BLK_SIZE                 (DNAND_FS_BLK_SECT_NUM*DNAND_DRV_SECTOR_BLK_SIZE)
#define DNAND_FS_BLK_NUM                  (DNAND_FS_PARTITION_SIZE/DNAND_FS_BLK_SIZE)
#define DNAND_FS_BLK_CKCD_NUM             (1)
#define DNAND_FS_BLK_CLID_NUM             ( (DNAND_FS_CLID_NUM*sizeof(uint32_t)+DNAND_FS_BLK_SIZE-1)/DNAND_FS_BLK_SIZE )
#define DNAND_FS_BLK_FAT_NUM              ( (DNAND_FS_BLK_NUM*sizeof(uint16_t)+DNAND_FS_BLK_SIZE-1)/DNAND_FS_BLK_SIZE )
#define DNAND_FS_SYSTEM_NUM               (2)
#define DNAND_FS_BLK_SYS_NUM              (DNAND_FS_BLK_CKCD_NUM+DNAND_FS_BLK_CLID_NUM+DNAND_FS_BLK_FAT_NUM)
#define DNAND_FS_BLK_DATA_ST              (DNAND_FS_BLK_SYS_NUM*DNAND_FS_SYSTEM_NUM)
#define DNAND_FS_CKCD_SIZE                (6)
#define DNAND_FS_CKCD                     ("DNAND")
#define DNAND_FS_BLK_UNUSE                (0xFFFF)
#define DNAND_FS_BLK_EOF                  (0x7FFF)

#define DNAND_FILL_HEX 0xFF
#define DNAND_CLEAR_HEX 0x00

typedef struct dnand_fs_flmng_st
{
	uint8_t  ckcd[DNAND_FS_CKCD_SIZE];
	uint8_t  dmya[DNAND_FS_BLK_SIZE-DNAND_FS_CKCD_SIZE];
	uint32_t clid[(DNAND_FS_BLK_CLID_NUM*DNAND_FS_BLK_SIZE)/sizeof(uint32_t)];
	uint16_t fat[(DNAND_FS_BLK_FAT_NUM*DNAND_FS_BLK_SIZE)/sizeof(uint16_t)];
} dnand_fs_flmng;

typedef struct dnand_fs_memng_st
{
	uint32_t clid[DNAND_FS_CLID_NUM];
	uint16_t fat[DNAND_FS_BLK_NUM];
} dnand_fs_memng;

static dnand_fs_memng dnand_fsinfo[DNAND_FS_SYSTEM_NUM];
static uint32_t dnand_alloc_start = DNAND_FS_BLK_DATA_ST;

static void inter_cpy_info(dnand_fs_flmng *pbuf)
{
	if(!pbuf)
		return;

	memcpy(&dnand_fsinfo[0].clid[0], &pbuf->clid[0],
	       sizeof(dnand_fsinfo[0].clid));
	memcpy(&dnand_fsinfo[0].fat[0], &pbuf->fat[0],
	       sizeof(dnand_fsinfo[0].fat));
	memcpy(&dnand_fsinfo[1], &dnand_fsinfo[0],
	       sizeof(dnand_fsinfo[1]));

	return;
}

static int32_t dnand_blk_read(uint32_t blk_no, uint32_t offset,
			      uint8_t *pbuf, uint32_t size)
{
	int32_t ret = DNAND_NO_ERROR;
	uint32_t s_sect, e_sect, n_sect, offset_sect, rest_sect;
	uint32_t r_sect, c_size;
	uint8_t *p_kbuf;

	s_sect = (blk_no * DNAND_FS_BLK_SECT_NUM) + (offset / DNAND_DRV_SECTOR_BLK_SIZE);
	e_sect = (offset + size + DNAND_DRV_SECTOR_BLK_SIZE - 1) / DNAND_DRV_SECTOR_BLK_SIZE;
	e_sect += (blk_no * DNAND_FS_BLK_SECT_NUM);
	n_sect = e_sect - s_sect;

	offset_sect = offset%DNAND_DRV_SECTOR_BLK_SIZE;
	rest_sect = (offset+size)%DNAND_DRV_SECTOR_BLK_SIZE;

	p_kbuf = (uint8_t*)kmalloc(DNAND_DRV_SECTOR_BLK_SIZE, GFP_KERNEL);
	if (!p_kbuf)
		return DNAND_NOMEM_ERROR;

	if (offset_sect > 0) {
		r_sect = 1;
		ret = dnand_drv_read(s_sect, r_sect, p_kbuf);
		if (ret != DNAND_NO_ERROR) {
			pr_err("Failed read from drv, s_sect=%d, r_sect=%d\n",
			       s_sect, r_sect);
			goto out;
		}
		if ((offset_sect+size) > DNAND_DRV_SECTOR_BLK_SIZE)
			c_size = DNAND_DRV_SECTOR_BLK_SIZE - offset_sect;
		else
			c_size = size;

		memcpy(pbuf, (p_kbuf+offset_sect), c_size);
		pbuf += c_size;
		size -= c_size;
		s_sect += r_sect;
		n_sect -= r_sect;
	}

	if (n_sect == 0)
		goto out;

	if (rest_sect > 0) {
		r_sect = n_sect -1;
	} else {
		r_sect = n_sect;
	}

	if (r_sect > 0) {
		ret = dnand_drv_read(s_sect, r_sect, pbuf);
		if (ret != DNAND_NO_ERROR) {
			pr_err("Failed read from drv, s_sect=%d, r_sect=%d\n",
			       s_sect, r_sect);
			goto out;
		}
		c_size = r_sect * DNAND_DRV_SECTOR_BLK_SIZE;
		pbuf += c_size;
		size -= c_size;
		s_sect += r_sect;
		n_sect -= r_sect;
	}

	if (n_sect > 0) {
		r_sect = 1;
		ret = dnand_drv_read(s_sect, r_sect, p_kbuf);
		if (ret != DNAND_NO_ERROR) {
			pr_err("Failed read from drv, s_sect=%d, r_sect=%d\n",
			       s_sect, r_sect);
			goto out;
		}
		memcpy(pbuf, p_kbuf, size);
	}
out:
	kfree(p_kbuf);
	return ret;
}

static int32_t dnand_blk_write(uint32_t blk_no, uint32_t offset,
			       uint8_t *pbuf, uint32_t size)
{
	int32_t ret = DNAND_NO_ERROR;
	uint32_t s_sect, e_sect, n_sect, offset_sect, rest_sect;
	uint32_t rw_sect, c_size;
	uint8_t *p_kbuf;

	s_sect = (blk_no * DNAND_FS_BLK_SECT_NUM) + (offset / DNAND_DRV_SECTOR_BLK_SIZE);
	e_sect = (offset + size + DNAND_DRV_SECTOR_BLK_SIZE - 1) / DNAND_DRV_SECTOR_BLK_SIZE;
	e_sect += (blk_no * DNAND_FS_BLK_SECT_NUM);
	n_sect = e_sect - s_sect;

	offset_sect = offset % DNAND_DRV_SECTOR_BLK_SIZE;
	rest_sect = (offset+size) % DNAND_DRV_SECTOR_BLK_SIZE;

	p_kbuf = (uint8_t*)kmalloc(DNAND_DRV_SECTOR_BLK_SIZE, GFP_KERNEL);
	if (!p_kbuf)
		return DNAND_NOMEM_ERROR;

	if (offset_sect > 0) {
		rw_sect = 1;
		ret = dnand_drv_read(s_sect, rw_sect, p_kbuf);
		if (ret != DNAND_NO_ERROR) {
			pr_err("Failed read from drv, s_sect=%d, rw_sect=%d\n",
			       s_sect, rw_sect);
			goto out;
		}
		if ((offset_sect + size) > DNAND_DRV_SECTOR_BLK_SIZE)
			c_size = DNAND_DRV_SECTOR_BLK_SIZE - offset_sect;
		else
			c_size = size;

		memcpy((p_kbuf + offset_sect), pbuf, c_size );
		ret = dnand_drv_write(s_sect, rw_sect, p_kbuf);
		if (ret != DNAND_NO_ERROR)
			goto out;

		pbuf += c_size;
		size -= c_size;
		s_sect += rw_sect;
		n_sect -= rw_sect;
	}

	if (n_sect == 0)
		goto out;

	if (rest_sect > 0) {
		rw_sect = n_sect -1;
	} else {
		rw_sect = n_sect;
	}

	if (rw_sect > 0) {
		ret = dnand_drv_write(s_sect, rw_sect, pbuf);
		if (ret != DNAND_NO_ERROR) {
			pr_err("Failed write to drv, s_sect=%d, rw_sect=%d\n",
			       s_sect, rw_sect);
			goto out;
		}
		c_size = rw_sect * DNAND_DRV_SECTOR_BLK_SIZE;
		pbuf += c_size;
		size -= c_size;
		s_sect += rw_sect;
		n_sect -= rw_sect;
	}

	if (n_sect > 0) {
		rw_sect = 1;
		ret = dnand_drv_read(s_sect, rw_sect, p_kbuf);
		if (ret != DNAND_NO_ERROR) {
			pr_err("Failed read from drv, s_sect=%d, rw_sect=%d\n",
			       s_sect, rw_sect);
			goto out;
		}
		c_size = size;
		memcpy(p_kbuf, pbuf, size);
		ret = dnand_drv_write(s_sect, rw_sect, p_kbuf);
		if (ret != DNAND_NO_ERROR) {
			pr_err("Failed write to drv, s_sect=%d, rw_sect=%d\n",
			       s_sect, rw_sect);
			goto out;
		}
	}
out:
	kfree(p_kbuf);
	return ret;
}

static int32_t update_flmng(uint32_t sysno, dnand_fs_flmng *p_flmng )
{
	uint32_t blk_no;
	int32_t ret;

	if (!p_flmng) {
		return DNAND_INTERNAL_ERROR;
	}
	if (sysno == 0)
		blk_no = 0;
	else
		blk_no = DNAND_FS_BLK_SYS_NUM;

	memset(&p_flmng->ckcd[0], DNAND_FILL_HEX, sizeof(p_flmng->ckcd));
	memset(&p_flmng->dmya[0], DNAND_FILL_HEX, sizeof(p_flmng->dmya));

	ret = dnand_blk_write(blk_no, 0, (uint8_t*)p_flmng, DNAND_FS_BLK_SIZE);
	if (ret != DNAND_NO_ERROR) {
		pr_err("Failed write to blk, blkno=%d\n", blk_no);
		return ret;
	}
	memset(&p_flmng->clid[0], DNAND_FILL_HEX, sizeof(p_flmng->clid));
	memcpy(&p_flmng->clid[0], &dnand_fsinfo[1].clid[0], sizeof(dnand_fsinfo[1].clid));
	memset(&p_flmng->fat[0], DNAND_FILL_HEX, sizeof(p_flmng->fat));
	memcpy(&p_flmng->fat[0], &dnand_fsinfo[1].fat[0], sizeof(dnand_fsinfo[1].fat));
	
	ret = dnand_blk_write(blk_no, 0, (uint8_t*)p_flmng, sizeof(dnand_fs_flmng));
	if (ret != DNAND_NO_ERROR) {
		pr_err("Failed write to blk, blkno=%d\n", blk_no);
		return ret;
	}
	strcpy((char*)&p_flmng->ckcd[0], DNAND_FS_CKCD);

	ret = dnand_blk_write(blk_no, 0, (uint8_t*)p_flmng, DNAND_FS_BLK_SIZE);
	return ret;
}

static void inter_memng_clr( uint32_t  cid )
{
	uint32_t i;
	uint16_t *pfat0, *pfat1;
	uint16_t blk0, blk1, nxtblk0, nxtblk1;

	if (cid >= DNAND_FS_CLID_NUM)
		return;

	blk0 = (uint16_t)dnand_fsinfo[0].clid[cid];
	blk1 = (uint16_t)dnand_fsinfo[1].clid[cid];
	pfat0 = &dnand_fsinfo[0].fat[0];
	pfat1 = &dnand_fsinfo[1].fat[0];
	nxtblk0 = DNAND_FS_BLK_UNUSE;
	nxtblk1 = DNAND_FS_BLK_UNUSE;
	
	dnand_fsinfo[1].clid[cid] = dnand_fsinfo[0].clid[cid];
	
	for(i=0; i<DNAND_FS_BLK_NUM; i++) {
		if (blk0 >= DNAND_FS_BLK_NUM && blk1 >= DNAND_FS_BLK_NUM)
			break;

		if (blk0 < DNAND_FS_BLK_NUM)
			nxtblk0 = pfat0[blk0];
		if (blk1 < DNAND_FS_BLK_NUM)
			nxtblk1 = pfat1[blk1];

		if (blk0 != blk1) {
			if (blk1 < DNAND_FS_BLK_NUM) {
				pfat0[blk1] = DNAND_FS_BLK_UNUSE;
				pfat1[blk1] = DNAND_FS_BLK_UNUSE;
			}
		}
		if (blk0 < DNAND_FS_BLK_NUM) {
			pfat1[blk0] = nxtblk0;
		}

		if (nxtblk0 < DNAND_FS_BLK_NUM)
			blk0 = nxtblk0;
		else
			blk0 = DNAND_FS_BLK_UNUSE;

		if (nxtblk1 < DNAND_FS_BLK_NUM)
			blk1 = nxtblk1;
		else
			blk1 = DNAND_FS_BLK_UNUSE;

	}
	return;
}

static int32_t update_memng(uint32_t cid)
{
	int32_t ret;
	uint8_t *p_kbuf;

	if (cid >= DNAND_FS_CLID_NUM)
		return DNAND_INTERNAL_ERROR;

	inter_memng_clr(cid);

	p_kbuf = (uint8_t*)kmalloc(sizeof(dnand_fs_flmng), GFP_KERNEL);
	if (!p_kbuf) {
		pr_err("Failed to kmalloc dnand_fs_flmng\n");
		return DNAND_NOMEM_ERROR;
	}

	ret = update_flmng(1, (dnand_fs_flmng*)p_kbuf);
	if (ret != DNAND_NO_ERROR) {
		pr_err("Failed to update_fs_flmng\n");
		goto out;
	}
	ret = update_flmng(0, (dnand_fs_flmng*)p_kbuf);
out:
	kfree(p_kbuf);
	return ret;
}

static uint16_t get_clid( uint32_t  cid )
{
	uint16_t blk;

	if (cid >= DNAND_FS_CLID_NUM) {
		return DNAND_FS_BLK_UNUSE;
	}
	blk = (uint16_t)dnand_fsinfo[0].clid[cid];
	if (blk < DNAND_FS_BLK_DATA_ST || blk >= DNAND_FS_BLK_NUM) {
		blk = DNAND_FS_BLK_UNUSE;
	}
	return blk;
}

static void set_clid(uint32_t  cid, uint16_t pos)
{
	uint32_t blk;

	blk = pos;
	if (cid >= DNAND_FS_CLID_NUM) {
		return;
	}
	if (blk < DNAND_FS_BLK_DATA_ST || blk >= DNAND_FS_BLK_NUM) {
		blk = DNAND_FS_BLK_UNUSE;
	}
	dnand_fsinfo[0].clid[cid] = blk;
	return;
}

static uint16_t get_fat(uint16_t blk)
{
	uint16_t nxtblk;

	if (blk < DNAND_FS_BLK_DATA_ST || blk >= DNAND_FS_BLK_NUM) {
		nxtblk = DNAND_FS_BLK_UNUSE;
	} else {
		nxtblk = (uint16_t)dnand_fsinfo[0].fat[blk];
	}
	return nxtblk;
}

static void set_fat(uint16_t pos, uint16_t blk)
{
	if (pos < DNAND_FS_BLK_DATA_ST || pos >= DNAND_FS_BLK_NUM) {
		return;
	}
	dnand_fsinfo[0].fat[pos] = blk;
	return;
}

static uint16_t alloc_fat(void)
{
	uint32_t i, blk;
	uint16_t retblk, tmpblk;

	if (dnand_alloc_start < DNAND_FS_BLK_DATA_ST ||
	    dnand_alloc_start >= DNAND_FS_BLK_NUM) {
		dnand_alloc_start = DNAND_FS_BLK_DATA_ST;
	}
	
	retblk = DNAND_FS_BLK_UNUSE;
	for (i = DNAND_FS_BLK_DATA_ST; i < DNAND_FS_BLK_NUM; i++) {
		blk = dnand_fsinfo[0].fat[dnand_alloc_start];
		tmpblk = dnand_alloc_start;

		dnand_alloc_start++;
		if (dnand_alloc_start >= DNAND_FS_BLK_NUM) {
			dnand_alloc_start = DNAND_FS_BLK_DATA_ST;
		}

		if (blk == DNAND_FS_BLK_UNUSE) {
			retblk = tmpblk;
			break;
		}
	}

	if(retblk < DNAND_FS_BLK_DATA_ST || retblk >= DNAND_FS_BLK_NUM) {
		retblk = DNAND_FS_BLK_UNUSE;
	}
	return retblk;
}

static int32_t inter_fs_read(uint16_t blk, uint32_t offset,
			     uint8_t *pbuf, uint32_t size)
{
	int32_t ret;
	uint32_t i, cnt, c_size;

	ret = DNAND_NO_ERROR;
	if (size == 0) {
		pr_warn("size is zero\n");
		return ret;
	}

	cnt = (offset + size + DNAND_FS_BLK_SIZE - 1)/DNAND_FS_BLK_SIZE;
	for (i = 0; i < cnt; i++) {
		if (offset >= DNAND_FS_BLK_SIZE) {
			offset -= DNAND_FS_BLK_SIZE;
			blk = get_fat(blk);
			continue;
		}
		if ((offset + size) >= DNAND_FS_BLK_SIZE)
			c_size = DNAND_FS_BLK_SIZE - offset;
		else
			c_size = size;

		if (blk < DNAND_FS_BLK_DATA_ST || blk >= DNAND_FS_BLK_NUM) {
			pr_warn("Got EOF\n");
			ret = DNAND_EOF_ERROR;
			break;
		} else {
			ret = dnand_blk_read(blk, offset, pbuf, c_size);
			if (ret != DNAND_NO_ERROR) {
				pr_warn("dnand_blk_read returns ERROR\n");
				break;
			}
		}
		offset = 0;
		size -= c_size;
		pbuf += c_size;
		blk = get_fat(blk);
	}
	return ret;
}

static int32_t inter_fs_write(uint32_t cid, uint16_t blk,
			      uint32_t offset, uint8_t *pbuf,
			      uint32_t size)
{
	int32_t ret;
	uint32_t i, cnt, c_size;
	uint16_t a_blk, prevblk, nextblk;
	uint8_t *p_kbuf;

	if (size == 0) {
		pr_warn("size is zero\n");
		return DNAND_NO_ERROR;
	}

	if ((blk < DNAND_FS_BLK_DATA_ST || blk >= DNAND_FS_BLK_NUM) &&
	    blk != DNAND_FS_BLK_UNUSE) {
		pr_warn("blk is not valid\n");
		return DNAND_MNG_ERROR;
	}

	ret = DNAND_NO_ERROR;
	cnt = (offset + size + DNAND_FS_BLK_SIZE - 1)/DNAND_FS_BLK_SIZE;
	a_blk = alloc_fat();
	if (a_blk >= DNAND_FS_BLK_NUM) {
		pr_warn("Failed to alloc fat\n");
		return DNAND_NOSPC_ERROR;
	}

	prevblk = DNAND_FS_BLK_UNUSE;
	p_kbuf = (uint8_t*)kmalloc(DNAND_FS_BLK_SIZE, GFP_KERNEL);
	if (!p_kbuf) {
		pr_warn("Failed to alloc fsblk\n");
		return DNAND_NOMEM_ERROR;
	}

	for (i = 0; i < cnt; i++) {
		if (offset >= DNAND_FS_BLK_SIZE) {
			if (blk >= DNAND_FS_BLK_DATA_ST && blk < DNAND_FS_BLK_NUM) {
				offset -= DNAND_FS_BLK_SIZE;
				prevblk = blk;
				blk = get_fat(prevblk);
				continue;
			}
			memset(p_kbuf, DNAND_CLEAR_HEX, DNAND_FS_BLK_SIZE);
			ret = dnand_blk_write(a_blk, 0, p_kbuf, DNAND_FS_BLK_SIZE);
			offset -= DNAND_FS_BLK_SIZE;
		} else {
			if ((offset + size) >= DNAND_FS_BLK_SIZE) {
				c_size = DNAND_FS_BLK_SIZE - offset;
			} else {
				c_size = size;
			}

			if (c_size < DNAND_FS_BLK_SIZE) {
				if(blk >= DNAND_FS_BLK_DATA_ST && blk < DNAND_FS_BLK_NUM) {
					ret = dnand_blk_read(blk, 0, p_kbuf, DNAND_FS_BLK_SIZE);
					if (ret == DNAND_NO_ERROR) {
						memcpy(p_kbuf + offset, pbuf, c_size);
						ret = dnand_blk_write(a_blk, 0, p_kbuf, DNAND_FS_BLK_SIZE);
					}
				} else {
					memset(p_kbuf, DNAND_CLEAR_HEX, DNAND_FS_BLK_SIZE);
					memcpy(p_kbuf + offset, pbuf, c_size);
					ret = dnand_blk_write(a_blk, 0, p_kbuf, DNAND_FS_BLK_SIZE);
				}
			} else {
				ret = dnand_blk_write(a_blk, 0, pbuf, c_size);
			}
			offset = 0;
			size -= c_size;
			pbuf += c_size;
		}

		if (ret != DNAND_NO_ERROR) {
			pr_warn("got something wrong\n");
			break;
		}
		nextblk = get_fat(blk);
		blk = nextblk;
		if ((nextblk >= DNAND_FS_BLK_DATA_ST && nextblk < DNAND_FS_BLK_NUM) ||
		    nextblk == DNAND_FS_BLK_EOF) {
			set_fat(a_blk, nextblk);
		} else {
			set_fat(a_blk, DNAND_FS_BLK_EOF);
		}

		if (prevblk >= DNAND_FS_BLK_DATA_ST && prevblk < DNAND_FS_BLK_NUM)
			set_fat(prevblk, a_blk);

		if(i == 0)
			set_clid(cid, a_blk);

		prevblk = a_blk;
		
		if((i+1) < cnt) {
			a_blk = alloc_fat();
			if(a_blk >= DNAND_FS_BLK_NUM) {
				pr_warn("a_blk is too big\n");
				ret = DNAND_NOSPC_ERROR;
				break;
			}
		}
	}

	kfree(p_kbuf);
	return(ret);
}

int32_t dnand_fs_read(uint32_t cid, uint32_t offset,
		      uint8_t *pbuf, uint32_t size)
{
	uint16_t blk;

	if(!pbuf) {
		pr_warn("param error\n");
		return DNAND_PARAM_ERROR;
	}

	if (cid >= DNAND_ID_ENUM_MAX ||
	    cid >= DNAND_FS_CLID_NUM) {
		pr_warn("param error\n");
		return DNAND_PARAM_ERROR;
	}

	memset(pbuf, DNAND_CLEAR_HEX, size);

	blk = get_clid(cid);
	if (blk < DNAND_FS_BLK_DATA_ST)
		return DNAND_MNG_ERROR;

	if (blk >= DNAND_FS_BLK_NUM)
		return DNAND_NOEXISTS_ERROR;

	return inter_fs_read(blk, offset, pbuf, size);
}

int32_t dnand_fs_write(uint32_t cid, uint32_t offset,
		       uint8_t *pbuf, uint32_t size)
{
	int32_t ret;
	uint16_t blk;

	if (!pbuf) {
		pr_warn("param error\n");
		return DNAND_PARAM_ERROR;
	}

	if (cid >= DNAND_ID_ENUM_MAX ||
	    cid >= DNAND_FS_CLID_NUM) {
		pr_warn("param error\n");
		return DNAND_PARAM_ERROR;
	}

	blk = get_clid(cid);
	ret  = inter_fs_write(cid, blk, offset, pbuf, size);
	if (ret == DNAND_NO_ERROR)
		ret = update_memng(cid);

	return ret;
}

int __init dnand_middle_init(void)
{
	int ret;
	int32_t cmpret;
	bool flg_valid[DNAND_FS_SYSTEM_NUM];
	uint8_t *p_kbuf0;
	uint8_t *p_kbuf1;
	dnand_fs_flmng *p_flmng0;
	dnand_fs_flmng *p_flmng1;

	ret = dnand_drv_init();
	if (ret != DNAND_NO_ERROR) {
		pr_err("Failed to init dnand_drv\n");
		return ret;
	}

	dnand_alloc_start = DNAND_FS_BLK_DATA_ST;

	p_kbuf0 = (uint8_t*)kmalloc(sizeof(dnand_fs_flmng), GFP_KERNEL);
	if(!p_kbuf0)
		return DNAND_NOMEM_ERROR;

	p_kbuf1 = (uint8_t*)kmalloc(sizeof(dnand_fs_flmng), GFP_KERNEL);
	if (!p_kbuf1) {
		ret = DNAND_NOMEM_ERROR;
		goto out_free;
	}

	ret = dnand_blk_read(0, 0, p_kbuf0, sizeof(dnand_fs_flmng));
	if (ret != DNAND_NO_ERROR)
		goto out_free;

	ret = dnand_blk_read(DNAND_FS_BLK_SYS_NUM, 0, p_kbuf1, sizeof(dnand_fs_flmng));
	if (ret != DNAND_NO_ERROR)
		goto out_free_all;

	p_flmng0 = (dnand_fs_flmng*)p_kbuf0;
	p_flmng1 = (dnand_fs_flmng*)p_kbuf1;
	flg_valid[0] = false;
	flg_valid[1] = false;

	cmpret = strcmp((char*)p_flmng0->ckcd, DNAND_FS_CKCD);
	if (cmpret == 0) {
		flg_valid[0] = true;
	}

	cmpret = strcmp((char*)p_flmng1->ckcd, DNAND_FS_CKCD);
	if (cmpret == 0) {
		flg_valid[1] = true;
	}

	if(flg_valid[0] != false && flg_valid[0] == flg_valid[1]) {
		inter_cpy_info(p_flmng0);
		cmpret = memcmp(p_flmng0, p_flmng1, sizeof(dnand_fs_flmng));
		if(cmpret != 0) {
			ret = update_flmng(1, p_flmng0);
		}
	} else if (flg_valid[0] != false) {
		inter_cpy_info(p_flmng0);
		ret = update_flmng(1, p_flmng0);
	} else if (flg_valid[1] != false) {
		inter_cpy_info(p_flmng1);
		ret = update_flmng(0, p_flmng1);
	} else {
		memset((void*)&dnand_fsinfo[0], DNAND_FILL_HEX, sizeof(dnand_fsinfo));
		ret = update_flmng(0, p_flmng0);
		if(ret == DNAND_NO_ERROR) {
			ret = update_flmng(1, p_flmng0);
		}
	}
out_free_all:
	kfree(p_kbuf1);
out_free:
	kfree(p_kbuf0);
	pr_info("dnand_middle initialized\n");

	return ret;
}
