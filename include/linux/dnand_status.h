#ifndef DNAND_STATUS_H
#define DNAND_STATUS_H
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2011 KYOCERA Corporation
 * (C) 2012 KYOCERA Corporation
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
#define DNAND_NO_ERROR        (0)
#define DNAND_DEV_ERROR       (-1)
#define DNAND_MNG_ERROR       (-10)
#define DNAND_NOMEM_ERROR     (-20)
#define DNAND_PARAM_ERROR     (-30)
#define DNAND_NOSPC_ERROR     (-40)
#define DNAND_INTERNAL_ERROR  (-50)
#define DNAND_NOEXISTS_ERROR  (-60)
#define DNAND_EOF_ERROR       (-70)

#endif
