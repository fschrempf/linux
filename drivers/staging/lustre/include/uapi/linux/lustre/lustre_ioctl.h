/*
 * GPL HEADER START
 *
 * DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 only,
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License version 2 for more details (a copy is included
 * in the LICENSE file that accompanied this code).
 *
 * You should have received a copy of the GNU General Public License
 * version 2 along with this program; If not, see
 * http://www.gnu.org/licenses/gpl-2.0.html
 *
 * GPL HEADER END
 */
/*
 * Copyright (c) 2007, 2010, Oracle and/or its affiliates. All rights reserved.
 * Use is subject to license terms.
 *
 * Copyright (c) 2011, 2015, Intel Corporation.
 */
#ifndef _UAPI_LUSTRE_IOCTL_H_
#define _UAPI_LUSTRE_IOCTL_H_

#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <uapi/linux/lustre/lustre_idl.h>

#if !defined(__KERNEL__) && !defined(LUSTRE_UTILS)
# error This file is for Lustre internal use only.
#endif

enum md_echo_cmd {
	ECHO_MD_CREATE		= 1, /* Open/Create file on MDT */
	ECHO_MD_MKDIR		= 2, /* Mkdir on MDT */
	ECHO_MD_DESTROY		= 3, /* Unlink file on MDT */
	ECHO_MD_RMDIR		= 4, /* Rmdir on MDT */
	ECHO_MD_LOOKUP		= 5, /* Lookup on MDT */
	ECHO_MD_GETATTR		= 6, /* Getattr on MDT */
	ECHO_MD_SETATTR		= 7, /* Setattr on MDT */
	ECHO_MD_ALLOC_FID	= 8, /* Get FIDs from MDT */
};

#define OBD_DEV_ID 1
#define OBD_DEV_NAME "obd"
#define OBD_DEV_PATH "/dev/" OBD_DEV_NAME

#define OBD_IOCTL_VERSION	0x00010004
#define OBD_DEV_BY_DEVNAME	0xffffd0de

struct obd_ioctl_data {
	__u32		ioc_len;
	__u32		ioc_version;

	union {
		__u64	ioc_cookie;
		__u64	ioc_u64_1;
	};
	union {
		__u32	ioc_conn1;
		__u32	ioc_u32_1;
	};
	union {
		__u32	ioc_conn2;
		__u32	ioc_u32_2;
	};

	struct obdo	ioc_obdo1;
	struct obdo	ioc_obdo2;

	__u64		ioc_count;
	__u64		ioc_offset;
	__u32		ioc_dev;
	__u32		ioc_command;

	__u64		ioc_nid;
	__u32		ioc_nal;
	__u32		ioc_type;

	/* buffers the kernel will treat as user pointers */
	__u32		ioc_plen1;
	char __user    *ioc_pbuf1;
	__u32		ioc_plen2;
	char __user    *ioc_pbuf2;

	/* inline buffers for various arguments */
	__u32		ioc_inllen1;
	char	       *ioc_inlbuf1;
	__u32		ioc_inllen2;
	char	       *ioc_inlbuf2;
	__u32		ioc_inllen3;
	char	       *ioc_inlbuf3;
	__u32		ioc_inllen4;
	char	       *ioc_inlbuf4;

	char		ioc_bulk[0];
};

struct obd_ioctl_hdr {
	__u32		ioc_len;
	__u32		ioc_version;
};

static inline __u32 obd_ioctl_packlen(struct obd_ioctl_data *data)
{
	__u32 len = __ALIGN_KERNEL(sizeof(*data), 8);

	len += __ALIGN_KERNEL(data->ioc_inllen1, 8);
	len += __ALIGN_KERNEL(data->ioc_inllen2, 8);
	len += __ALIGN_KERNEL(data->ioc_inllen3, 8);
	len += __ALIGN_KERNEL(data->ioc_inllen4, 8);

	return len;
}

/*
 * OBD_IOC_DATA_TYPE is only for compatibility reasons with older
 * Linux Lustre user tools. New ioctls should NOT use this macro as
 * the ioctl "size". Instead the ioctl should get a "size" argument
 * which is the actual data type used by the ioctl, to ensure the
 * ioctl interface is versioned correctly.
 */
#define OBD_IOC_DATA_TYPE	long

/*	IOC_LDLM_TEST		_IOWR('f', 40, long) */
/*	IOC_LDLM_DUMP		_IOWR('f', 41, long) */
/*	IOC_LDLM_REGRESS_START	_IOWR('f', 42, long) */
/*	IOC_LDLM_REGRESS_STOP	_IOWR('f', 43, long) */

#define OBD_IOC_CREATE		_IOWR('f', 101, OBD_IOC_DATA_TYPE)
#define OBD_IOC_DESTROY		_IOW('f', 104, OBD_IOC_DATA_TYPE)
/*	OBD_IOC_PREALLOCATE	_IOWR('f', 105, OBD_IOC_DATA_TYPE) */

#define OBD_IOC_SETATTR		_IOW('f', 107, OBD_IOC_DATA_TYPE)
#define OBD_IOC_GETATTR		_IOWR('f', 108, OBD_IOC_DATA_TYPE)
#define OBD_IOC_READ		_IOWR('f', 109, OBD_IOC_DATA_TYPE)
#define OBD_IOC_WRITE		_IOWR('f', 110, OBD_IOC_DATA_TYPE)

#define OBD_IOC_STATFS		_IOWR('f', 113, OBD_IOC_DATA_TYPE)
#define OBD_IOC_SYNC		_IOW('f', 114, OBD_IOC_DATA_TYPE)
/*	OBD_IOC_READ2		_IOWR('f', 115, OBD_IOC_DATA_TYPE) */
/*	OBD_IOC_FORMAT		_IOWR('f', 116, OBD_IOC_DATA_TYPE) */
/*	OBD_IOC_PARTITION	_IOWR('f', 117, OBD_IOC_DATA_TYPE) */
/*	OBD_IOC_COPY		_IOWR('f', 120, OBD_IOC_DATA_TYPE) */
/*	OBD_IOC_MIGR		_IOWR('f', 121, OBD_IOC_DATA_TYPE) */
/*	OBD_IOC_PUNCH		_IOWR('f', 122, OBD_IOC_DATA_TYPE) */

/*	OBD_IOC_MODULE_DEBUG	_IOWR('f', 124, OBD_IOC_DATA_TYPE) */
#define OBD_IOC_BRW_READ	_IOWR('f', 125, OBD_IOC_DATA_TYPE)
#define OBD_IOC_BRW_WRITE	_IOWR('f', 126, OBD_IOC_DATA_TYPE)
#define OBD_IOC_NAME2DEV	_IOWR('f', 127, OBD_IOC_DATA_TYPE)
#define OBD_IOC_UUID2DEV	_IOWR('f', 130, OBD_IOC_DATA_TYPE)
#define OBD_IOC_GETNAME		_IOWR('f', 131, OBD_IOC_DATA_TYPE)
#define OBD_IOC_GETMDNAME	_IOR('f', 131, char[MAX_OBD_NAME])
#define OBD_IOC_GETDTNAME	OBD_IOC_GETNAME
#define OBD_IOC_LOV_GET_CONFIG	_IOWR('f', 132, OBD_IOC_DATA_TYPE)
#define OBD_IOC_CLIENT_RECOVER	_IOW('f', 133, OBD_IOC_DATA_TYPE)
#define OBD_IOC_PING_TARGET	_IOW('f', 136, OBD_IOC_DATA_TYPE)

/*	OBD_IOC_DEC_FS_USE_COUNT _IO('f', 139) */
#define OBD_IOC_NO_TRANSNO	_IOW('f', 140, OBD_IOC_DATA_TYPE)
#define OBD_IOC_SET_READONLY	_IOW('f', 141, OBD_IOC_DATA_TYPE)
#define OBD_IOC_ABORT_RECOVERY	_IOR('f', 142, OBD_IOC_DATA_TYPE)
/*	OBD_IOC_ROOT_SQUASH	_IOWR('f', 143, OBD_IOC_DATA_TYPE) */
#define OBD_GET_VERSION		_IOWR('f', 144, OBD_IOC_DATA_TYPE)
/*	OBD_IOC_GSS_SUPPORT	_IOWR('f', 145, OBD_IOC_DATA_TYPE) */
/*	OBD_IOC_CLOSE_UUID	_IOWR('f', 147, OBD_IOC_DATA_TYPE) */
#define OBD_IOC_CHANGELOG_SEND	_IOW('f', 148, OBD_IOC_DATA_TYPE)
#define OBD_IOC_GETDEVICE	_IOWR('f', 149, OBD_IOC_DATA_TYPE)
#define OBD_IOC_FID2PATH	_IOWR('f', 150, OBD_IOC_DATA_TYPE)
/*	lustre/lustre_user.h	151-153 */
/*	OBD_IOC_LOV_SETSTRIPE	154 LL_IOC_LOV_SETSTRIPE */
/*	OBD_IOC_LOV_GETSTRIPE	155 LL_IOC_LOV_GETSTRIPE */
/*	OBD_IOC_LOV_SETEA	156 LL_IOC_LOV_SETEA */
/*	lustre/lustre_user.h	157-159 */
/*	OBD_IOC_QUOTACHECK	_IOW('f', 160, int) */
/*	OBD_IOC_POLL_QUOTACHECK	_IOR('f', 161, struct if_quotacheck *) */
#define OBD_IOC_QUOTACTL	_IOWR('f', 162, struct if_quotactl)
/*	lustre/lustre_user.h	163-176 */
#define OBD_IOC_CHANGELOG_REG	_IOW('f', 177, struct obd_ioctl_data)
#define OBD_IOC_CHANGELOG_DEREG	_IOW('f', 178, struct obd_ioctl_data)
#define OBD_IOC_CHANGELOG_CLEAR	_IOW('f', 179, struct obd_ioctl_data)
/*	OBD_IOC_RECORD		_IOWR('f', 180, OBD_IOC_DATA_TYPE) */
/*	OBD_IOC_ENDRECORD	_IOWR('f', 181, OBD_IOC_DATA_TYPE) */
/*	OBD_IOC_PARSE		_IOWR('f', 182, OBD_IOC_DATA_TYPE) */
/*	OBD_IOC_DORECORD	_IOWR('f', 183, OBD_IOC_DATA_TYPE) */
#define OBD_IOC_PROCESS_CFG	_IOWR('f', 184, OBD_IOC_DATA_TYPE)
/*	OBD_IOC_DUMP_LOG	_IOWR('f', 185, OBD_IOC_DATA_TYPE) */
/*	OBD_IOC_CLEAR_LOG	_IOWR('f', 186, OBD_IOC_DATA_TYPE) */
#define OBD_IOC_PARAM		_IOW('f', 187, OBD_IOC_DATA_TYPE)
#define OBD_IOC_POOL		_IOWR('f', 188, OBD_IOC_DATA_TYPE)
#define OBD_IOC_REPLACE_NIDS	_IOWR('f', 189, OBD_IOC_DATA_TYPE)

#define OBD_IOC_CATLOGLIST	_IOWR('f', 190, OBD_IOC_DATA_TYPE)
#define OBD_IOC_LLOG_INFO	_IOWR('f', 191, OBD_IOC_DATA_TYPE)
#define OBD_IOC_LLOG_PRINT	_IOWR('f', 192, OBD_IOC_DATA_TYPE)
#define OBD_IOC_LLOG_CANCEL	_IOWR('f', 193, OBD_IOC_DATA_TYPE)
#define OBD_IOC_LLOG_REMOVE	_IOWR('f', 194, OBD_IOC_DATA_TYPE)
#define OBD_IOC_LLOG_CHECK	_IOWR('f', 195, OBD_IOC_DATA_TYPE)
/*	OBD_IOC_LLOG_CATINFO	_IOWR('f', 196, OBD_IOC_DATA_TYPE) */
#define OBD_IOC_NODEMAP		_IOWR('f', 197, OBD_IOC_DATA_TYPE)

/*	ECHO_IOC_GET_STRIPE	_IOWR('f', 200, OBD_IOC_DATA_TYPE) */
/*	ECHO_IOC_SET_STRIPE	_IOWR('f', 201, OBD_IOC_DATA_TYPE) */
/*	ECHO_IOC_ENQUEUE	_IOWR('f', 202, OBD_IOC_DATA_TYPE) */
/*	ECHO_IOC_CANCEL		_IOWR('f', 203, OBD_IOC_DATA_TYPE) */

#define OBD_IOC_GET_OBJ_VERSION	_IOR('f', 210, OBD_IOC_DATA_TYPE)

/*	lustre/lustre_user.h	212-217 */
#define OBD_IOC_GET_MNTOPT	_IOW('f', 220, mntopt_t)
#define OBD_IOC_ECHO_MD		_IOR('f', 221, struct obd_ioctl_data)
#define OBD_IOC_ECHO_ALLOC_SEQ	_IOWR('f', 222, struct obd_ioctl_data)
#define OBD_IOC_START_LFSCK	_IOWR('f', 230, OBD_IOC_DATA_TYPE)
#define OBD_IOC_STOP_LFSCK	_IOW('f', 231, OBD_IOC_DATA_TYPE)
#define OBD_IOC_QUERY_LFSCK	_IOR('f', 232, struct obd_ioctl_data)
/*	lustre/lustre_user.h	240-249 */
/*	LIBCFS_IOC_DEBUG_MASK	250 */

#define IOC_OSC_SET_ACTIVE	_IOWR('h', 21, void *)

#endif /* _UAPI_LUSTRE_IOCTL_H_ */
