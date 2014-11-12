/* $NetBSD$ */

/*
 * Copyright (C) 2006-2013 Oracle Corporation
 *
 * This file is part of VirtualBox Open Source Edition (OSE), as
 * available from http://www.virtualbox.org. This file is free software;
 * you can redistribute it and/or modify it under the terms of the GNU
 * General Public License (GPL) as published by the Free Software
 * Foundation, in version 2 as it comes in the "COPYING" file of the
 * VirtualBox OSE distribution. VirtualBox OSE is distributed in the
 * hope that it will be useful, but WITHOUT ANY WARRANTY of any kind.
 *
 * The contents of this file may alternatively be used under the terms
 * of the Common Development and Distribution License Version 1.0
 * (CDDL) only, as it comes in the "COPYING.CDDL" file of the
 * VirtualBox OSE distribution, in which case the provisions of the
 * CDDL are applicable instead of those of the GPL.
 *
 * You may elect to license modified versions of this file under the
 * terms and conditions of either the GPL or the CDDL or both.
 */

/*
 * Definitions of the interface for VirtualBox VMM device.
 */

/*
 * See include/iprt/err.h.
 */
#define VINF_SUCCESS			0
#define VERR_GENERAL_FAILURE		-1

/*
 * See include/VBox/ostypes.h.
 */
typedef enum {
	VBOXOSTYPE_NetBSD =		0x62000,
	VBOXOSTYPE_NetBSD_x64 =		0x62100,
	VBOXOSTYPE_32BIT_HACK =		0x7fffffff
} VBOXOSTYPE;

/*
 * See include/VBox/VMMDev.h.
 */
#define VMMDEV_VERSION			0x00010004

#define VMMDEV_PORT_OFF_REQUEST		0

typedef enum {
	VMMDevReq_GetHostTime =		10,
	VMMDevReq_ReportGuestInfo =	50,
	VMMDevReq_SizeHack =		0x7fffffff
} VMMDevRequestType;

#define VMMDEV_REQUEST_HEADER_VERSION	0x10001
typedef struct {
	uint32_t size;
	uint32_t version;
	VMMDevRequestType request_type;
	int32_t rc;
	uint32_t reserved1;
	uint32_t reserved2;
} VMMDevRequestHeader;

typedef struct {
	VMMDevRequestHeader header;
	uint64_t time;
} VMMDevReqHostTime;

typedef struct {
	VMMDevRequestHeader header;
	struct VBoxGuestInfo {
		uint32_t interface_version;
		VBOXOSTYPE os_type;
	} guest_info;
} VMMDevReportGuestInfo;

typedef union {
	VMMDevReqHostTime req_host_time;
	VMMDevReportGuestInfo report_host_info;
} VMMDevRequestStorage;

#define VMMDEV_MEMORY_VERSION		1
/* This is not the full version of VMMDevMemory. */
typedef struct {
	uint32_t size;
	uint32_t version;
} VMMDevMemory_head;
