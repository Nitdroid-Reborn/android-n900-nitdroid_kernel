/**********************************************************************
 *
 * Copyright(c) 2008 Imagination Technologies Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful but, except
 * as otherwise stated in writing, without any warranty; without even the
 * implied warranty of merchantability or fitness for a particular purpose.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * Imagination Technologies Ltd. <gpl-support@imgtec.com>
 * Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK
 *
 ******************************************************************************/

#ifndef AUTOCONF_INCLUDED
#include <linux/config.h>
#endif

#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>

#include "services_headers.h"

#include "queue.h"
#include "resman.h"
#include "pvrmmap.h"
#include "pvr_debug.h"
#include "pvrversion.h"
#include "proc.h"

static struct proc_dir_entry *dir;

static off_t procDumpSysNodes(char *buf, size_t size, off_t off);
static off_t procDumpVersion(char *buf, size_t size, off_t off);

off_t printAppend(char *buffer, size_t size, off_t off, const char *format, ...)
{
	int n;
	int space = size - off;
	va_list ap;

	va_start(ap, format);
	n = vsnprintf(buffer + off, space, format, ap);
	va_end(ap);

	if (n > space || n < 0)
		return size;
	else
		return off + n;
}

static int pvr_read_proc(char *page, char **start, off_t off,
			 int count, int *eof, void *data)
{
	off_t (*pprn)(char *, size_t, off_t) = data;

	off_t len = pprn(page, count, off);

	if (len == END_OF_FILE) {
		len = 0;
		*eof = 1;
	} else if (!len) {
		*start = (char *)0;
	} else {
		*start = (char *)1;
	}

	return len;
}

int CreateProcEntry(const char *name, read_proc_t rhandler,
		    write_proc_t whandler, void *data)
{
	struct proc_dir_entry *file;
	mode_t mode;

	if (!dir) {
		PVR_DPF(PVR_DBG_ERROR, "CreateProcEntry: "
			"cannot make proc entry /proc/pvr/%s: no parent",
			 name);
		return -ENOMEM;
	}

	mode = S_IFREG;

	if (rhandler)
		mode |= S_IRUGO;

	if (whandler)
		mode |= S_IWUSR;

	file = create_proc_entry(name, mode, dir);

	if (file) {
		file->owner = THIS_MODULE;
		file->read_proc = rhandler;
		file->write_proc = whandler;
		file->data = data;

		PVR_DPF(PVR_DBG_MESSAGE, "Created /proc/pvr/%s", name);

		return 0;
	}

	PVR_DPF(PVR_DBG_ERROR, "CreateProcEntry: "
		 "cannot make proc entry /proc/pvr/%s: no memory", name);

	return -ENOMEM;
}

int CreateProcReadEntry(const char *name,
			off_t (handler)(char *, size_t, off_t))
{
	struct proc_dir_entry *file;

	if (!dir) {
		PVR_DPF(PVR_DBG_ERROR, "CreateProcReadEntry: "
			 "cannot make proc entry /proc/pvr/%s: no parent",
			 name);

		return -ENOMEM;
	}

	file =
	    create_proc_read_entry(name, S_IFREG | S_IRUGO, dir, pvr_read_proc,
				   (void *)handler);

	if (file) {
		file->owner = THIS_MODULE;

		return 0;
	}

	PVR_DPF(PVR_DBG_ERROR, "CreateProcReadEntry: "
			"cannot make proc entry /proc/pvr/%s: no memory",
		 name);

	return -ENOMEM;
}

int CreateProcEntries(void)
{
	dir = proc_mkdir("pvr", NULL);

	if (!dir) {
		PVR_DPF(PVR_DBG_ERROR, "CreateProcEntries: "
			"cannot make /proc/pvr directory");

		return -ENOMEM;
	}

	if (CreateProcReadEntry("queue", QueuePrintQueues) ||
	    CreateProcReadEntry("version", procDumpVersion) ||
	    CreateProcReadEntry("nodes", procDumpSysNodes)) {
		PVR_DPF(PVR_DBG_ERROR,
			 "CreateProcEntries: couldn't make /proc/pvr files");

		return -ENOMEM;
	}
#ifdef DEBUG
	if (CreateProcEntry
	    ("debug_level", PVRDebugProcGetLevel, PVRDebugProcSetLevel, NULL)) {
		PVR_DPF(PVR_DBG_ERROR, "CreateProcEntries: "
			"couldn't make /proc/pvr/debug_level");

		return -ENOMEM;
	}
#endif

	return 0;
}

void RemoveProcEntry(const char *name)
{
	if (dir)
		remove_proc_entry(name, dir);

	PVR_DPF(PVR_DBG_MESSAGE, "Removing /proc/pvr/%s", name);
}

void RemoveProcEntries(void)
{
#ifdef DEBUG
	RemoveProcEntry("debug_level");
#endif
	RemoveProcEntry("queue");
	RemoveProcEntry("nodes");
	RemoveProcEntry("version");

	while (dir->subdir) {
		PVR_DPF(PVR_DBG_WARNING, "Belatedly removing /proc/pvr/%s",
			 dir->subdir->name);

		RemoveProcEntry(dir->subdir->name);
	}

	remove_proc_entry("pvr", NULL);
}

static off_t procDumpVersion(char *buf, size_t size, off_t off)
{
	struct SYS_DATA *psSysData;

	if (off == 0)
		return printAppend(buf, size, 0, "Version %s (%s) %s\n",
				   PVRVERSION_STRING, PVR_BUILD_TYPE,
				   PVR_BUILD_DIR);

	if (SysAcquireData(&psSysData) != PVRSRV_OK)
		return PVRSRV_ERROR_GENERIC;

	if (off == 1) {
		char *pszSystemVersionString = "None";

		if (psSysData->pszVersionString)
			pszSystemVersionString = psSysData->pszVersionString;

		if (strlen(pszSystemVersionString) +
		    strlen("System Version String: \n") + 1 > size)
			return 0;
		return printAppend(buf, size, 0, "System Version String: %s\n",
				   pszSystemVersionString);
	}

	return END_OF_FILE;
}

static const char *deviceTypeToString(enum PVRSRV_DEVICE_TYPE deviceType)
{
	switch (deviceType) {
	default:
		{
			static char text[10];
			sprintf(text, "?%x", deviceType);
			return text;
		}
	}
}

static const char *deviceClassToString(enum PVRSRV_DEVICE_CLASS deviceClass)
{
	switch (deviceClass) {
	case PVRSRV_DEVICE_CLASS_3D:
		{
			return "3D";
		}
	case PVRSRV_DEVICE_CLASS_DISPLAY:
		{
			return "display";
		}
	case PVRSRV_DEVICE_CLASS_BUFFER:
		{
			return "buffer";
		}
	default:
		{
			static char text[10];

			sprintf(text, "?%x", deviceClass);
			return text;
		}
	}
}

static
off_t procDumpSysNodes(char *buf, size_t size, off_t off)
{
	struct SYS_DATA *psSysData;
	struct PVRSRV_DEVICE_NODE *psDevNode;
	off_t len;

	if (size < 80)
		return 0;

	if (off == 0)
		return printAppend(buf, size, 0, "Registered nodes\n"
			"Addr     Type     Class    Index Ref pvDev     "
			"Size Res\n");

	if (SysAcquireData(&psSysData) != PVRSRV_OK)
		return PVRSRV_ERROR_GENERIC;

	for (psDevNode = psSysData->psDeviceNodeList;
	     --off && psDevNode; psDevNode = psDevNode->psNext)
		;

	if (!psDevNode)
		return END_OF_FILE;

	len = printAppend(buf, size, 0,
			  "%p %-8s %-8s %4d  %2u  %p  %3u  %p\n",
			  psDevNode,
			  deviceTypeToString(psDevNode->sDevId.eDeviceType),
			  deviceClassToString(psDevNode->sDevId.eDeviceClass),
			  psDevNode->sDevId.eDeviceClass,
			  psDevNode->ui32RefCount,
			  psDevNode->pvDevice,
			  psDevNode->ui32pvDeviceSize,
			  psDevNode->hResManContext);
	return len;
}
