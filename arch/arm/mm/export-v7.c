/*
 *  linux/arch/arm/mm/export-v7.c
 *
 *  Copyright (C) 2010 Nokia Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  This is the "shell" of the ARMv7 processor support.
 */

#include <linux/module.h>
#include <linux/mm.h>

extern void v7_flush_dcache_all(void);

EXPORT_SYMBOL(v7_flush_dcache_all);

