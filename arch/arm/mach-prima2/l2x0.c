/*
 * l2 cache initialization for CSR SiRFprimaII
 *
 * Copyright (c) 2011 Cambridge Silicon Radio Limited, a CSR plc group company.
 *
 * Licensed under GPLv2 or later.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <asm/hardware/cache-l2x0.h>

static int __init sirfsoc_l2x0_init(void)
{
	return l2x0_of_init(0, ~0);
}
early_initcall(sirfsoc_l2x0_init);
