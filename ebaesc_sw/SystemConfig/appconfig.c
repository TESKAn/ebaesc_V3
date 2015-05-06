/*******************************************************************************
*
* Freescale Semiconductor Inc.
* (c) Copyright 2004-2007 Freescale Semiconductor, Inc.
* ALL RIGHTS RESERVED.
*
********************************************************************************
*
* $File Name: appconfig.c$
*
* $Date:      Oct-12-2010$
*
* $Version:   2.3.9.0$
*
* Description: User configuration routines
*
*****************************************************************************/

#include "qs.h"

#ifdef __cplusplus
extern "C" {
#endif

void userPreMain(void);
void userPostMain(void);

/* function called before the main() is enterred */

void userPreMain(void)
{
}

/* function called after the main() is done */

void userPostMain(void)
{
	/* halt in debugger */
	archDebugHalt();
	
	/* hangup */
	for(;;)
	    /* nop */;
}

#ifdef __cplusplus
}
#endif

