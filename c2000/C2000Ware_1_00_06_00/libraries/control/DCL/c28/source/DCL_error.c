/* DCL_error.c */

#include "DCL.h"
static unsigned int error_handler_count = 0;


void DCL_runErrorHandler(DCL_CSS *p)
{
    if (ERR_NONE != p->err)
    {
#ifdef __TMS320C28XX__
//        asm(" ESTOP0");
          asm("nop");
#else
//        while(1);
#endif
        error_handler_count++;
    }
}

/* end of file */
