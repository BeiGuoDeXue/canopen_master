
/* File generated by gen_cfile.py. Should not be modified. */

#ifndef TESTALL_H
#define TESTALL_H

#include "data.h"

/* Prototypes of function provided by object dictionnary */
UNS32 TestAll_valueRangeTest (UNS8 typeValue, void * value);
const indextable * TestAll_scanIndexOD (CO_Data *d, UNS16 wIndex, UNS32 * errorCode);

/* Master node data struct */
extern CO_Data TestAll_Data;
extern UNS8 data[4];		/* Mapped at index 0x2000, subindex 0x01 - 0x04 */

#endif // TESTALL_H
