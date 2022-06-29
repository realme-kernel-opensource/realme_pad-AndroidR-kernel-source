/*************************************************************************/ /*!
@File
@Title          Header for secure firmware memory PMR factory
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Part of the memory management. This module is responsible for
                implementing the function callbacks importing secure firmware
                code allocations.
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/

#ifndef PHYSMEM_TDFWMEM_H
#define PHYSMEM_TDFWMEM_H

#include "img_types.h"
#include "pvrsrv_error.h"
#include "pvrsrv_memallocflags.h"
#include "pmr.h"

/*
 * PhysmemNewTDFWMemPMR
 *
 * This function is used as part of the facility to provide secure Firmware
 * memory. A default implementation is provided but it can be replaced by
 * the SoC implementer if necessary.
 *
 * Calling this function will create a PMR for a memory allocation made
 * in "secure Firmware memory".
 */
PVRSRV_ERROR PhysmemNewTDFWMemPMR(CONNECTION_DATA *psConnection,
                                  PVRSRV_DEVICE_NODE *psDevNode,
                                  IMG_DEVMEM_SIZE_T uiSize,
                                  PMR_LOG2ALIGN_T uiLog2Align,
                                  PVRSRV_MEMALLOCFLAGS_T uiFlags,
                                  PVRSRV_TD_FW_MEM_REGION eRegion,
                                  PMR **ppsPMRPtr);

#endif /* PHYSMEM_TDFWMEM_H */
