/*******************************************************************************
* Copyright @ LongCheer Technologies Co., Ltd. 1998-2014. All rights reserved.
* File name: securec.h
* Decription:
*             the user of this secure c library should include this header file
*             in you source code. This header file declare all supported API
*             prototype of the library, such as memcpy_s, strcpy_s, wcscpy_s,
*             strcat_s, strncat_s, sprintf_s, scanf_s, and so on.
* History:
* 1. Date:
* Author:
* Modification:
********************************************************************************/
#ifndef _SECURE_H_
#define _SECURE_H_
#include <linux/string.h>
/*define error code*/
#ifndef errno_t
typedef int errno_t;
#endif

/* success */
#ifdef EOK
#undef EOK
#endif
#define EOK (0)
/* invalid parameter */
#ifdef EINVAL
#undef EINVAL
#endif
#define EINVAL (22)
#define EINVAL_AND_RESET (22 | 0X80)

/* invalid parameter range */
#ifdef ERANGE
#undef ERANGE  /* to avoid redefinition */
#endif
#define ERANGE (34)
#define ERANGE_AND_RESET  (34 | 0X80)

//typedef unsigned int size_t;

errno_t memcpy_s(
    void* dest_buf,
    size_t dest_size,
    const void* src_buf,
    size_t src_size)
{
    if ((src_buf == NULL) || (dest_buf == NULL)) {
        return EINVAL;
    }

    if ((dest_size < src_size) || (src_size == 0)) {
        return ERANGE;
    }

    memcpy(dest_buf, src_buf, src_size);
    return EOK;
}

#endif
