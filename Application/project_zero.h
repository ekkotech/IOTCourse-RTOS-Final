/******************************************************************************

 @file  project_zero.h

 @brief This file contains the Project Zero sample application
        definitions and prototypes.
 Group: CMCU, LPRF
 Target Device: cc2640r2

 ******************************************************************************
 
 Copyright (c) 2013-2019, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/
#ifndef PROJECTZERO_H
#define PROJECTZERO_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "lss_service.h"
#include "als_service.h"
/*********************************************************************
*  EXTERNAL VARIABLES
*/
/*********************************************************************
 * TYPEDEFS
 */
typedef struct snv_config {
    offon_char_t        offOn;
    rgb_char_t          colour;
    hyst_char_t         lmHyst;
    thresh_char_t       lmThresh;
    lmoffon_char_t      lmOffOn;
} snv_config_t;

/*********************************************************************
 * CONSTANTS
 */
//
// Non-volatile memory related
//
#define SNV_APP_ID                      0x80

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task creation function for the Simple BLE Peripheral.
 */
extern void ProjectZero_createTask(void);

char *Util_convertArrayToHexString(uint8_t const *src, uint8_t src_len,
                                          uint8_t *dst, uint8_t dst_len);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* PROJECTZERO_H */
