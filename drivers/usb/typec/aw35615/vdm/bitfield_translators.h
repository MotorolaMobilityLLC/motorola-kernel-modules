/* SPDX-License-Identifier: GPL-2.0 */
/*******************************************************************************
 **** Copyright (C), 2020-2021,Awinic.All rights reserved. ************
 *******************************************************************************
 * File Name     : bitfield_translators.h
 * Author        : awinic
 * Date          : 2021-09-10
 * Description   : .C file function description
 * Version       : 1.0
 * Function List :
 ******************************************************************************/
#ifndef __VDM_BITFIELD_TRANSLATORS_H__
#define __VDM_BITFIELD_TRANSLATORS_H__

#include "../aw_types.h"

#ifdef AW_HAVE_VDM
/*
 * Functions that convert bits into internal header representations...
 */
UnstructuredVdmHeader getUnstructuredVdmHeader(AW_U32 in);
StructuredVdmHeader getStructuredVdmHeader(AW_U32 in);
IdHeader getIdHeader(AW_U32 in);
VdmType getVdmTypeOf(AW_U32 in);
/*
 * Functions that convert internal header representations into bits...
 */
AW_U32 getBitsForUnstructuredVdmHeader(UnstructuredVdmHeader in);
AW_U32 getBitsForStructuredVdmHeader(StructuredVdmHeader in);
AW_U32 getBitsForIdHeader(IdHeader in);

/*
 * Functions that convert bits into internal VDO representations...
 */
CertStatVdo getCertStatVdo(AW_U32 in);
ProductVdo getProductVdo(AW_U32 in);
CableVdo getCableVdo(AW_U32 in);
AmaVdo getAmaVdo(AW_U32 in);

/*
 * Functions that convert internal VDO representations into bits...
 */
AW_U32 getBitsForProductVdo(ProductVdo in);
AW_U32 getBitsForCertStatVdo(CertStatVdo in);
AW_U32 getBitsForCableVdo(CableVdo in);
AW_U32 getBitsForAmaVdo(AmaVdo in);

#endif /* __VDM_BITFIELD_TRANSLATORS_H__ */

#endif /* AW_HAVE_VDM */
