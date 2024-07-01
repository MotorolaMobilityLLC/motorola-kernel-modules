/*
 * Copyright (C) 2021 Motorola Mobility LLC
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __TRUSTED_SHASH_LIB_H__
#define __TRUSTED_SHASH_LIB_H__
#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/random.h>
#include <crypto/hash.h>
#include <crypto/hash_info.h>

#define SHA_NUM                  4
typedef struct trusted_shash{
  u32 random_num[SHA_NUM];
  u8 sha1[SHA1_DIGEST_SIZE];
  u8 hmac_sha256[SHA256_DIGEST_SIZE];
}TRUSTED_SHASH_RESULT, *PTRUSTED_SHASH_RESULT;

#define SHA256_NUM                  4
extern int  trusted_shash_alloc(void);
extern void trusted_shash_release(void);
extern int trusted_sha1(u32 *random_num, u32 random_size,
		    unsigned char *digest);
extern int trusted_hmac(u32 *random_num, u32 random_size,
		    unsigned char *digest);

#endif
