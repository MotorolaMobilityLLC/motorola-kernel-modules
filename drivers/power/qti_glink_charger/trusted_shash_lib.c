/*
 * Copyright (C) 2020-2021 Motorola Mobility LLC
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

#include "trusted_shash_lib.h"

static const char hmac_alg[] = "hmac(sha256)";
static const char hash_alg[] = "sha1";
static char sha1_key[] = "MOTO-EXTREME-WLC";
static char hmac_key[] = "Motorola-Hyper-Proprietary-2021Gen1";

static struct crypto_shash *hashalg;
static struct crypto_shash *hmacalg;
struct sdesc {
	struct shash_desc shash;
	char ctx[];
};

static struct sdesc *init_sdesc(struct crypto_shash *alg)
{
	struct sdesc *sdesc;
	int size;

	size = sizeof(struct shash_desc) + crypto_shash_descsize(alg);
	sdesc = kmalloc(size, GFP_KERNEL);
	if (!sdesc)
		return ERR_PTR(-ENOMEM);
	sdesc->shash.tfm = alg;
	return sdesc;
}

int  trusted_shash_alloc(void)
{
	int ret;

	hmacalg = crypto_alloc_shash(hmac_alg, 0, 0);
	if (IS_ERR(hmacalg)) {
		pr_info("trusted_key: could not allocate crypto %s\n",
			hmac_alg);
		return PTR_ERR(hmacalg);
	}

	hashalg = crypto_alloc_shash(hash_alg, 0, 0);
	if (IS_ERR(hashalg)) {
		pr_info("trusted_key: could not allocate crypto %s\n",
			hash_alg);
		ret = PTR_ERR(hashalg);
		goto hashalg_fail;
	}

	return 0;

hashalg_fail:
	crypto_free_shash(hmacalg);
	return ret;
}

void trusted_shash_release(void)
{
	if (hashalg)
		crypto_free_shash(hashalg);
	if (hmacalg)
		crypto_free_shash(hmacalg);
}

int trusted_sha1_set(const unsigned char *data, unsigned int datalen,
		    unsigned char *digest)
{
	struct sdesc *sdesc;
	int ret;

	if (!hashalg) {
		pr_info("trusted: hashlg is NULL\n");
		return -1;
	}

	sdesc = init_sdesc(hashalg);
	if (IS_ERR(sdesc)) {
		pr_info("trusted_key: can't alloc %s\n", hash_alg);
		return PTR_ERR(sdesc);
	}

	ret = crypto_shash_digest(&sdesc->shash, data, datalen, digest);
	kfree_sensitive(sdesc);
	return ret;
}

int trusted_sha1(u32 *data, u32 data_size,
		    unsigned char *digest)
{
	u8 num[4] = {0};
	u32 random_num[4] = {0};
	int i = 0, ret = 0;
	char desc[SHA1_DIGEST_SIZE] = {0};

	if (data_size != 4)
		return -EPERM;

	memcpy(random_num, data, sizeof(u32) * data_size);

	for (i = 0; i < 4; i++) {
		num[i] = random_num[i];
	}

	memcpy(desc, num, sizeof(char) * 4);
	memcpy(desc + sizeof(char) * 4, sha1_key, strlen(sha1_key));

	ret = trusted_sha1_set(desc, SHA1_DIGEST_SIZE, digest);
	return ret;
}

int trusted_hmac_set(unsigned char *digest, const unsigned char *key,
		       unsigned int keylen, uint32_t *data, unsigned int dsize)
{
	struct sdesc *sdesc;
	int ret = 0, i = 0,j = 0;
	uint8_t *data_in;

	if (!hmacalg) {
		pr_info("trusted: hmacalg is NULL\n");
		return -1;
	}

	data_in = kzalloc(sizeof(*data_in) * dsize *4, GFP_KERNEL);
	if (!data_in) {
		return -ENOMEM;
	}

	for (i = 0;i < dsize;i++) {
		for (j = 0; j < sizeof(*data); j++) {
			data_in[(i * 4) + 3 - j] = (data[i] >> (j * 8)) & 0xff;
		}
	}

	sdesc = init_sdesc(hmacalg);
	if (IS_ERR(sdesc)) {
		pr_info("trusted_key: can't alloc %s\n", hmac_alg);
		ret = PTR_ERR(sdesc);
		goto out;
	}

	ret = crypto_shash_setkey(hmacalg, key, keylen);
	if (ret < 0)
		goto out;

	ret = crypto_shash_digest(&sdesc->shash, data_in, dsize * 4, digest);

out:
	kfree(data_in);
	kfree_sensitive(sdesc);
	return ret;
}

int trusted_hmac(u32 *data, u32 data_size,
		    unsigned char *digest)
{
	int ret = 0;
	u32 random_num[4] = {0};
	if (data_size != 4)
		return -EPERM;
	memcpy(random_num, data, sizeof(u32) * data_size);
 	ret = trusted_hmac_set(digest, hmac_key,
		sizeof(hmac_key), random_num, data_size);

	return ret;
}
