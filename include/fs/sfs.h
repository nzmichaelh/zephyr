/*
 * Copyright (c) 2018 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _FS_SFS_H_
#define _FS_SFS_H_

#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_FILE_NAME 12

typedef s16_t sfs_block;

struct sfs_fp {
	sfs_block block;
	s16_t offset;
	s16_t size;
	u8_t name[MAX_FILE_NAME+1];
};

struct sfs_dir {
	s16_t block;
};

FS_FILE_DEFINE(struct sfs_fp fp);
FS_DIR_DEFINE(struct sfs_dir dp);


#ifdef __cplusplus
}
#endif

#endif
