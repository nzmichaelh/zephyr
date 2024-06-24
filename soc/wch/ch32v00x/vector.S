/*
 * Copyright (c) 2024 Google LLC.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/toolchain.h>

/* exports */
GTEXT(__start)

/* imports */
GTEXT(__initialize)

SECTION_FUNC(vectors, __start)
	/* Jump to __initialize */
	tail __initialize
