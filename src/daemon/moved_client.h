 /**
 * PS Move API - An interface for the PS Move Motion Controller
 * Copyright (c) 2011, 2012 Thomas Perl <m@thp.io>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 **/


#ifndef MOVED_CLIENT_H
#define MOVED_CLIENT_H

#include "psmove_platform_config.h"
#include "psmove_moved_protocol.h"

struct moved_client;

typedef struct _moved_client_list {
	struct moved_client *client;
	struct _moved_client_list *next;
} moved_client_list;

#ifdef __cplusplus
extern "C" {
#endif

ADDAPI moved_client_list *
ADDCALL moved_client_list_open();

ADDAPI void
ADDCALL moved_client_list_destroy(moved_client_list *client_list);

ADDAPI struct moved_client *
ADDCALL moved_client_create(const char *hostname);

ADDAPI int
ADDCALL moved_client_send(struct moved_client *client, char req, char id, const unsigned char *data);

ADDAPI unsigned char *
ADDCALL moved_client_get_read_response_buffer(struct moved_client *client);

ADDAPI void
ADDCALL moved_client_destroy(struct moved_client *client);

#ifdef __cplusplus
}
#endif

#endif
