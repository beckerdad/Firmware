// James has hacked this to make a data logging application.

/**
 * @file logbuffer.h
 *
 * Ring FIFO buffer for binary log data.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#ifndef SDLOG2_RINGBUFFER_H_
#define SDLOG2_RINGBUFFER_H_

#include <stdbool.h>

struct logbuffer_s {
	// pointers and size are in bytes
	int write_ptr;
	int read_ptr;
	int size;
	char *data;
};

int logbuffer_init(struct logbuffer_s *lb, int size);

int logbuffer_count(struct logbuffer_s *lb);

int logbuffer_is_empty(struct logbuffer_s *lb);

bool logbuffer_write(struct logbuffer_s *lb, void *ptr, int size);

int logbuffer_get_ptr(struct logbuffer_s *lb, void **ptr, bool *is_part);

void logbuffer_mark_read(struct logbuffer_s *lb, int n);

#endif
