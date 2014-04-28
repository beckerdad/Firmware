// James has hacked this to make a data logging application.

/**
 * @file logbuffer.c
 *
 * Ring FIFO buffer for binary log data.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <string.h>
#include <stdlib.h>

#include "logbuffer.h"

int logbuffer_init(struct logbuffer_s *lb, int size)
{
	lb->size  = size;
	lb->write_ptr = 0;
	lb->read_ptr = 0;
	lb->data = malloc(lb->size);
	return (lb->data == 0) ? ERROR : OK;
}

int logbuffer_count(struct logbuffer_s *lb)
{
	int n = lb->write_ptr - lb->read_ptr;

	if (n < 0) {
		n += lb->size;
	}

	return n;
}

int logbuffer_is_empty(struct logbuffer_s *lb)
{
	return lb->read_ptr == lb->write_ptr;
}

bool logbuffer_write(struct logbuffer_s *lb, void *ptr, int size)
{
	// bytes available to write
	int available = lb->read_ptr - lb->write_ptr - 1;

	if (available < 0)
		available += lb->size;

	if (size > available) {
		// buffer overflow
		return false;
	}

	char *c = (char *) ptr;
	int n = lb->size - lb->write_ptr;	// bytes to end of the buffer

	if (n < size) {
		// message goes over end of the buffer
		memcpy(&(lb->data[lb->write_ptr]), c, n);
		lb->write_ptr = 0;

	} else {
		n = 0;
	}

	// now: n = bytes already written
	int p = size - n;	// number of bytes to write
	memcpy(&(lb->data[lb->write_ptr]), &(c[n]), p);
	lb->write_ptr = (lb->write_ptr + p) % lb->size;
	return true;
}

int logbuffer_get_ptr(struct logbuffer_s *lb, void **ptr, bool *is_part)
{
	// bytes available to read
	int available = lb->write_ptr - lb->read_ptr;

	if (available == 0) {
		return 0;	// buffer is empty
	}

	int n = 0;

	if (available > 0) {
		// read pointer is before write pointer, all available bytes can be read
		n = available;
		*is_part = false;

	} else {
		// read pointer is after write pointer, read bytes from read_ptr to end of the buffer
		n = lb->size - lb->read_ptr;
		*is_part = lb->write_ptr > 0;
	}

	*ptr = &(lb->data[lb->read_ptr]);
	return n;
}

void logbuffer_mark_read(struct logbuffer_s *lb, int n)
{
	lb->read_ptr = (lb->read_ptr + n) % lb->size;
}
