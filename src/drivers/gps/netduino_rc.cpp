/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Thomas Gubler <thomasgubler@student.ethz.ch>
 *           Julian Oes <joes@student.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* @file mkt.cpp */

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include "netduino_rc.h"


NETD::NETD(const int &fd, struct vehicle_gps_position_s *gps_position) :
_fd(fd),
_gps_position(gps_position),
_mtk_revision(0)
{
	decode_init();
}

NETD::~NETD()
{
}

int
NETD::configure(unsigned &baudrate)
{
	/* set baudrate first */
	if (GPS_Helper::set_baudrate(_fd, NETDUINO_BAUDRATE) != 0)
		return -1;

	baudrate = NETDUINO_BAUDRATE;

	return 0;
}

int
NETD::receive(unsigned timeout)
{
	/* poll descriptor */
	pollfd fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	uint8_t buf[32];
	netduino_packet_t packet;

	/* timeout additional to poll */
	uint64_t time_started = hrt_absolute_time();

	int j = 0;
	ssize_t count = 0;

	while (true) {

		/* first read whatever is left */
		if (j < count) {
			/* pass received bytes to the packet decoder */
			while (j < count) {
				if (parse_char(buf[j], packet) > 0) {
					handle_message(packet);
					return 1;
				}
				/* in case we keep trying but only get crap from GPS */
				if (time_started + timeout*1000 < hrt_absolute_time() ) {
					return -1;
				}
				j++;
			}
			/* everything is read */
			j = count = 0;
		}

		/* then poll for new data */
		int ret = ::poll(fds, sizeof(fds) / sizeof(fds[0]), timeout);

		if (ret < 0) {
			/* something went wrong when polling */
			return -1;

		} else if (ret == 0) {
			/* Timeout */
			return -1;

		} else if (ret > 0) {
			/* if we have new data from GPS, go handle it */
			if (fds[0].revents & POLLIN) {
				/*
				 * We are here because poll says there is some data, so this
				 * won't block even on a blocking device.  If more bytes are
				 * available, we'll go back to poll() again...
				 */
				count = ::read(_fd, buf, sizeof(buf));
			}
		}
	}
}

void
NETD::decode_init(void)
{
	_rx_ck_a = 0;
	_rx_ck_b = 0;
	_rx_count = 0;
	_decode_state = NETDUINO_DECODE_UNINIT;
}
int
NETD::parse_char(uint8_t b, netduino_packet_t &packet)
{
	int ret = 0;

	if (_decode_state == NETDUINO_DECODE_UNINIT) {

		if (b == NETDUINO_SYNC1) {
			_decode_state = NETDUINO_DECODE_GOT_CK_A;
		}

	} else if (_decode_state == NETDUINO_DECODE_GOT_CK_A) {
		if (b == NETDUINO_SYNC2) {
			_decode_state = NETDUINO_DECODE_GOT_CK_B;

		} else {
			// Second start symbol was wrong, reset state machine
			decode_init();
		}

	} else if (_decode_state == NETDUINO_DECODE_GOT_CK_B) {

		// Fill packet buffer
		((uint8_t*)(&packet))[_rx_count] = b;
		_rx_count++;

		/* Packet size minus checksum, XXX ? */
		if (_rx_count >= sizeof(packet)) {
			// Reset state machine to decode next packet
			decode_init();
		}
	}
	return ret;
}

void
NETD::handle_message(netduino_packet_t &packet)
{

	return;
}

void
NETD::add_byte_to_checksum(uint8_t b)
{
	_rx_ck_a = _rx_ck_a + b;
	_rx_ck_b = _rx_ck_b + _rx_ck_a;
}
