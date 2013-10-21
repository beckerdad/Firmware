/****************************************************************************
 *
 *   Copyright (C) 2008-2013 PX4 Development Team. All rights reserved.
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

/* @file mtk.h */

#ifndef NETD_H_
#define NETD_H_

#include "gps_helper.h"

#define NETDUINO_SYNC1 0xd0
#define NETDUINO_SYNC2 0xdd

#define MTK_OUTPUT_5HZ		"$PMTK220,200*2C\r\n"
#define MTK_SET_BINARY		"$PGCMD,16,0,0,0,0,0*6A\r\n"
#define SBAS_ON	        	"$PMTK313,1*2E\r\n"
#define WAAS_ON				"$PMTK301,2*2E\r\n"
#define MTK_NAVTHRES_OFF 	"$PMTK397,0*23\r\n"

#define MTK_TIMEOUT_5HZ 400
#define NETDUINO_BAUDRATE 115200

typedef enum {
	NETDUINO_DECODE_UNINIT = 0,
	NETDUINO_DECODE_GOT_CK_A = 1,
	NETDUINO_DECODE_GOT_CK_B = 2
} netduino_decode_state_t;

/** the structures of the binary packets */
#pragma pack(push, 1)

typedef struct {
	uint16_t rc1;
	uint16_t rc2;
	uint16_t rc3;
	uint16_t rc4;
	uint16_t rc5;
	uint16_t rc6;
} netduino_packet_t;

#pragma pack(pop)

#define MTK_RECV_BUFFER_SIZE 40

class NETD : public GPS_Helper
{
public:
	NETD(const int &fd, struct rc_over_uart_s *rc_position);
	~NETD();
	int				receive(unsigned timeout);
	int				configure(unsigned &baudrate);

private:
	/**
	 * Parse the binary MTK packet
	 */
	int				parse_char(uint8_t b, netduino_packet_t &packet);

	/**
	 * Handle the package once it has arrived
	 */
	void				handle_message(netduino_packet_t &packet);

	/**
	 * Reset the parse state machine for a fresh start
	 */
	void				decode_init(void);

	/**
	 * While parsing add every byte (except the sync bytes) to the checksum
	 */
	void				add_byte_to_checksum(uint8_t);

	int					_fd;
	struct rc_over_uart_s *_rc_position;
	netduino_decode_state_t	_decode_state;
	uint8_t				_mtk_revision;
	uint8_t				_rx_buffer[MTK_RECV_BUFFER_SIZE];
	unsigned			_rx_count;
	uint8_t 			_rx_ck_a;
	uint8_t				_rx_ck_b;
};

#endif /* MTK_H_ */
