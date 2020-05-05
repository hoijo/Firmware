/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "CanardNuttXCDev.hpp"

#include <fcntl.h>
#include <poll.h>

#include <nuttx/can/can.h>
#include <arch/board/board.h>

#include "stm32_can.h"

#include <px4_platform_common/log.h>

int16_t CanardNuttXCDev::open(const char *const can_iface_name, const bool can_fd)
{
	struct can_dev_s *can = stm32_caninitialize(1);

	if (can == nullptr) {
		PX4_ERR("Failed to get CAN interface");

	} else {
		/* Register the CAN driver at "/dev/can0" */
		int ret = can_register("/dev/can0", can);

		if (ret < 0) {
			PX4_ERR("can_register failed: %d", ret);

		} else {
			_fd = ::open("/dev/can0", O_RDWR | O_NONBLOCK);
		}
	}

	return 0;
}

int16_t CanardNuttXCDev::transmit(const CanardFrame &txf)
{
	if (_fd < 0) {
		return -1;
	}

	/* Copy CanardFrame to can_frame/canfd_frame */
	// txf.extended_can_id;
	// ins->send_frame.can_id |= CAN_EFF_FLAG;
	// ins->send_frame.len = txf.payload_size;
	// memcpy(&ins->send_frame.data, txf.payload, txf.payload_size);



	//struct can_frame *frame = (struct can_frame *)&ins->send_frame;
	// frame->can_id = txf.extended_can_id;
	// frame->can_id |= CAN_EFF_FLAG;
	// frame->can_dlc = txf.payload_size;
	// memcpy(&frame->data, txf.payload, txf.payload_size);

	/* Set CAN_RAW_TX_DEADLINE timestamp  */

	//ins->send_tv->tv_usec = txf.timestamp_usec % 1000000ULL;
	//ins->send_tv->tv_sec = (txf.timestamp_usec - ins->send_tv->tv_usec) / 1000000ULL;

	return 0; //(ins->s, &ins->send_msg, 0);
}

int16_t CanardNuttXCDev::receive(CanardFrame *received_frame)
{
	if ((_fd < 0) || (received_frame == nullptr)) {
		return -1;
	}

	// File desriptor for CAN.
	struct pollfd fds {};
	fds.fd = _fd;
	fds.events = POLLIN;

	// Any recieved CAN messages will cause the poll statement to unblock and run
	// This way CAN read runs with minimal latency.
	// Note that multiple messages may be received in a short time, so this will try to read any availible in a loop
	::poll(&fds, 1, 10);

	// Only execute this part if can0 is changed.
	if (fds.revents & POLLIN) {

		// Try to read.
		struct can_msg_s receive_msg;
		const ssize_t nbytes = ::read(fds.fd, &receive_msg, sizeof(receive_msg));

		if (nbytes < 0 || (size_t)nbytes < CAN_MSGLEN(0) || (size_t)nbytes > sizeof(receive_msg)) {
			// error
			return -1;

		} else {
			received_frame->extended_can_id = receive_msg.cm_hdr.ch_id;
			received_frame->payload_size = receive_msg.cm_hdr.ch_dlc;
			received_frame->payload = &receive_msg.cm_data;
		}
	}

	return 0;
}
