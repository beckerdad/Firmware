/****************************************************************************
 *
 *   Copyright (C) 2012-2013 PX4 Development Team. All rights reserved.
 *   Author: Tobias Naegeli <naegelit@student.ethz.ch>
 *           Lorenz Meier <lm@inf.ethz.ch>
 *           Anton Babushkin <anton.babushkin@me.com>
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

/**
 * @file lockrotor_rate_control.c
 *
 * Implementation of rate controller for lockrotors.
 *
 * @author Tobias Naegeli <naegelit@student.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Julian Oes <joes@student.ethz.ch>
 */

#include "lockrotor_rate_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <float.h>
#include <math.h>
#include <systemlib/pid/pid.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

PARAM_DEFINE_FLOAT(LR_YAWRATE_P, 0.1f); /* same on Flamewheel */
PARAM_DEFINE_FLOAT(LR_YAWRATE_D, 0.01f);
PARAM_DEFINE_FLOAT(LR_YAWRATE_I, 0.0f);
//PARAM_DEFINE_FLOAT(LR_YAWRATE_AWU, 0.0f);
//PARAM_DEFINE_FLOAT(LR_YAWRATE_LIM, 1.0f);

PARAM_DEFINE_FLOAT(LR_RATTRATE_P, 0.1f); /* 0.15 F405 Flamewheel */
PARAM_DEFINE_FLOAT(LR_RATTRATE_D, 0.01f);
PARAM_DEFINE_FLOAT(LR_RATTRATE_I, 0.0f);

PARAM_DEFINE_FLOAT(LR_PATTRATE_P, 0.1f); /* 0.15 F405 Flamewheel */
PARAM_DEFINE_FLOAT(LR_PATTRATE_D, 0.01f);
PARAM_DEFINE_FLOAT(LR_PATTRATE_I, 0.0f);
//PARAM_DEFINE_FLOAT(LR_ATTRATE_AWU, 0.05f);
//PARAM_DEFINE_FLOAT(LR_ATTRATE_LIM, 1.0f);	/**< roughly < 500 deg/s limit */

struct lock_rate_control_params {

	float yawrate_p;
	float yawrate_d;
	float yawrate_i;
	//float yawrate_awu;
	//float yawrate_lim;

	float rattrate_p;
	float rattrate_d;
	float rattrate_i;

	float pattrate_p;
	float pattrate_d;
	float pattrate_i;
	//float attrate_awu;
	//float attrate_lim;

	float rate_lim;
};

struct lock_rate_control_param_handles {

	param_t yawrate_p;
	param_t yawrate_i;
	param_t yawrate_d;
	//param_t yawrate_awu;
	//param_t yawrate_lim;

	param_t rattrate_p;
	param_t rattrate_i;
	param_t rattrate_d;
	param_t pattrate_p;
	param_t pattrate_i;
	param_t pattrate_d;
	//param_t attrate_awu;
	//param_t attrate_lim;
};

/**
 * Initialize all parameter handles and values
 *
 */
static int parameters_init(struct lock_rate_control_param_handles *h);

/**
 * Update all parameters
 *
 */
static int parameters_update(const struct lock_rate_control_param_handles *h, struct lock_rate_control_params *p);


static int parameters_init(struct lock_rate_control_param_handles *h)
{
	/* PID parameters */
	h->yawrate_p 	=	param_find("LR_YAWRATE_P");
	h->yawrate_i 	=	param_find("LR_YAWRATE_I");
	h->yawrate_d 	=	param_find("LR_YAWRATE_D");
	//h->yawrate_awu 	=	param_find("LR_YAWRATE_AWU");
	//h->yawrate_lim 	=	param_find("LR_YAWRATE_LIM");

	h->rattrate_p 	= 	param_find("LR_RATTRATE_P");
	h->rattrate_i 	= 	param_find("LR_RATTRATE_I");
	h->rattrate_d 	= 	param_find("LR_RATTRATE_D");
	h->pattrate_p 	= 	param_find("LR_PATTRATE_P");
	h->pattrate_i 	= 	param_find("LR_PATTRATE_I");
	h->pattrate_d 	= 	param_find("LR_PATTRATE_D");
	//h->attrate_awu 	= 	param_find("LR_ATTRATE_AWU");
	//h->attrate_lim 	= 	param_find("LR_ATTRATE_LIM");

	return OK;
}

static int parameters_update(const struct lock_rate_control_param_handles *h, struct lock_rate_control_params *p)
{
	param_get(h->yawrate_p, &(p->yawrate_p));
	param_get(h->yawrate_i, &(p->yawrate_i));
	param_get(h->yawrate_d, &(p->yawrate_d));
	//param_get(h->yawrate_awu, &(p->yawrate_awu));
	//param_get(h->yawrate_lim, &(p->yawrate_lim));

	param_get(h->rattrate_p, &(p->rattrate_p));
	param_get(h->rattrate_i, &(p->rattrate_i));
	param_get(h->rattrate_d, &(p->rattrate_d));
	param_get(h->pattrate_p, &(p->pattrate_p));
	param_get(h->pattrate_i, &(p->pattrate_i));
	param_get(h->pattrate_d, &(p->pattrate_d));
	//param_get(h->attrate_awu, &(p->attrate_awu));
	//param_get(h->attrate_lim, &(p->attrate_lim));

	return OK;
}

void lockrotor_control_rates(const struct vehicle_rates_setpoint_s *rate_sp,
			      const float rates[], struct actuator_controls_s *actuators)
{
	static uint64_t last_run = 0;
	const float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
	static uint64_t last_input = 0;

	if (last_input != rate_sp->timestamp) {
		last_input = rate_sp->timestamp;
	}

	last_run = hrt_absolute_time();

	static int motor_skip_counter = 0;

	static PID_t pitch_rate_controller;
	static PID_t roll_rate_controller;

	static struct lock_rate_control_params p;
	static struct lock_rate_control_param_handles h;

	static bool initialized = false;

	/* initialize the pid controllers when the function is called for the first time */
	if (initialized == false) {
		parameters_init(&h);
		parameters_update(&h, &p);
		initialized = true;

		pid_init(&pitch_rate_controller, p.pattrate_p, p.pattrate_i, p.pattrate_d, 1.0f, 1.0f, PID_MODE_DERIVATIV_CALC_NO_SP, 0.003f);
		pid_init(&roll_rate_controller, p.rattrate_p, p.rattrate_i, p.rattrate_d, 1.0f, 1.0f, PID_MODE_DERIVATIV_CALC_NO_SP, 0.003f);

	}

	/* load new parameters with lower rate */
	if (motor_skip_counter % 2500 == 0) {
		/* update parameters from storage */
		parameters_update(&h, &p);
		pid_set_parameters(&pitch_rate_controller, p.pattrate_p, p.pattrate_i, p.pattrate_d, 1.0f, 1.0f);
		pid_set_parameters(&roll_rate_controller,  p.rattrate_p, p.rattrate_i, p.rattrate_d, 1.0f, 1.0f);
	}

	/* reset integral if on ground */
	if (rate_sp->thrust < 0.01f) {
		pid_reset_integral(&pitch_rate_controller);
		pid_reset_integral(&roll_rate_controller);
	}

	/* control pitch (forward) output */
	float pitch_control = pid_calculate(&pitch_rate_controller, rate_sp->pitch ,
					    rates[1], 0.0f, deltaT);

	/* control roll (left/right) output */
	float roll_control = pid_calculate(&roll_rate_controller, rate_sp->roll ,
					   rates[0], 0.0f, deltaT);

	/* control yaw rate */ //XXX use library here
	float yaw_rate_control = p.yawrate_p * (rate_sp->yaw - rates[2]);

	/* increase resilience to faulty control inputs */
	if (!isfinite(yaw_rate_control)) {
		yaw_rate_control = 0.0f;
		warnx("rej. NaN ctrl yaw");
	}

	actuators->control[0] = roll_control;
	actuators->control[1] = pitch_control;
	actuators->control[2] = yaw_rate_control;
	actuators->control[3] = rate_sp->thrust;

	motor_skip_counter++;
}
