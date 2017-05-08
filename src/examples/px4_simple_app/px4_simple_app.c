/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot which drives circles and gives out the position of the simulated object
 * as well as the acceleration data.
 *
 * @author Nils Rottann <Nils.Rottmann@tuhh.de>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

// Include uORB and the required topics for this app
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>                // this topics hold the acceleration data
#include <uORB/topics/actuator_controls.h>              // this topic gives the actuators control input
#include <uORB/topics/att_pos_mocap.h>                  // this topic holds the position from gazebo

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{
    PX4_INFO("px4_simple_app has been started!");

    /* subscribe to sensor_combined topic */
    int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
    /* limit the update rate to 5 Hz */
    orb_set_interval(sensor_sub_fd, 200);

    /* subscribe to att_pos_mocap topic */
    int position_sub_fd = orb_subscribe(ORB_ID(att_pos_mocap));
    /* limit the update rate to 5 Hz */
    orb_set_interval(position_sub_fd, 200);

    /* advertise to actuator_control topic */
    struct actuator_controls_s act;
    memset(&act, 0, sizeof(act));
    orb_advert_t act_pub = orb_advertise(ORB_ID(actuator_controls_0), &act);

    /* one could wait for multiple topics with this technique, just using one here */
    px4_pollfd_struct_t fds[] = {
        { .fd = sensor_sub_fd,   .events = POLLIN },
        { .fd = position_sub_fd,   .events = POLLIN },
        /* there could be more file descriptors here, in the form like:
         * { .fd = other_sub_fd,   .events = POLLIN },
         */
    };

    int error_counter = 0;

    for (int i = 0; i < 100; i++) {
        /* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
        int poll_ret = px4_poll(fds, 1, 1000);

        /* handle the poll result */
        if (poll_ret == 0) {
            /* this means none of our providers is giving us data */
            PX4_ERR("Got no data within a second");

        } else if (poll_ret < 0) {
            /* this is seriously bad - should be an emergency */
            if (error_counter < 10 || error_counter % 50 == 0) {
                /* use a counter to prevent flooding (and slowing us down) */
                PX4_ERR("ERROR return value from poll(): %d", poll_ret);
            }

            error_counter++;

        } else {

            if (fds[0].revents & POLLIN) {
                /* obtained data for the first file descriptor */
                struct sensor_combined_s raw_sensor;
                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw_sensor);
                // printing the sensor data into the terminal
                PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
                     (double)raw_sensor.accelerometer_m_s2[0],
                     (double)raw_sensor.accelerometer_m_s2[1],
                     (double)raw_sensor.accelerometer_m_s2[2]);

                /* obtained data for the second file descriptor */
                struct att_pos_mocap_s raw_position;
                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(att_pos_mocap), position_sub_fd, &raw_position);
                // printing the position data into the terminal
                PX4_INFO("Local Position:\t%8.4f\t%8.4f\t%8.4f",
                     (double)raw_position.x,
                     (double)raw_position.y,
                     (double)raw_position.z);
            }
        }

    // Give actuator input to the HippoC, this will result in a circle
	act.control[2] = 1.0f;		// yaw
    act.control[3] = 1.0f;		// thrust
    orb_publish(ORB_ID(actuator_controls_0), act_pub, &act);

    }


    PX4_INFO("exiting px4_simple_app!");

    return 0;
}


