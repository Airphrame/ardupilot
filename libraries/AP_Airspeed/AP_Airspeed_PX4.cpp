/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 *   PX4 airspeed backend
 */


#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4  || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN

#include "AP_Airspeed_PX4.h"

uint8_t AP_Airspeed_PX4::num_px4_instances = 0;

#include <drivers/drv_airspeed.h>
#include <uORB/topics/differential_pressure.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

extern const AP_HAL::HAL &hal;

AP_Airspeed_PX4::AP_Airspeed_PX4(AP_Airspeed &_frontend, uint8_t instance, AP_Airspeed::Airspeed_State &_state) :
    AP_Airspeed_Backend(_frontend, instance, _state)
{
    _fd = open_driver();

    // consider this path used up
    num_px4_instances++;

    if (_fd == -1) {
        hal.console->printf("Unable to open PX4 airspeed sensor %u\n", num_px4_instances);
        state.status = AP_Airspeed::Airspeed_NotConnected;
        return;
    }

    if (OK != ioctl(_fd, SENSORIOCSPOLLRATE, 100) ||
        OK != ioctl(_fd, SENSORIOCSQUEUEDEPTH, 15)) {
        hal.console->println("Failed to setup airspeed driver rate and queue");
        return;
    }
}

/*
   close the file descriptor
*/
AP_Airspeed_PX4::~AP_Airspeed_PX4()
{
    if (_fd != -1) {
        close(_fd);
    }
}

/*
   open the PX4 driver, returning the file descriptor
*/
int AP_Airspeed_PX4::open_driver(void)
{
    // work out the device path based on how many PX4 drivers we have loaded
    char path[] = AIRSPEED_BASE_DEVICE_PATH "n";
    path[strlen(path)-1] = '0' + num_px4_instances;
    return open(path, O_RDONLY);
}

/*
   see if the PX4 driver is available
*/
bool AP_Airspeed_PX4::detect(AP_Airspeed &frontend, uint8_t instance)
{
    int fd = open_driver();
    if (fd == -1) {
        return false;
    }
    close(fd);
    return true;
}

// read the airspeed sensor
void AP_Airspeed_PX4::update(void)
{
    if (_fd == -1) {
        state.status = AP_Airspeed::Airspeed_NotConnected;
        return;
    }

    // read from the PX4 airspeed sensor
    float psum = 0;
    float tsum = 0;
    uint16_t count = 0;
    struct differential_pressure_s report;

    while (::read(_fd, &report, sizeof(report)) == sizeof(report) &&
           report.timestamp != _last_timestamp) {
        psum += report.differential_pressure_raw_pa;
        tsum += report.temperature;
        count++;
        _last_timestamp = report.timestamp;
    }
    if (count == 0) {
        return;
    }

    state.pressure = psum / count;

    if (state.temperature < -80) {
        // almost certainly a bad reading. The ETS driver on PX4
        // returns -1000
        state.temperature = tsum / count;
    }
}

#endif // CONFIG_HAL_BOARD
