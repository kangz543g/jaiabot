// Copyright 2024:
//   JaiaRobotics LLC
// File authors:
//   Matt Ferro <matt.ferro@jaia.tech>
//
//
// This file is part of the JaiaBot Project Binaries
// ("The Jaia Binaries").
//
// The Jaia Binaries are free software: you can redistribute them and/or modify
// them under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 2 of the License, or
// (at your option) any later version.
//
// The Jaia Binaries are distributed in the hope that they will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with the Jaia Binaries.  If not, see <http://www.gnu.org/licenses/>.

#include "goby/util/sci.h" // for linear_interpolate
#include <algorithm>
#include <boost/units/io.hpp>
#include <goby/middleware/io/udp_point_to_point.h>
#include <vector>

#include "jaiabot/messages/arduino.pb.h"
#include "jaiabot/messages/low_control.pb.h"
#include "jaiabot/messages/motor.pb.h"
#include "jaiabot/messages/production.pb.h"

#include "system_thread.h"

#include "jaiabot/groups.h"

using goby::glog;

constexpr int thermistor_ohms_neutral = 10000;
constexpr int thermistor_voltage = 5;

jaiabot::apps::MotorStatusThread::MotorStatusThread(
    const jaiabot::config::MotorStatusConfig& cfg)
    : HealthMonitorThread(cfg, "motor_status", 5.0 * boost::units::si::hertz)
{
    status_.set_motor_harness_info_type(cfg.motor_harness_info_type());

    interthread().subscribe<jaiabot::groups::motor_udp_in>([this](const goby::middleware::protobuf::IOData& data) {
        jaiabot::protobuf::Motor motor;
        if (!motor.ParseFromString(data.data()))
        {
            glog.is_warn() && glog << "Couldn't deserialize Motor message from UDP packet"
                                   << std::endl;
            return;
        }
        glog.is_debug2() && glog << "Publishing Motor message: " << motor.ShortDebugString()
                                 << std::endl;

        rpm_value_ = motor.rpm();        
        last_motor_rpm_report_time_ = goby::time::SteadyClock::now();
    });

    interprocess().subscribe<jaiabot::groups::arduino_to_pi>(
        [this](const jaiabot::protobuf::ArduinoResponse& arduino_response) {
            if (arduino_response.has_thermistor_voltage())
            {
                float voltage = arduino_response.thermistor_voltage();
                float resistance = thermistor_ohms_neutral * voltage / (thermistor_voltage - voltage);
                float temperature =
                    goby::util::linear_interpolate(resistance, resistance_to_temperature_);
                float temperature_celsius = (temperature - 32) / 1.8;

                status_.mutable_thermistor()->set_temperature(temperature_celsius);
                status_.mutable_thermistor()->set_resistance(resistance);
                status_.mutable_thermistor()->set_voltage(voltage);

                last_motor_thermistor_report_time_ = goby::time::SteadyClock::now();
            }

            if (arduino_response.has_motor())
            {
                if (arduino_response.motor() > 1500)
                {
                    // motor is positive 
                    status_.set_rpm(std::abs(rpm_value_));
                }
                else if (arduino_response.motor() < 1500)
                {
                    // motor is negative
                    status_.set_rpm(-std::abs(rpm_value_));
                }
                else
                {
                    // motor is off
                    status_.set_rpm(0);
                }

                if (is_motor_test_)
                {
                    rpm_test_values_.push_back(status_.rpm());
                }
            }

            if (arduino_response.has_vcccurrent() && is_motor_test_)
            {
                vcc_current_test_values_.push_back(arduino_response.vcccurrent());
            }
        });

    interprocess().subscribe<jaiabot::groups::low_control>(
        [this](const jaiabot::protobuf::LowControl& low_control) {
            if (low_control.test_mode())
            {
                is_motor_test_ = true;
            }
            else if (is_motor_test_ && !low_control.test_mode())
            {
                terminate_motor_test();
            }
        });
}

void jaiabot::apps::MotorStatusThread::issue_status_summary()
{
    send_rpm_query();
    glog.is_debug2() && glog << group(thread_name()) << "Status: " << status_.DebugString()
                             << std::endl;
    interprocess().publish<jaiabot::groups::motor_status>(status_);
}

void jaiabot::apps::MotorStatusThread::send_rpm_query()
{
    glog.is_debug2() && glog << group(thread_name()) << "Senging RPM Query: " << std::endl;
    // Just send an empty packet to provide the python driver with a return address
    auto io_data = std::make_shared<goby::middleware::protobuf::IOData>();
    io_data->set_data("hello\n");
    interthread().publish<jaiabot::groups::motor_udp_out>(io_data);
}

void jaiabot::apps::MotorStatusThread::health(goby::middleware::protobuf::ThreadHealth& health)
{
    auto health_state = goby::middleware::protobuf::HEALTH__OK;

    if (last_motor_rpm_report_time_ + std::chrono::seconds(cfg().motor_rpm_report_timeout_seconds()) <
        goby::time::SteadyClock::now())
    {
        glog.is_warn() && glog << "Timeout on rpm driver" << std::endl;
        health_state = goby::middleware::protobuf::HEALTH__DEGRADED;
        health.MutableExtension(jaiabot::protobuf::jaiabot_thread)
            ->add_warning(protobuf::WARNING__NOT_RESPONDING__JAIABOT_RPM_DRIVER);
    }

    if (last_motor_thermistor_report_time_ + std::chrono::seconds(cfg().motor_thermistor_report_timeout_seconds()) <
        goby::time::SteadyClock::now())
    {
        glog.is_warn() && glog << "Timeout on thermistor data" << std::endl;
        health_state = goby::middleware::protobuf::HEALTH__DEGRADED;
        health.MutableExtension(jaiabot::protobuf::jaiabot_thread)
            ->add_warning(protobuf::WARNING__NOT_RESPONDING__JAIABOT_ARDUINO_MOTOR_TEMP);
    }

    health.set_state(health_state);
}

void discard_vector_ends(std::vector<double>& vector, int discard_size)
{
    vector.erase(vector.begin(), vector.begin() + discard_size);
    vector.erase(vector.end() - discard_size, vector.end());
}

double calculate_median(std::vector<double> vector)
{
    int size = vector.size();

    std::sort(vector.begin(), vector.end());

    // vector size is odd, take the midpoint
    if (size % 2 != 0)
    {
        return vector[size / 2];
    }
    // vector size is even, take the average of the two middle points
    else
    {
        return (vector[size / 2] + vector[size / 2 - 1]) / 2.0;
    }
}

void jaiabot::apps::MotorStatusThread::terminate_motor_test()
{
    // data points to be removed from each end of the vector
    const int discard_size = 3;

    discard_vector_ends(rpm_test_values_, discard_size);
    discard_vector_ends(vcc_current_test_values_, discard_size);

    double median_rpm = calculate_median(rpm_test_values_);
    double max_vcc_current =
        *std::max_element(vcc_current_test_values_.begin(), vcc_current_test_values_.end());
    motor_test_count_ += 1;

    jaiabot::protobuf::MotorTest motor_test_msg;
    motor_test_msg.set_test_count(motor_test_count_);
    motor_test_msg.set_median_rpm(median_rpm);
    motor_test_msg.set_max_vcc_current(max_vcc_current);

    interprocess().publish<jaiabot::groups::production>(motor_test_msg);

    is_motor_test_ = false;
    rpm_test_values_.clear();
    vcc_current_test_values_.clear();
}
