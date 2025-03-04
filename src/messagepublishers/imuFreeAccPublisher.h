// BSD 3-Clause License
//
// Copyright (c) 2021, BlueSpace.ai, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//  Copyright (c) 2003-2020 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//

#ifndef IMUFREEPUBLISHER_H
#define IMUFREEPUBLISHER_H

#include <iomanip> // std::get_time
#include <ctime>   // strcut std::tm

#include "packetcallback.h"
#include "publisherhelperfunctions.h"

#include <sensor_msgs/msg/imu.hpp>
#include <eigen3/Eigen/Dense>

#include "kalman_filter.h"

struct ImuFreeAccPublisher : public PacketCallback, PublisherHelperFunctions
{
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuRawPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr kalmanFilterPublisher;
    double orientation_variance[3];
    double linear_acceleration_variance[3];
    double angular_velocity_variance[3];
    int frameId = 0;
    int timeZoneOffset;
    rclcpp::Node &node_handle;
    bool useImuTime = false;
    // Kalman filter vars
    bool doKalmanFilter = false;
    int numberOfStates = 6;
    int numberOfMesurments = 6;
    float freq = 200;
    KalmanFilter kf;

    ImuFreeAccPublisher(rclcpp::Node &node)
        : node_handle(node)
    {
        std::vector<double> variance = {0, 0, 0};
        double freeAccVariance = 0.0;
        double angVelVariance = 0.0;
        double processNoiseFactor;

        // declare kalman params
        node.declare_parameter("do_kalman_filter", doKalmanFilter);
        node.declare_parameter("free_acc_var", freeAccVariance);
        node.declare_parameter("ang_vel_var", angVelVariance);
        node.declare_parameter("process_noise_factor", processNoiseFactor);
        node.declare_parameter("freq", freq);

        // declare generel params
        node.declare_parameter("use_imu_time", useImuTime);
        
        // Init publisher
        int pub_queue_size = 5;
        node.get_parameter("publisher_queue_size", pub_queue_size);
        node.get_parameter("do_kalman_filter", doKalmanFilter);
        imuRawPublisher = node.create_publisher<sensor_msgs::msg::Imu>("/imu/data", pub_queue_size);

        if (doKalmanFilter)
        {

            kalmanFilterPublisher = node.create_publisher<sensor_msgs::msg::Imu>("/imu/kalman_filter", pub_queue_size);
            node.get_parameter("free_acc_var", freeAccVariance);
            node.get_parameter("ang_vel_var", angVelVariance);
            node.get_parameter("process_noise_factor", processNoiseFactor);
            node.get_parameter("freq", freq);
            initKalman(freeAccVariance, angVelVariance, processNoiseFactor);
        }
        // get time parameters
        node.get_parameter("use_imu_time", useImuTime);
        node.get_parameter("time_zone_offset", timeZoneOffset);
        // REP 145: Conventions for IMU Sensor Drivers (http://www.ros.org/reps/rep-0145.html)
        variance_from_stddev_param("orientation_stddev", orientation_variance, node);
        variance_from_stddev_param("angular_velocity_stddev", angular_velocity_variance, node);
        variance_from_stddev_param("linear_acceleration_stddev", linear_acceleration_variance, node);
    }

    void initKalman(double freeAccVariance, double angVelVariance, double processNoiseFactor)
    {

        double dt = 1.0 / float(freq);
        std::cout << "frequency is: " << freq << " dt is " << dt << std::endl; 
        Eigen::MatrixXd A = Eigen::MatrixXd::Identity(numberOfStates, numberOfStates);         // System dynamics matrix
        Eigen::MatrixXd C = Eigen::MatrixXd::Identity(numberOfMesurments, numberOfStates);     // Output matrix
        Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(numberOfStates, numberOfStates);         // Process noise covariance
        Eigen::MatrixXd R = Eigen::MatrixXd::Identity(numberOfMesurments, numberOfMesurments); // Measurement noise covariance
        Eigen::MatrixXd P = Eigen::MatrixXd::Identity(numberOfMesurments, numberOfMesurments); // Initial state error

        // set Process noise covariance
        Q(0, 0) = processNoiseFactor * angVelVariance;
        Q(1, 1) = processNoiseFactor * angVelVariance;
        Q(2, 2) = processNoiseFactor * angVelVariance;
        Q(3, 3) = processNoiseFactor * freeAccVariance;
        Q(4, 4) = processNoiseFactor * freeAccVariance;
        Q(5, 5) = processNoiseFactor * freeAccVariance;

        // set Measurement noise covariance
        R(0, 0) = angVelVariance;
        R(1, 1) = angVelVariance;
        R(2, 2) = angVelVariance;
        R(3, 3) = freeAccVariance;
        R(4, 4) = freeAccVariance;
        R(5, 5) = freeAccVariance;

        // set Initial state error
        P(0, 0) = angVelVariance;
        P(1, 1) = angVelVariance;
        P(2, 2) = angVelVariance;
        P(3, 3) = freeAccVariance;
        P(4, 4) = freeAccVariance;
        P(5, 5) = freeAccVariance;

        std::cout << "[BLUESPACE IMU] A: \n"
                  << A << std::endl;
        std::cout << "[BLUESPACE IMU] C: \n"
                  << C << std::endl;
        std::cout << "[BLUESPACE IMU] Q: \n"
                  << Q << std::endl;
        std::cout << "[BLUESPACE IMU] R: \n"
                  << R << std::endl;
        std::cout << "[BLUESPACE IMU] P: \n"
                  << P << std::endl;
        kf = KalmanFilter(dt, A, C, Q, R, P);
    }

    rclcpp::Time get_utc_time(const XsDataPacket &packet)
    {
        uint32_t sec, nsec;
        auto packetTime = packet.utcTime();

        // convert time struct to utc float time
        std::ostringstream date;
        date << packetTime.m_year << "-" << std::setw(2) << std::setfill('0') << std::to_string(packetTime.m_month) << "-" << std::setw(2) << std::setfill('0') << std::to_string(packetTime.m_day) << "T" << std::setw(2) << std::setfill('0') << std::to_string(packetTime.m_hour + timeZoneOffset) << ":" << std::setw(2) << std::setfill('0') << std::to_string(packetTime.m_minute) << ":" << std::setw(2) << std::setfill('0') << std::to_string(packetTime.m_second) << "Z";

        std::istringstream ss(date.str());
        std::tm t{};
        ss >> std::get_time(&t, "%Y-%m-%dT%H:%M:%S");
        if (ss.fail())
        {
            throw std::runtime_error{"failed to parse time string"};
        }
        std::time_t time_stamp = mktime(&t);

        sec = int(time_stamp);
        nsec = packetTime.m_nano;

        rclcpp::Time sample_time(sec, nsec);
        return sample_time;
    }

    void preformKalmanFilter(geometry_msgs::msg::Vector3& angularVel, geometry_msgs::msg::Vector3& linearAcc)
    {

        if (!kf.isIntialized())
        {
            // Best guess of initial states
            Eigen::VectorXd x0(numberOfStates);
            x0 << angularVel.x, angularVel.y, angularVel.z, linearAcc.x, linearAcc.y, linearAcc.z;
            double t = 0;
            kf.init(t, x0);
        }
        Eigen::VectorXd currMeasurnent(numberOfMesurments);
        currMeasurnent << angularVel.x, angularVel.y, angularVel.z, linearAcc.x, linearAcc.y, linearAcc.z;
        kf.update(currMeasurnent);
        Eigen::VectorXd newState = kf.state();
        angularVel.x = newState(0);
        angularVel.y = newState(1);
        angularVel.z = newState(2);
        linearAcc.x = newState(3);
        linearAcc.y = newState(4);
        linearAcc.z = newState(5);

    }

    void operator()(const XsDataPacket &packet, rclcpp::Time timestamp)
    {
        bool quaternion_available = packet.containsOrientation();
        bool gyro_available = packet.containsCalibratedGyroscopeData();
        bool accel_available = packet.containsFreeAcceleration();

        geometry_msgs::msg::Quaternion quaternion;
        if (quaternion_available)
        {
            XsQuaternion q = packet.orientationQuaternion();

            quaternion.w = q.w();
            quaternion.x = q.x();
            quaternion.y = q.y();
            quaternion.z = q.z();
        }

        geometry_msgs::msg::Vector3 gyro, filterGyro;
        if (gyro_available)
        {
            XsVector g = packet.calibratedGyroscopeData();
            gyro.x = filterGyro.x =  g[0];
            gyro.y = filterGyro.y =  g[1];
            gyro.z = filterGyro.z =  g[2];
        }

        geometry_msgs::msg::Vector3 accel, filterAccel;
        if (accel_available)
        {
            XsVector a = packet.freeAcceleration();
            accel.x = filterAccel.x =  a[0];
            accel.y = filterAccel.y =  a[1];
            accel.z = filterAccel.z =  a[2];
        }

        // Imu message, publish if any of the fields is available
        if (quaternion_available || accel_available || gyro_available)
        {
            sensor_msgs::msg::Imu msg;

            if (packet.containsUtcTime() && useImuTime)
                msg.header.stamp = get_utc_time(packet);
            else
                msg.header.stamp = timestamp;
            msg.header.frame_id = std::to_string(frameId);
            frameId++;
            
            msg.orientation = quaternion;
            if (quaternion_available)
            {
                msg.orientation_covariance[0] = orientation_variance[0];
                msg.orientation_covariance[4] = orientation_variance[1];
                msg.orientation_covariance[8] = orientation_variance[2];
            }
            else
            {
                msg.orientation_covariance[0] = -1; // mark as not available
            }

            msg.angular_velocity = gyro;
            if (gyro_available)
            {
                msg.angular_velocity_covariance[0] = angular_velocity_variance[0];
                msg.angular_velocity_covariance[4] = angular_velocity_variance[1];
                msg.angular_velocity_covariance[8] = angular_velocity_variance[2];
            }
            else
            {
                msg.angular_velocity_covariance[0] = -1; // mark as not available
            }

            msg.linear_acceleration = accel;
            if (accel_available)
            {
                msg.linear_acceleration_covariance[0] = linear_acceleration_variance[0];
                msg.linear_acceleration_covariance[4] = linear_acceleration_variance[1];
                msg.linear_acceleration_covariance[8] = linear_acceleration_variance[2];
            }
            else
            {
                msg.linear_acceleration_covariance[0] = -1; // mark as not available
            }

            imuRawPublisher->publish(msg);
            if (doKalmanFilter)
            {
                preformKalmanFilter(filterGyro, filterAccel);
                msg.angular_velocity = filterGyro;
                msg.linear_acceleration = filterAccel;
                kalmanFilterPublisher->publish(msg);
            }
            
        }
    }
};

#endif
