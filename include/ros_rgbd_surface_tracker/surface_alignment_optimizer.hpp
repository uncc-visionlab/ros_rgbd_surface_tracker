/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   transformations.hpp
 * Author: arwillis
 *
 * Created on July 2, 2017, 1:05 PM
 */

#ifndef SURFACE_ALIGNMENT_OPTIMIZER_HPP
#define SURFACE_ALIGNMENT_OPTIMIZER_HPP

#ifdef __cplusplus

#include <iostream>

// Eigen includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <ros_rgbd_surface_tracker/AlgebraicSurface.hpp>

template <typename ScalarType>
Eigen::Matrix<ScalarType, 3, 7> jacobianTransformedPointQuat(
        ScalarType px, ScalarType py, ScalarType pz,
        ScalarType tx, ScalarType ty, ScalarType tz,
        ScalarType s, ScalarType vx, ScalarType vy, ScalarType vz) {
    // Jacobian rows from top to bottom: x, y, z
    // Jacobian columns from left to right: s, vx, vy, vz, tx, ty, tz

    Eigen::Matrix<ScalarType, 3, 7> jacobian;

    ScalarType s2_vx2_vy2_vz2 = pow(s, 2) + pow(vx, 2) + pow(vy, 2) + pow(vz, 2);
    ScalarType s2_vx2_vy2_vz2_sq = pow(s2_vx2_vy2_vz2, 2);
    ScalarType pdotv = px * vx + py * vy + pz*vz;
    ScalarType restx = px * pow(s, 2) - 2 * s * (py * vz - pz * vy) - px * (pow(vy, 2)
            + pow(vz, 2)) + py * vx * vy + pz * vx*vz;
    ScalarType resty = py * pow(s, 2) + 2 * s * (px * vz - pz * vx) - py * (pow(vx, 2)
            + pow(vz, 2)) + px * vx * vy + pz * vy*vz;
    ScalarType restz = pz * pow(s, 2) - 2 * s * (px * vy - py * vx) - pz * (pow(vx, 2)
            + pow(vy, 2)) + px * vx * vz + py * vy*vz;
    ScalarType _2pxs_2pyvz_2pzvy = 2 * px * s - 2 * py * vz + 2 * pz*vy;
    ScalarType _2pxvx_2pyvy_2pzvy = 2 * px * vx + 2 * py * vy + 2 * pz*vz;
    ScalarType _2pzs_2pxvy_2pyvx = 2 * pz * s - 2 * px * vy + 2 * py*vx;
    ScalarType _2pys_2pxvz_2pzvx = 2 * py * s + 2 * px * vz - 2 * pz*vx;

    jacobian(0, 0) = (_2pxs_2pyvz_2pzvy) / (s2_vx2_vy2_vz2)
            - (2 * s * (vx * (pdotv) + restx)) / s2_vx2_vy2_vz2_sq;
    jacobian(0, 1) = (_2pxvx_2pyvy_2pzvy) / (s2_vx2_vy2_vz2)
            - (2 * vx * (vx * (pdotv) + restx)) / s2_vx2_vy2_vz2_sq;
    jacobian(0, 2) = (_2pzs_2pxvy_2pyvx) / (s2_vx2_vy2_vz2)
            - (2 * vy * (vx * (pdotv) + restx)) / s2_vx2_vy2_vz2_sq;
    jacobian(0, 3) = -(_2pys_2pxvz_2pzvx) / (s2_vx2_vy2_vz2)
            - (2 * vz * (vx * (pdotv) + restx)) / s2_vx2_vy2_vz2_sq;
    jacobian(0, 4) = 1;
    jacobian(0, 5) = 0;
    jacobian(0, 6) = 0;

    jacobian(1, 0) = (_2pys_2pxvz_2pzvx) / (s2_vx2_vy2_vz2)
            - (2 * s * (vy * (pdotv) + resty)) / s2_vx2_vy2_vz2_sq;
    jacobian(1, 1) = -(_2pzs_2pxvy_2pyvx) / (s2_vx2_vy2_vz2)
            - (2 * vx * (vy * (pdotv) + resty)) / s2_vx2_vy2_vz2_sq;
    jacobian(1, 2) = (_2pxvx_2pyvy_2pzvy) / (s2_vx2_vy2_vz2)
            - (2 * vy * (vy * (pdotv) + resty)) / s2_vx2_vy2_vz2_sq;
    jacobian(1, 3) = (_2pxs_2pyvz_2pzvy) / (s2_vx2_vy2_vz2)
            - (2 * vz * (vy * (pdotv) + resty)) / s2_vx2_vy2_vz2_sq;
    jacobian(1, 4) = 0;
    jacobian(1, 5) = 1;
    jacobian(1, 6) = 0;

    jacobian(2, 0) = (_2pzs_2pxvy_2pyvx) / (s2_vx2_vy2_vz2)
            - (2 * s * (vz * (pdotv) + restz)) / s2_vx2_vy2_vz2_sq;
    jacobian(2, 1) = (_2pys_2pxvz_2pzvx) / (s2_vx2_vy2_vz2)
            - (2 * vx * (vz * (pdotv) + restz)) / s2_vx2_vy2_vz2_sq;
    jacobian(2, 2) = -(_2pxs_2pyvz_2pzvy) / (s2_vx2_vy2_vz2)
            - (2 * vy * (vz * (pdotv) + restz)) / s2_vx2_vy2_vz2_sq;
    jacobian(2, 3) = (_2pxvx_2pyvy_2pzvy) / (s2_vx2_vy2_vz2)
            - (2 * vz * (vz * (pdotv) + restz)) / s2_vx2_vy2_vz2_sq;
    jacobian(2, 4) = 0;
    jacobian(2, 5) = 0;
    jacobian(2, 6) = 1;

    return jacobian;
}

template <typename ScalarType>
Eigen::Matrix<ScalarType, 3, Eigen::Dynamic> jacobianTransformedPointAxisAngle(
        ScalarType px, ScalarType py, ScalarType pz,
        ScalarType tx, ScalarType ty, ScalarType tz,
        ScalarType vx, ScalarType vy, ScalarType vz, ScalarType theta) {
    // Omission of optional argument theta assumes axis vector magnitude -> angle
    // Jacobian rows from top to bottom: x, y, z
    // Jacobian columns from left to right: vx, vy, vz, tx, ty, tz
    //                       or with theta: theta, vx, vy, vz, tx, ty, tz

    Eigen::Matrix<ScalarType, 3, Eigen::Dynamic> jacobian;

    if (std::isnan(theta)) {
        ScalarType mag_v = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));
        ScalarType sin_mag_v = sin(mag_v);
        ScalarType cos_mag_v = cos(mag_v);
        ScalarType cos_mag_v_m1 = cos_mag_v - 1;

        jacobian.resize(3, 6);
        jacobian(0, 0) = pz * ((vx * vy * cos_mag_v) / mag_v - vz * cos_mag_v_m1
                + (pow(vx, 2) * vz * sin_mag_v) / mag_v) - py * (vy * cos_mag_v_m1
                + (vx * vz * cos_mag_v) / mag_v - (pow(vx, 2) * vy * sin_mag_v) / mag_v)
                - px * ((vx * pow(vy, 2) * sin_mag_v) / mag_v + (vx * pow(vz, 2) * sin_mag_v) / mag_v);
        jacobian(0, 1) = pz * (sin_mag_v + (pow(vy, 2) * cos_mag_v) / mag_v
                + (vx * vy * vz * sin_mag_v) / mag_v) - px * ((pow(vy, 3) * sin_mag_v) / mag_v
                - 2 * vy * cos_mag_v_m1 + (vy * pow(vz, 2) * sin_mag_v) / mag_v)
                - py * (vx * cos_mag_v_m1 + (vy * vz * cos_mag_v) / mag_v
                - (vx * pow(vy, 2) * sin_mag_v) / mag_v);
        jacobian(0, 2) = pz * ((vy * vz * cos_mag_v) / mag_v - vx * cos_mag_v_m1
                + (vx * pow(vz, 2) * sin_mag_v) / mag_v) - px * ((pow(vz, 3) * sin_mag_v) / mag_v
                - 2 * vz * cos_mag_v_m1 + (pow(vy, 2) * vz * sin_mag_v) / mag_v)
                - py * (sin_mag_v + (pow(vz, 2) * cos_mag_v) / mag_v
                - (vx * vy * vz * sin_mag_v) / mag_v);
        jacobian(0, 3) = 1;
        jacobian(0, 4) = 0;
        jacobian(0, 5) = 0;

        jacobian(1, 0) = px * ((vx * vz * cos_mag_v) / mag_v - vy * cos_mag_v_m1
                + (pow(vx, 2) * vy * sin_mag_v) / mag_v) - py * ((pow(vx, 3) * sin_mag_v) / mag_v
                - 2 * vx * cos_mag_v_m1 + (vx * pow(vz, 2) * sin_mag_v) / mag_v)
                - pz * (sin_mag_v + (pow(vx, 2) * cos_mag_v) / mag_v
                - (vx * vy * vz * sin_mag_v) / mag_v);
        jacobian(1, 1) = px * ((vy * vz * cos_mag_v) / mag_v - vx * cos_mag_v_m1
                + (vx * pow(vy, 2) * sin_mag_v) / mag_v) - py * ((pow(vx, 2) * vy * sin_mag_v) / mag_v
                + (vy * pow(vz, 2) * sin_mag_v) / mag_v) - pz * (vz * cos_mag_v_m1
                + (vx * vy * cos_mag_v) / mag_v - (pow(vy, 2) * vz * sin_mag_v) / mag_v);
        jacobian(1, 2) = px * (sin_mag_v + (pow(vz, 2) * cos_mag_v) / mag_v
                + (vx * vy * vz * sin_mag_v) / mag_v) - py * ((pow(vz, 3) * sin_mag_v) / mag_v
                - 2 * vz * cos_mag_v_m1 + (pow(vx, 2) * vz * sin_mag_v) / mag_v)
                - pz * (vy * cos_mag_v_m1 + (vx * vz * cos_mag_v) / mag_v
                - (vy * pow(vz, 2) * sin_mag_v) / mag_v);
        jacobian(1, 3) = 0;
        jacobian(1, 4) = 1;
        jacobian(1, 5) = 0;

        jacobian(2, 0) = py * (sin_mag_v + (pow(vx, 2) * cos_mag_v) / mag_v
                + (vx * vy * vz * sin_mag_v) / mag_v) - pz * ((pow(vx, 3) * sin_mag_v) / mag_v
                - 2 * vx * cos_mag_v_m1 + (vx * pow(vy, 2) * sin_mag_v) / mag_v)
                - px * (vz * cos_mag_v_m1 + (vx * vy * cos_mag_v) / mag_v
                - (pow(vx, 2) * vz * sin_mag_v) / mag_v);
        jacobian(2, 1) = py * ((vx * vy * cos_mag_v) / mag_v - vz * cos_mag_v_m1
                + (pow(vy, 2) * vz * sin_mag_v) / mag_v) - pz * ((pow(vy, 3) * sin_mag_v) / mag_v
                - 2 * vy * cos_mag_v_m1 + (pow(vx, 2) * vy * sin_mag_v) / mag_v)
                - px * (sin_mag_v + (pow(vy, 2) * cos_mag_v) / mag_v
                - (vx * vy * vz * sin_mag_v) / mag_v);
        jacobian(2, 2) = py * ((vx * vz * cos_mag_v) / mag_v - vy * cos_mag_v_m1
                + (vy * pow(vz, 2) * sin_mag_v) / mag_v) - px * (vx * cos_mag_v_m1
                + (vy * vz * cos_mag_v) / mag_v - (vx * pow(vz, 2) * sin_mag_v) / mag_v)
                - pz * ((pow(vx, 2) * vz * sin_mag_v) / mag_v + (pow(vy, 2) * vz * sin_mag_v) / mag_v);
        jacobian(2, 3) = 0;
        jacobian(2, 4) = 0;
        jacobian(2, 5) = 1;

    } else {
        ScalarType costh = cos(theta);
        ScalarType sinth = sin(theta);
        ScalarType costh_m1 = costh - 1;

        jacobian.resize(3, 7);
        jacobian(0, 0) = pz * (vy * costh + vx * vz * sinth) - py * (vz * costh - vx * vy * sinth)
                - px * (sinth * pow(vy, 2) + sinth * pow(vz, 2));
        jacobian(0, 1) = -costh_m1 * (py * vy + pz * vz);
        jacobian(0, 2) = pz * sinth + 2 * px * vy * costh_m1 - py * vx*costh_m1;
        jacobian(0, 3) = 2 * px * vz * costh_m1 - py * sinth - pz * vx*costh_m1;
        jacobian(0, 4) = 1;
        jacobian(0, 5) = 0;
        jacobian(0, 6) = 0;

        jacobian(1, 0) = px * (vz * costh + vx * vy * sinth) - pz * (vx * costh - vy * vz * sinth)
                - py * (sinth * pow(vx, 2) + sinth * pow(vz, 2));
        jacobian(1, 1) = 2 * py * vx * costh_m1 - px * vy * costh_m1 - pz*sinth;
        jacobian(1, 2) = -costh_m1 * (px * vx + pz * vz);
        jacobian(1, 3) = px * sinth + 2 * py * vz * costh_m1 - pz * vy*costh_m1;
        jacobian(1, 4) = 0;
        jacobian(1, 5) = 1;
        jacobian(1, 6) = 0;

        jacobian(2, 0) = py * (vx * costh + vy * vz * sinth) - px * (vy * costh - vx * vz * sinth)
                - pz * (sinth * pow(vx, 2) + sinth * pow(vy, 2));
        jacobian(2, 1) = py * sinth - px * vz * costh_m1 + 2 * pz * vx*costh_m1;
        jacobian(2, 2) = 2 * pz * vy * costh_m1 - py * vz * costh_m1 - px*sinth;
        jacobian(2, 3) = -costh_m1 * (px * vx + py * vy);
        jacobian(2, 4) = 0;
        jacobian(2, 5) = 0;
        jacobian(2, 6) = 1;

    }

    return jacobian;
}

struct LMFunctor {
    const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>* points;

    Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> transformed_points;

    AlgebraicSurfaceProduct<double>* surface;

    // Compute errors, one for each data point, for the given parameter values in 'x'

    int operator()(const Eigen::VectorXd& x, Eigen::VectorXd& fvec) {
        // 'x' has dimensions paraemters x 1
        // It contains the current estimates for the parameters.

        // 'fvec' has dimensions points x 1
        // It will contain the error for each data point and will be returned

        std::size_t N = num_data_points;

        Eigen::Quaternion<double> quat(x(0), x(1), x(2), x(3));
        quat.normalize();

        Eigen::Matrix3d R0 = quat.toRotationMatrix();
        Eigen::RowVector3d t0 = x.tail(3);

        transformed_points = (*points) * R0.transpose() + Eigen::VectorXd::Ones(N) * t0;

        //        Eigen::Matrix<double, Eigen::Dynamic, 1> fvec(N);
        for (std::size_t i = 0; i != N; i++) {
            fvec(i) = surface->evaluate(transformed_points.row(i));
        }

        return 0;
    }

    // Compute the jacobian of the errors

    int df(const Eigen::VectorXd& x, Eigen::MatrixXd& fjac) {
        // 'x' has dimensions parameters x 1
        // It contains the current estimates for the parameters.

        // 'fjac' has dimensions points x parameters and will be returned
        for (std::size_t i = 0; i != num_data_points; i++) {
            fjac.row(i) = surface->evaluateGradient(transformed_points.row(i)) *
                    jacobianTransformedPointQuat<double>(transformed_points(i, 0),
                    transformed_points(i, 1), transformed_points(i, 2),
                    x(4), x(5), x(6), x(0), x(1), x(2), x(3));
        }

        return 0;
    }

    // Number of data points, i.e. values.
    int num_data_points;

    int values() {
        return num_data_points;
    }

    // The number of parameters, i.e. inputs.
    int num_parameters;

    int inputs() {
        return num_parameters;
    }

};

#endif /* __cplusplus */

#endif /* SURFACE_ALIGNMENT_OPTIMIZER_HPP */

