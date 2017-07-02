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

// Eigen includes
#include <Eigen/Dense>
#include <Eigen/Geometry>

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

bool leastSquaresSurfaceFitLM(AlgebraicSurfaceProduct<double>& surface,
        const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>& points,
        Eigen::Matrix4d& transform) {

    std::cout << "-----------------------------------------------------------\n"
            "Nonlinear Surface Fit:\n";

    bool convergence = false;
    transform.setIdentity();
    std::size_t N = points.rows();

    Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> transformed_points = points;

    //    std::cout << "Points:\n" << points << "\n";

    double error_a_k = 0;
    Eigen::Matrix<double, Eigen::Dynamic, 1> residuals_a_k(N);
    for (std::size_t i = 0; i != N; i++) {
        double res = pow(surface.evaluate(points.row(i)), 2);
        residuals_a_k(i) = res;
        error_a_k += res;
    }
    double initial_error = error_a_k;

    if (initial_error < .01) {
        std::cout << "Initial error below threshold, stopping...\n";
        return true;
    }

    //    std::cout << "Residuals:\n" << residuals_a_k << "\n";
    //    std::cout << "error_a_k:\n" << error_a_k << "\n";

    //    Eigen::Matrix<double, Eigen::Dynamic, 3> grad_vecs(N, 3);
    Eigen::Matrix<double, 3, 7> jac_pt;
    Eigen::Matrix<double, Eigen::Dynamic, 7> J(N, 7);

    Eigen::Matrix<double, 1, 7> a_k;
    a_k << 1, 0, 0, 0, 0, 0, 0;
    Eigen::Matrix<double, 1, 7> a_k_p1;
    Eigen::Matrix<double, Eigen::Dynamic, 1> ONE_VEC =
            Eigen::Matrix<double, Eigen::Dynamic, 1>::Ones(N, 1);
    Eigen::Matrix<double, 7, 7> EYE7 = Eigen::Matrix<double, 7, 7>::Identity();

    Eigen::Matrix<double, 3, 3> R0;
    Eigen::Matrix<double, 1, 3> t0;
    Eigen::Matrix<double, Eigen::Dynamic, 1> residuals_a_k_p1(N);

    //    std::cout << "a_k:\n" << a_k << "\n";

    double rate = 500;
    double lambda_k = rate;
    double delta_error = 0;
    std::size_t iterations = 0;
    bool min_found = false;

    while (!min_found) {

        //        for (std::size_t i = 0; i != N; i++) {
        //            grad_vecs.row(i) = surface.evaluateSquaredGradient(transformed_points.row(i));
        //        }

        //        std::cout << "grad_vecs:\n" << grad_vecs << "\n";

        for (std::size_t i = 0; i != N; i++) {
            jac_pt = jacobianTransformedPointQuat<double>(transformed_points(i, 0),
                    transformed_points(i, 1), transformed_points(i, 2),
                    a_k(4), a_k(5), a_k(6), a_k(0), a_k(1), a_k(2), a_k(3));
            //            J.row(i) = grad_vecs.row(i)*jac_pt;
            J.row(i) = surface.evaluateSquaredGradient(transformed_points.row(i)) * jac_pt;

        }

        //        std::cout << "J:\n" << J << "\n";

        double lambda_k_p1 = lambda_k;
        while (lambda_k_p1 <= lambda_k) {
            lambda_k = lambda_k_p1;

            //            std::cout << "lambda_k: " << lambda_k << "\n";
            //            std::cout << "J.transpose()*J + lambda_k*EYE7:\n" << J.transpose()*J + lambda_k*EYE7 << "\n";
            //            std::cout << "above.inverse():\n" << (J.transpose()*J + lambda_k*EYE7).inverse() << "\n";
            //            std::cout << "J.transpose()*residuals_a_k:\n" << J.transpose()*residuals_a_k << "\n";

            // check other methods for Ax = b than inverse(A)*b
            //            a_k_p1 = a_k - ((J.transpose()*J + lambda_k*EYE7).inverse()*J.transpose()*residuals_a_k).transpose();

            Eigen::HouseholderQR<Eigen::MatrixXd> qr(J.transpose() * J + lambda_k * EYE7);
            a_k_p1 = a_k - qr.solve(J.transpose() * residuals_a_k).transpose();


            //            std::cout << "a_k_p1:\n" << a_k_p1 << "\n";

            if (a_k_p1.hasNaN()) {
                std::cout << "NaN in a_k_p1! -----------------------------------\n";
                min_found = true;
                break;
            }

            Eigen::Quaternion<double> quat(a_k_p1(0), a_k_p1(1), a_k_p1(2), a_k_p1(3));
            quat.normalize();

            R0 = quat.toRotationMatrix();
            t0 = a_k_p1.tail(3);

            transformed_points = points * R0.transpose() + ONE_VEC*t0;

            //            std::cout << "R0:\n" << R0 << "\n";
            //            std::cout << "t0:\n" << t0 << "\n";
            //            std::cout << "New points:\n" << transformed_points << "\n";

            double error_a_k_p1 = 0;
            for (std::size_t i = 0; i != N; i++) {

                residuals_a_k_p1(i) = pow(surface.evaluate(transformed_points.row(i)), 2);
                error_a_k_p1 += residuals_a_k_p1(i);
            }

            //            std::cout << "error_a_k_p1:\n" << error_a_k_p1 << "\n";
            //            std::cout << "residuals_a_k_p1:\n" << residuals_a_k_p1 << "\n";

            a_k = a_k_p1;
            iterations++;
            delta_error = error_a_k_p1 - error_a_k;

            if (delta_error > 0)
                lambda_k_p1 = rate * lambda_k;
            else {
                lambda_k_p1 = (1 / rate) * lambda_k;
                if (delta_error > -1e-03) {
                    std::cout << "Stopping criteria satisfied.\n";
                    min_found = true;
                    break;
                }
            }

            residuals_a_k = residuals_a_k_p1;
            error_a_k = error_a_k_p1;

            //            std::cout << "iteration: " << iterations << "\n";
            if (iterations > 50) {
                std::cout << "Iterations exceeded... stopping\n";
                min_found = true;
                break;
            }

        }

    }

    transform.block<3, 3>(0, 0) = R0.transpose();
    transform.block<3, 1>(0, 3) = -R0.transpose() * t0.transpose();

    if (!std::isnan(error_a_k) && (error_a_k < initial_error))
        convergence = true;

    std::cout << "Iterations: " << iterations << "\n";
    std::cout << "Initial Error: " << initial_error << "\n";
    std::cout << "Final Error: " << error_a_k << "\n";
    std::cout << "Transform:\n" << transform << "\n";

    return convergence;

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

bool EigenLeastSquaresSurfaceFitLM(AlgebraicSurfaceProduct<double>& surface,
        const Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>& points,
        Eigen::Matrix4d& transform) {

    LMFunctor functor;
    functor.surface = &surface;
    functor.points = &points;
    functor.num_data_points = points.rows();
    functor.num_parameters = 7;
    Eigen::VectorXd x(7);
    x << 1, 0, 0, 0, 0, 0, 0;

    Eigen::LevenbergMarquardt<LMFunctor, double> lm(functor);
    lm.parameters.maxfev = 50; // maximum # function evaluations

    int status = lm.minimize(x);

    Eigen::Quaternion<double> quat(x(0), x(1), x(2), x(3));
    quat.normalize();
    Eigen::Matrix3d R0 = quat.toRotationMatrix();

    transform.setIdentity();
    transform.block<3, 3>(0, 0) = R0.transpose();
    transform.block<3, 1>(0, 3) = -R0.transpose() * x.tail(3);

    std::cout << "-----------------------------------------------------------\n"
            "Nonlinear Surface Fit:\n";
    std::cout << "LM optimization status: " << status << "\n";
    std::cout << "Iterations: " << lm.iter << "\n";
    std::cout << "Transform:\n" << transform << "\n";

    return ((status > 0) ? true : false);
}

#endif /* __cplusplus */

#endif /* SURFACE_ALIGNMENT_OPTIMIZER_HPP */

