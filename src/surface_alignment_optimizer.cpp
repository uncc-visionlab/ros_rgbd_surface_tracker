/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <boost/make_shared.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/NonLinearOptimization>

#include <ros_rgbd_surface_tracker/surface_alignment_optimizer.hpp>
#include <ros_rgbd_surface_tracker/rgbd_image_uncc.hpp>
#include <ros_rgbd_surface_tracker/rgbd_tracker_uncc.hpp>

#define PROFILE_CALLGRIND false

#ifdef PROFILE_CALLGRIND
#include <valgrind/callgrind.h>
#endif

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

namespace cv {
    namespace rgbd {

        void RgbdSurfaceTracker::iterativeAlignment(cv::rgbd::RgbdImage& rgbd_img) {
#ifdef PROFILE_CALLGRIND
            CALLGRIND_TOGGLE_COLLECT;
#endif
            int tile_width = 100, tile_height = 100;
            int x0 = floor(0.5 * (rgbd_img.getWidth() - tile_width));
            int y0 = floor(0.5 * (rgbd_img.getHeight() - tile_height));
            int num_samples = 100;

            std::vector<cv::Point3f> data;
            data.reserve(num_samples);

            bool got_all_samples = rgbd_img.getTileData_Uniform(
                    x0, y0, tile_width, tile_height, data, num_samples);

            //for (int ii = 0; ii < data.size(); ii++) {
            //    std::cout << "datapt[ " << ii << "] = " << data[ii] << std::endl;
            //}

            static bool have_initial_guess;
            static cv::Plane3f plane;
            cv::RectWithError rect;

            AlgebraicSurfaceProduct<double> surface(3);
            Eigen::Matrix4d transform_matrix;
            transform_matrix.setIdentity();

            bool convergence = true;

            if (data.size() < num_samples / 2) {
                return;
            }

            // convert points to matrix form
            Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> data_mat(data.size(), 3);
            for (std::size_t i = 0; i != data.size(); ++i) {
                data_mat.row(i) << data[i].x, data[i].y, data[i].z;
            }

            if (have_initial_guess) {
                // run nonlinear optimization to estimate frame-frame odometry
                boost::shared_ptr<AlgebraicSurface<double>> subsurface_ptr =
                        boost::make_shared<PlanarSurface<double>>(
                        Eigen::RowVector4d(plane.d, plane.x, plane.y, plane.z));
                surface.addSubSurface(subsurface_ptr);
                convergence = EigenLeastSquaresSurfaceFitLM(surface, data_mat, transform_matrix);

            } else {
                // fit a plane to use as an initial guess
                rgbd_img.fitImplicitPlaneLeastSquares(data, plane, rect.error,
                        rect.noise, rect.inliers, rect.outliers, rect.invalid);
                have_initial_guess = true;
            }

            if (convergence) { // transform the plane and publish
                cv::Point2i middle_pixel;
                middle_pixel.x = floor(0.5 * rgbd_img.getWidth());
                middle_pixel.y = floor(0.5 * rgbd_img.getHeight());

                if (!std::isnan(rgbd_img.getDepth().at<float>(middle_pixel.y, middle_pixel.x))) {
                    // rgbd_targeting::plane planedata;
                    cv::Point3f middle = rgbd_img.backproject(middle_pixel);
                    middle.z = -(plane.d + plane.x * middle.x + plane.y * middle.y) / plane.z;
                    PlaneVisualizationData *vizData = this->getPlaneVisualizationData();
                    vizData->plane_point << middle.x, middle.y, middle.z, 1.0;
                    vizData->plane_norm << plane.x, plane.y, plane.z, 0.0;
                    vizData->transformed_point = transform_matrix.cast<float>() * vizData->plane_point;
                    vizData->transformed_norm = transform_matrix.cast<float>() * vizData->plane_norm;
                    plane.x = vizData->transformed_norm(0);
                    plane.y = vizData->transformed_norm(1);
                    plane.z = vizData->transformed_norm(2);
                    plane.d = -(plane.x * vizData->transformed_point(0)
                            + plane.y * vizData->transformed_point(1)
                            + plane.z * vizData->transformed_point(2));
                    plane.convertHessianNormalForm();
                }
            }
            std::cout << "plane coeffs: " << plane.toString() << "\n";
#ifdef PROFILE_CALLGRIND
            CALLGRIND_TOGGLE_COLLECT;
#endif
        }
    } /* namespace rgbd */
} /* namespace cv */

