#include <boost/make_shared.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/NonLinearOptimization>

#include <opencv2/imgproc.hpp>

#include <ros_rgbd_surface_tracker/surface_alignment_optimizer.hpp>
#include <ros_rgbd_surface_tracker/rgbd_image_uncc.hpp>
#include <ros_rgbd_surface_tracker/rgbd_tracker_uncc.hpp>
#include <ros_rgbd_surface_tracker/Polygonizer.hpp>

#include <chrono> // for timing & benchmarking
#include <fstream>
#include <iomanip>


#define OUTLIER_THRESHOLD 0.02

namespace cv {
    namespace rgbd {

        void planeAlignment(cv::rgbd::RgbdImage& rgbd_img, RgbdSurfaceTracker& surface_tracker) {

            int tile_width = 100, tile_height = 100;
            int x0 = floor(0.5 * (rgbd_img.getWidth() - tile_width));
            int y0 = floor(0.5 * (rgbd_img.getHeight() - tile_height));
            int num_samples = 100;

            cv::Point2i tl, tr, br, bl;

            tl.x = x0;
            tl.y = y0;

            tr.x = x0 + tile_width;
            tr.y = y0;

            br.x = x0 + tile_width;
            br.y = y0 + tile_height;

            bl.x = x0;
            bl.y = y0 + tile_height;

            std::vector<cv::Point2i> corners;
            corners.reserve(4);
            corners.push_back(bl);
            corners.push_back(br);
            corners.push_back(tr);
            corners.push_back(tl);

            std::vector<cv::Point3f> data;
            data.reserve(num_samples);

            bool got_all_samples = rgbd_img.getTileData_Uniform(
                    x0, y0, tile_width, tile_height, data, num_samples);

            static bool have_initial_guess;
            static AlgebraicSurfaceProduct<double> surface(3);
            Eigen::Matrix4d transform_matrix;
            transform_matrix.setIdentity();

            bool convergence = false;

            if (data.size() > num_samples / 2) {

                // convert points to matrix form
                Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> data_mat(data.size(), 3);
                for (std::size_t i = 0; i != data.size(); ++i) {
                    data_mat.row(i) << data[i].x, data[i].y, data[i].z;
                }

                if (have_initial_guess) {
                    // run nonlinear optimization to estimate frame-frame odometry
                    convergence = leastSquaresSurfaceFitLM<double>(surface, data_mat, transform_matrix);

                } else {
                    // fit a plane to use as an initial guess
                    cv::Plane3f plane = rgbd_img.fitPlaneImplicitLeastSquares(data);
                    AlgebraicSurface<double>::Ptr subsurface_ptr =
                            boost::make_shared<PlanarSurface<double>>(
                            Eigen::RowVector4d(plane.d, plane.x, plane.y, plane.z));
                    surface.addSubSurface(subsurface_ptr);
                    have_initial_guess = true;
                }

                if (convergence) { // transform the plane and publish
                    AlgebraicSurface<double>::Ptr subsurface = surface.subsurfaces[0];
                    subsurface->affineTransform(transform_matrix);
                    std::cout << "surface: " << subsurface->toString() << "\n";

                    PlaneVisualizationData* vis_data = surface_tracker.getPlaneVisualizationData();
                    vis_data->rect_points.clear();

                    for (std::size_t i = 0; i != 4; i++) {

                        if (!std::isnan(rgbd_img.getDepthImage().at<float>(corners[i].y, corners[i].x))) {

                            cv::Point3f point = rgbd_img.backproject(corners[i]);

                            point.z = -(subsurface->coeffs(0)
                                    + subsurface->coeffs(1) * point.x
                                    + subsurface->coeffs(2) * point.y) / subsurface->coeffs(3);


                            vis_data->rect_points.push_back(Eigen::Vector3f(point.x, point.y, point.z));

                        }
                    }
                }
            }

        }

        void edgeAlignment(cv::rgbd::RgbdImage& rgbd_img, RgbdSurfaceTracker& surface_tracker) {

            // collecting data from two tiles in depth image
            int tile_width = 100, tile_height = 100;
            int x1 = floor(0.5 * (rgbd_img.getWidth() - tile_width)) - 100;
            int x2 = x1 + 200;
            int y1 = floor(0.5 * (rgbd_img.getHeight() - tile_height));
            int y2 = y1;
            int samples_per_tile = 100;

            std::vector<cv::Point3f> data, tileA_data, tileB_data;
            data.reserve(2 * samples_per_tile);
            tileA_data.reserve(samples_per_tile);
            tileB_data.reserve(samples_per_tile);

            rgbd_img.getTileData_Uniform(
                    x1, y1, tile_width, tile_height, tileA_data, samples_per_tile);

            rgbd_img.getTileData_Uniform(
                    x2, y1, tile_width, tile_height, tileB_data, samples_per_tile);

            if (tileA_data.size() + tileB_data.size() > samples_per_tile) {

                static bool have_initial_guess;

                static AlgebraicSurfaceProduct<double> surface(3);
                Eigen::Matrix4d transform_matrix;
                transform_matrix.setIdentity();

                bool convergence = false;

                if (have_initial_guess) {

                    // move points to common std::vector (this avoids copy)
                    data = std::move(tileA_data);
                    std::move(std::begin(tileB_data), std::end(tileB_data), std::back_inserter(data));

                    // convert points to matrix form (copying unfortunatly)
                    Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> data_mat(data.size(), 3);
                    for (std::size_t i = 0; i != data.size(); ++i) {
                        data_mat.row(i) << data[i].x, data[i].y, data[i].z;
                    }

                    // run nonlinear optimization to estimate frame-frame odometry
                    convergence = leastSquaresSurfaceFitLM<double>(surface, data_mat, transform_matrix);

                } else {
                    // fit edge for initial guess
                    cv::Plane3f planeA = rgbd_img.fitPlaneImplicitLeastSquares(tileA_data);
                    cv::Plane3f planeB = rgbd_img.fitPlaneImplicitLeastSquares(tileB_data);
                    AlgebraicSurface<double>::Ptr subsurfaceA_ptr =
                            boost::make_shared<PlanarSurface<double>>(
                            Eigen::RowVector4d(planeA.d, planeA.x, planeA.y, planeA.z));
                    AlgebraicSurface<double>::Ptr subsurfaceB_ptr =
                            boost::make_shared<PlanarSurface<double>>(
                            Eigen::RowVector4d(planeB.d, planeB.x, planeB.y, planeB.z));
                    surface.addSubSurface(subsurfaceA_ptr);
                    surface.addSubSurface(subsurfaceB_ptr);
                    have_initial_guess = true;
                }

                if (convergence) { // transform the surface and publish

                    surface.affineTransform(transform_matrix);

                    PlaneVisualizationData* vis_data_ptr = surface_tracker.getPlaneVisualizationData();
                    vis_data_ptr->rect_points.clear();
                    vis_data_ptr->triangles.clear();

                    static Polygonizer<double> poly(
                            Polygonizer<double>::EvaluationVolume(0, 0, 2.5, 6, 4, 5, 10, 10, 10),
                            &surface,
                            vis_data_ptr);
                    poly.polygonize();

                }

            }

        }

        constexpr double pi() {
            return std::atan(1)*4;
        }

        void cornerAlignment(cv::rgbd::RgbdImage& rgbd_img, RgbdSurfaceTracker& surface_tracker,
                cv::Mat& rgb_result) {

            // collecting data from three tiles in depth image
            int tile_width = 30, tile_height = 30;
            int samples_per_tile = 100;

            float radius = 150; //pixels
            std::array<float, 3> angles = {pi() / 2 + pi() / 3, 7 * pi() / 6 + pi() / 3, 11 * pi() / 6 + pi() / 3};
            std::vector<cv::Rect2i> rects;
            rects.reserve(3);

            for (auto& angle : angles) {
                cv::Rect2i rectVal(radius * std::cos(angle) + (rgbd_img.getWidth() - tile_width) / 2.0,
                        radius * std::sin(angle) + (rgbd_img.getHeight() - tile_height) / 2.0,
                        tile_width, tile_height);
                rects.emplace_back(rectVal
                        );
                cv::rectangle(rgb_result, rectVal, cv::Scalar(0, 255, 0), 3);
            }

            std::array<std::vector<cv::Point3f>, 3> tile_data; // data for each tile in array of vectors
            int amount_data = 0;

            for (std::size_t rectid = 0; rectid != rects.size(); ++rectid) {

                tile_data[rectid].reserve(samples_per_tile);

                rgbd_img.getTileData_Uniform(rects[rectid].x, rects[rectid].y,
                        rects[rectid].width, rects[rectid].height,
                        tile_data[rectid], samples_per_tile);

                amount_data += tile_data[rectid].size();

            }

            std::vector<cv::Point3f> data; // this will hold the points from all of tile_data
            data.reserve(amount_data);

            if (amount_data > 2 * samples_per_tile) {

                static bool have_initial_guess;

                static CornerSurfaceProduct<double> surface;
                Eigen::Matrix4d transform_matrix;
                transform_matrix.setIdentity();

                bool convergence = false;

                if (have_initial_guess) {

                    // move points to common std::vector (this avoids copy)
                    data = std::move(tile_data[0]);
                    for (std::size_t rectid = 1; rectid != tile_data.size(); ++rectid)
                        std::move(std::begin(tile_data[rectid]),
                            std::end(tile_data[rectid]),
                            std::back_inserter(data));

                    // convert points to matrix form (copying unfortunatly)
                    Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor> data_mat(data.size(), 3);
                    for (std::size_t i = 0; i != data.size(); ++i) {
                        data_mat.row(i) << data[i].x, data[i].y, data[i].z;
                    }

                    // run nonlinear optimization to estimate frame-frame odometry
                    convergence = leastSquaresSurfaceFitLM<double>(surface, data_mat, transform_matrix);

                } else {
                    // fit surface for initial guess

                    for (std::size_t rectid = 0; rectid != rects.size(); ++rectid) {
                        cv::Plane3f plane = rgbd_img.fitPlaneImplicitLeastSquares(tile_data[rectid]);

                        AlgebraicSurface<double>::Ptr subsurface_ptr =
                                boost::make_shared<PlanarSurface<double>>(
                                Eigen::RowVector4d(plane.d, plane.x, plane.y, plane.z));

                        surface.addSubSurface(subsurface_ptr);
                    }

                    Eigen::RowVector3d corner_pt = surface.getPoint(*surface.subsurfaces.at(0), *surface.subsurfaces.at(1), *surface.subsurfaces[2]);
                    std::cout << "Corner pt: " << corner_pt << std::endl;
                    have_initial_guess = true;
                }

                if (convergence) { // transform the surface and publish

                    surface.affineTransform(transform_matrix);

                    PlaneVisualizationData* vis_data_ptr = surface_tracker.getPlaneVisualizationData();
                    vis_data_ptr->rect_points.clear();
                    vis_data_ptr->triangles.clear();

                    static Polygonizer<double> poly(
                            Polygonizer<double>::EvaluationVolume(0, 0, 3, 1, 1, 6, 25, 25, 25),
                            &surface,
                            vis_data_ptr);
                    poly.polygonize();

                }

            }

        }

        int planeListAlignmentCV(
                std::vector<cv::Plane3f::Ptr>& moving_planes,
                std::vector<cv::Plane3f::Ptr>& fixed_planes,
                Eigen::Matrix4f& transformation_matrix) {

            int result = 0;

            if (fixed_planes.size() == 0 || moving_planes.size() == 0 ||
                    fixed_planes.size() != moving_planes.size()) {
                result = -1;
                return result;
            }

            Eigen::Matrix3d normalsCovMat = Eigen::Matrix3d::Zero();
            Eigen::Vector3d deltaDt_NA = Eigen::Vector3d::Zero();
            Eigen::Vector3d deltaDt_NB = Eigen::Vector3d::Zero();

            for (int indexA = 0; indexA < fixed_planes.size(); indexA++) {
                cv::Plane3f::Ptr& plane_tgt = fixed_planes.at(indexA);
                cv::Plane3f::Ptr& plane_src = moving_planes.at(indexA);

                if (plane_tgt && plane_src) {
                    float norm =
                            (plane_tgt->x - plane_src->x)*(plane_tgt->x - plane_src->x) +
                            (plane_tgt->y - plane_src->y)*(plane_tgt->y - plane_src->y) +
                            (plane_tgt->z - plane_src->z)*(plane_tgt->z - plane_src->z) +
                            (plane_tgt->d - plane_src->d)*(plane_tgt->d - plane_src->d);

                    //if (norm < OUTLIER_THRESHOLD || true) {
                    //std::cout << *plane_src << ", " << *plane_tgt << std::endl;

                    normalsCovMat(0, 0) += plane_tgt->x * plane_src->x;
                    normalsCovMat(0, 1) += plane_tgt->x * plane_src->y;
                    normalsCovMat(0, 2) += plane_tgt->x * plane_src->z;
                    normalsCovMat(1, 0) += plane_tgt->y * plane_src->x;
                    normalsCovMat(1, 1) += plane_tgt->y * plane_src->y;
                    normalsCovMat(1, 2) += plane_tgt->y * plane_src->z;
                    normalsCovMat(2, 0) += plane_tgt->z * plane_src->x;
                    normalsCovMat(2, 1) += plane_tgt->z * plane_src->y;
                    normalsCovMat(2, 2) += plane_tgt->z * plane_src->z;

                    //}

                } else {
                    //std::cout << "NULL" << std::endl;
                }

            }

            // Compute the Singular Value Decomposition
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(normalsCovMat, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix3d u = svd.matrixU();
            Eigen::Matrix3d v = svd.matrixV();
            Eigen::Matrix3d R = v * u.transpose();
            //std::cout << "singular values = " << svd.singularValues() << std::endl;
            //std::cout << "det(R) = " << R.determinant() << std::endl;

            float detR = R.determinant();

            if (detR < 0) {
                //                std::cout << "|R| = " << detR << ", fixing...\n";
                //                Eigen::Matrix3d eye_with_neg = Eigen::Matrix3d::Identity();
                //                eye_with_neg(2, 2) = -1;
                //                R = v*eye_with_neg*u.transpose();
                result = 1; //R contains reflection!
            }

            normalsCovMat.setZero();

            for (int indexA = 0; indexA < fixed_planes.size(); indexA++) {

                cv::Plane3f::Ptr& plane_tgt = fixed_planes.at(indexA);
                cv::Plane3f::Ptr plane_src = moving_planes.at(indexA);

                if (plane_src && plane_tgt) {

                    // Rotating plane_src!
                    Eigen::Vector4d plane_src_coeffs(plane_src->x, plane_src->y, plane_src->z, plane_src->d);
                    plane_src_coeffs.head(3) = R.transpose() * plane_src_coeffs.head(3);
                    plane_src = boost::make_shared<cv::Plane3f>(plane_src_coeffs(0), plane_src_coeffs(1), plane_src_coeffs(2), plane_src_coeffs(3));
                    //plane_src->convertHessianNormalForm();

                    float norm = (plane_tgt->x - plane_src->x)*(plane_tgt->x - plane_src->x) +
                            (plane_tgt->y - plane_src->y)*(plane_tgt->y - plane_src->y) +
                            (plane_tgt->z - plane_src->z)*(plane_tgt->z - plane_src->z) +
                            (plane_tgt->d - plane_src->d)*(plane_tgt->d - plane_src->d);

                    //if (norm < OUTLIER_THRESHOLD || true) {

                    normalsCovMat(0, 0) += plane_tgt->x * plane_tgt->x + plane_src->x * plane_src->x;
                    normalsCovMat(1, 1) += plane_tgt->y * plane_tgt->y + plane_src->y * plane_src->y;
                    normalsCovMat(2, 2) += plane_tgt->z * plane_tgt->z + plane_src->z * plane_src->z;

                    normalsCovMat(0, 1) += plane_tgt->x * plane_tgt->y + plane_src->x * plane_src->y;
                    normalsCovMat(0, 2) += plane_tgt->x * plane_tgt->z + plane_src->x * plane_src->z;
                    normalsCovMat(1, 2) += plane_tgt->y * plane_tgt->z + plane_src->y * plane_src->z;

                    double d_diff = plane_src->d - plane_tgt->d;

                    deltaDt_NA(0) += d_diff * plane_src->x;
                    deltaDt_NA(1) += d_diff * plane_src->y;
                    deltaDt_NA(2) += d_diff * plane_src->z;
                    deltaDt_NB(0) += d_diff * plane_tgt->x;
                    deltaDt_NB(1) += d_diff * plane_tgt->y;
                    deltaDt_NB(2) += d_diff * plane_tgt->z;

                    //}
                }
            }

            normalsCovMat(1, 0) = normalsCovMat(0, 1);
            normalsCovMat(2, 0) = normalsCovMat(0, 2);
            normalsCovMat(2, 1) = normalsCovMat(1, 2);

            Eigen::Matrix<double, 3, 1> t = normalsCovMat.inverse()*(deltaDt_NA + deltaDt_NB);

            if (t.hasNaN()) {
                result = -1;
                return result;
            }
            // Return the correct transformation
            //Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Zero();
            transformation_matrix.topLeftCorner(3, 3) = R.cast<float>();
            transformation_matrix.block<3, 1>(0, 3) = t.cast<float>();
            transformation_matrix(3, 3) = 1.0f;

            return result;
            //                    std::cout << "Estimated Transform:\n" << transformation_matrix << "\n";
            //                    
            //                    std::cout << "Actual Transform:\n" << actual_trans.matrix() << "\n";
            //
            //                    Eigen::Quaterniond quat(transformation_matrix.block<3, 3>(0, 0));
            //                    Eigen::Vector3d translation(transformation_matrix.block<3, 1>(0, 3));
            //                    tf::Quaternion tf_quat(quat.x(), quat.y(), quat.z(), quat.w());
            //                    tf::Transform xform(tf_quat,
            //                            tf::Vector3(translation[0], translation[1], translation[2]));
        }

        float minimizePointToPointError(std::vector<cv::Point3f>& fixed_points,
                std::vector<cv::Point3f>& moving_points,
                Pose& delta_pose) {
            int result = 0;

            if (fixed_points.size() == 0 || moving_points.size() == 0 ||
                    fixed_points.size() != moving_points.size()) {
                result = -1;
                return result;
            }

            float errorIn = 0;
            cv::Vec3f errorVecIn;
            for (int pairIdx = 0; pairIdx < moving_points.size(); ++pairIdx) {
                errorVecIn = (cv::Vec3f)fixed_points[pairIdx] - (cv::Vec3f) moving_points[pairIdx];
                errorIn += std::sqrt(errorVecIn.dot(errorVecIn));
            }
            std::cout << "errorIn = " << errorIn << std::endl;

            cv::Vec3f fixed_mean(0, 0, 0), moving_mean(0, 0, 0);
            for (int pairIdx = 0; pairIdx < moving_points.size(); ++pairIdx) {
                fixed_mean += (cv::Vec3f) fixed_points[pairIdx];
                moving_mean += (cv::Vec3f) moving_points[pairIdx];
            }
            fixed_mean *= 1.0f / fixed_points.size();
            moving_mean *= 1.0f / moving_points.size();
            //vc::Vec3f zero_ctr_movingpt, zero_ctr_fixedpt;
            cv::Mat covMat = cv::Mat::zeros(3, 3, CV_32F);
            for (int pairIdx = 0; pairIdx < moving_points.size(); ++pairIdx) {
                float *covMat_ptr = covMat.ptr<float>(0, 0);
                //zero_ctr_movingpt = (cv::Vec3f) moving_points[pairIdx] - moving_mean;
                //zero_ctr_fixedpt = (cv::Vec3f) fixed_points[pairIdx] - moving_mean;
                //for (int mpt_dim_idx = 0; mpt_dim_idx < 3; ++mpt_dim_idx) {
                //    for (int fpt_dim_idx = 0; fpt_dim_idx < 3; ++fpt_dim_idx) {
                //        *covMat_ptr++ += (zero_ctr_movingpt[mpt_dim_idx])*(zero_ctr_fixedpt[fpt_dim_idx]);
                //    }
                //}
                *covMat_ptr++ += (moving_points[pairIdx].x - moving_mean[0])*(fixed_points[pairIdx].x - fixed_mean[0]);
                *covMat_ptr++ += (moving_points[pairIdx].x - moving_mean[0])*(fixed_points[pairIdx].y - fixed_mean[1]);
                *covMat_ptr++ += (moving_points[pairIdx].x - moving_mean[0])*(fixed_points[pairIdx].z - fixed_mean[2]);
                *covMat_ptr++ += (moving_points[pairIdx].y - moving_mean[1])*(fixed_points[pairIdx].x - fixed_mean[0]);
                *covMat_ptr++ += (moving_points[pairIdx].y - moving_mean[1])*(fixed_points[pairIdx].y - fixed_mean[1]);
                *covMat_ptr++ += (moving_points[pairIdx].y - moving_mean[1])*(fixed_points[pairIdx].z - fixed_mean[2]);
                *covMat_ptr++ += (moving_points[pairIdx].z - moving_mean[2])*(fixed_points[pairIdx].x - fixed_mean[0]);
                *covMat_ptr++ += (moving_points[pairIdx].z - moving_mean[2])*(fixed_points[pairIdx].y - fixed_mean[1]);
                *covMat_ptr += (moving_points[pairIdx].z - moving_mean[2])*(fixed_points[pairIdx].z - fixed_mean[2]);
            }
            //std::cout << "covMat = " << covMat << std::endl;
            cv::Mat w, u, vt;
            cv::SVDecomp(covMat, w, u, vt, cv::SVD::FULL_UV);
            cv::Mat R = u * vt;
            //std::cout << "u = " << u << std::endl;
            //std::cout << "vt = " << vt << std::endl;
            //std::cout << "R = " << R << std::endl;
            if (cv::determinant(R) < 0) {
                cv::Mat fixReflection = (Mat_<float>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, -1);
                R = u * fixReflection * vt;
                result = 1;
            }
            cv::Matx33f iR33f = Pose::cvtMat_to_Matx33(R.t());
            //std::cout << "iR33f = " << iR33f << std::endl;
            //std::cout << "fixed_mean = " << fixed_mean << std::endl;
            //std::cout << "moving_mean = " << moving_mean << std::endl;
            cv::Vec3f t = fixed_mean - iR33f * moving_mean;
            //std::cout << "t = " << t << std::endl;

            float error = 0;
            cv::Vec3f errorVec;
            for (int pairIdx = 0; pairIdx < moving_points.size(); ++pairIdx) {
                errorVec = (cv::Vec3f)fixed_points[pairIdx] - ((cv::Vec3f)(iR33f * moving_points[pairIdx]) + t);
                error += std::sqrt(errorVec.dot(errorVec));
            }
            std::cout << "errorOut = " << error << std::endl;
            delta_pose.set(t, R);
            return error;
        }

        void planeListAlignmentEigen(
                std::vector<cv::Plane3f::Ptr>& moving_planes,
                std::vector<cv::Plane3f::Ptr>& fixed_planes,
                Eigen::Matrix4f& transformation_matrix) {


            if (fixed_planes.size() > 0 &&
                    fixed_planes.size() == moving_planes.size()) {


                std::size_t N = fixed_planes.size();

                Eigen::Matrix<double, Eigen::Dynamic, 3> fixed_norms(N, 3);
                Eigen::Matrix<double, Eigen::Dynamic, 1> fixed_d(N, 1);
                Eigen::Matrix<double, Eigen::Dynamic, 3> moving_norms(N, 3);
                Eigen::Matrix<double, Eigen::Dynamic, 1> moving_d(N, 1);
                Eigen::Matrix<double, Eigen::Dynamic, 1> diff_d(N, 1);

                for (std::size_t planeid = 0; planeid < N; ++planeid) {
                    auto& plane_tgt = fixed_planes.at(planeid);
                    auto& plane_src = moving_planes.at(planeid);

                    if (plane_tgt && plane_src) {

                        fixed_norms.row(planeid) << plane_tgt->x, plane_tgt->y, plane_tgt->z;
                        moving_norms.row(planeid) << plane_src->x, plane_src->y, plane_src->z;
                        diff_d(planeid) = plane_src->d - plane_tgt->d;

                    } else {

                        fixed_norms.row(planeid).setZero();
                        moving_norms.row(planeid).setZero();
                        diff_d(planeid) = 0;

                    }


                }

                Eigen::Matrix3d normals_covariance = fixed_norms.transpose() * moving_norms;

                // Compute the Singular Value Decomposition
                Eigen::JacobiSVD<Eigen::Matrix3d> svd(normals_covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
                Eigen::Matrix3d u = svd.matrixU();
                Eigen::Matrix3d v = svd.matrixV();
                Eigen::Matrix3d R = v * u.transpose();

                Eigen::Matrix3Xd sum_norms = (moving_norms * R + fixed_norms).transpose();

                Eigen::Matrix3d rot_normals_covariance = sum_norms*fixed_norms;

                Eigen::Vector3d t = rot_normals_covariance.inverse()*(sum_norms * diff_d);


                if (!t.hasNaN()) {

                    // Return the correct transformation
                    //Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Zero();
                    transformation_matrix.topLeftCorner(3, 3) = R.cast<float>();
                    transformation_matrix.block<3, 1>(0, 3) = t.cast<float>();
                    transformation_matrix(3, 3) = 1.0f;
                    //                    std::cout << "Estimated Transform:\n" << transformation_matrix << "\n";
                    //                    
                    //                    std::cout << "Actual Transform:\n" << actual_trans.matrix() << "\n";
                    //
                    //                    Eigen::Quaterniond quat(transformation_matrix.block<3, 3>(0, 0));
                    //                    Eigen::Vector3d translation(transformation_matrix.block<3, 1>(0, 3));
                    //                    tf::Quaternion tf_quat(quat.x(), quat.y(), quat.z(), quat.w());
                    //                    tf::Transform xform(tf_quat,
                    //                            tf::Vector3(translation[0], translation[1], translation[2]));

                }

            }

        }

        void benchmarkPointAlignment() {
            std::vector<cv::Point3f> fixedPts = {cv::Point3f(-0.4274, 0.7586, -0.2772),
                cv::Point3f(-1.0350, 0.6493, 0.0538),
                cv::Point3f(1.7751, -1.4690, 1.3040),
                cv::Point3f(-2.3125, -2.3650, -0.1002)};
            std::vector<cv::Point3f> movingPts = {cv::Point3f(-0.9662, -1.0617, 0.0316),
                cv::Point3f(-1.2795, -1.6596, -0.1554),
                cv::Point3f(-1.4948, 1.0347, -2.7323),
                cv::Point3f(-4.4956, -1.5442, -0.7760)};
            Pose delta_pose;
            minimizePointToPointError(fixedPts, movingPts, delta_pose);
            std::cout << "delta_pose = " << delta_pose.toString() << std::endl;
        }

        void benchmarkPlaneListAlignment(cv::rgbd::RgbdImage& rgbd_img, RgbdSurfaceTracker& surface_tracker,
                cv::Mat& rgb_result) {

            Eigen::Matrix<float, 3, 4> plane_coeffs;
            plane_coeffs.row(0) << 0.4784, -0.5571, -0.6788, 30.8483;
            plane_coeffs.row(1) << 0.6469, 0.4714, -0.5994, 2.8847;
            plane_coeffs.row(2) << 0.0193, 0.6558, -0.7546, 39.4952;

            Eigen::Affine3f actual_trans;
            actual_trans.matrix().block<3, 3>(0, 0) = (Eigen::Matrix3f() << 0.4930, 0.4808, -0.7251, 0.6240, 0.3855, 0.6798, 0.6064, -0.7876, -0.1100).finished().transpose();
            actual_trans.translation() = Eigen::Vector3f(8.9612, 82.5965, 38.9587);

            Eigen::Matrix3f R_actual = (Eigen::Matrix3f() << 0.4930, 0.4808, -0.7251, 0.6240, 0.3855, 0.6798, 0.6064, -0.7876, -0.1100).finished().transpose();
            Eigen::Vector3f t_actual = Eigen::Vector3f(8.9612, 82.5965, 38.9587);

            std::vector<cv::Plane3f::Ptr> moving_planes;
            std::vector<cv::Plane3f::Ptr> fixed_planes;

            for (std::size_t i = 0; i < plane_coeffs.rows(); ++i) {

                Eigen::Vector4f this_fixed = plane_coeffs.row(i);
                Eigen::Vector4f this_moving;
                this_moving << R_actual * this_fixed.head(3), t_actual.dot(this_fixed.head(3)) + this_fixed(3);
                this_moving = this_moving / this_moving.head(3).norm();

                fixed_planes.emplace_back(boost::make_shared<cv::Plane3f>(this_fixed(0), this_fixed(1), this_fixed(2), this_fixed(3)));
                moving_planes.emplace_back(boost::make_shared<cv::Plane3f>(this_moving(0), this_moving(1), this_moving(2), this_moving(3)));

            }

            auto begin1 = std::chrono::high_resolution_clock::now();
            Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Zero();
            for (std::size_t i = 0; i < 10000; i++)
                planeListAlignmentCV(moving_planes, fixed_planes, transformation_matrix);

            auto end1 = std::chrono::high_resolution_clock::now();
            std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(end1 - begin1).count() << "ns" << std::endl;

            auto begin2 = std::chrono::high_resolution_clock::now();

            for (std::size_t i = 0; i < 10000; i++)
                planeListAlignmentEigen(moving_planes, fixed_planes, transformation_matrix);

            auto end2 = std::chrono::high_resolution_clock::now();
            std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(end2 - begin2).count() << "ns" << std::endl;

        }

        void analyzePlaneFits(cv::rgbd::RgbdImage& rgbd_img, RgbdSurfaceTracker& surface_tracker) {

            int tile_width = 100, tile_height = 100;
            int x1 = floor(0.5 * (rgbd_img.getWidth() - tile_width)) - 100;
            int y1 = floor(0.5 * (rgbd_img.getHeight() - tile_height));
            int samples_per_tile = 100;

            std::vector<cv::Point3f> tile_data;
            tile_data.reserve(samples_per_tile);


            rgbd_img.getTileData_Uniform(
                    x1, y1, tile_width, tile_height, tile_data, samples_per_tile);

            cv::Plane3f plane = rgbd_img.fitPlaneImplicitLeastSquares(tile_data);

            //            std::fstream plane_data_file("plane_data.csv", std::fstream::in | std::fstream::out | std::fstream::app);
            //            std::cout << plane.toString();
            //            
            //            if (plane_data_file.is_open()) {
            //
            //                plane_data_file << std::fixed << std::setprecision(8) << plane.x << "," << plane.y << "," << plane.z << "," << plane.d << "\n";
            //            
            //                plane_data_file.close();
            //            }

        }

        void RgbdSurfaceTracker::iterativeAlignment(cv::rgbd::RgbdImage& rgbd_img, cv::Mat& rgb_result) {

            //            cornerAlignment(rgbd_img, *this, rgb_result);

        }

    } /* namespace rgbd */
} /* namespace cv */

