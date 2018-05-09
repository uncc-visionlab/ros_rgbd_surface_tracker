/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Gaussian.hpp
 * Author: John Papadakis
 *
 * Created on May 8, 2018, 2:19 PM
 */

#ifndef GAUSSIAN_HPP
#define GAUSSIAN_HPP

#include <cmath>

#include <opencv2/core.hpp>

template <typename scalar_t, int size>
class Gaussian {
public:

    Gaussian() {
    }

    Gaussian(const cv::Vec<scalar_t, size>& mean, const cv::Matx<scalar_t, size, size>& covariance) {
        this->mean = mean;
        this->covariance = covariance;
        this->precomputeTerms();
    }

    scalar_t evaluate(const cv::Vec<scalar_t, size>& x) const {
        cv::Vec<scalar_t, size> x_minus_mu = x - mean;
        return normalization_factor*std::exp(-0.5*(x_minus_mu.t()*inv_covariance*x_minus_mu)[0]);
    }

    scalar_t evaluateLogLikelihood(const cv::Vec<scalar_t, size>& x) const {
        cv::Vec<scalar_t, size> x_minus_mu = x - mean;
        return -0.5*(nlog2pi + log_det_covariance + (x_minus_mu.t()*inv_covariance*x_minus_mu)[0]);
    }

    template <typename T, int n, int m>
    static Gaussian<T, n> transform(const Gaussian<T, m>& input, const cv::Matx<T, n, m>& transformation_matrix) {
        return Gaussian<T, n>(transformation_matrix*input.getMean(), transformation_matrix*input.getCovariance()*transformation_matrix.t());
    }

    template <int newsize>
    void transform(const cv::Matx<scalar_t, newsize, size>& transformation_matrix) {
        *this = Gaussian::transform(*this, transformation_matrix);
    }

    void setMean(const cv::Vec<scalar_t, size>& mean) {
        this->mean = mean;
    }

    const cv::Vec<scalar_t, size>& getMean() const {
        return mean;
    }

    void setCovariance(const cv::Matx<scalar_t, size, size>& covariance) {
        this->covariance = covariance;
        this->precomputeTerms();
    }

    const cv::Matx<scalar_t, size, size>& getCovariance() const {
        return covariance;
    }

    const cv::Matx<scalar_t, size, size>& getInverseCovariance() const {
        return inv_covariance;
    }
    
private:
    
    cv::Vec<scalar_t, size> mean;
    cv::Matx<scalar_t, size, size> covariance;
    cv::Matx<scalar_t, size, size> inv_covariance;
    scalar_t det_covariance;
    scalar_t log_det_covariance;
    scalar_t normalization_factor;
    static constexpr scalar_t pi = std::acos(-1);
    static constexpr scalar_t nlog2pi = size*std::log(2*pi);

    void precomputeTerms() {
        inv_covariance = covariance.inv();
        det_covariance = cv::determinant(covariance);
        normalization_factor = 1/std::sqrt(std::pow(2*pi, size)*det_covariance);
    }
        
};


#endif /* GAUSSIAN_HPP */

