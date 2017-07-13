/**
 *
 * A class for describing and operating upon algebraic implicit polynomial functions
 * of arbitrary dimension and degree.
 *
 * @author Andrew Willis, Brown University, April 7, 2003
 *         transcoded and extended by John Papadakis, UNCC, June 23, 2017
 *
 **/
//
// Copyright (C) 2003 Andrew R. Willis
// This program is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation; either version 2 of the License, or (at your
// option) any later version.
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
// for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 675 Mass Ave, Cambridge, MA 02139, USA.
//

#ifndef ALGEBRAICSURFACE_HPP
#define ALGEBRAICSURFACE_HPP

#ifdef __cplusplus

#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#define SAFETY_CHECKS true

template <typename ScalarType>
class AlgebraicSurface {
    using RowVectorXs = Eigen::Matrix<ScalarType, 1, Eigen::Dynamic>;
    using MatrixXs = Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic>;
    
public:
    using Ptr = boost::shared_ptr<AlgebraicSurface<ScalarType>>;

    AlgebraicSurface() {
    }

    AlgebraicSurface(const AlgebraicSurface& orig) {

        this->coeffs = orig.coeffs;
        this->dimension = orig.dimension;
        this->order = orig.order;

    }

    AlgebraicSurface(int dimension, int order) {
        int num_monomials = polyDim(dimension, order);
        this->coeffs.setZero(num_monomials);
        this->dimension = dimension;
        this->order = order;

    }

    AlgebraicSurface(const Eigen::Ref<const RowVectorXs>& coeffs, int dimension, int order) {
        int num_monomials = polyDim(dimension, order);
        this->dimension = dimension;
        this->order = order;
        this->coeffs.resize(num_monomials);
        this->coeffs = coeffs;
    }
    
    virtual ~AlgebraicSurface() = default;

    /**
     * Compute the number of monomial terms for a polynomial of the specified
     * dimension and degree.
     * @param dim - the dimension of the polynomial.
     * @param degree - the degree of the polynomial.
     * @return	the number of monomials in the polynomial.
     * */
    static int polyDim(int dim, int degree) {
        int size = 1;
        for (int i = 0; i < dim; i++) {
            size *= (degree + i + 1);
            size /= (i + 1);
        }
        return size;
    }

    /**
     * Given a data row vector X, this function computes a matrix, of the 
     * independent coordinate monomials based on the degree and dimension of 
     * this implicit polynomial.
     * <pre>
     * X = [x_1, x_2, ...,   x_dim]
     *
     * Mx: |1 x_1 x_1^2 ...   x_1^d|
     *     |1 x_2 x_2^2 ...   x_2^d|
     *     |... ...     ...    ... |
     *     |1 x_dim x_dim^2 x_dim^d|
     *
     * </pre>
     * @param X - the data point.
     * @return a matrix Mx (dimension x degree+1) containing the described 
     * terms.
     * */
    MatrixXs computePowerMatrix(const Eigen::Ref<const RowVectorXs>& x) {

        MatrixXs powers(this->dimension, this->order + 1);
        powers.col(0).setOnes();

        if (this->order > 0) {
            powers.col(1) = x;
        }

        for (int j = 0; j < this->dimension; j++) {
            for (int i = 2; i < (this->order + 1); i++) {
                powers(j, i) = x(j) * powers(j, i - 1);
            }
        }

        return powers;
    }

    /**
     * Given an integer array of the current monomial index in the implicit 
     * polynomial, this function generates a new integer array which corresponds
     * to the monomial immediately following the current monomial in 
     * lexographical order.
     * @param coordinate_powers An array of integers specifying the current 
     * monomial. The format of this array is as follows: 
     *      [x1^a, x2^b, x3^c,....]. 
     * Where the array coordinate_powers = [a, b, c, ...].
     * @return The new array of integers [a', b', c', ...] of the next monomial 
     * in lexographical order.
     * */
    Eigen::RowVectorXi generateNextMonomial(
            const Eigen::Ref<const Eigen::RowVectorXi>& coordinate_powers) const {

        Eigen::RowVectorXi res(coordinate_powers.size() + 1);

        int sum = 0;
        for (int i = 0; i < coordinate_powers.size(); i++) {
            res(i + 1) = coordinate_powers(i);
            sum += res(i);
        }
        res(0) = this->order - sum;
        int dim = res.size();
        for (int i = dim - 2; i >= 0; i--) {
            //find the lower bits with nonzero power
            int j;
            for (j = i; j >= 0; j--) {
                if (res(j) != 0) {
                    break;
                } else {
                    continue;
                }
            }
            if (j >= 0) {
                // change the nonzero bit
                res(j) = res(j) - 1;
                res(j + 1) = res(j + 1) + 1;
                for (int k = dim - 1; k > j + 1; k--) {
                    res(j + 1) += res(k);
                    res[k] = 0;
                }
                break; // finished the job
            }
        }

        Eigen::RowVectorXi ans(coordinate_powers.size());
        for (int i = 0; i < coordinate_powers.size(); i++) {
            ans(i) = res(i + 1);
        }

        return ans;
    }

    /**
     * Computes the monomial vector for a single data point using the matrix
     * obtained from {@link #computePowerMatrix(const Eigen::Ref<const RowVectorXs>& x)}.
     * @param X - the data point.
     * @return A row vector of monomials to be appended to the data monomial 
     * matrix.
     * */
    RowVectorXs computeMonomialVector(
            const Eigen::Ref<const RowVectorXs>& x) {
        MatrixXs powers = computePowerMatrix(x);

        int num_monomials = polyDim(this->dimension, this->order);
        RowVectorXs monomial_vector(num_monomials);
        monomial_vector.setOnes();
        Eigen::RowVectorXi coordinate_powers(x.size());
        coordinate_powers.setZero();

        for (int monomial_index = 0; monomial_index < num_monomials; monomial_index++) {
            monomial_vector(monomial_index) = 1;
            for (int k = 0; k < this->dimension; k++) {
                monomial_vector(monomial_index) *= powers(k, coordinate_powers(k));
            }

            coordinate_powers = generateNextMonomial(coordinate_powers);
        }
        return monomial_vector;
    }
    
    static int signum(ScalarType d) {
        if (d == 0) {
            return 0;
        } else if (d < 0) {
            return -1;
        } else {
            return 1;
        }
    }
    
    static bool coincident(
            const AlgebraicSurface<ScalarType>& p, const AlgebraicSurface<ScalarType>& q) {
        ScalarType det_nx_ny = p.coeffs(1) * q.coeffs(2) - q.coeffs(1) * p.coeffs(2);
        ScalarType det_nx_nz = p.coeffs(1) * q.coeffs(3) - q.coeffs(1) * p.coeffs(3);
        ScalarType det_nx_d = p.coeffs(1) * q.coeffs(0) - q.coeffs(1) * p.coeffs(0);
        ScalarType det_ny_nz = p.coeffs(2) * q.coeffs(3) - q.coeffs(2) * p.coeffs(3);
        ScalarType det_ny_d = p.coeffs(2) * q.coeffs(0) - q.coeffs(2) * p.coeffs(0);
        ScalarType det_nz_d = p.coeffs(3) * q.coeffs(0) - q.coeffs(3) * p.coeffs(0);
        return (det_nx_ny == 0 && det_nx_nz == 0 && det_nx_d == 0
                && det_ny_nz == 0 && det_ny_d == 0 && det_nz_d == 0);
        
    }
    
    /**
     * Computes integer factorials for n : n!
     * @param n Number to factorialize.
     * @return n!.
     * */
    static int factorial(int n) {
        if (n == 0) {
            return (1);
        }
        return (n * factorial(n - 1));
    }

    /**
     * Compute the binomial coefficient n choose m
     * @param n How many items to choose.
     * @param m Total number of available items.
     * @return Number of ways to choose n items out of m.
     * */
    static ScalarType getBinomialCoefficient(int n, int m) {
        int r1, r2, r3, r;
        r1 = factorial(n);
        r2 = factorial(m);
        r3 = factorial(n - m);
        if (r2 * r3 != 0) {
            r = r1 / (r2 * r3);
        } else {
            r = 0;
        }
        return ((ScalarType) r);
    }
    
    /**
     * Compute the multinomial coefficient for an array of numbers.
     * @param n How many items to choose.
     * @param m Total number of available items.
     * @return Number of ways to choose n items out of m.
     * */
    static ScalarType getMultinomialCoefficient(
            const Eigen::Ref<const Eigen::RowVectorXi>& coordinate_powers) {

        Eigen::RowVectorXi K(coordinate_powers.size());
        K = coordinate_powers;

        int N = K.sum();
        
        // find the maximal element in K
        int max_element = -1;
        int max_index = 0;
        for (int i = 0; i < K.size(); i++) {
            if (K(i) > max_element) {
                max_element = K(i);
                max_index = i;
            }
        }
        
        int res = 1;
        for (int i = max_element + 1; i <= N; i++) {
            res *= i;
        }
        
        for (int i = 0; i < K.size(); i++) {
            if (i != max_index) {
                for (int j = 1; j <= K[i]; j++) {
                    res /= j;
                }
            }
        }
        
        return res;
    }

    int getMonomialIndex(
            const Eigen::Ref<const Eigen::RowVectorXi>& coordinate_powers) {
        return getMonomialIndex(coordinate_powers, this->dimension);
    }

    static int getMonomialIndex(
            const Eigen::Ref<const Eigen::RowVectorXi>& coordinate_powers, int poly_dimension) {

        // get dimension and degree
        int dim = coordinate_powers.size();
        int deg = 0;
        for (int i = 0; i < dim; i++) {
            deg += coordinate_powers(i);
        }
        // compute position index
        int pos = polyDim(poly_dimension, deg - 1);
        int rest_deg = deg;
        for (int digits = 1; digits < dim; digits++) {
            // the last digist always determined
            int rest_digits = dim - digits;
            int current_power = coordinate_powers[digits - 1];

            // due to the formula \sum{k}{0}(\binom{a+i}{i})=\binom{a+k+1}{k},
            // the above part can be reduced to a simple form.
            if (rest_deg - current_power > 0) {
                // if there are still freedom of distribute power
                pos += getBinomialCoefficient(
                        rest_digits + rest_deg - current_power - 1,
                        rest_deg - current_power - 1);
            } else {
                pos += 0;
            }
            rest_deg -= current_power;
        }
        return pos;
    }
    
    /**
     * Adds two implicit polynomials and returns the result.
     * @param surfaceA - The first implicit polynomial.
     * @param surfaceB - The second implicit polynomial.
     * @return a new  implicit polynomial representing surfaceA + surfaceB.
     * */
    static AlgebraicSurface<ScalarType> add(
            const AlgebraicSurface<ScalarType>& surfaceA, 
            const AlgebraicSurface<ScalarType>& surfaceB) {
        
        int new_order = (surfaceA.order > surfaceB.order) ? surfaceA.order : surfaceB.order;
        int new_dimension = surfaceA.dimension;
        int num_monomials = polyDim(new_dimension, new_order);
        RowVectorXs new_coeffs(num_monomials);
        double coeffA, coeffB;
        for (int i = 0; i < new_coeffs.size(); i++) {
            if (i < surfaceA.coeffs.size()) {
                coeffA = surfaceA.coeffs(i);
            } else {
                coeffA = 0.0;
            }
            if (i < surfaceB.coeffs.size()) {
                coeffB = surfaceB.coeffs(i);
            } else {
                coeffB = 0.0;
            }
            new_coeffs[i] = coeffA + coeffB;
        }
        
        return AlgebraicSurface<ScalarType>(new_coeffs, new_dimension, new_order);
        
    }
    
    static AlgebraicSurface<ScalarType> scale(ScalarType scale, const AlgebraicSurface<ScalarType>& surface) {
        return AlgebraicSurface<ScalarType>(scale*surface.coeffs, surface.dimension, surface.order);
    }
    
    /**
     * Scale the coefficients of an implicit polynomial.
     * @param scale the value which will be multiplied into each of the polynomial coefficients.
     * @return a new scaled implicit polynomial.
     * */
    AlgebraicSurface<ScalarType> scale(ScalarType scale) {
        return AlgebraicSurface<ScalarType>(scale*this->coeffs, this->dimension, this->order);
    }

    /**
     * Multiplies the two passed AlgebraicSurface objects and returns a new
     * AlgebraicSurface object of the same dimension but a degree equal to the
     * sum of the multiplied polynomial degrees.
     * @param surfA The first of the two implicit polynomials to multiply.
     * @param surfB The second of the two implicit polynomials to multiply.
     * @return A new AlgebraicSurface object representing surfA*surfB.
     */
    static AlgebraicSurface<ScalarType> multiply(
            const AlgebraicSurface<ScalarType>& surfA,
            const AlgebraicSurface<ScalarType>& surfB) {

        if (surfA.dimension != surfB.dimension) {
            throw "Cannot multiply polynomials of differing dimension";
        }

        int num_monomialsA = polyDim(surfA.dimension, surfA.order);
        int num_monomialsB = polyDim(surfB.dimension, surfB.order);
        AlgebraicSurface<ScalarType> product(surfA.dimension, 
                surfA.order + surfB.order);
        int num_monomials = polyDim(product.dimension, product.order);

        Eigen::RowVectorXi surfA_monomial(surfA.dimension);
        Eigen::RowVectorXi surfB_monomial(surfB.dimension);
        Eigen::RowVectorXi product_monomial(surfA.dimension);
        RowVectorXs product_coeffs(num_monomials);
        product_coeffs.setZero();

        surfA_monomial.setZero();
        product_monomial.setZero();

        for (int i = 0; i < num_monomialsA; i++) {
            surfB_monomial.setZero();
            for (int j = 0; j < num_monomialsB; j++) {
                for (int k = 0; k < product_monomial.size(); k++) {
                    product_monomial[k] = surfA_monomial[k] + surfB_monomial[k];
                }
                int pos = product.getMonomialIndex(product_monomial);
                product_coeffs[pos] += surfA.coeffs(i) * surfB.coeffs(j);
                surfB_monomial = surfB.generateNextMonomial(surfB_monomial);
            }
            surfA_monomial = surfA.generateNextMonomial(surfA_monomial);
        }
        product.coeffs = product_coeffs;

        return product;
    }

    virtual ScalarType evaluate(const Eigen::Ref<const RowVectorXs>& pt) {
        RowVectorXs monomial_vector = computeMonomialVector(pt);
        return this->coeffs * monomial_vector.transpose();
    }

    virtual ScalarType evaluateDerivative(int coord_index, 
            const Eigen::Ref<const RowVectorXs>& pt) {
        return computeSymbolicDerivative(coord_index).evaluate(pt);
    }

    /**
     * Computes the derivative of the implicit polynomial with respect to the
     * coordinate specified by the passed arguement.
     * @param coordIndex - The index of the coordinate whose derivative we are computing.
     * @return The ImplicitPolynomial object which corresponds to the derivative of this polynomial
     *         with respect to the coordinate specified by coordIndex.
     * */
    AlgebraicSurface<ScalarType> computeSymbolicDerivative(int coord_index) {
        
        int num_monomials = polyDim(this->dimension, this->order);
        int num_monomials2 = polyDim(this->dimension, this->order - 1);
        
        RowVectorXs derivative_coeffs(num_monomials2);
        derivative_coeffs.setZero();

        Eigen::RowVectorXi coordinate_powers(this->dimension);
        coordinate_powers.setZero();
        
        int current_dimension = coord_index;
        for (int monom_idx = 0; monom_idx < num_monomials; monom_idx++) {
            if (coordinate_powers(current_dimension) > 0) {
                coordinate_powers(current_dimension)--;
                int monomialIndex = getMonomialIndex(coordinate_powers);
                coordinate_powers(current_dimension)++;
                derivative_coeffs(monomialIndex) = 
                        coordinate_powers(current_dimension) * this->coeffs(monom_idx);
            }
            coordinate_powers = generateNextMonomial(coordinate_powers);
        }
        
        return AlgebraicSurface<ScalarType>(derivative_coeffs, this->dimension, 
                this->order - 1);
        
    }
    
    virtual RowVectorXs evaluateGradient(const Eigen::Ref<const RowVectorXs>& pt) {
        
        RowVectorXs gradient(this->dimension);
        gradient.setZero();

        for (std::size_t d = 0; d != this->dimension; d++) {
            gradient(d) += this->evaluateDerivative(d, pt);
        }

        return gradient;
    }
    
    
    static AlgebraicSurface<ScalarType> affineTransform(
            const Eigen::Ref<const MatrixXs>& transform, 
            const AlgebraicSurface<ScalarType>& surface) {
        
        int num_monomials = polyDim(surface.dimension, surface.order);
        AlgebraicSurface<ScalarType> res = 
                AlgebraicSurface<ScalarType>(surface.dimension, surface.order);

        // notice we need to reverse the order to satisfy the order of [a_n, ..., a2, a1] like
        Eigen::RowVectorXi powers(surface.dimension);
        powers.setZero();

        for (int i = 0; i < num_monomials; i++) {
            // for each monomial
            RowVectorXs one_coeff(1);
            one_coeff.setOnes();

            AlgebraicSurface<ScalarType> poly(one_coeff, surface.dimension, 0); // 0 degree polynomial
            for (int j = 0; j < surface.dimension; j++) {
                // for each dimension
                RowVectorXs trans(surface.dimension + 1);
                for (int k = 1; k < surface.dimension + 1; k++) {
                    trans(k) = transform(j, k - 1);
                }
                trans(0) = transform(j, surface.dimension);
                poly = AlgebraicSurface<ScalarType>::multiply(poly, surface.expandLinearFormPower(trans, powers(j)));
            }
            res = AlgebraicSurface<ScalarType>::add(res, poly.scale(surface.coeffs(i)));
            powers = surface.generateNextMonomial(powers);
        }
        
        return res;
    }
    
    
    virtual void affineTransform(const Eigen::Ref<const MatrixXs>& transform) {
        *this = AlgebraicSurface<ScalarType>::affineTransform(transform, *this);
        return;
    }
    
    AlgebraicSurface<ScalarType> expandLinearFormPower(
            const Eigen::Ref<const RowVectorXs>& linear_coeffs, int power) const {
        
        Eigen::RowVectorXi indx(this->dimension + 1);
        indx.setZero();
        indx(0) = power;

        int num_monomials = polyDim(this->dimension, power);
        AlgebraicSurface<ScalarType> res = AlgebraicSurface<ScalarType>(this->dimension, power);
        RowVectorXs coeffs(num_monomials);
        coeffs.setZero();
        
        for (int i = 0; i < num_monomials; i++) {
            double prod = getMultinomialCoefficient(indx);
            for (int j = 0; j < this->dimension + 1; j++) {
                prod *= pow(linear_coeffs(j), indx(j));
            }
            
            coeffs(i) = prod;
            indx = generateNextMonomial(indx);
        }
        
        res.coeffs = coeffs;
        return res;
    }
    
    
    std::string coordinatePowerstoString(
            const Eigen::Ref<const Eigen::RowVectorXi>& coordinate_powers) {
        
        std::string str;
        for (int i = 0; i < coordinate_powers.size(); i++) {
            if (coordinate_powers(i) > 0) {
                str += "x" + std::to_string(i + 1) + "^" + 
                        std::to_string(coordinate_powers(i));
                if ((i < coordinate_powers.size() - 1) && 
                        (coordinate_powers(i + 1) > 0)) {
                    str += "*";
                }
            }
        }
        str += " ";
        return str;
        
    }

    std::string toString() {
        
        std::string str;
        int num_monomials = polyDim(this->dimension, this->order);
        Eigen::RowVectorXi coordinate_powers(this->dimension);
        coordinate_powers.setZero();

        for (int i = 0; i < num_monomials; i++) {
            str += std::to_string(this->coeffs(i)) + "*" +
                    coordinatePowerstoString(coordinate_powers);
            if (i < num_monomials - 1 && this->coeffs(i + 1) >= 0) {
                str += "+";
            }
            coordinate_powers = generateNextMonomial(coordinate_powers);
        }
        str += "\n";
        return str;
        
    }


    int dimension;

    int order;

    RowVectorXs coeffs;

};

template <typename ScalarType>
class PlanarSurface : public AlgebraicSurface<ScalarType> {
    typedef Eigen::Matrix<ScalarType, 1, Eigen::Dynamic> RowVectorXs;
    typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic> MatrixXs;

public:

    PlanarSurface() {
        this->order = 1;
        this->dimension = 3;
        this->coeffs.setZero(4);
        
    }

    PlanarSurface(const Eigen::Ref<const RowVectorXs>& coeffs) : 
        AlgebraicSurface<ScalarType>(coeffs, 3, 1) {}

    void convertHessianNormalForm() {
        this->coeffs = this->coeffs/this->coeffs.tail(3).norm();
    }
    
    ScalarType evaluate(const Eigen::Ref<const RowVectorXs>& pt) {
        
        return (Eigen::Matrix<ScalarType, 1, 4>() << 1.0, pt).finished() *
                this->coeffs.transpose();
        
    }

    ScalarType evaluateDerivative(int coord_index, const Eigen::Ref<const RowVectorXs>& pt) {
        
        return this->coeffs(coord_index + 1);
        
    }
    
    void affineTransform(
            const Eigen::Ref<const Eigen::Matrix<ScalarType, 4, 4>>& transform) {
        
        // inverse method 1
//        transform.row(3).swap(transform.col(3));
//        transform.block<1, 3>(3, 0) *= -transform.block<3, 3>(0, 0);
        
        // inverse method 2
//        transform.template block<3, 3>(0, 0).transposeInPlace();
//        transform.block<3, 1>(0, 3) = -transform.block<3, 3>(0, 0)*transform.block<3, 1>(0, 3);
        
        // this method seems to take ~600% less time to run than 
        // superclass affineTransform()
        Eigen::PermutationMatrix<4, 4> perm;
        perm.indices() << 1, 2, 3, 0;
        this->coeffs = perm.transpose()*transform*perm*this->coeffs.transpose();

//        this->AlgebraicSurface<ScalarType>::affineTransform(transform);
        this->convertHessianNormalForm();
        
        return;
    }

};

template <typename ScalarType>
class AlgebraicSurfaceProduct {
    using RowVectorXs =  Eigen::Matrix<ScalarType, 1, Eigen::Dynamic>;
    using MatrixXs = Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic>;
    using VecAlgebraicSurfacePtrs = 
            std::vector<boost::shared_ptr<AlgebraicSurface<ScalarType>>>;
    
public:

    AlgebraicSurfaceProduct() {}
    
    AlgebraicSurfaceProduct(int new_dimension) {
        
        this->dimension = new_dimension;
        
    }
    
    AlgebraicSurfaceProduct(const VecAlgebraicSurfacePtrs& new_subsurfaces) {
        
        this->subsurfaces = new_subsurfaces;
        this->dimension = this->computeDimension();
        this->order = this->computeOrder();
        
    }
    
    int computeDimension() {
        
        if (subsurfaces.size() == 0)
            throw "No subsurfaces found, cannot compute dimension.";
        
        int dim = this->subsurfaces[0]->dimension;
        for (auto const& surface : this->subsurfaces) {
            if (dim != surface->dimension)
                throw "Subsurfaces have different dimension!";
        }
        return dim;
        
    }
    
    int computeOrder() {
        
        if (subsurfaces.size() == 0)
            throw "No subsurfaces found, cannot compute order.";
        
        int order = 0;
        for (auto const& surface : this->subsurfaces) {
            order += surface->order;
        }
        return order;
        
    }
    
    void addSubSurface(const typename AlgebraicSurface<ScalarType>::Ptr& surface) {
        
        if (this->subsurfaces.size() == 0 && this->dimension == 0) {
            this->dimension = surface->dimension;
        } else if (this->dimension != surface->dimension)
                throw "Candidate surface dimension does not match collection!";
        
        this->subsurfaces.push_back(surface);
        this->order = this->computeOrder();
    }
    
    ScalarType evaluate(const Eigen::Ref<const RowVectorXs>& pt) {
        if (subsurfaces.size() == 0 && SAFETY_CHECKS)
            throw "No subsurfaces found!";
        
        ScalarType algebraic_dist = 1;

        for (auto const& surface : this->subsurfaces) {
            algebraic_dist *= surface->evaluate(pt);
        }

        return algebraic_dist;
        
    }
    
    ScalarType evaluateDerivative(int coord_index, const Eigen::Ref<const RowVectorXs>& pt) {
        if (subsurfaces.size() == 0 && SAFETY_CHECKS)
            throw "No subsurfaces found!";
        
        ScalarType algebraic_dist = 1;
        ScalarType derivative = 0;
        ScalarType this_surface_dist;

        for (std::size_t i = 0; i != this->subsurfaces.size(); i++) {
            this_surface_dist = this->subsurfaces[i]->evaluate(pt);
            if (std::abs<ScalarType>(this_surface_dist) < 1e-20)
                return ((ScalarType) 0.0);
            algebraic_dist *= this_surface_dist;
            derivative += this->subsurfaces[i]->evaluateDerivative(coord_index, pt)/this_surface_dist;
        }
        
        derivative *= algebraic_dist;

        return derivative;
        
    }
    
    RowVectorXs evaluateGradient(const Eigen::Ref<const RowVectorXs>& pt) {
        
        if (subsurfaces.size() == 0 && SAFETY_CHECKS)
            throw "No subsurfaces found!";
        
        if (this->dimension == 0)
            this->dimension = this->computeDimension();
        
        ScalarType algebraic_dist = 1;
        ScalarType this_surface_dist;
        RowVectorXs gradient(this->dimension);
        gradient.setZero();
        
        for (std::size_t i = 0; i != this->subsurfaces.size(); i++) {
            this_surface_dist = this->subsurfaces[i]->evaluate(pt);
            if (std::abs<ScalarType>(this_surface_dist) < 1e-20)
                return gradient.setZero();
            algebraic_dist *= this_surface_dist;
            
            for (std::size_t d = 0; d != this->dimension; d++) {
                gradient(d) += this->subsurfaces[i]->evaluateDerivative(d, pt)/this_surface_dist;
            }
        }
        
        gradient = algebraic_dist*gradient;

        return gradient;
        
    }

    RowVectorXs evaluateSquaredGradient(const Eigen::Ref<const RowVectorXs>& pt) {
        
        if (subsurfaces.size() == 0 && SAFETY_CHECKS)
            throw "No subsurfaces found!";
        
        if (this->dimension == 0)
            this->dimension = this->computeDimension();
        
        ScalarType algebraic_dist = 1;
        ScalarType this_surface_dist;
        RowVectorXs gradient(this->dimension);
        gradient.setZero();
        
        for (std::size_t i = 0; i != this->subsurfaces.size(); i++) {
            this_surface_dist = this->subsurfaces[i]->evaluate(pt);
            if (std::abs<ScalarType>(this_surface_dist) < 1e-20)
                return gradient.setZero();
            algebraic_dist *= this_surface_dist;
            
            for (std::size_t d = 0; d != this->dimension; d++) {
                gradient(d) += this->subsurfaces[i]->evaluateDerivative(d, pt)/this_surface_dist;
            }
        }
        
        gradient = 2*algebraic_dist*algebraic_dist*gradient;

        return gradient;
        
    }
    
    void affineTransform(const Eigen::Ref<const MatrixXs>& transform) {
        
        for (auto& surface : this->subsurfaces) {
            surface->affineTransform(transform);
        }
        
        return;
    }
    
    AlgebraicSurface<ScalarType> toAlgebraicSurface() {
        
        if (subsurfaces.size() == 0 && SAFETY_CHECKS)
            throw "No subsurfaces found!";
        
        AlgebraicSurface<ScalarType> new_surface(this->subsurfaces[0]->dimension, 0);
        new_surface.coeffs(0) = 1;
        
        for (auto const& surface : this->subsurfaces) {
            new_surface = AlgebraicSurface<ScalarType>::multiply(new_surface, *surface);
        }
        
        return new_surface;

    }
    
    int dimension = 0;

    int order = 0;
    
    VecAlgebraicSurfacePtrs subsurfaces;
};

template <typename ScalarType>
class CornerSurfaceProduct : public AlgebraicSurfaceProduct<ScalarType> {
    using RowVector3s =  Eigen::Matrix<ScalarType, 1, 3>;
    using RowVectorXs =  Eigen::Matrix<ScalarType, 1, Eigen::Dynamic>;
    using Matrix3s =  Eigen::Matrix<ScalarType, 3, 3>;
    using MatrixXs = Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic>;
    using VecAlgebraicSurfacePtrs = 
            std::vector<boost::shared_ptr<AlgebraicSurface<ScalarType>>>;
    
public:
    
    CornerSurfaceProduct() : AlgebraicSurfaceProduct<ScalarType>(3) {}
    
    CornerSurfaceProduct(const VecAlgebraicSurfacePtrs& new_subsurfaces) {
        this->subsurfaces = new_subsurfaces;
        if (this->computeDimension() != 3)
            throw "New subsurfaces are not of dimension 3!";
    }
    
    static RowVector3s getPoint(
            const AlgebraicSurface<ScalarType>& p, 
            const AlgebraicSurface<ScalarType>& q, 
            const AlgebraicSurface<ScalarType>& r) {
        
        Eigen::Matrix<ScalarType, 3, 4> coeffs_mat;
        coeffs_mat << p.coeffs, q.coeffs, r.coeffs;
         
        ScalarType x = (Matrix3s() << coeffs_mat.col(0), coeffs_mat.col(2), coeffs_mat.col(3)).finished().determinant();
        ScalarType y = (Matrix3s() << coeffs_mat.col(1), coeffs_mat.col(0), coeffs_mat.col(3)).finished().determinant();
        ScalarType z = (Matrix3s() << coeffs_mat.col(1), coeffs_mat.col(2), coeffs_mat.col(0)).finished().determinant();
        ScalarType w = -(Matrix3s() << coeffs_mat.col(1), coeffs_mat.col(2), coeffs_mat.col(3)).finished().determinant();
        
        if (AlgebraicSurface<ScalarType>::signum(w) == 0) {
            bool is_coincident = AlgebraicSurface<ScalarType>::coincident(p, q);
            return  RowVector3s().setConstant(std::numeric_limits<ScalarType>::quiet_NaN());
        }
        
        RowVector3s pt(x, y, z);
        
        return pt;
    }
    
//
//    static double inexactDeterminant3x3(double[][] m) {
//        double[][] m00_minor = {
//            {m[1][1], m[1][2]},
//            {m[2][1], m[2][2]}};
//        double[][] m01_minor = {
//            {m[1][0], m[1][2]},
//            {m[2][0], m[2][2]}};
//        double[][] m02_minor = {
//            {m[1][0], m[1][1]},
//            {m[2][0], m[2][1]}};
//        double det = m[0][0] * inexactDeterminant2x2(m00_minor)
//                - m[0][1] * inexactDeterminant2x2(m01_minor)
//                + m[0][2] * inexactDeterminant2x2(m02_minor);
//        return det;
//    }
//
//    static double inexactDeterminant2x2(double[][] m) {
//        double det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
//        return det;
//    }



};


#endif /* __cplusplus */
#endif /* ALGEBRAICSURFACE_H */
