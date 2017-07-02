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
            const Eigen::Ref<const Eigen::RowVectorXi>& coordinate_powers) {

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
     * Multiplies the two passed AlgebraicSurface objects and returns a new
     * AlgebraicSurface object of the same dimension but a degree equal to the
     * sum of the multiplied polynomial degrees.
     * @param surfA The first of the two implicit polynomials to multiply.
     * @param surfB The second of the two implicit polynomials to multiply.
     * @return A new AlgebraicSurface object representing surfA*surfB.
     */
    static AlgebraicSurface<ScalarType> multiply(AlgebraicSurface& surfA,
            AlgebraicSurface& surfB) {

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

    ScalarType evaluate(const Eigen::Ref<const RowVectorXs>& pt) {
        
        return (Eigen::Matrix<ScalarType, 1, 4>() << 1.0, pt).finished() *
                this->coeffs.transpose();
        
    }

    ScalarType evaluateDerivative(int coord_index, const Eigen::Ref<const RowVectorXs>& pt) {
        
        return this->coeffs(coord_index + 1);
        
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
    
    AlgebraicSurfaceProduct(
            int new_dimension, int new_order, 
            const VecAlgebraicSurfacePtrs& new_subsurfaces) {
        
        this->dimension = new_dimension;
        this->order = new_order;
        this->subsurfaces = new_subsurfaces;
        
    }
    
    AlgebraicSurfaceProduct(
            const VecAlgebraicSurfacePtrs& new_subsurfaces) {
        
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
        
        if (this->subsurfaces.size() == 0) {
            this->dimension = surface->dimension;
            this->order = 0;
        } else if (this->dimension != surface->dimension)
                throw "Candidate surface dimension does not match collection!";
        
        this->order += surface->order;
        this->subsurfaces.push_back(surface);
        
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
    
    int dimension;

    int order;
    
    VecAlgebraicSurfacePtrs subsurfaces;
};
#endif /* __cplusplus */
#endif /* ALGEBRAICSURFACE_H */
