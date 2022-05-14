/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once
#include "config.h"

#include <cmath>
#include <iostream>

namespace Eigen
{
template <typename _Scalar, int _Rows, int _Cols, int _Options>
class Matrix;
template <typename _Scalar, int _Rows, int _Cols, int _Options>
class MatrixView;
template <typename _Scalar, int _Rows, int _Cols, int _Options>
class Array;
namespace internal {

template <typename T>
struct traits;


template <typename _Scalar, int _Rows, int _Cols, int _Options>
struct traits<Matrix<_Scalar, _Rows, _Cols, _Options>>
{
   public:
    typedef _Scalar Scalar;
    using DenseReturnType = Matrix<_Scalar, _Rows, _Cols, _Options>;
};

template <typename _Scalar, int _Rows, int _Cols, int _Options>
struct traits<MatrixView<_Scalar, _Rows, _Cols, _Options>>
{
   public:
    typedef _Scalar Scalar;
    using DenseReturnType = Matrix<_Scalar, _Rows, _Cols, _Options>;
};

template <typename _Scalar, int _Rows, int _Cols, int _Options>
struct traits<Array<_Scalar, _Rows, _Cols, _Options>>
{
   public:
    typedef _Scalar Scalar;
    using DenseReturnType = Array<_Scalar, _Rows, _Cols, _Options>;
};

}

enum StorageOptions {
    /** Storage order is column major (see \ref TopicStorageOrders). */
    ColMajor = 0,
    /** Storage order is row major (see \ref TopicStorageOrders). */
    RowMajor = 0x1,  // it is only a coincidence that this is equal to RowMajorBit -- don't rely on that
    /** Align the matrix itself if it is vectorizable fixed-size */
    AutoAlign = 0,
    /** Don't require alignment for the matrix itself (the array of coefficients, if dynamically allocated, may still be requested to be aligned) */ // FIXME --- clarify the situation
    DontAlign = 0x2
};

template <typename Derived>
class MatrixBase
{
   public:
    using Scalar          = typename internal::traits<Derived>::Scalar;
    using DenseReturnType = typename internal::traits<Derived>::DenseReturnType;
    using SameMatrix      = Derived;
    using PlainObject     = Derived;

    const Derived& derived() const { return *static_cast<const Derived*>(this); }
    Derived& derived() { return *static_cast<Derived*>(this); }

    Scalar& operator()(int i, int j) { return derived()(i, j); }
    const Scalar& operator()(int i, int j) const { return derived()(i, j); }



    Scalar& at(int i) { return derived().at(i); }
    const Scalar& at(int i) const { return derived().at(i); }


    int rows() const { return derived().rows(); }
    int cols() const { return derived().cols(); }
    int size() const { return derived().size(); }

    Scalar minCoeff() const
    {
        Scalar result = derived().at(0);
        for (int i = 1; i < derived().size(); ++i)
        {
            result = std::min(result, derived().at(i));
        }
        return result;
    }

    Scalar sum() const
    {
        Scalar result = 0;
        for (int i = 0; i < derived().size(); ++i)
        {
            result += derived().at(i);
        }
        return result;
    }

    Scalar maxCoeff() const
    {
        Scalar result = derived().at(0);
        for (int i = 1; i < derived().size(); ++i)
        {
            result = std::max(result, derived().at(i));
        }
        return result;
    }


    Scalar norm() const
    {
        Scalar result = 0;
        for (int i = 0; i < derived().size(); ++i)
        {
            result += derived().at(i) * derived().at(i);
        }
        return sqrt(result);
    }

    Scalar squaredNorm() const
    {
        Scalar result = 0;
        for (int i = 0; i < derived().size(); ++i)
        {
            result += derived().at(i) * derived().at(i);
        }
        return result;
    }

    SameMatrix eval() const { return *this; }

    SameMatrix normalized() const { return *this / norm(); }


    SameMatrix& normalize()
    {
        derived() = derived() / norm();
        return derived();
    }

    Scalar dot(const SameMatrix& other) const
    {
        Scalar result = 0;
        for (int i = 0; i < derived().size(); ++i)
        {
            result += derived().at(i) * other.derived().at(i);
        }
        return result;
    }

    DenseReturnType cross(const SameMatrix& other) const
    {
        DenseReturnType result;
        return result;
    }

    DenseReturnType transpose() const
    {
        DenseReturnType result;
        return result;
    }

    DenseReturnType operator-() const
    {
        DenseReturnType result;
        for (int i = 0; i < derived().size(); ++i)
        {
            result.at(i) = -derived().at(i);
        }
        return result;
    }


    void setZero()
    {
        for (int i = 0; i < derived().size(); ++i)
        {
            derived().at(i) = 0;
        }
    }


    void setOnes()
    {
        for (int i = 0; i < derived().size(); ++i)
        {
            derived().at(i) = 1;
        }
    }



    DenseReturnType inverse() const
    {
        DenseReturnType result;
        return result;
    }

    DenseReturnType round() const
    {
        DenseReturnType result;
        for (int i = 0; i < derived().size(); ++i)
        {
            result.at(i) = std::round(derived().at(i));
        }
        return result;
    }

    bool operator==(SameMatrix other) const
    {
        bool result = true;
        for (int i = 0; i < derived().size(); ++i)
        {
            result &= derived().at(i) == other.derived().at(i);
        }
        return result;
    }

    DenseReturnType operator&&(SameMatrix other) const
    {
        DenseReturnType result;
        for (int i = 0; i < derived().size(); ++i)
        {
            result.at(i) = derived().at(i) && other.derived().at(i);
        }
        return result;
    }


    SameMatrix& operator+=(const SameMatrix& other)
    {
        derived() = derived() + other;
        return derived();
    }

    SameMatrix& operator-=(const SameMatrix& other)
    {
        derived() = derived() - other;
        return derived();
    }


    SameMatrix& operator*=(Scalar value)
    {
        derived() = derived() * value;
        return derived();
    }

    SameMatrix& operator/=(Scalar other)
    {
        *this = *this / other;
        return derived();
    }
};

template <typename _Scalar, int _Rows, int _Cols, int _Options = ColMajor>
class Array : public MatrixBase<Array<_Scalar, _Rows, _Cols, _Options>>
{
   public:
    using Scalar              = _Scalar;
    using SameMatrix          = Array<_Scalar, _Rows, _Cols, _Options>;
    static constexpr int Size = _Rows * _Cols;
    static_assert(_Rows > 0, "Rows must be positive");
    static_assert(_Options == ColMajor, "Only colmajor supportet");

    Array() {}

    _Scalar* data() { return data; }

    _Scalar& operator()(int i, int j) { return _data[j * _Rows + i]; }
    const _Scalar& operator()(int i, int j) const { return _data[j * _Rows + i]; }

    template <typename T>
    Array<T, _Rows, _Cols, _Options> cast() const
    {
        Array<T, _Rows, _Cols, _Options> result;
        for (int i = 0; i < result.size(); ++i)
        {
            result.at(i) = (T)at(i);
        }

        return result;
    }

    SameMatrix min(SameMatrix other) const
    {
        SameMatrix result;
        for (int i = 0; i < size(); ++i)
        {
            result.at(i) = std::min(at(i), other.at(i));
        }
        return result;
    }

    SameMatrix max(SameMatrix other) const
    {
        SameMatrix result;
        for (int i = 0; i < size(); ++i)
        {
            result.at(i) = std::max(at(i), other.at(i));
        }
        return result;
    }

    SameMatrix abs() const
    {
        SameMatrix result;
        for (int i = 0; i < size(); ++i)
        {
            result.at(i) = std::abs(at(i));
        }
        return result;
    }


    SameMatrix floor() const
    {
        SameMatrix result;
        for (int i = 0; i < size(); ++i)
        {
            result.at(i) = std::floor(at(i));
        }
        return result;
    }

    int rows() const { return _Rows; }
    int cols() const { return _Cols; }
    int size() const { return Size; }

    _Scalar& at(int index) { return _data[index]; }
    const _Scalar& at(int index) const { return _data[index]; }

   private:
    _Scalar _data[Size];
};

template <typename _Scalar, int _Rows, int _Cols, int _Options>
class MatrixView : public MatrixBase<MatrixView<_Scalar, _Rows, _Cols, _Options>>
{
   public:
    using Scalar              = _Scalar;
    using SameMatrix          = MatrixView<_Scalar, _Rows, _Cols, _Options>;
    static constexpr int Size = _Rows * _Cols;
    static_assert(_Rows > 0, "Rows must be positive");
    static_assert(_Options == ColMajor, "Only colmajor supportet");

    MatrixView(Scalar* data, int row_stride, int col_stride)
        : _data(data), _row_stride(row_stride), _col_stride(col_stride)
    {
    }


    template <typename OtherType>
    MatrixView(const MatrixBase<OtherType> other)
    {
        for (int i = 0; i < rows(); ++i)
        {
            for (int j = 0; j < cols(); ++j)
            {
                (*this)(i, j) = other(i, j);
            }
        }
    }

    int rows() const { return _Rows; }
    int cols() const { return _Cols; }
    int size() const { return Size; }

   private:
    _Scalar* _data;
    // distance between neighboring rows/cols
    int _row_stride, _col_stride;
};

template <typename _Scalar, int _Rows, int _Cols, int _Options = ColMajor>
class Matrix : public MatrixBase<Matrix<_Scalar, _Rows, _Cols, _Options>>
{
   public:
    using Scalar              = _Scalar;
    using SameMatrix          = Matrix<_Scalar, _Rows, _Cols, _Options>;
    static constexpr int Size = _Rows * _Cols;
    static_assert(_Rows > 0, "Rows must be positive");
    static_assert(_Options == ColMajor, "Only colmajor supportet");

    static SameMatrix Zero()
    {
        SameMatrix result;
        for (int i = 0; i < result.size(); ++i)
        {
            result.at(i) = 0;
        }
        return result;
    }

    static SameMatrix Ones()
    {
        SameMatrix result;
        for (int i = 0; i < result.size(); ++i)
        {
            result.at(i) = 1;
        }
        return result;
    }

    static SameMatrix Identity()
    {
        static_assert(_Rows == _Cols, "Only valid for square matrices.");
        SameMatrix result = Zero();
        for (int i = 0; i < _Rows; ++i)
        {
            result(i, i) = 1;
        }
        return result;
    }

    Matrix() {}

    template <typename OtherType>
    Matrix(const MatrixBase<OtherType> other)
    {
        for (int i = 0; i < rows(); ++i)
        {
            for (int j = 0; j < cols(); ++j)
            {
                (*this)(i, j) = other(i, j);
            }
        }
    }

    Matrix(const Array<_Scalar, _Rows, _Cols, _Options> other) { this->array() = other; }

    // Vector constructors
    Matrix(_Scalar x0, _Scalar x1)
    {
        static_assert(_Rows == 2 && _Cols == 1, "Constructor only valid for vectors.");
        _data[0] = x0;
        _data[1] = x1;
    }
    Matrix(_Scalar x0, _Scalar x1, _Scalar x2)
    {
        static_assert(_Rows == 3 && _Cols == 1, "Constructor only valid for vectors.");
        _data[0] = x0;
        _data[1] = x1;
        _data[2] = x2;
    }
    Matrix(_Scalar x0, _Scalar x1, _Scalar x2, _Scalar x3)
    {
        static_assert(_Rows == 4 && _Cols == 1, "Constructor only valid for vectors.");
        _data[0] = x0;
        _data[1] = x1;
        _data[2] = x2;
        _data[3] = x3;
    }

    _Scalar* data() { return data; }

    _Scalar& operator[](int i)
    {
        static_assert(_Rows == 1 || _Cols == 1, "Constructor only valid for vectors.");
        return _data[i];
    }
    const _Scalar& operator[](int i) const
    {
        static_assert(_Rows == 1 || _Cols == 1, "Constructor only valid for vectors.");
        return _data[i];
    }

    _Scalar& operator()(int i)
    {
        static_assert(_Rows == 1 || _Cols == 1, "Constructor only valid for vectors.");
        return _data[i];
    }
    const _Scalar& operator()(int i) const
    {
        static_assert(_Rows == 1 || _Cols == 1, "Constructor only valid for vectors.");
        return _data[i];
    }

    _Scalar& operator()(int i, int j) { return _data[j * _Rows + i]; }
    const _Scalar& operator()(int i, int j) const { return _data[j * _Rows + i]; }

    Scalar& x() { return _data[0]; }
    Scalar& y() { return _data[1]; }
    Scalar& z() { return _data[2]; }
    Scalar& w() { return _data[3]; }
    const Scalar& x() const { return _data[0]; }
    const Scalar& y() const { return _data[1]; }
    const Scalar& z() const { return _data[2]; }
    const Scalar& w() const { return _data[3]; }

    int rows() const { return _Rows; }
    int cols() const { return _Cols; }
    int size() const { return Size; }

    template <typename T>
    Matrix<T, _Rows, _Cols, _Options> cast() const
    {
        Matrix<T, _Rows, _Cols, _Options> result;
        for (int i = 0; i < result.size(); ++i)
        {
            result.at(i) = (T)at(i);
        }

        return result;
    }
    MatrixView<Scalar, _Rows, 1, _Options> col(int id)
    {
        return MatrixView<Scalar, _Rows, 1, _Options>(&((*this)(0, id)), 1, 1);
    }

    Matrix<Scalar, _Rows, 1, _Options> col(int id) const
    {
        return MatrixView<const Scalar, _Rows, 1, _Options>(&((*this)(0, id)), 1, 1);
    }

    template <int NewRows, int NewCols>
    MatrixView<Scalar, NewRows, NewCols, _Options> block(int i, int j)
    {
        return MatrixView<Scalar, NewRows, NewCols, _Options>(&((*this)(i, j)), 1, _Cols);
    }

    template <int NewRows, int NewCols>
    Matrix<Scalar, NewRows, NewCols, _Options> block(int i, int j) const
    {
        return MatrixView<const Scalar, NewRows, NewCols, _Options>(&((*this)(i, j)), 1, _Cols);
    }


    template <int NewRows>
    Matrix<Scalar, NewRows, 1, _Options> head()
    {
        return MatrixView<Scalar, NewRows, 1, _Options>(&((*this)(0, 0)), 1, 1);
    }

    template <int NewRows>
    Matrix<Scalar, NewRows, 1, _Options> head() const
    {
        return MatrixView<const Scalar, NewRows, 1, _Options>(&((*this)(0, 0)), 1, 1);
    }


    _Scalar& at(int index) { return _data[index]; }
    const _Scalar& at(int index) const { return _data[index]; }

    const Array<_Scalar, _Rows, _Cols, _Options>& array() const
    {
        return *reinterpret_cast<const Array<_Scalar, _Rows, _Cols, _Options>*>(this);
    }

    Array<_Scalar, _Rows, _Cols, _Options>& array()
    {
        return *reinterpret_cast<Array<_Scalar, _Rows, _Cols, _Options>*>(this);
    }


   private:
    _Scalar _data[Size];
};

template <typename Derived>
typename Derived::PlainObject operator-(const MatrixBase<Derived>& m1, const MatrixBase<Derived>& m2)
{
    typename Derived::PlainObject result;
    for (int i = 0; i < result.size(); ++i)
    {
        result.at(i) = m1.at(i) - m2.at(i);
    }
    return result;
}

template <typename Derived>
typename Derived::PlainObject operator+(const MatrixBase<Derived>& m1, const MatrixBase<Derived>& m2)
{
    typename Derived::PlainObject result;
    for (int i = 0; i < result.size(); ++i)
    {
        result.at(i) = m1.at(i) + m2.at(i);
    }
    return result;
}


template <typename Derived>
typename Derived::PlainObject operator*(const MatrixBase<Derived>& m1, typename Derived::Scalar v)
{
    typename Derived::PlainObject result;
    return result;
}

template <typename Derived>
typename Derived::PlainObject operator*(typename Derived::Scalar v, const MatrixBase<Derived>& m1)
{
    typename Derived::PlainObject result;
    return result;
}

template <typename Derived>
typename Derived::PlainObject operator/(const MatrixBase<Derived>& m1, typename Derived::Scalar v)
{
    typename Derived::PlainObject result;
    return result;
}

template <typename Derived>
typename Derived::PlainObject operator/(typename Derived::Scalar v, const MatrixBase<Derived>& m1)
{
    typename Derived::PlainObject result;
    return result;
}

template <typename _Scalar, int _Rows0, int _Cols0, int _Options0, int _Rows1, int _Cols1, int _Options1>
Matrix<_Scalar, _Rows0, _Cols1, _Options0> operator*(const Matrix<_Scalar, _Rows0, _Cols0, _Options0>& m1,
                                                     Matrix<_Scalar, _Rows1, _Cols1, _Options1> m2)
{
    Matrix<_Scalar, _Rows0, _Cols1, _Options0> result;
    return result;
}

template <typename _Scalar, int _Rows0, int _Cols0, int _Options0>
Array<_Scalar, _Rows0, _Cols0, _Options0> operator*(const Array<_Scalar, _Rows0, _Cols0, _Options0>& m1,
                                                    Array<_Scalar, _Rows0, _Cols0, _Options0> m2)
{
    Array<_Scalar, _Rows0, _Cols0, _Options0> result;
    return result;
}


template <typename Derived>
std::ostream& operator<<(std::ostream& strm, const MatrixBase<Derived>& m1)
{
    return strm;
}

}  // namespace Eigen