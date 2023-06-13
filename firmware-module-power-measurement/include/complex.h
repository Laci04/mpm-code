//
// Created by dominik on 6/3/23.
//

#ifndef MCP3911_SPI_TEST_COMPLEX_H
#define MCP3911_SPI_TEST_COMPLEX_H

#include <math.h>

namespace math {

  template <class T>
  class complex {
  public:
    complex();
    complex(complex& other);
    complex(T re, T im);
    complex(T len, T phi, bool);

    template <class Other>
    complex<T>& operator = (complex<Other>&& other);

    complex operator +(complex&& other);
    template <class S>
    auto operator +(S scalar);
    complex operator -(complex&& other);
    template <class S>
    auto operator -(S scalar);
    complex operator *(complex&& other);
    template <class S>
    auto operator *(S scalar);
    complex operator /(complex&& other);
    template <class S>
    auto operator /(S scalar);

    complex<T> conj();
    T abso();
    T arg();

    T re;
    T im;
  };

  template <class T>
  template <class Other>
  complex<T>& complex<T>::operator=(complex<Other> &&other) {
    re = other.re;
    im = other.im;
    return *this;
  }

  template <class T> complex<T>::complex() = default;
  template <class T> complex<T>::complex(complex &other) : re{other.re}, im{other.im} {}
  template <class T> complex<T>::complex(T re, T im) : re{re}, im{im} {}
  template <class T> complex<T>::complex(T len, T phi, bool) {}

  template <class T>
  complex<T> complex<T>::operator+(complex &&other) {
    return complex(this->re + other.re, this->im + other.im);
  }
  template <class T>
  template <class S>
  auto complex<T>::operator+(S scalar) {
    return complex(this->re + scalar, this->im + scalar);
  }
  template <class T>
  complex<T> complex<T>::operator-(complex &&other) {
    return complex(this->re - other.re, this->im - other.im);
  }
  template <class T>
  template <class S>
  auto complex<T>::operator-(S scalar) {
    return complex(this->re - scalar, this->im - scalar);
  }
  template <class T>
  complex<T> complex<T>::operator*(complex &&other) {
    return complex(this->re * other.re - this->im * other->im, this->re * other.im + this->im * other.re);
  }
  template <class T>
  template <class S>
  auto complex<T>::operator*(S scalar) {
    return complex(this->re * scalar, this->im * scalar);
  }
  template <class T>
  complex<T> complex<T>::operator/(complex &&other) {
    return (other * (*this).conj())/(this->re * this->re + this->im * this->im);
  }
  template <class T>
  template <class S>
  auto complex<T>::operator/(S scalar) {
    return complex(this->re / scalar, this->im / scalar);
  }
  template <class T> complex<T> complex<T>::conj() { return complex<T>(re, -im); }
  template <class T> T complex<T>::abso() { return sqrt(this->re * this->re + this->im * this->im); }
  template <class T> T complex<T>::arg() { return atan(this->re/this->im); }
  };

#endif // MCP3911_SPI_TEST_COMPLEX_H
