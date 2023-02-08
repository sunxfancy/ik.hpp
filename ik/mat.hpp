#pragma once
#include <iostream>
#include "ik/vec.hpp"

namespace ik
{


template <typename T, unsigned C, unsigned R>
class mat {
public:
    mat() { clear(); }
    mat(const mat<T, C, R>& rhs) = default;
    mat(mat<T, C, R>&& rhs) = default;
    ~mat() = default;

    mat(std::initializer_list<vec<T, C>> list) {
        unsigned i = 0;
        for (auto v : list) {
            data[i++] = v;
        }
    }

    vec<T, C>& operator[](unsigned i) { return data[i]; }
    const vec<T, C>& operator[](unsigned i) const { return data[i]; }
    bool operator==(const mat<T, C, R>& rhs) const {
        for (unsigned i = 0; i < R; ++i) {
            for (unsigned j = 0; j < C; ++j) {
                if (data[i][j] != rhs[i][j]) {
                    return false;
                }
            }
        }
        return true;
    }

    mat<T, C, R> operator=(const mat<T, C, R>& rhs) {
        for (unsigned i = 0; i < R; ++i) {
            data[i] = rhs[i];
        }
        return *this;
    }

    void clear() {
        for (unsigned i = 0; i < R; ++i) {
            for (unsigned j = 0; j < C; ++j) {
                data[i][j] = 0;
            }
        }
    }
protected:
    vec<T, C> data[R];
};


template <typename T, unsigned C, unsigned R>
std::ostream& operator<<(std::ostream& os, const mat<T, C, R>& m) {
    os << "[";
    for (unsigned i = 0; i < R; ++i) {
        os << m[i] << std::endl;
    }
    os << "]";
    return os;
}

template <typename T, unsigned C, unsigned R>
mat<T, C, R> operator+(const mat<T, C, R>& lhs, const mat<T, C, R>& rhs) {
    mat<T, C, R> result;
    for (unsigned i = 0; i < R; ++i) {
        for (unsigned j = 0; j < C; ++j) {
            result[i][j] = lhs[i][j] + rhs[i][j];
        }
    }
    return result;
}

template <typename T, unsigned C, unsigned R>
mat<T, C, R> operator+(const mat<T, C, R>& lhs, T rhs) {
    mat<T, C, R> result;
    for (unsigned i = 0; i < R; ++i) {
        for (unsigned j = 0; j < C; ++j) {
            result[i][j] = lhs[i][j] + rhs;
        }
    }
    return result;
}

template <typename T, unsigned C, unsigned R>
mat<T, C, R> operator+(T lhs, const mat<T, C, R>& rhs) {
    return rhs + lhs;
}

template <typename T, unsigned C, unsigned R>
mat<T, C, R> operator-(const mat<T, C, R>& m) {
    mat<T, C, R> result;
    for (unsigned i = 0; i < R; ++i) {
        for (unsigned j = 0; j < C; ++j) {
            result[i][j] = -m[i][j];
        }
    }
    return result;
}


template <typename T, unsigned C, unsigned R>
mat<T, C, R> operator-(const mat<T, C, R>& lhs, const mat<T, C, R>& rhs) {
    mat<T, C, R> result;
    for (unsigned i = 0; i < R; ++i) {
        for (unsigned j = 0; j < C; ++j) {
            result[i][j] = lhs[i][j] - rhs[i][j];
        }
    }
    return result;
}

template <typename T, unsigned C, unsigned R>
mat<T, C, R> operator-(const mat<T, C, R>& lhs, T rhs) {
    mat<T, C, R> result;
    for (unsigned i = 0; i < R; ++i) {
        for (unsigned j = 0; j < C; ++j) {
            result[i][j] = lhs[i][j] - rhs;
        }
    }
    return result;
}

template <typename T, unsigned C, unsigned R>
mat<T, C, R> operator-(T lhs, const mat<T, C, R>& rhs) {
    return -rhs + lhs;
}


template <typename T, unsigned C, unsigned R>
mat<T, C, R> operator*(const mat<T, C, R>& m, T s) {
    mat<T, C, R> result;
    for (unsigned i = 0; i < R; ++i) {
        for (unsigned j = 0; j < C; ++j) {
            result[i][j] = m[i][j] * s;
        }
    }
    return result;
}

template <typename T, unsigned C, unsigned R>
mat<T, C, R> operator*(T s, const mat<T, C, R>& m) {
    return m * s;
}

template <typename T, unsigned C, unsigned R>
vec<T, C> operator*(const mat<T, C, R>& m, const vec<T, R>& v) {
    vec<T, C> result;
    for (unsigned i = 0; i < C; ++i) {
        for (unsigned j = 0; j < R; ++j) {
            result[i] += (m[i][j] * v[j]);
        }
    }
    return result;
}

template <typename T, unsigned C, unsigned R>
vec<T, C> operator*(const vec<T, R>& v, const mat<T, C, R>& m) {
    vec<T, C> result;
    for (unsigned i = 0; i < R; ++i) {
        for (unsigned j = 0; j < C; ++j) {
            result[j] += v[i] * m[i][j];
        }
    }
    return result;
}

template <typename T, unsigned C, unsigned R, unsigned Q>
mat<T, R, Q> operator*(const mat<T, C, R>& lhs, const mat<T, Q, C>& rhs) {
    mat<T, R, Q> result;
    for (unsigned i = 0; i < R; ++i) {
        for (unsigned k = 0; k < C; ++k) {
            for (unsigned j = 0; j < Q; ++j) {
                result[i][j] += lhs[i][k] * rhs[k][j];
            }
        }
    }
    return result;
}

#define MAT_TYPE(T, N, C) \
    typedef mat<T, C, C> N##mat##C; 
    
#define MAT_TYPE_D(T, N, C, R) \
    typedef mat<T, C, R> N##mat##C##x##R; \

#define MAT_TYPES(T, N) \
    MAT_TYPE(T, N, 2) \
    MAT_TYPE(T, N, 3) \
    MAT_TYPE(T, N, 4) \
    MAT_TYPE_D(T, N, 2, 3) \
    MAT_TYPE_D(T, N, 2, 4) \
    MAT_TYPE_D(T, N, 3, 2) \
    MAT_TYPE_D(T, N, 3, 4) \
    MAT_TYPE_D(T, N, 4, 2) \
    MAT_TYPE_D(T, N, 4, 3)

MAT_TYPES(float,)
MAT_TYPES(double, d)
MAT_TYPES(unsigned, u)
MAT_TYPES(int, i)
MAT_TYPES(uint8_t, u8)
MAT_TYPES(uint16_t, u16)
MAT_TYPES(uint32_t, u32)
MAT_TYPES(uint64_t, u64)
MAT_TYPES(int8_t, i8)
MAT_TYPES(int16_t, i16)
MAT_TYPES(int32_t, i32)
MAT_TYPES(int64_t, i64)

#undef MAT_TYPE
#undef MAT_TYPE_D
#undef MAT_TYPES

} // namespace ik
