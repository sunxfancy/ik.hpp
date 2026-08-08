#pragma once

#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <ostream>
#include <type_traits>

namespace ik
{

// vec_base is used to specialize the vec class for different dimensions with different fields
template <typename T, unsigned N>
struct vec_base {
    T data[N];
};

template <typename T>
struct vec_base<T, 2> {
    union {
        struct { T x, y; };
        T data[2];
    };
};

template <typename T>
struct vec_base<T, 3> {
    union {
        struct { T x, y, z; };
        struct { T r, g, b; };
        T data[3];
    };
};

template <typename T>
struct vec_base<T, 4> {
    union {
        struct { T x, y, z, w; };
        struct { T r, g, b, a; };
        T data[4];
    };
};


template <typename T, unsigned N>
class vec : public vec_base<T, N>
{
public:
    using vec_base<T, N>::data;

    vec() {
        for (unsigned i = 0; i < N; ++i) {
            this->data[i] = 0;
        }
    }
    vec(std::initializer_list<T> list) {
        unsigned i = 0;
        for(auto& val : list) {
            this->data[i++] = val;
        }
    }
    vec(const vec<T, N>& rhs) = default;
    vec(vec<T, N>&& rhs) = default;
    ~vec() = default;

    T operator[](unsigned i) const { return this->data[i]; }
    T& operator[](unsigned i) { return this->data[i]; }

    vec<T, N> operator=(vec<T, N> rhs) {
        for (unsigned i = 0; i < N; ++i) {
            this->data[i] = rhs[i];
        }
        return *this;
    }
};

template <typename T, unsigned N>
T sum(vec<T, N> v) {
    T result = 0;
    for(unsigned i = 0; i < N; ++i) {
        result += v[i];
    }
    return result;
}


template <typename T, unsigned N>
T dot(vec<T, N> lhs, vec<T, N> rhs) {
    T result = T{};
    for(unsigned i = 0; i < N; ++i) {
        result += (lhs[i] * rhs[i]);
    }
    return result;
}

template <typename T, unsigned N>
T length_squared(vec<T, N> v) {
    return dot(v, v);
}

template <typename T, unsigned N>
T length(vec<T, N> v) {
    return std::sqrt(length_squared(v));
}

template <typename T, unsigned N>
T distance(vec<T, N> a, vec<T, N> b) {
    return length(a - b);
}

template <typename T, unsigned N>
vec<T, N> normalize(vec<T, N> v) {
    T len = length(v);
    if (len <= T{}) {
        return v;
    }
    return v / len;
}

template <typename T>
vec<T, 2> rotate(vec<T, 2> v, T angle) {
    T c = std::cos(angle);
    T s = std::sin(angle);
    return vec<T, 2>{v[0] * c - v[1] * s, v[0] * s + v[1] * c};
}

template <typename T>
T angle(vec<T, 2> v) {
    return std::atan2(v[1], v[0]);
}

template <typename T>
T wrap_angle(T a) {
    const T pi = static_cast<T>(3.14159265358979323846);
    const T two_pi = static_cast<T>(2) * pi;
    while (a <= -pi) a += two_pi;
    while (a > pi) a -= two_pi;
    return a;
}

template <typename T>
T clamp_angle(T value, T min_value, T max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

template <typename T, unsigned N>
vec<T, N> cross(vec<T, N> lhs, vec<T, N> rhs) {
    // static_assert(false, "Cross product is only defined for 2D and 3D vectors");
    return vec<T, N>();
}

template <typename T>
vec<T, 2> cross(vec<T, 2> lhs, vec<T, 2> rhs) {
    vec<T, 2> result;
    result[0] = lhs[0] * rhs[1] - lhs[1] * rhs[0];
    return result;
}

template <typename T>
vec<T, 3> cross(vec<T, 3> lhs, vec<T, 3> rhs) {
    vec<T, 3> result;
    result[0] = lhs[1] * rhs[2] - lhs[2] * rhs[1];
    result[1] = lhs[2] * rhs[0] - lhs[0] * rhs[2];
    result[2] = lhs[0] * rhs[1] - lhs[1] * rhs[0];
    return result;
}


template <typename T, unsigned N>
vec<T, N> operator+(vec<T, N> lhs, vec<T, N> rhs) {
    vec<T, N> result;
    for(unsigned i = 0; i < N; ++i) {
        result[i] = lhs[i] + rhs[i];
    }
    return result;
}

template <typename T, typename U, unsigned N>
vec<T, N> operator+(vec<T, N> lhs, U rhs) {
    vec<T, N> result;
    for(unsigned i = 0; i < N; ++i) {
        result[i] = lhs[i] + rhs;
    }
    return result;
}

template <typename T, typename U, unsigned N>
vec<T, N> operator+(U lhs, vec<T, N> rhs) {
    return rhs + lhs;
}

template <typename T, unsigned N>
vec<T, N> operator-(vec<T, N> lhs, vec<T, N> rhs) {
    vec<T, N> result;
    for(unsigned i = 0; i < N; ++i) {
        result[i] = lhs[i] - rhs[i];
    }
    return result;
}

template <typename T, typename U, unsigned N>
vec<T, N> operator-(vec<T, N> lhs, U rhs) {
    vec<T, N> result;
    for(unsigned i = 0; i < N; ++i) {
        result[i] = lhs[i] - rhs;
    }
    return result;
}

template <typename T, unsigned N>
vec<T, N> operator-(vec<T, N> rhs) {
    vec<T, N> result;
    for(unsigned i = 0; i < N; ++i) {
        result[i] = - rhs[i];
    }
    return result;
}

template <typename T, unsigned N>
vec<T, N> operator*(vec<T, N> lhs, vec<T, N> rhs) {
    return cross(lhs, rhs);
}

template <typename T, typename U, unsigned N>
vec<T, N> operator*(vec<T, N> lhs, U rhs) {
    vec<T, N> result;
    for(unsigned i = 0; i < N; ++i) {
        result[i] = lhs[i] * rhs;
    }
    return result;
}

template <typename T, unsigned N>
vec<T, N> operator*(T lhs, vec<T, N> rhs) {
    return rhs * lhs;
}

template <typename T, unsigned N>
vec<T, N> operator/(vec<T, N> lhs, vec<T, N> rhs) {
    vec<T, N> result;
    for(unsigned i = 0; i < N; ++i) {
        result[i] = lhs[i] / rhs[i];
    }
    return result;
}

template <typename T, typename U, unsigned N>
vec<T, N> operator/(vec<T, N> lhs, U rhs) {
    vec<T, N> result;
    for(unsigned i = 0; i < N; ++i) {
        result[i] = lhs[i] / rhs;
    }
    return result;
}

template <typename T, typename U, unsigned N>
vec<T, N> operator/(U lhs, vec<T, N> rhs) {
    vec<T, N> result;
    for(unsigned i = 0; i < N; ++i) {
        result[i] = lhs / rhs[i];
    }
    return result;
}

template <typename T, unsigned N>
bool operator==(vec<T, N> lhs, vec<T, N> rhs) {
    for(unsigned i = 0; i < N; ++i) 
        if(lhs[i] != rhs[i]) return false;
    return true;
}

template <typename T, unsigned N>
std::ostream& operator<<(std::ostream& os, vec<T, N> v) {
    os << "(";
    for(unsigned i = 0; i < N; ++i) {
        os << v[i];
        if(i < N - 1) os << ", ";
    }
    os << ")";
    return os;
}


#define VEC_TYPE(T, N, C) \
    typedef vec<T, C> N##vec##C;

#define VEC_TYPES(T, N) \
    VEC_TYPE(T, N, 2) \
    VEC_TYPE(T, N, 3) \
    VEC_TYPE(T, N, 4)

VEC_TYPES(float,)
VEC_TYPES(double, d)
VEC_TYPES(int, i)
VEC_TYPES(unsigned, u)
VEC_TYPES(bool, b)
VEC_TYPES(uint8_t, u8)
VEC_TYPES(uint16_t, u16)
VEC_TYPES(uint32_t, u32)
VEC_TYPES(uint64_t, u64)
VEC_TYPES(int8_t, i8)
VEC_TYPES(int16_t, i16)
VEC_TYPES(int32_t, i32)
VEC_TYPES(int64_t, i64)

#undef VEC_TYPES
#undef VEC_TYPE

} // namespace ik
