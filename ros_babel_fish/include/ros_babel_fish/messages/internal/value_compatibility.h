// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef ROS_BABEL_FISH_VALUE_COMPATIBILITY_H
#define ROS_BABEL_FISH_VALUE_COMPATIBILITY_H

#include <limits>

namespace ros_babel_fish
{
//! Internal namespace not for public use, may change at any time.
namespace internal
{
// Backport of C++20's std::remove_cvref
template<class T>
struct rm_cvref
{
  typedef typename std::remove_cv<typename std::remove_reference<T>::type>::type type;
};
template<class T>
using rm_cvref_t = typename rm_cvref<T>::type;

// is_integral is necessary to avoid a CLang tidy warning
template<typename T, typename U>
typename std::enable_if<
  std::is_integral<T>::value &&
  std::numeric_limits<T>::is_signed && !std::numeric_limits<U>::is_signed, bool>::type
constexpr inBounds( const T &val )
{
  return val >= 0 && static_cast<typename std::make_unsigned<T>::type >( val ) <= std::numeric_limits<U>::max();
}

template<typename T, typename U>
typename std::enable_if<
  std::is_integral<T>::value &&
  !std::numeric_limits<T>::is_signed && std::numeric_limits<U>::is_signed &&
  std::numeric_limits<T>::digits >= std::numeric_limits<U>::digits, bool>::type
constexpr inBounds( const T &val )
{
  return val <= static_cast<T>(std::numeric_limits<U>::max());
}

template<typename T, typename U>
typename std::enable_if<
  std::is_integral<T>::value &&
!std::numeric_limits<T>::is_signed && std::numeric_limits<U>::is_signed &&
std::numeric_limits<T>::digits < std::numeric_limits<U>::digits, bool>::type
constexpr inBounds( const T &val )
{
  return static_cast<U>(val) <= std::numeric_limits<U>::max();
}

template<typename T, typename U>
typename std::enable_if<
  std::is_integral<T>::value &&
  std::numeric_limits<T>::is_signed == std::numeric_limits<U>::is_signed, bool>::type
constexpr inBounds( const T &val )
{
  return std::numeric_limits<U>::min() <= val && val <= std::numeric_limits<U>::max();
}

template<typename T, typename U>
typename std::enable_if<std::is_floating_point<T>::value, bool>::type
constexpr inBounds( const T &val )
{
  return static_cast<T>(std::numeric_limits<U>::min()) <= val && val <= static_cast<T>(std::numeric_limits<U>::max());
}

/**
 * Returns true if type T can always be stored in U without overflowing.
 */
template<typename T, typename U>
typename std::enable_if<std::is_arithmetic<T>::value && std::is_arithmetic<U>::value, bool>::type
constexpr isCompatible()
{
  // See https://en.cppreference.com/w/cpp/types/numeric_limits/digits
  // + 1 added because ::digits returns nbits-1 for signed types.
  return std::is_same<rm_cvref_t<T>, rm_cvref_t<U>>::value ||
         std::is_floating_point<U>::value ||
         (std::is_integral<T>::value &&
          !(std::is_signed<T>::value && std::is_unsigned<U>::value) &&
          (std::numeric_limits<T>::digits + 1 < std::numeric_limits<U>::digits));
}

}
}

#endif //ROS_BABEL_FISH_VALUE_COMPATIBILITY_H
