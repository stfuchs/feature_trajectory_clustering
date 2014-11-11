/**
 * @file   utils.h
 * @author Steffen Fuchs <steffenfuchs@samson.informatik.uni-stuttgart.de>
 * @date   Thu Nov  6 17:44:04 2014
 * 
 * @brief  utilities for visualization
 */

#ifndef VISUALIZATION_UTILS_H
#define VISUALIZATION_UTILS_H

namespace Visualization
{
  namespace Utils
  {
    struct ColorRGB
    {
      uint8_t r;
      uint8_t g;
      uint8_t b;
    };

    struct ColorHSV
    {
      uint16_t h;
      uint8_t s;
      uint8_t v;
    };

    template<typename T> inline T min3 (const T& a, const T& b, const T& c) {
      return (a<b ? std::min<T>(a,c) : std::min<T>(b,c));
    }

    template<typename T> inline T max3 (const T& a, const T& b, const T& c) {
      return (a>b ? std::max<T>(a,c) : std::max<T>(b,c));
    }


    void rgb2hsv(const ColorRGB& i, ColorHSV& o);

    void hsv2rgb(const ColorHSV& i, ColorRGB& o);

    void generateColor(int offset, ColorRGB& out);
  }
}

#include "mlr_visualization/impl/utils.hpp"

#endif
