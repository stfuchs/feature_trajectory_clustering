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
      ColorRGB() : r(220),g(220),b(220) {}
      ColorRGB(uint8_t _r, uint8_t _g, uint8_t _b) : r(_r),g(_g),b(_b) {}
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

    const int N = 17;
    const ColorRGB palette[17] = {
      {231, 76, 60},
      {46, 204, 113},
      {155, 89, 182},
      {241, 196, 15},
      {52, 152, 219},
      {52, 73, 94},
      {230, 126, 34},
      {26, 188, 156},
      {224, 246, 53},
      {149, 165, 166},
      {245, 160, 167},
      {218, 202, 143},
      {141, 177, 115},
      {199, 74, 108},
      {101, 70, 65},
      {54, 99, 120},
      {146, 39, 41}
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
