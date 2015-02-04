/**
 * @file   utils.hpp
 * @author Steffen Fuchs <steffenfuchs@samson.informatik.uni-stuttgart.de>
 * @date   Thu Nov  6 17:49:55 2014
 * 
 * @brief  utilities for visualization
 */



void Visualization::Utils::rgb2hsv(const ColorRGB& i, ColorHSV& o)
{
  uint8_t rgb_min = min3(i.r, i.g, i.b);
  uint8_t rgb_max = max3(i.r, i.g, i.b);
  uint8_t delta = rgb_max - rgb_min;
  o.v = rgb_max;
  if (o.v == 0) { o.s = o.h = 0; return; }

  o.s = round(float(delta) / float(o.v) * 100.f);
  o.v /= 2.55f;
  float h_tmp;
  if (o.s == 0) { o.h = 0; return; }

  if (i.r==rgb_max)     { h_tmp =       float(i.g - i.b)/float(delta); }
  else if (i.g==rgb_max){ h_tmp = 2.f + float(i.b - i.r)/float(delta); }
  else                  { h_tmp = 4.f + float(i.r - i.g)/float(delta); }

  h_tmp *= 60.f;
  if (h_tmp < 0) o.h = h_tmp + 360;
  else o.h = h_tmp;
}

void Visualization::Utils::hsv2rgb(const ColorHSV& i, ColorRGB& o)
{
  if (i.s==0) { o.r = o.g = o.b = i.v*2.55f; return; }

  float hh = i.h / 60.f; // sector 0..5
  uint8_t idx = floor (hh);
  float f = hh - idx;
  uint8_t v = round (i.v * 2.55f);
  uint8_t p = v * (100.f - i.s) * 0.01f;
  uint8_t q = v * (100.f - i.s * f) * 0.01f;
  uint8_t t = v * (100.f - i.s * (1.f - f)) * 0.01f;
  switch (idx)
  {
  case 0: { o.r = v; o.g = t; o.b = p; break; }
  case 1: { o.r = q; o.g = v; o.b = p; break; }
  case 2: { o.r = p; o.g = v; o.b = t; break; }
  case 3: { o.r = p; o.g = q; o.b = v; break; }
  case 4: { o.r = t; o.g = p; o.b = v; break; }
  default:{ o.r = v; o.g = p; o.b = q; break; }
  }
}

void Visualization::Utils::generateColor(int position, ColorRGB& out)
{
  int wheel_step = position % 3;
  int mode = position / 3;
  int h = (wheel_step + mode % 2) * 60;
  int v = 100;
  int s = 100;
  if ((position % 12)>=6) h += 30;
  if (position > 12)
  {
    s = 75;
  }
  if (position > 24)
  {
    v = 75;
    s = 100;
  }
  if (position > 36)
  {
    h = rand()%360;
    v = 25+rand()%75;
    s = 25+rand()%75;
  }
  ColorHSV hsv = {uint16_t(h),uint8_t(s),uint8_t(v)};
  hsv2rgb(hsv,out);
}
