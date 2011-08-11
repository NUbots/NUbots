/*!
  @file ColorModelConversions.h
  @author Steven Nicklin
  @brief Declaration and implementation of class ColorModelConversions.
  */

#ifndef __ColorModelConversions_h_
#define __ColorModelConversions_h_

#include "Tools/Math/General.h"

/*!
    @brief A class that defines static conversions between color models for single pixels.

    Colour conversions are available between each colour space YCbCR,
    RGB and HSV into any of the other colour spaces mentioned.

    Input and outputs all have 8-bits of precision, however all of the
    conversion are done using float values so as to minimise the loss of
    precision during the conversion. However the limitation on the input and output
    values still restict the accuracy of the conversions and therefore the subsequent
    result of converting from one colour space to another and then back to the
    original colour space may still result in a different answer.
  */
class ColorModelConversions
{
public:
  /*!
    @brief Converts an YCbCr pixel into an RGB pixel.

    From: http://softpixel.com/~cwright/programming/colorspace/yuv/
    @param Y The Y channel of the source pixel.
    @param Cb The Cb channel of the source pixel.
    @param Cr The Cr channel of the source pixel.
    @param R The R channel of the target pixel.
    @param G The G channel of the target pixel.
    @param B The B channel of the target pixel.
   */
  static void fromYCbCrToRGB(unsigned char Y,
                             unsigned char Cb,
                             unsigned char Cr,
                             unsigned char& R,
                             unsigned char& G,
                             unsigned char& B)
  {
    float r = Y + (1.4075 * (Cr - 128));;
    float g = Y - (0.3455 * (Cb - 128)) - (0.7169 * (Cr - 128));
    float b = Y + (1.7790 * (Cb - 128));

    if(r < 0) r = 0; else if(r > 255) r = 255;
    if(g < 0) g = 0; else if(g > 255) g = 255;
    if(b < 0) b = 0; else if(b > 255) b = 255;
    R = (unsigned char) mathGeneral::roundNumberToInt(r);
    G = (unsigned char) mathGeneral::roundNumberToInt(g);
    B = (unsigned char) mathGeneral::roundNumberToInt(b);
  }

  /*!
    @brief Converts an RGB pixel into an YCbCr pixel.

    From: http://softpixel.com/~cwright/programming/colorspace/yuv/
    @param R The R channel of the source pixel.
    @param G The G channel of the source pixel.
    @param B The B channel of the source pixel.
    @param Y The Y channel of the target pixel.
    @param Cb The Cb channel of the target pixel.
    @param Cr The Cr channel of the target pixel.
    */
  static void fromRGBToYCbCr(unsigned char R, unsigned char G, unsigned char B, unsigned char& Y, unsigned char& Cb, unsigned char& Cr)
  {
    float y = (R * .299) + (G * .587) + (B * .114);
    float cb = (R * -.169) + (G * -.332) + (B * .500) + 128;
    float cr = (R * .500) + (G * -.419) + (B * -.0813) + 128;
    if(y < 0) y = 0; else if(y > 255) y = 255;
    if(cb < 0) cb = 0; else if(cb > 255) cb = 255;
    if(cr < 0) cr = 0; else if(cr > 255) cr = 255;

    Y = (unsigned char) mathGeneral::roundNumberToInt(y);
    Cb = (unsigned char) mathGeneral::roundNumberToInt(cb);
    Cr = (unsigned char) mathGeneral::roundNumberToInt(cr);
  }

   /*!
    @brief Converts an RGB pixel into an HSV pixel.

    From: http://en.wikipedia.org/wiki/HSL_and_HSV
    @param R The R channel of the source pixel.
    @param G The G channel of the source pixel.
    @param B The B channel of the source pixel.
    @param H The H channel of the target pixel.
    @param S The S channel of the target pixel.
    @param V The V channel of the target pixel.
    */
  static void fromRGBToHSV(unsigned char R,
                             unsigned char G,
                             unsigned char B,
                             unsigned char& H,
                             unsigned char& S,
                             unsigned char& V)
  {
      float h, s, v, hdeg=0;
      unsigned char min, max, maxDiff;

      min = R;
      if(min > G) min = G;
      if(min > B) min = B;

      max = R;
      if(max < G) max = G;
      if(max < B) max = B;

      maxDiff = max - min;

      if(min == max)
      {
          hdeg = 0.0;
      }
      else if (max == R)
      {
          hdeg = fmod((60.0 * (G - B) / maxDiff + 360.0), 360.0);
      }
      else if (max == G)
      {
          hdeg = (60.0 * (B - R) / maxDiff + 120.0);
      }
      else if (max == B)
      {
          hdeg = (60.0 * (R - G) / maxDiff + 240.0);
      }

      h = hdeg * 255.0 / 360.0;

      if(max == 0)
      {
          s = 0.0;
      }
      else
      {
          s = (1.0 - (float)min / (float)max) * 255;
      }

      v = max;

    if(h < 0) h = 0; else if(h > 255) h = 255;
    if(s < 0) s = 0; else if(s > 255) s = 255;
    if(v < 0) v = 0; else if(v > 255) v = 255;
    H = (unsigned char) mathGeneral::roundNumberToInt(h);
    S = (unsigned char) mathGeneral::roundNumberToInt(s);
    V = (unsigned char) mathGeneral::roundNumberToInt(v);
  }

   /*!
    @brief Converts an HSV pixel into an RGB pixel.

    From: http://en.wikipedia.org/wiki/HSL_and_HSV
    @param H The H channel of the source pixel.
    @param S The S channel of the source pixel.
    @param V The V channel of the source pixel.
    @param R The R channel of the target pixel.
    @param G The G channel of the target pixel.
    @param B The B channel of the target pixel.
    */
  static void fromHSVToRGB( unsigned char H,
                            unsigned char S,
                            unsigned char V,
                            unsigned char& R,
                            unsigned char& G,
                            unsigned char& B)
  {
    #define RGB_WRITE(_r,_g,_b) r=_r; g=_g; b=_b;

    float r, g, b, hDeg;
    float f, p, q, t;
    hDeg = 360.0 / 255.0 * (float)H;
    float hfloor = floor(hDeg / 60.0);
      int hi = (int)(std::fmod(hfloor, 6.0f));
    f = (hDeg / 60 - hfloor);
    p = V * (255 - S) / 255.0;
    q = V * (255 - f*S) / 255.0;
    t = V * (255 - (1.0 - f)*S) / 255.0;
    switch(hi)
    {
        case 0: RGB_WRITE(V,t,p);
                break;
        case 1: RGB_WRITE(q,V,p);
                break;
        case 2: RGB_WRITE(p,V,t);
                break;
        case 3: RGB_WRITE(p,q,V);
                break;
        case 4: RGB_WRITE(t,p,V);
                break;
        case 5: RGB_WRITE(V,p,q);
                break;
        default:
                RGB_WRITE(0,0,0);
                break;
    };
    if(r < 0) r = 0; else if(r > 255) r = 255;
    if(g < 0) g = 0; else if(g > 255) g = 255;
    if(b < 0) b = 0; else if(b > 255) b = 255;
    R = (unsigned char) mathGeneral::roundNumberToInt(r);
    G = (unsigned char) mathGeneral::roundNumberToInt(g);
    B = (unsigned char) mathGeneral::roundNumberToInt(b);
    return;
    #undef RGB_WRITE
  }

   /*!
    @brief Converts an HSI pixel into an YCbCr pixel.
    @param H The H channel of the source pixel.
    @param S The S channel of the source pixel.
    @param V The V channel of the source pixel.
    @param Y The Y channel of the target pixel.
    @param Cb The Cb channel of the target pixel.
    @param Cr The Cr channel of the target pixel.
    */
static void fromHSVToYCbCr(unsigned char H,
                             unsigned char S,
                             unsigned char V,
                             unsigned char& Y,
                             unsigned char& Cb,
                             unsigned char& Cr)
{
    unsigned char r, g, b;
    fromHSVToRGB(H, S, V, r, g, b);
    fromRGBToYCbCr(r, g, b, Y, Cb, Cr);
    return;
}


  /*!
    @brief Converts an HSI pixel into an YCbCr pixel.
    @param H The H channel of the source pixel.
    @param S The S channel of the source pixel.
    @param V The V channel of the source pixel.
    @param Y The Y channel of the target pixel.
    @param Cb The Cb channel of the target pixel.
    @param Cr The Cr channel of the target pixel.
    */
static void fromYCbCrToHSV( unsigned char Y,
                            unsigned char Cb,
                            unsigned char Cr,
                            unsigned char& H,
                            unsigned char& S,
                            unsigned char& V)
{
    unsigned char r, g, b;
    fromYCbCrToRGB( Y, Cb, Cr, r, g, b);
    fromRGBToHSV(r, g, b, H, S, V);
    return;
}

};
#endif //__ColorModelConversions_h_
