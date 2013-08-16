#include "ColorModelConversions.h"

#include <cmath>
namespace utility {
namespace image {
     RGB toRGB(YCbCr ycbcr) {
        unsigned char Y = ycbcr.Y;
        unsigned char Cr = ycbcr.Cr;
        unsigned char Cb = ycbcr.Cb;

        float r = Y + (1.4075 * (Cr - 128));;
        float g = Y - (0.3455 * (Cb - 128)) - (0.7169 * (Cr - 128));
        float b = Y + (1.7790 * (Cb - 128));

        if(r < 0) r = 0; else if(r > 255) r = 255;
        if(g < 0) g = 0; else if(g > 255) g = 255;
        if(b < 0) b = 0; else if(b > 255) b = 255;

        RGB rgb;
        rgb.r = (unsigned char) std::rint(r);
        rgb.g = (unsigned char) std::rint(g);
        rgb.b = (unsigned char) std::rint(b);

        return rgb;
    }

     RGB toRGB(HSV hsv) {
        unsigned char H = hsv.h;
        unsigned char S = hsv.s;
        unsigned char V = hsv.v;

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

        RGB rgb;
        rgb.r = (unsigned char) std::rint(r);
        rgb.g = (unsigned char) std::rint(g);
        rgb.b = (unsigned char) std::rint(b);

        return rgb;
        #undef RGB_WRITE
    }

     HSV toHSV(RGB rgb) {
        unsigned char R = rgb.r;
        unsigned char G = rgb.g;
        unsigned char B = rgb.b;

        float h, s, v, hdeg=0;
        unsigned char min, max, maxDiff;

        min = R;
        if(min > G) min = G;
        if(min > B) min = B;

        max = R;
        if(max < G) max = G;
        if(max < B) max = B;

        maxDiff = max - min;

        if(min == max) {
          hdeg = 0.0;
        } else if (max == R) {
          hdeg = fmod((60.0 * (G - B) / maxDiff + 360.0), 360.0);
        } else if (max == G) {
          hdeg = (60.0 * (B - R) / maxDiff + 120.0);
        } else if (max == B) {
          hdeg = (60.0 * (R - G) / maxDiff + 240.0);
        }

        h = hdeg * 255.0 / 360.0;

        if(max == 0) {
          s = 0.0;
        } else {
          s = (1.0 - (float)min / (float)max) * 255;
        }

        v = max;

        if(h < 0) h = 0; else if(h > 255) h = 255;
        if(s < 0) s = 0; else if(s > 255) s = 255;
        if(v < 0) v = 0; else if(v > 255) v = 255;

        HSV hsv;
        hsv.h = (unsigned char) std::rint(h);
        hsv.s = (unsigned char) std::rint(s);
        hsv.v = (unsigned char) std::rint(v);
        return hsv;
    }

     HSV toHSV(YCbCr ycbcr) {
        RGB rgb = toRGB(ycbcr);
        return toHSV(rgb);
    }

     YCbCr toYCbCr(RGB rgb) {
        unsigned char R = rgb.r;
        unsigned char G = rgb.g;
        unsigned char B = rgb.b;

        float y = (R * .299) + (G * .587) + (B * .114);
        float cb = (R * -.169) + (G * -.332) + (B * .500) + 128;
        float cr = (R * .500) + (G * -.419) + (B * -.0813) + 128;
        if(y < 0) y = 0; else if(y > 255) y = 255;
        if(cb < 0) cb = 0; else if(cb > 255) cb = 255;
        if(cr < 0) cr = 0; else if(cr > 255) cr = 255;

        YCbCr ycbcr;
        ycbcr.Y = (unsigned char) std::rint(y);
        ycbcr.Cb = (unsigned char) std::rint(cb);
        ycbcr.Cr = (unsigned char) std::rint(cr);
        return ycbcr;
    }

     YCbCr toYCbCr(HSV hsv) {
        RGB rgb = toRGB(hsv);
        return toYCbCr(rgb);
    }
}
}
