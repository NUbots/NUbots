#ifndef PIXELS_H
#define PIXELS_H

namespace pixels
{
//! Available colour spaces in which an image can be stored.
	enum ColourSpace 
	{
		RGB,
		YUYV,
		UNKNOWN
	};

	/**
	* The union defines a pixel in YCbCr space.
	*/
	union Pixel
	{
		unsigned color; /**< Representation as single machine word. */
		unsigned char channels[4];  /**< Representation as an array of channels. */
		struct
		{
		unsigned char yCbCrPadding, /**< Y channel. */
                    cb, /**< Cb channel. */
                    y, /**< Ignore. */
                    cr; /**< Cr channel. */
    };
    struct
    {
      unsigned char r, /**< R channel. */
                    g, /**< G channel. */
                    b, /**< B channel. */
                    rgbPadding; /**< Ignore. */
    };
    struct
    {
      unsigned char h, /**< H channel. */
                    s, /**< S channel. */
                    v, /**< V channel. */
                    hsvPadding; /**< Ignore. */
    };
  };
};
#endif
