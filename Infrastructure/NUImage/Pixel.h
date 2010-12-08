#ifndef PIXEL_H
#define PIXEL_H
union Pixel
{
        unsigned color;             //!< Representation as single machine word.
        unsigned char channel[4];   //!< Representation as an array of channels.
	struct
	{
		unsigned char	yCbCrPadding,	//!< Ignore
                                cb,		//!< Cb channel.
                                y,		//!< Y channel.
                                cr;             //!< Cr channel.
	};
};
#endif
