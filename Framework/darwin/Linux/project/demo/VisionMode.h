/*
 * VisionMode.h
 *
 *  Created on: 2011. 1. 21.
 *      Author: zerom
 */

#ifndef VISIONMODE_H_
#define VISIONMODE_H_

namespace Robot
{

class VisionMode
{
public:
    enum {
        RED     = 1,
        YELLOW  = 2,
        BLUE    = 4
    };

    static void Play(int color);
};

}

#endif /* VISIONMODE_H_ */
