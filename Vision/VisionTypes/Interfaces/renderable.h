#ifndef RENDERABLE_H
#define RENDERABLE_H

#include <opencv2/core/core.hpp>

class Renderable {
public:
    virtual ~Renderable() {}
    virtual void render(cv::Mat& mat) const = 0;
};

#endif // RENDERABLE_H
