#ifndef FILTER_H
#define FILTER_H

#include "framebuffer.h"

class Filter
{
public:
    virtual void filter(FrameBuffer *buffer) = 0;

};

#endif // FILTER_H
