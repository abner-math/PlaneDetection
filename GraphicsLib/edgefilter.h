#ifndef EDGEFILTER_H
#define EDGEFILTER_H

#include "filter.h"

class EdgeFilter : public Filter
{
public:
    void filter(FrameBuffer *buffer) override;

};

#endif // EDGEFILTER_H
