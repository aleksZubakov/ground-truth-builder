#pragma once
#ifndef HEDGEHOG_PROXY_H
#define HEDGEHOG_PROXY_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#include <unistd.h>
#include <signal.h>

#include <stdio.h>
#include "marvelmind.h"

class HedgehogProxy
{
public:
    HedgehogProxy();
    ~HedgehogProxy();
    PositionValue get_data();

    bool terminationRequired() { return hedge->terminationRequired; }
private:
    struct MarvelmindHedge* hedge;
};

#endif // HEDGEHOG_PROXY_H

