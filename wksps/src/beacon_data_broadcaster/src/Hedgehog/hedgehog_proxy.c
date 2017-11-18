#include "hedgehog_proxy.h"


HedgehogProxy::HedgehogProxy() 
{
    puts("Start proxy...\n");
    
    const char* ttyFileName = DEFAULT_TTY_FILENAME; // todo... get as parameter
    hedge = createMarvelmindHedge();

    if (!hedge)
    {
        puts ("Error: Unable to create MarvelmindHedge");
        return;
    }

    hedge->ttyFileName=ttyFileName;
    hedge->verbose=true; // show errors and warnings
    startMarvelmindHedge (hedge);
}
HedgehogProxy::~HedgehogProxy()
{
    if (hedge)
    {
        stopMarvelmindHedge(hedge);
        destroyMarvelmindHedge(hedge);
        if (hedge) delete hedge;
        hedge = 0;
    }
}
PositionValue HedgehogProxy::get_data()
{
    struct PositionValue position;
    position.ready = false;

    while (!position.ready) {
    bool onlyNew = true;
    uint8_t i,j;
    double xm,ym,zm;
    if (hedge->haveNewValues_ || (!onlyNew))
    {        
        uint8_t real_values_count=hedge->maxBufferedPositions;
        uint8_t addresses[real_values_count];
        uint8_t addressesNum= 0;

        for(i=0;i<real_values_count;i++)
        {
           uint8_t address= hedge->positionBuffer[i].address;
           bool alreadyProcessed= false;
           if (addressesNum != 0)
                for(j=0;j<addressesNum;j++)
                {
                    if (address == addresses[j])
                    {
                        alreadyProcessed= true;
                        break;
                    }
               }
            if (alreadyProcessed)
                continue;
            addresses[addressesNum++]= address;

            getPositionFromMarvelmindHedgeByAddress (hedge, &position, address);
            xm= ((double) position.x)/1000.0;
            ym= ((double) position.y)/1000.0;
            zm= ((double) position.z)/1000.0;
            if (position.ready)
            {
                /*
                if (position.highResolution)
                {
                    printf ("Address: %d, X: %.3f, Y: %.3f, Z: %.3f at time T: %u\n",
                            position.address, xm, ym, zm, position.timestamp);
                } else
                {
                    printf ("Address: %d, X: %.2f, Y: %.2f, Z: %.2f at time T: %u\n",
                            position.address, xm, ym, zm, position.timestamp);
                }
                */

                return position;
            }
            hedge->haveNewValues_=false;
        }
    }
    }
    return position;
}


