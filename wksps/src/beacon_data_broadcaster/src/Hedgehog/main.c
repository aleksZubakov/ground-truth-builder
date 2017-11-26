#include <iostream>
#include <stdexcept>
#include <ctime>

#include "hedgehog_proxy.h"

bool terminateProgram=false;
void CtrlHandler(int signum)
{
    terminateProgram=true;
}

int main()
{
    std::cout << "Starting example..." << std::endl;
    HedgehogProxy hedge_proxy; 
    
    signal (SIGINT, CtrlHandler);
    signal (SIGQUIT, CtrlHandler);


     // Main loop
    while ((!terminateProgram) && (!hedge_proxy.terminationRequired()))
    {

        PositionValue pos = hedge_proxy.get_position();
        if (pos.ready)
        {
            std::cout << "X: " << pos.x << " Y: " << pos.y << " Z: " << pos.z << std::endl; 
        }                       
    }
    return 0;
}
