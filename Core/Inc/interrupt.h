#ifndef INTERRUPT_H
#define INTERRUPT_H

#include "gpio.h"

class Switch
{
    private:
        bool interrupt;

    public:
        Switch();
        ~Switch();
        void read_state();
        bool get_state();

};

#endif