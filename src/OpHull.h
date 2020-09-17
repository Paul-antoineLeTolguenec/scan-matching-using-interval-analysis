#ifndef __OPHULL_H__
#define __OPHULL_H__

#include <ibex.h>
#include <tubex.h>
#include <tubex-rob.h>
/* #include <tubex-3rd.h> */

class OpHull  {
    public:
        OpHull();
        IntervalVector eval(std::vector<ibex::IntervalVector>& assobox);

    
};


#endif //__OPHULL_H__
