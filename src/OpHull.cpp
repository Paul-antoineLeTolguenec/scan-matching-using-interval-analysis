#include "OpHull.h"

OpHull::OpHull(){};

IntervalVector OpHull::eval(std::vector<ibex::IntervalVector>& assobox) {
    IntervalVector a(2);
    IntervalVector out = IntervalVector::empty(2);
    for(auto const& box: assobox)
        out |= (a & box);
    a = out;
    return(a);
};