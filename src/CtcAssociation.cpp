#include "CtcAssociation.h"
#include "OpHull.h"
void CtcAssociation::contract(ibex::IntervalVector& a) {
    IntervalVector out = IntervalVector::empty(2);
    vector<IntervalVector> asso;
    OpHull hull;
    IntervalVector hull_box;
    IntervalVector contracted=a;
    for(auto const& box: m_map){
        if (a.intersects(box))
        {
            asso.push_back(box);
        }
    }
    hull_box=hull.eval(asso);
    contracted[0]=Interval(hull_box[0].lb(),a[0].ub());
    /* i++;
    cout<<i<<endl; */
    a=a&contracted;
    
}