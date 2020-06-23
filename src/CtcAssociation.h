#ifndef __CTCASSOCIATION_H__
#define __CTCASSOCIATION_H__

#include <ibex.h>
#include <tubex.h>
#include <tubex-rob.h>
#include <tubex-3rd.h>

class CtcAssociation : public ibex::Ctc {
    public:
        CtcAssociation(const std::vector<ibex::IntervalVector>& map): ibex::Ctc(2), m_map(map){}
        void contract(ibex::IntervalVector& a);

    protected:
        const std::vector<ibex::IntervalVector> m_map;
};


#endif //__CTCASSOCIATION_H__
