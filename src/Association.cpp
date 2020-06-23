#include <ibex.h>
#include <tubex.h>
#include <tubex-rob.h>
#include <tubex-3rd.h>
#include <iostream>
#include "CtcAssociation.h"

using namespace std;
using namespace ibex;
using namespace tubex;

int main(int argc, char** argv) {
    /* =========== Association Tests ===========*/

    vector<ibex::IntervalVector> map;
    ibex::IntervalVector b1{{2,3},{2,3}};
    ibex::IntervalVector b2{{5,6},{4,5}};
    ibex::IntervalVector b3{{9,10},{2,3}};
    map.push_back(b1);
    map.push_back(b2);
    map.push_back(b3);

    ibex::IntervalVector a1{{0,4},{0,4}};
    ibex::IntervalVector a2{{3.5,11},{1,6}};
    ibex::IntervalVector a3{{4.5,8},{-0.5,3}};

    // =========== Graphics ===========

    vibes::beginDrawing();
    VIBesFigMap fig_asso("Association");
    fig_asso.set_properties(50,50,1000,600);
    ibex::IntervalVector win_area{{-1,11},{-1, 7}};
    fig_asso.axis_limits(win_area);

    fig_asso.draw_box(b1, "#2c3e50[#7f8c8d]");
    fig_asso.draw_box(b2, "#2c3e50[#7f8c8d]");
    fig_asso.draw_box(b3, "#2c3e50[#7f8c8d]");

    fig_asso.draw_box(a1, "#2980b9");
    fig_asso.draw_box(a2, "#2980b9");
    fig_asso.draw_box(a3, "#2980b9");

    CtcAssociation ctc_asso(map);
    ctc_asso.contract(a1);
    ctc_asso.contract(a2);

    fig_asso.draw_box(a1, "#FF5D00");
    fig_asso.draw_box(a2, "#FF5D00");
    fig_asso.show();
    vibes::endDrawing();
    
  /* =========== ENDING =========== */

    return EXIT_SUCCESS;
}