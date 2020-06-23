#include <ibex.h>
#include <tubex.h>
#include <tubex-rob.h>
#include <tubex-3rd.h>
#include <iostream>
#include "CtcAssociation.h"
#include <random>

using namespace std;
using namespace ibex;
using namespace tubex;


int main(int argc, char** argv) {
  /* ================================================= CREATING DATA ============================================================ */
  IntervalVector map_area(2, Interval(-20.,20.));

  // Truth (unknown pose)
  Vector truth1(3,0.);
  truth1[2] = M_PI/6.; // heading

/* =========== CREATING LANDMARKS =========== */
    int nb_marks=100;
    //let's create 2 wall
    vector<IntervalVector> upwall(nb_marks);
    vector<IntervalVector> bwall(nb_marks);
    float i=0;
    for (auto& mark : upwall){ //landmark from -10 to 10
      //upwall
      mark=IntervalVector(2);
      mark[0]=Interval(-20+i/2.5);
      mark[1]=Interval(2*sin(-M_PI+(i/nb_marks)*2*M_PI)+10);
      i++;
      } 
    i=0;
    for (auto& mark : bwall){ //landmark from -10 to 10
      //bwall
      mark=IntervalVector(2);
      mark[0]=Interval(-20+i/2.5);
      mark[1]=Interval(2*sin(-M_PI+(i/nb_marks)*2*M_PI)-10);
      i++;
      } 
    //let's create a vector containing all the landmarks
    vector<IntervalVector> v_b =upwall;
    v_b.insert( v_b.end(), bwall.begin(), bwall.end() );

/* =========== OBSERVATION INSTANT T1 =========== */
    int nb_observations=50;


    // Creating  landmarks observations at t1 for the two walls
   /*  Interval visi_range(0.,100.); // [0m,75m]
    Interval visi_angle=Interval(-M_PI/4.+M_PI/2,M_PI/4.+M_PI/2)|Interval(-M_PI/4.-M_PI/2,M_PI/4.-M_PI/2); // frontal sonar
     */

    //Keep only some random observations:
    vector<IntervalVector> obs_t1(nb_observations);
    for (auto& obs : obs_t1)
    {
      std::random_device seed ;
      // generator 
      std::mt19937 engine( seed( ) ) ;
      // number distribution
      std::uniform_int_distribution<int> choose( 0 , v_b.size( ) - 1 ) ;
      obs= v_b[ choose( engine ) ] ;
    }

    vector<IntervalVector> v_obs_t1 = DataLoader::generate_observations(truth1, obs_t1); //, true, visi_range, visi_angle); if you to keep the sonar range

    //box inflating due to sonar loss
    for(auto& obs : v_obs_t1) {
      obs[0].inflate(0.2); // range
      obs[1].inflate(0.025); // bearing
    }

/* =========== LOCALIZATION PROCESSING T1 =========== */
    /* all the sensor information is in v_obs_t1 */
    IntervalVector x1(3);
    //Let's consider that at T1 the position is known with an accuracy of one meter
    x1[0] = Interval(truth1[0]-0.5,truth1[0]+0.5);
    x1[1] = Interval(truth1[1]-0.5,truth1[1]+0.5);
    x1[2] = Interval(truth1[2]);

    /* ------------TO DO--------------- */
    
    












    
/* =========== OBSERVATION INSTANT T2 =========== */
    /* first updating of the pose */
    // Truth (unknown pose)
    Vector truth2(3,0.);
    truth2[0] = 6.;
    truth2[1] = -1.; // y
    truth2[2] = 0.; // heading

    //True displacement qtrue
    Vector qtrue(3,0.);
    qtrue[0] =truth2[0]-truth1[0] ;
    qtrue[1] =truth2[1]-truth1[1] ; 
    qtrue[2] =truth2[2]-truth1[2] ; 
    



    nb_observations=50;
    // Creating  landmarks observations at t1 for the two walls
   /*  Interval visi_range(0.,100.); // [0m,75m]
    Interval visi_angle=Interval(-M_PI/4.+M_PI/2,M_PI/4.+M_PI/2)|Interval(-M_PI/4.-M_PI/2,M_PI/4.-M_PI/2); // frontal sonar
     */

    //Keep only some random observations:
    vector<IntervalVector> obs_t2(nb_observations);
    for (auto& obs : obs_t2)
    {
      std::random_device seed ;
      // generator 
      std::mt19937 engine( seed( ) ) ;
      // number distribution
      std::uniform_int_distribution<int> choose( 0 , v_b.size( ) - 1 ) ;
      obs= v_b[ choose( engine ) ] ;
    }
  //the following line is used to generate the observation given the pose of the robot 
  //you can use the extension of the function in order to limit the bearing range as it is in the reality.
    vector<IntervalVector> v_obs_t2 = DataLoader::generate_observations(truth2, obs_t2); //, true, visi_range, visi_angle); if you to keep the sonar range

    //box inflating due to sonar loss
    for(auto& obs : v_obs_t2) {
      obs[0].inflate(0.2); // range
      obs[1].inflate(0.025); // bearing
    }


/* The IMU measurement gives us the evolution q of the robot between t1 and t2 */
  IntervalVector q(3);
  q[0]=Interval(-1,1)+qtrue[0] ;
  q[1]=Interval(-1,1)+qtrue[1] ; 
  q[2]=Interval(-M_PI/8,M_PI/8)+qtrue[2] ; 











/* =========== LOCALIZATION PROCESSING T2 =========== */
    /* all the sensor information is in v_obs_t2 */

/* The IMU gives us an approximation of the displacement, so the position of the robot at T2 is known but with a loss of accuracy */
/* By integrating with an Euler method we have : */
IntervalVector x2(3);
x2[0]=x1[0]+q[0];
x2[1]=x1[1]+q[1];
x2[2]=x1[2]+q[2];

/*========================= TO DO ===============================================*/


    /* =========== GRAPHICS =========== */

    vibes::beginDrawing();
    VIBesFigMap fig_map("Map");
    fig_map.set_properties(10, 10, 600, 600);
    for(const auto& iv : v_b){
        fig_map.add_beacon(Beacon(iv), 0.2);}
    /* observation t1 */
    fig_map.add_observations(v_obs_t1, truth1);
    for(int i = 0 ; i < obs_t1.size() ; i++){
        fig_map.add_beacon(obs_t1[i], 0.2, "##A50108[#A50108]");}
    fig_map.draw_box(x1.subvector(0,1), "#2980b9[#ECF0F1]"); // estimated position (2d box)
    fig_map.draw_vehicle(truth1, 1.);
    
    /* observation t2 */
    fig_map.add_observations(v_obs_t2, truth2);
    for(int i = 0 ; i < obs_t2.size() ; i++){
        fig_map.add_beacon(obs_t2[i], 0.2, "#00A53B[#00A53B]");}
    fig_map.draw_box(x2.subvector(0,1), "#2980b9[#ECF0F1]"); // estimated position (2d box)
    fig_map.draw_vehicle(truth2, 1.);

    fig_map.axis_limits(map_area);
    fig_map.show();
    vibes::endDrawing();
  /* =========== ENDING =========== */

    return EXIT_SUCCESS;
    
}




