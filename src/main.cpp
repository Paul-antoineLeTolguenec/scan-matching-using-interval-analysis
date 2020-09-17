#include <ibex.h>
#include <tubex.h>
#include <tubex-rob.h>
#include <tubex-3rd.h>
#include <iostream>
#include "OpHull.h"
#include <random>
#include "CtcAssociation.h"

using namespace std;
using namespace ibex;
using namespace tubex;

void SIVIA(IntervalVector q, IntervalVector x1, vector<IntervalVector> v_obs_t2, vector<vector<IntervalVector>> assot2,vector<IntervalVector>asso_hull, int dim, IntervalVector*qopt=new IntervalVector, float *Min=new float,float *Minq=new float, int k=0){
  pyibex::CtcPolar ctc_polar;
  //Let's bring t2 measures into t1 frame
  vector<IntervalVector> Bt2(v_obs_t2.size());
  Interval theta2;
  int i=0;
  for (auto& obs:Bt2)
    { obs=IntervalVector(2);
      theta2=Interval();
      theta2=v_obs_t2[i][1]+q[2]+x1[2];
      ctc_polar.contract(obs[0], obs[1], v_obs_t2[i][0], theta2);
      obs[0]=obs[0]+q[0];
      obs[1]=obs[1]+q[1];
      i++;
      }
  //Let's calculate the sum we want to minimize
  float Sum=0;
  for (int i = 0; i < Bt2.size(); i++)
  {
    /* Sum+=pow(distance(Bt2[i],asso_hull[i]),4); */
    for (int j = 0; j < assot2[i].size(); j++)
    { 
      Sum+=pow(distance(Bt2[i],assot2[i][j]),2);
    }
  }
  //Update Min value
  if (Sum<=*Min or k==0)
  { 
   /*  cout<<k<<endl; */
    *Min=Sum;
    *qopt=q;
    /* cout<<Sum<<"     "<<*qopt<<endl; */
  }
  
  // cut the boxes and do it again
  if (k<dim )
   {  
      IntervalVector q1=q;
      IntervalVector q2=q;
      q1[k%3]=Interval(q[k%3].lb(),q[k%3].mid());
      q2[k%3]=Interval(q[k%3].mid(),q[k%3].ub());
      k++;
      SIVIA(q2,x1,v_obs_t2,assot2,asso_hull,dim,qopt,Min,Minq,k);
      SIVIA(q1,x1,v_obs_t2,assot2,asso_hull,dim,qopt,Min,Minq,k);
     
   }
   
  }





















void SIVIA_Contraction(IntervalVector q, IntervalVector x1,IntervalVector x2, vector<IntervalVector> v_obs_t2, vector<IntervalVector>b_obs_t1, int dim, IntervalVector*qopt=new IntervalVector, float *Min=new float,float *Minq=new float, int k=0){
  pyibex::CtcPolar ctc_polar;
  //Let's bring t2 measures into t1 frame
  vector<IntervalVector> Bt2(v_obs_t2.size());
  Interval theta2;
  int i=0;
  for (auto& obs:Bt2)
    { obs=IntervalVector(2);
      theta2=Interval();
      theta2=v_obs_t2[i][1]+q[2]+x1[2];
      ctc_polar.contract(obs[0], obs[1], v_obs_t2[i][0], theta2);
      obs[0]=obs[0]+q[0];
      obs[1]=obs[1]+q[1];
      i++;
      }

  /* Now that they are in the same frame, let's link one box to the others. */
  /* so everybody is in the robot frame at t1 */
  OpHull hull;
  vector<vector<IntervalVector>> asso_t1at2(Bt2.size());
  vector<IntervalVector> asso_hull(Bt2.size());
  for (size_t i = 0; i < Bt2.size(); i++){
    for (size_t j = 0; j < b_obs_t1.size(); j++){
      if (Bt2[i].intersects(b_obs_t1[j]))
      {
        asso_t1at2[i].push_back(b_obs_t1[j]);
      }
      
    }  
    asso_hull[i]=hull.eval( asso_t1at2[i]);
  }

  vector<IntervalVector> box_included_into_hull(Bt2.size());
  for (int i = 0; i < Bt2.size(); i++)
  { 
      box_included_into_hull[i]=Bt2[i];
      box_included_into_hull[i][1]=asso_hull[i][1];
      if (asso_hull[i][1].lb()>x2[1].ub())
      {box_included_into_hull[i][1]=Interval(asso_hull[i][1].lb(),Bt2[i][1].ub());
      }
      else
      {
        box_included_into_hull[i][1]=Interval(Bt2[i][1].lb(),asso_hull[i][1].ub());
      }
    
  }
  
  //Let's make the contraction
  cout<<"         "<<endl;
  
  cout<<q<<endl;
 
  
  ContractorNetwork cn;
  ibex::CtcFwdBwd ctc_add(*new ibex::Function( "a", "b","c", "a+b-c"));
  vector<IntervalVector> test(Bt2.size());
  int counter=0;
  for (int i = 0; i < Bt2.size(); i++)
  {   
    if (box_included_into_hull[i][1].ub()<1000000 )
    { counter++;

      Interval& thetavar = cn.create_dom(Interval());
      Interval& theta2 = cn.create_dom(v_obs_t2[i][1]+x1[2]);
      Interval& rho = cn.create_dom( v_obs_t2[i][0]);
      IntervalVector& box = cn.create_dom(IntervalVector(2));
      IntervalVector& boxf = cn.create_dom(IntervalVector(2));
    
      cn.add(ctc_add,{theta2,q[2],thetavar});
      cn.add(ctc_polar,{box[0], box[1],rho, thetavar});
      cn.add(ctc_add,{box[1],q[1],box_included_into_hull[i][1]});
      
    }
  }
  cn.contract();
 
 cout<<q<<endl;
  
  
  // cut the boxes and do it again
  if (k<dim)
   {  
      /* IntervalVector q1=q;
      IntervalVector q2=q;
      q1[k%3]=Interval(q[k%3].lb(),q[k%3].mid());
      q2[k%3]=Interval(q[k%3].mid(),q[k%3].ub());
      k++; */
      k++;
      SIVIA_Contraction(q,x1,x2,v_obs_t2,b_obs_t1,dim,qopt,Min,Minq,k);
      /* SIVIA_Contraction(q1,x1,x2,v_obs_t2,b_obs_t1,dim,qopt,Min,Minq,k); */
     
   }
   
  }






void SIVIA_Polar(vector<IntervalVector> v_obs_t2,vector<IntervalVector> v_obs_t1,IntervalVector q,int dim,int k=0){
  
ContractorNetwork cn;
ibex::CtcFwdBwd ctc_add(*new ibex::Function( "a", "b","c", "a+b-c"));
ibex::CtcFwdBwd ctc_polX(*new ibex::Function( "xq","dnew", "w","dref","aref","xq+dnew*cos(w)-dref*cos(aref)"));
ibex::CtcFwdBwd ctc_polY(*new ibex::Function( "yq","dnew", "w","dref","aref","yq+dnew*sin(w)-dref*sin(aref)"));
ibex::CtcFwdBwd ctc_D(*new ibex::Function( "dref","xq","yq","dnew","w","dref-sqrt(((xq+dnew*cos(w))^2+(yq+dnew*sin(w))^2))"));
ibex::CtcFwdBwd ctc_A(*new ibex::Function( "aref","xq","yq","dnew","w","atan2(yq+dnew*sin(w),xq+dnew*cos(w))-aref"));

ibex::CtcFwdBwd ctc_TX(*new ibex::Function( "mxq","xq", "yq","tq","(xq*cos(tq)+yq*sin(tq))+mxq"));
ibex::CtcFwdBwd ctc_TY(*new ibex::Function( "myq","xq", "yq","tq","(yq*cos(tq)-xq*sin(tq))+myq"));
ibex::CtcFwdBwd ctc_TT(*new ibex::Function( "mtq","tq","tq+mtq"));

CtcAssociation ctc_asso1(v_obs_t1);
CtcAssociation ctc_asso2(v_obs_t2);

pyibex::CtcPolar ctc_polar;

IntervalVector& mq = cn.create_dom(IntervalVector(3));
IntervalVector mq1=-q;
mq1[0]=-(cos(q[2])*q[0]+sin(q[2])*q[1]);
mq1[1]=-(cos(q[2])*q[1]-sin(q[2])*q[0]);
mq=mq&mq1;
cout<<q<<endl;
for (int i = 0; i < v_obs_t2.size(); i++)
{  
  IntervalVector& p1 = cn.create_dom(IntervalVector(2));
  Interval& w1 = cn.create_dom(Interval());
  IntervalVector& c1 = cn.create_dom(IntervalVector(2));
  IntervalVector& c2 = cn.create_dom(IntervalVector(2));
  cn.add(ctc_add,{q[2],v_obs_t2[i][1],w1});
  cn.add(ctc_polX,{q[0],v_obs_t2[i][0],w1,p1[0],p1[1]});
  cn.add(ctc_polY,{q[1],v_obs_t2[i][0],w1,p1[0],p1[1]});
  cn.add(ctc_D,{p1[0],q[0],q[1],v_obs_t2[i][0],w1});
  cn.add(ctc_A,{p1[1],q[0],q[1],v_obs_t2[i][0],w1});
  cn.add(ctc_asso1,{p1});
  /* ctc_polar(cartesian way) */
  cn.add(ctc_polar,{c1[0],c1[1],v_obs_t2[i][0],w1});
  cn.add(ctc_polar,{c2[0],c2[1],p1[0],p1[1]});
  cn.add(ctc_add,{q[0],c1[0],c2[0]});
  cn.add(ctc_add,{q[1],c1[1],c2[1]});

  /* IntervalVector& p2 = cn.create_dom(IntervalVector(2));
  Interval& w2 = cn.create_dom(Interval());
  cn.add(ctc_TX,{ mq[0],q[0], q[1],q[2]});
  cn.add(ctc_TY,{ mq[1],q[0], q[1],q[2]});
  cn.add(ctc_TT,{ mq[2],q[2]});

  cn.add(ctc_add,{mq[2],v_obs_t1[i][1],w2});
  cn.add(ctc_polX,{mq[0],v_obs_t1[i][0],w2,p2[0],p2[1]});
  cn.add(ctc_polY,{mq[1],v_obs_t1[i][0],w2,p2[0],p2[1]});
  cn.add(ctc_D,{p2[0],mq[0],mq[1],v_obs_t1[i][0],w2});
  cn.add(ctc_A,{p2[1],mq[0],mq[1],v_obs_t1[i][0],w2});
  cn.add(ctc_asso2,{p2}); */
  
}
cn.contract();
cout<<q<<endl;

if (k<dim and q.is_empty()==false)
   {  
      IntervalVector q1=q;
      IntervalVector q2=q;
      q1[k%3]=Interval(q[k%3].lb(),q[k%3].mid());
      q2[k%3]=Interval(q[k%3].mid(),q[k%3].ub());
      k++;
      SIVIA_Polar(v_obs_t2,v_obs_t1,q1,dim,k);
      SIVIA_Polar(v_obs_t2,v_obs_t1,q2,dim,k);
     
   }
}





















int main(int argc, char** argv) {
  vector<IntervalVector>*set=new vector<IntervalVector>;
  
  
  /* ================================================= CREATING DATA ============================================================ */
  IntervalVector map_area(2, Interval(-20.,20.));

  // Truth (unknown pose)
  Vector truth1(3,0.);
  truth1[0]=-10;
  truth1[2] = 0; // heading

/* =========== CREATING LANDMARKS =========== */
    int nb_marks=100;
    //let's create 4 wall of measurement
    //2 wall for T1
    vector<IntervalVector> upwall1(nb_marks/2);
    vector<IntervalVector> bwall1(nb_marks/2);
    float i=0;
    for (auto& mark : upwall1){ //landmark from -15 to 15
      //upwall
      mark=IntervalVector(2);
      mark[0]=Interval(-20+i/2.5);
      mark[1]=Interval(2*sin(-M_PI+(i/nb_marks)*2*M_PI)+15+nb_marks/2*sin(i)/nb_marks);
      i+=2;} 
    i=0;
    for (auto& mark : bwall1){ //landmark from -15 to 15
      //bwall
      mark=IntervalVector(2);
      mark[0]=Interval(-20+i/2.5);
      mark[1]=Interval(2*sin(-M_PI+(i/nb_marks)*2*M_PI)-15+nb_marks/2*sin(i)/nb_marks);
      i+=2;} 
    //let's create a vector containing all the landmarks
    vector<IntervalVector> v_b1 =upwall1;
    v_b1.insert( v_b1.end(), bwall1.begin(), bwall1.end() );

    // 2 wall for T2
    vector<IntervalVector> upwall2(nb_marks/2);
    vector<IntervalVector> bwall2(nb_marks/2);
    i=1;
    for (auto& mark : upwall2){ //landmark from -15 to 15
      //upwall
      mark=IntervalVector(2);
      mark[0]=Interval(-20+i/2.5);
      mark[1]=Interval(2*sin(-M_PI+(i/nb_marks)*2*M_PI)+15+nb_marks/2*sin(i)/nb_marks);
      i+=2;} 
    i=1;
    for (auto& mark : bwall2){ //landmark from -15 to 15
      //bwall
      mark=IntervalVector(2);
      mark[0]=Interval(-20+i/2.5);
      mark[1]=Interval(2*sin(-M_PI+(i/nb_marks)*2*M_PI)-15+nb_marks/2*sin(i)/nb_marks);
      i+=2;} 
    //let's create a vector containing all the landmarks
    vector<IntervalVector> v_b2 =upwall2;
    v_b2.insert( v_b2.end(), bwall2.begin(), bwall2.end() );



/* =========== OBSERVATION INSTANT T1 =========== */
    int nb_observations=90;
    // Creating  landmarks observations at t1 for the two walls
   /*  Interval visi_range(0.,100.); // [0m,75m]
    Interval visi_angle=Interval(-M_PI/4.+M_PI/2,M_PI/4.+M_PI/2)|Interval(-M_PI/4.-M_PI/2,M_PI/4.-M_PI/2); // frontal sonar
     */
    //Keep only some random observations:
    vector<IntervalVector> obs_t1(nb_observations);
    int index;
    for (auto& obs : obs_t1)
    {std::random_device seed ;
      // generator 
      std::mt19937 engine( seed( ) ) ;
      // number distribution
      std::uniform_int_distribution<int> choose( 0 , v_b1.size( ) - 1 ) ;
      index=choose( engine ) ;
      obs= v_b1[ index ];
      v_b1.erase(v_b1.begin()+index);
      }
    vector<IntervalVector> v_obs_t1 = DataLoader::generate_observations(truth1, obs_t1); //, true, visi_range, visi_angle); if you to keep the sonar range
    //box inflating due to sonar loss
    for(auto& obs : v_obs_t1) {
      obs[0].inflate(0.2); // range
      obs[1].inflate(0.025); // bearing
    }


    /* =========== OBSERVATION INSTANT T2 =========== */
    /* first updating of the pose */
    // Truth (unknown pose)
    Vector truth2(3,0.);
    truth2[0] = 6.;
    truth2[1] = -1.; // y
    truth2[2] = -M_PI/6.; // heading

    //True displacement qtrue
    Vector qtrue(3,0.);
    qtrue[0] =truth2[0]-truth1[0] ;
    qtrue[1] =truth2[1]-truth1[1] ; 
    qtrue[2] =truth2[2]-truth1[2] ; 
    //number of measurement
    nb_observations=80;
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
      std::uniform_int_distribution<int> choose( 0 , v_b2.size( ) - 1 ) ;
      index=choose( engine ) ;
      obs= v_b2[ index ];
      v_b2.erase(v_b2.begin()+index);
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
  q[0]=Interval(-0.5,0.5)+qtrue[0] ;
  q[1]=Interval(-0.5,0.5)+qtrue[1] ; 
  q[2]=Interval(-M_PI/32,M_PI/32)+qtrue[2] ; 











/* ==========================================================DATA PROCESSING=========================================================================== */


/* =========== LOCALIZATION PROCESSING T1 =========== */
    /* all the sensor information is in v_obs_t1 */
    IntervalVector x1(3);
    //Let's consider that at T1 the position is known with an accuracy of one meter
    x1[0] = Interval(truth1[0]-0.5,truth1[0]+0.5);
    x1[1] = Interval(truth1[1]-0.5,truth1[1]+0.5);
    x1[2] = Interval(truth1[2]);

    /* ------------TO DO--------------- */
    pyibex::CtcPolar ctc_polar;
    //first let's create the vector with the box containing the landmark at t1
    IntervalVector b_t1(2);
    Interval theta=Interval();
    vector<IntervalVector> b_obs_t1g(v_obs_t1);
    vector<IntervalVector> b_obs_t1(v_obs_t1);

    i=0;
    for (auto& obs : b_obs_t1g)
      { obs=IntervalVector(2);
        theta=Interval();
        theta=v_obs_t1[i][1]+x1[2];
        ctc_polar.contract(obs[0], obs[1], v_obs_t1[i][0], theta);
        obs[0]=obs[0]+x1[0];
        obs[1]=obs[1]+x1[1];
        i++;
        }

    i=0;
    for (auto& obs : b_obs_t1)
      { obs=IntervalVector(2);
        theta=Interval();
        theta=v_obs_t1[i][1]+x1[2];
        ctc_polar.contract(obs[0], obs[1], v_obs_t1[i][0], theta);
        obs[0]=obs[0];
        obs[1]=obs[1];
        i++;
        }




    








/* =========== LOCALIZATION PROCESSING T2 =========== */
    /* all the sensor information is in v_obs_t2 */

  /* The IMU gives us an approximation of the displacement, so the position of the robot at T2 is known but with a loss of accuracy */
  /* By integrating with an Euler method we have : */
  IntervalVector x2(3);
  x2[0]=x1[0]+q[0];
  x2[1]=x1[1]+q[1];
  x2[2]=x1[2]+q[2];

/*========================= TO DO ===============================================*/
// the first step envisaged is to bring the Rnew measurements back into Rref
//let's take the observations of v_obs_t2 and bring it back to general frame
  IntervalVector b_t2(2);
  Interval theta2=Interval();
  vector<IntervalVector> b_obs_t2g(v_obs_t2);
  vector<IntervalVector> b_obs_t2(v_obs_t2);
  vector<IntervalVector> v_obs_t2_in_t1(v_obs_t2);
  i=0;
  for (auto& obs : b_obs_t2g)
    { obs=IntervalVector(2);
      theta2=Interval();
      theta2=v_obs_t2[i][1]+x2[2];
      ctc_polar.contract(obs[0], obs[1], v_obs_t2[i][0], theta2);
      obs[0]=obs[0]+x2[0];
      obs[1]=obs[1]+x2[1];
      i++;
      }
  i=0;
  for (auto& obs : b_obs_t2)
    { obs=IntervalVector(2);
      theta2=Interval();
      theta2=v_obs_t2[i][1]+q[2]+x1[2];
      ctc_polar.contract(obs[0], obs[1], v_obs_t2[i][0], theta2);
      obs[0]=obs[0]+q[0];
      obs[1]=obs[1]+q[1];
      i++;
      }

  i=0;
  for (auto& obs : v_obs_t2_in_t1)
    { obs=IntervalVector(2);
      theta2=Interval();
      theta2=v_obs_t2[i][1]+q[2]+x1[2];
      ctc_polar.contract(obs[0], obs[1], v_obs_t2[i][0], theta2);
      obs[0]=obs[0]+q[0];
      obs[1]=obs[1]+q[1];
      i++;
      }  

  /* Now that they are in the same frame, let's link one box to the others. */
  /* so everybody is in the robot frame at t1 */
 /*  OpHull hull;
  vector<vector<IntervalVector>> asso_t1at2(b_obs_t2.size());
  vector<IntervalVector> asso_hull(b_obs_t2.size());
  for (size_t i = 0; i < b_obs_t2.size(); i++){
    for (size_t j = 0; j < b_obs_t1.size(); j++){
      if (b_obs_t2[i].intersects(b_obs_t1[j]))
      {
        asso_t1at2[i].push_back(b_obs_t1[j]);
      }
      
    }  
    asso_hull[i]=hull.eval( asso_t1at2[i]);
  }
   */

/* ======================================================================+SIVIA+=============================================================== */
/* IntervalVector*qopt=new IntervalVector;
SIVIA(q, x1, v_obs_t2, asso_t1at2,asso_hull, 3,qopt);
cout<<*qopt<<endl;
cout<<qtrue<<endl;

vector<IntervalVector> b_q_opt_t2(v_obs_t2);
i=0;
IntervalVector qcoco ;
qcoco=*qopt;

for (auto& obs : b_q_opt_t2)
  { obs=IntervalVector(2);
    theta2=Interval();
    theta2=v_obs_t2[i][1]+qcoco[2]+x1[2];
    ctc_polar.contract(obs[0], obs[1], v_obs_t2[i][0], theta2);
    obs[0]=obs[0]+qcoco[0];
    obs[1]=obs[1]+qcoco[1];
    i++;
    } */

/* ========================================================================+CONTRACTION METHODE+========================================================== */
/* let's make the hypothesis that the sonars tend to take the shortest distance. */
/* let's create a vector that contains all the measurments from t2 that are included into their hull box associated */
/* vector<IntervalVector> box_included_into_hull(b_obs_t2.size());
for (int i = 0; i < b_obs_t2.size(); i++)
{ 
    box_included_into_hull[i]=b_obs_t2[i];
    box_included_into_hull[i][1]=asso_hull[i][1];
    if (asso_hull[i][1].lb()>x2[1].ub())
    {box_included_into_hull[i][1]=Interval(asso_hull[i][1].lb(),b_obs_t2[i][1].ub());
    }
    else
    {
      box_included_into_hull[i][1]=Interval(b_obs_t2[i][1].lb(),asso_hull[i][1].ub());
    }
  
} */
/* let's create our contractor network */
/* ContractorNetwork cn;
ibex::CtcFwdBwd ctc_add(*new ibex::Function( "a", "b","c", "a+b-c"));
vector<IntervalVector> test(b_obs_t2.size());
IntervalVector qvar= q;
int counter=0;
for (int i = 0; i < b_obs_t2.size(); i++)
{   
  if (box_included_into_hull[i][1].ub()<1000000 )
  { counter++;

    Interval& thetavar = cn.create_dom(Interval());
    Interval& theta2 = cn.create_dom(v_obs_t2[i][1]+x1[2]);
    Interval& rho = cn.create_dom( v_obs_t2[i][0]);
    IntervalVector& box = cn.create_dom(IntervalVector(2));
    IntervalVector& boxf = cn.create_dom(IntervalVector(2));
   
    cn.add(ctc_add,{theta2,qvar[2],thetavar});
    cn.add(ctc_polar,{box[0], box[1],rho, thetavar});
    cn.add(ctc_add,{box[1],qvar[1],box_included_into_hull[i][1]});   
    
  }
}
cn.contract();
cout<<q<<endl;
cout<<qvar<<endl;
cout<<"         "<<endl; */
/* =====================================================================PAVING+CONTRACTION============================================================================== */
/* cout<<"contraction"<<endl;
IntervalVector*qo=new IntervalVector;
SIVIA_Contraction(q, x1,x2, v_obs_t2, b_obs_t1, 10);
 */





/* =======================================================================Polar contraction======================================================================= */

/* First let's try to bring T2 mesure into T1 frame in polar  */
ContractorNetwork cn;
ibex::CtcFwdBwd ctc_add(*new ibex::Function( "a", "b","c", "a+b-c"));
ibex::CtcFwdBwd ctc_polX(*new ibex::Function( "xq","dnew", "w","dref","aref","xq+dnew*cos(w)-dref*cos(aref)"));
ibex::CtcFwdBwd ctc_polY(*new ibex::Function( "yq","dnew", "w","dref","aref","yq+dnew*sin(w)-dref*sin(aref)"));
ibex::CtcFwdBwd ctc_D(*new ibex::Function( "dref","xq","yq","dnew","w","dref-sqrt(((xq+dnew*cos(w))^2+(yq+dnew*sin(w))^2))"));
ibex::CtcFwdBwd ctc_D2(*new ibex::Function( "dnew","xq","yq","dref","aref","dnew-sqrt(((xq-dref*cos(aref))^2+(yq-dref*sin(aref))^2))"));

ibex::CtcFwdBwd ctc_A(*new ibex::Function( "aref","xq","yq","dnew","w","atan2(yq+dnew*sin(w),xq+dnew*cos(w))-aref"));
ibex::CtcFwdBwd ctc_A2(*new ibex::Function( "aref","xq","yq","dref","w","atan2(dref*sin(aref)-yq,dref*cos(aref)-xq)-w"));

/* Contractor inverse evolution */
ibex::CtcFwdBwd ctc_TX(*new ibex::Function( "mxq","xq", "yq","tq","(xq*cos(tq)+yq*sin(tq))+mxq"));
ibex::CtcFwdBwd ctc_TY(*new ibex::Function( "myq","xq", "yq","tq","(yq*cos(tq)-xq*sin(tq))+myq"));
ibex::CtcFwdBwd ctc_TT(*new ibex::Function( "mtq","tq","tq+mtq"));

CtcAssociation ctc_asso1(v_obs_t1);
CtcAssociation ctc_asso2(v_obs_t2);
vector<IntervalVector> obs2_polar(v_obs_t2.size());
vector<vector<IntervalVector>> asso(v_obs_t2.size());

OpHull hull;
IntervalVector& mq = cn.create_dom(IntervalVector(3));
IntervalVector mq1=-q;
mq1[0]=-(cos(q[2])*q[0]+sin(q[2])*q[1]);
mq1[1]=-(cos(q[2])*q[1]-sin(q[2])*q[0]);
mq=mq&mq1;

cout<<q<<endl;
/* cout<<mq<<endl; */


for (int i = 0; i < obs2_polar.size(); i++)
{  
  IntervalVector& p1 = cn.create_dom(IntervalVector(2));
  Interval& w1 = cn.create_dom(Interval());
  IntervalVector& c1 = cn.create_dom(IntervalVector(2));
  IntervalVector& c2 = cn.create_dom(IntervalVector(2));
  cn.add(ctc_add,{q[2],v_obs_t2[i][1],w1});
  cn.add(ctc_polX,{q[0],v_obs_t2[i][0],w1,p1[0],p1[1]});
  cn.add(ctc_polY,{q[1],v_obs_t2[i][0],w1,p1[0],p1[1]});
  cn.add(ctc_D,{p1[0],q[0],q[1],v_obs_t2[i][0],w1});
  cn.add(ctc_D2,{v_obs_t2[i][0],q[0],q[1],p1[0],p1[1]});
  cn.add(ctc_A,{p1[1],q[0],q[1],v_obs_t2[i][0],w1});
  cn.add(ctc_A2,{p1[1],q[0],q[1],p1[0],w1});
  cn.add(ctc_asso1,{p1});
  /* ctc_polar(cartesian way) */
  cn.add(ctc_polar,{c1[0],c1[1],v_obs_t2[i][0],w1});
  cn.add(ctc_polar,{c2[0],c2[1],p1[0],p1[1]});
  cn.add(ctc_add,{q[0],c1[0],c2[0]});
  cn.add(ctc_add,{q[1],c1[1],c2[1]});

 /*  cn.contract(); */

  /* IntervalVector& p2 = cn.create_dom(IntervalVector(2));
  Interval& w2 = cn.create_dom(Interval());
  cn.add(ctc_TX,{ mq[0],q[0], q[1],q[2]});
  cn.add(ctc_TY,{ mq[1],q[0], q[1],q[2]});
  cn.add(ctc_TT,{ mq[2],q[2]});

  cn.add(ctc_add,{mq[2],v_obs_t1[i][1],w2});
  cn.add(ctc_polX,{mq[0],v_obs_t1[i][0],w2,p2[0],p2[1]});
  cn.add(ctc_polY,{mq[1],v_obs_t1[i][0],w2,p2[0],p2[1]});
  cn.add(ctc_D,{p2[0],mq[0],mq[1],v_obs_t1[i][0],w2});
  cn.add(ctc_A,{p2[1],mq[0],mq[1],v_obs_t1[i][0],w2});
  cn.add(ctc_asso2,{p2});
   */


 /*  obs2_polar[i]=IntervalVector(2);
  obs2_polar[i][0]=p1[0];
  obs2_polar[i][1]=p1[1]; */

  
}
cn.contract();
cout<<q<<endl;
cout<<qtrue<<endl;

/* =========================================================================SIVIA_Polar========================================================================= */


SIVIA_Polar(v_obs_t2,v_obs_t1,q,6);


    /* ===================================================================== GRAPHICS ===================================================================== */

    vibes::beginDrawing();


    // Map : into a general frame
    VIBesFigMap fig_map("Map");
    fig_map.set_properties(10, 10, 600, 600);
    for(const auto& iv : v_b1){
        fig_map.add_beacon(Beacon(iv), 0.2);}
    for(const auto& iv : v_b2){
        fig_map.add_beacon(Beacon(iv), 0.2);}
    /* observation t1 */
    /* fig_map.add_observations(v_obs_t1, truth1); */
    for(int i = 0 ; i < obs_t1.size() ; i++){
        fig_map.add_beacon(obs_t1[i], 0.2, "##A50108[#A50108]");}
    for(auto& obs : b_obs_t1g){
           fig_map.draw_box(obs, "red");} 

    fig_map.draw_box(x1.subvector(0,1), "#2980b9[#A50108]"); // estimated position (2d box)
    fig_map.draw_vehicle(truth1, 1.);
    
    /* observation t2 */
    for(auto& obs : b_obs_t2g){
        fig_map.draw_box(obs, "green");} 


    for(int i = 0 ; i < obs_t2.size() ; i++){
        fig_map.add_beacon(obs_t2[i], 0.2, "#00A53B[#00A53B]");}
    fig_map.draw_box(x2.subvector(0,1), "#2980b9[#00A53B]"); // estimated position (2d box)
    fig_map.draw_vehicle(truth2, 1.);

    fig_map.axis_limits(map_area);
    fig_map.show();

    //Map2 : each observation represented into its on frame.
    VIBesFigMap fig_map2("Map2");
    fig_map2.set_properties(10, 10, 600, 600);
    //observation t1
    
    /* for(auto& obs : b_obs_t1){
           fig_map2.draw_box(obs, "red");} 
 */

    //hull
   /*  for(auto& obs : asso_hull){
           fig_map2.draw_box(obs, "red");} 
 */
    //observation t2
    
    for(auto& obs : b_obs_t2){
           fig_map2.draw_box(obs, "green");} 

    //Included
    /* for(auto& obs : box_included_into_hull){
           fig_map2.draw_box(obs, "black");}  */

   /*  fig_map2.axis_limits(map_area); */
    fig_map2.show();

  //Map3 
    VIBesFigMap fig_map3("Map3");
    fig_map3.set_properties(10, 10, 600, 600);
    for(const auto& iv : v_b1){
        fig_map3.add_beacon(Beacon(iv), 0.2);}
    for(const auto& iv : v_b2){
        fig_map3.add_beacon(Beacon(iv), 0.2);}
      
    //observation t1
    fig_map3.add_observations(v_obs_t2, truth2);
    /* fig_map3.add_observations(obs2_polar, truth1); */

    //observation t2
    /* fig_map3.add_observations(v_obs_t2, truth2); */
    
     for(int i = 0 ; i < obs_t1.size() ; i++){
            fig_map3.add_beacon(obs_t1[i], 0.2, "##A50108[#A50108]");}
            
     for(int i = 0 ; i < obs_t2.size() ; i++){
        fig_map3.add_beacon(obs_t2[i], 0.2, "#00A53B[#00A53B]");}
    fig_map3.axis_limits(map_area);
    fig_map3.show();


  
    vibes::endDrawing();
  /* =========== ENDING =========== */

    return EXIT_SUCCESS;
    
}




