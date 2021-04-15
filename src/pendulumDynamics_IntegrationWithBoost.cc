/*====================================================================
 * Simple pendulum example
 * Copyright (c) 2015 Matthew Millard 
 * <matthew.millard@iwr.uni-heidelberg.de>
 *
 *///=================================================================


#include <string>
#include <iostream>
#include <stdio.h> 
#include <rbdl/rbdl.h>
#include "csvtools.h"

#include <boost/numeric/odeint/stepper/runge_kutta_cash_karp54.hpp>
#include <boost/numeric/odeint/stepper/controlled_runge_kutta.hpp>
#include <boost/numeric/odeint/integrate/integrate_adaptive.hpp>
#include <boost/numeric/odeint/stepper/generation/make_controlled.hpp>
//using namespace std;
using namespace boost::numeric::odeint;


#ifndef RBDL_BUILD_ADDON_LUAMODEL
    #error "Error: RBDL addon LuaModel not enabled."
#endif

#include <rbdl/addons/luamodel/luamodel.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;



//====================================================================
// Boost stuff
//====================================================================

typedef std::vector< double > state_type;

typedef runge_kutta_cash_karp54< state_type > error_stepper_type;
typedef controlled_runge_kutta< error_stepper_type > controlled_stepper_type;


class rbdlToBoost {

    public:
        rbdlToBoost(Model* model) : model(model) {
            q = VectorNd::Zero(model->dof_count);
            qd = VectorNd::Zero(model->dof_count);
            qdd = VectorNd::Zero(model->dof_count);
            tau = VectorNd::Zero(model->dof_count);

        }

        void operator() (const state_type &x, 
                         state_type &dxdt, 
                         const double t){

            //qd
            int j = 0;
            for(int i=0; i<model->dof_count; i++){                
                qd[i] = (double)x[j];
                j++;
            }

            //q
            for(int i=0; i<model->dof_count; i++){                
                q[i] = (double)x[j];
                j++;
            }

            //tau = for now. This could call a control
            //               function.
            for(int i=0; i<model->dof_count; i++){                
                tau[i] = 0;
            }

            ForwardDynamics (*model,q,qd,tau,qdd);

            //populate dxdt
            j = 0;
            for(int i = 0; i < model->dof_count; i++){
                dxdt[j] = (double)qdd[i];
                j++;
            }
            for(int i = 0; i < model->dof_count; i++){
                dxdt[j] = (double)qd[i];
                j++;
            }


        }

    private:
        Model* model;
        VectorNd q, qd, qdd, tau;
};

struct pushBackStateAndTime
{
    std::vector< state_type >& states;
    std::vector< double >& times;

    pushBackStateAndTime( std::vector< state_type > &states , 
                              std::vector< double > &times )
    : states( states ) , times( times ) { }

    void operator()( const state_type &x , double t )
    {
        states.push_back( x );
        times.push_back( t );
    }
};

void f(const state_type &x, state_type &dxdt, const double t);

/* Problem Constants */
int main (int argc, char* argv[]) {
    rbdl_check_api_version (RBDL_API_VERSION);


    //problem specific constants
    double  pi      = 3.14159265358979e+00;
    int     nPts    = 100;
    double  t0      = 0;
    double  t1      = 1;
    double  th0     = -1.57079632679490;
    double  th1     = 3.14159265358979e+00;

    //Integration settings
    double absTolVal   = 1e-10;
    double relTolVal   = 1e-6;

    //Model* model = NULL;
    VectorNd Q, QDot, QDDot, Tau;

    Model* model  = NULL;
    model         = new Model();

    if (!Addons::LuaModelReadFromFile ("./../data/pendulum_luamodel.lua", 
                                       model, false)             ){        
        std::cerr     << "Error loading model ./../data/pendulum_luamodel.lua" 
                    << std::endl;
        abort();
    }

    Q       = VectorNd::Zero (model->dof_count);
    QDot    = VectorNd::Zero (model->dof_count);
    QDDot   = VectorNd::Zero (model->dof_count);
    Tau     = VectorNd::Zero (model->dof_count);

    double t        = 0;             //time
    double ts       = 0;            //scaled time
    double dtsdt    = pi/(t1-t0);    //dertivative scaled time 
                                    //w.r.t. time

    printf("DoF: %i\n",model->dof_count);

    printf("==============================\n");
    printf("0. Inverse Dynamics \n");
    printf("==============================\n");    
    printf("   ts,        q,        qd,       qdd,      tau\n");

    for(int i=0; i < nPts; i++){

        t           = t0 + (t1-t0)*((double)i/((double)nPts-1.0));
        ts          = pi*(t-t0)/(t1-t0);

        Q[0]        = th0 + (th1-th0)*0.5*(1-cos(ts));
        QDot[0]     =       (th1-th0)*0.5  * sin(ts)*dtsdt;
        QDDot[0]    = 0;    //(th1-th0)*0.5  * cos(ts)*dtsdt*dtsdt;
        Tau         = VectorNd::Zero (model->dof_count);



        InverseDynamics(*model, Q, QDot, QDDot, Tau, NULL);        

        printf("%f, %f, %f, %f, %f\n",
                t,Q[0],QDot[0],QDDot[0],Tau[0]);
    }

    printf("==============================\n");
    printf("1. Forward Dynamics \n");
    printf("==============================\n");    

    QDot[0]     = 1.239845546;
    Q[0]        = 0.6453246986897345;
    Tau[0]      = 0;
    QDDot[0]    = 0;

    rbdlToBoost rbdlModel(model);
    state_type xState(2);
    int steps = 0;
    xState[0] = 0;//(double)QDot[0];
    xState[1] = -pi/2.0;//(double)Q[0];


    double dt   = (t1-t0)/((double)nPts);    
    //steps       = integrate(rbdlModel, x0, t0, t1, dt,
    //                       pushBackStateAndTime(xVec, times) );

    double ke,pe,vx,vy,th,dth = 0;
    double keRBDL, peRBDL = 0;
    double m,J,l,g = 0;
    m = 1;
    J = 1;
    l = 1;
    g = 9.81;


    std::vector<std::vector< double > > matrixData;
    std::vector<std::vector< double > > matrixErrorData;
    std::vector< double > rowData(model->dof_count+1);
    std::vector< double > rowErrorData(2);

    double a_x = 1.0 , a_dxdt = 1.0;
    controlled_stepper_type  
    controlled_stepper(
        default_error_checker< double , 
                               range_algebra , 
                               default_operations >
        ( absTolVal , relTolVal , a_x , a_dxdt ) );
    

    double tp = 0;
    rowData[0] = 0;
    for(int z=0; z < model->dof_count; z++){
        rowData[z+1] = xState[model->dof_count + z];
    }
    matrixData.push_back(rowData);

    double kepe0 = 0;

    printf("   t,        q,        qd,       ke,       keRBDL,   keErr,    pe,      peRBDL,   peErr\n");
    for(int i = 0; i <= nPts; i++){
        //t   = times[i];

        t = t0 + dt*i;


        integrate_adaptive( 
            controlled_stepper ,
            rbdlModel , xState , tp , t , (t-tp)/10 );
        tp = t;

        dth = xState[0];
        th  = xState[1];

        Q[0]    = th;
        QDot[0] = dth;

        peRBDL = Utils::CalcPotentialEnergy(*model, Q, true);
        keRBDL = Utils::CalcKineticEnergy(*model, Q, QDot, true);


        //x = l*sin(th)
        //y = -l*cos(th)
        vx =  l*cos(th)*dth;
        vy =  l*sin(th)*dth;

        ke =    0.5*m*(vx*vx + vy*vy)
             +  0.5*J*dth*dth;

        pe = (-l*cos(th))*m*g;

        printf("%f, %f, %f, %f, %f, %f, %f, %f, %f\n",
                    t, Q[0],QDot[0],ke,
                    keRBDL,(keRBDL-ke), 
                    pe,peRBDL,(peRBDL-pe)); 

        rowData[0] = t;
        for(int z=0; z < model->dof_count; z++){
            rowData[z+1] = xState[model->dof_count + z];
        }
        matrixData.push_back(rowData);

        if(i==0) kepe0 = (ke+pe);

        rowErrorData[0] = t;
        rowErrorData[1] = (keRBDL + peRBDL) - kepe0;
        matrixErrorData.push_back(rowErrorData);
    }

    /*
    std::string header = "# MeshUp animation file\nNAME:\nPendulum\nCOLUMNS:\nTime, ";
    char buf[2000];
    for(int z=0; z < model->dof_count; z++){
            sprintf(buf,"%s:%s:%s:%s, ",
                    "K1",
                    "rotation",
                    "Z",
                    "rad");
            header.append( buf);
    }   
    header.append("\nDATA:");
    */
    std::string header = "DATA:";

    std::string fname   = "../data/pendulumDynamicsBoostMeshupFile.csv";    
    printMatrixToFile(matrixData, header, fname);

    fname               = "../data/pendulumDynamicsBoostEnergyError.csv";
    printMatrixToFile(matrixErrorData,header,fname);

    delete model;

    return 0;
        
}

