/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2
 * @brief:      can_adaptive_ctrl
 * @date        2022.07.23
 * @changelog:
 * date         author          notes
 * 2022.07.23   wangchongwei    Create a file  
 **/

#include "can_adaptive_ctrl.hpp"


CAN_AdaptiveCtrl::CAN_AdaptiveCtrl(SimObj *parent):SimObj(parent)
{
    dx = 0;
    x = 0;
    u = 0;
    a = 0.5;

    xd = 2;
    dotxd = 0;
    k = 10;
}

CAN_AdaptiveCtrl::~CAN_AdaptiveCtrl()
{

}


void CAN_AdaptiveCtrl::dynamic_ode(double *dx, double *x,double *u)
{
    *dx = a * (*x) * (*x) + (*u);
}

void CAN_AdaptiveCtrl::run(void)
{

    while (1)
    {
        if (simPrm.real_time <= simPrm.end_time)
        {
            simPrm.real_time += simPrm.step_size;

            simulation();

            /*****wave data sample *******/
            if (simPrm.sim_cnt++ >= simPrm.sample_freq_div)
            {
                simPrm.sim_cnt = 0;
                wavePlot();
            }
        }
        else
        {
            emit signal_showAllGraph();
            break;
        }
    }
}


void CAN_AdaptiveCtrl::sim_run(void)
{
    run();
}



void CAN_AdaptiveCtrl::simulation(void)
{
    double err = xd - x;

    static double sum  = 0;

    sum += err *x*x * simPrm.step_size;

    u = dotxd + x*x *sum + k*err;
    
    this->rk4(&dx,&x,&u,simPrm.step_size,1);

    if (simPrm.real_time > 10) a = 1.5;
}

void CAN_AdaptiveCtrl::wavePlot(void)
{
    emit signal_appendWave("x","xd",simPrm.real_time,xd);
    emit signal_appendWave("x","x",simPrm.real_time,x);
    emit signal_appendWave("x","a",simPrm.real_time,a);
}

