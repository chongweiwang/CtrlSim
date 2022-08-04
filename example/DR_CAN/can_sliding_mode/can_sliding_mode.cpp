/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2
 * @brief:      can_sliding_mode
 * @date        2022.07.24
 * @changelog:
 * date         author          notes
 * 2022.07.24   wangchongwei    Create a file  
 **/

#include "can_sliding_mode.hpp"
#include <random>
#include <ctime>

CAN_SlidingMode::CAN_SlidingMode(SimObj *parent):SimObj(parent)
{
    dx = 0;
    x = 0;
    u = 0;
    a = 0.5;
    a_bar = 5;

    xd = 2;
    dotxd = 0;
    k = 1;
}

CAN_SlidingMode::~CAN_SlidingMode()
{

}


void CAN_SlidingMode::dynamic_ode(double *dx, double *x,double *u)
{
    *dx = a * (*x) * (*x) + (*u);
}

void CAN_SlidingMode::run(void)
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


void CAN_SlidingMode::sim_run(void)
{
    run();
}


#define MABS(a)  (a<0?-a:a)
void CAN_SlidingMode::simulation(void)
{

    a = (rand() % 100)*0.01;

    double err = xd - x;

    u =  k*err + dotxd + a_bar * (x*x + 0.1) * (MABS(err)/err) ;
    
    this->rk4(&dx,&x,&u,simPrm.step_size,1);


}

void CAN_SlidingMode::wavePlot(void)
{
    emit signal_appendWave("x","xd",simPrm.real_time,xd);
    emit signal_appendWave("x","x",simPrm.real_time,x);
    emit signal_appendWave("a","a",simPrm.real_time,a);
    emit signal_appendWave("u","u",simPrm.real_time,a);
}

