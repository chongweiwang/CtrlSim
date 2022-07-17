/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2
 * @brief:      CAN_follow_desired
 * @date        2022.07.09
 * @changelog:
 * date         author          notes
 * 2022.07.09   wangchongwei    first version   
 **/

#include "can_follow_desired.hpp"


CAN_FollowDesired::CAN_FollowDesired(SimObj *parent):SimObj(parent)
{
    for (int i = 0; i < 2; i++)
    {
        dx[i] = 0;
        x[i]  = 0;
        u[i]  = 0;
    }

    x[0] = 0;
}

CAN_FollowDesired::~CAN_FollowDesired()
{

}


void CAN_FollowDesired::dynamic_ode(double *dx, double *x,double *u)
{
    dx[0] = x[1];
    dx[1] = 10*x[0] - u[1];
}

void CAN_FollowDesired::run(void)
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

void CAN_FollowDesired::sim_run(void)
{
    run();
}

void CAN_FollowDesired::simulation(void)
{
    u[1] = -xd+k1*x[0]+2*x[1];
    this->rk4(dx,x,u,simPrm.step_size,2);
}

void CAN_FollowDesired::wavePlot(void)
{
    emit signal_appendWave("x0","xd",simPrm.real_time,xd);
    emit signal_appendWave("x0","x0",simPrm.real_time,x[0]);
    emit signal_appendWave("x1","x1",simPrm.real_time,x[1]);
    emit signal_appendWave("u","u",simPrm.real_time,u[1]);
}

