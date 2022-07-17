/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2
 * @brief:      can_basic_feedback
 * @date        2022.07.16
 * @changelog:
 * date         author          notes
 * 2022.07.16   wangchongwei    first version   
 **/

#include "can_basic_feedback.hpp"


CAN_BasicFeedback::CAN_BasicFeedback(SimObj *parent):SimObj(parent)
{
    for (int i = 0; i < 2; i++)
    {
        dx[i] = 0;
        x[i]  = 0;
        u[i]  = 0;

        dz[i] = 0;
        z[i]  = 0;
    }

    z[1] = 1;
}

CAN_BasicFeedback::~CAN_BasicFeedback()
{

}


void CAN_BasicFeedback::dynamic_ode(double *dx, double *x,double *u)
{
    dx[0] = x[1];
    dx[1] = -1*x[0] -0.5*x[1] + u[1];
}

void CAN_BasicFeedback::run(void)
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


void CAN_BasicFeedback::sim_run(void)
{
    run();
}

void CAN_BasicFeedback::observer(void)
{
    dz[0] = -2.5*z[0] + z[1] + 2.5*y;
    dz[1] = 0.25*z[0] - 0.5*z[1] + u[1] -1.25*y;


    z[0] =  z[0] + dz[0] * simPrm.step_size;
    z[1] =  z[1] + dz[1] * simPrm.step_size;
}

void CAN_BasicFeedback::simulation(void)
{
    u[1] = 1;
    this->rk4(dx,x,u,simPrm.step_size,2);

    // observer
    y = x[0];
    observer();
}

void CAN_BasicFeedback::wavePlot(void)
{
    emit signal_appendWave("x0","x0",simPrm.real_time,x[0]);
    emit signal_appendWave("x0","z0",simPrm.real_time,z[0]);
    emit signal_appendWave("x1","x1",simPrm.real_time,x[1]);
    emit signal_appendWave("x1","z1",simPrm.real_time,z[1]);
    emit signal_appendWave("u","u",simPrm.real_time,u[1]);
}

