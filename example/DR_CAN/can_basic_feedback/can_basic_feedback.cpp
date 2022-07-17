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
    for (int i = 0; i < 3; i++)
    {
        dx[i] = 0;
        x[i]  = 10;
        u[i]  = 0;
    }

}

CAN_BasicFeedback::~CAN_BasicFeedback()
{

}


void CAN_BasicFeedback::dynamic_ode(double *dx, double *x,double *u)
{
    dx[0] = x[0]*x[0] - x[0]*x[0]*x[0] + u[0];
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



void CAN_BasicFeedback::simulation(void)
{

    u[0] = -x[0]*x[0] + x[0]* x[0]*x[0] - x[0];
    this->rk4(&dx[0],&x[0],&u[0],simPrm.step_size,1);

    u[1] = -x[1]*x[1] - x[1];
    this->rk4(&dx[1],&x[1],&u[1],simPrm.step_size,1);

    u[2] = -x[2]*x[2];
    this->rk4(&dx[2],&x[2],&u[2],simPrm.step_size,1);

}

void CAN_BasicFeedback::wavePlot(void)
{
    emit signal_appendWave("x","x0",simPrm.real_time,x[0]);
    emit signal_appendWave("x","x1",simPrm.real_time,x[1]);
    emit signal_appendWave("x","x2",simPrm.real_time,x[2]);
    emit signal_appendWave("u","u1",simPrm.real_time,u[0]);
    emit signal_appendWave("u","u2",simPrm.real_time,u[1]);
    emit signal_appendWave("u","u3",simPrm.real_time,u[2]);
}

