/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2
 * @brief:      CAN_follow_desired
 * @date        2022.07.09
 * @changelog:
 * date         author          notes
 * 2022.07.09   wangchongwei    first version   
 **/

#include "can_luenberger.hpp"


CAN_Luenberger::CAN_Luenberger(SimObj *parent):SimObj(parent)
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

CAN_Luenberger::~CAN_Luenberger()
{

}


void CAN_Luenberger::dynamic_ode(double *dx, double *x,double *u)
{
    dx[0] = x[1];
    dx[1] = -1*x[0] -0.5*x[1] + u[1];
}

void CAN_Luenberger::run(void)
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


void CAN_Luenberger::sim_run(void)
{
    run();
}

void CAN_Luenberger::observer(void)
{
    dz[0] = -2.5*z[0] + z[1] + 2.5*y;
    dz[1] = 0.25*z[0] - 0.5*z[1] + u[1] -1.25*y;


    z[0] =  z[0] + dz[0] * simPrm.step_size;
    z[1] =  z[1] + dz[1] * simPrm.step_size;
}

void CAN_Luenberger::simulation(void)
{
    u[1] = 1;
    this->rk4(dx,x,u,simPrm.step_size,2);

    // observer
    y = x[0];
    observer();
}

void CAN_Luenberger::wavePlot(void)
{
    emit signal_appendWave("x0","x0",simPrm.real_time,x[0]);
    emit signal_appendWave("x0","z0",simPrm.real_time,z[0]);
    emit signal_appendWave("x1","x1",simPrm.real_time,x[1]);
    emit signal_appendWave("x1","z1",simPrm.real_time,z[1]);
    emit signal_appendWave("u","u",simPrm.real_time,u[1]);
}

