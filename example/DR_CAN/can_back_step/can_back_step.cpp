/**
 * @copyright   Copyright wangchongwei
 * @license:    GNU GPLv2 
 * @brief:      can_back_step
 * @date        2022.08.03
 * @changelog:
 * date         author          notes
 * 2022.08.03   wangchongwei    Create a file   
 **/

#include "can_back_step.hpp"
#include <math.h>

CAN_BackStep::CAN_BackStep(SimObj *parent):SimObj(parent)
{
    m = 1;
    alpha = 1;
    k1 = 10;
    k2 = 10;

}

CAN_BackStep::~CAN_BackStep()
{

}


void CAN_BackStep::dynamic_ode(double *dx, double *x,double *u)
{
    dx[0] = x[1];
    dx[1] = -(alpha/m)*x[0]*x[0]*x[0] + (1/m) * (*u);
}

void CAN_BackStep::run(void)
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

void CAN_BackStep::sim_run(void)
{
    run();
}

#define PI  3.141592
void CAN_BackStep::generate_Xd(void)
{
    double Stair[5] = {3,5,3,5,2};
    double osc =1;
    double peroid = 10;

    uint32_t value = (((uint32_t)simPrm.real_time)/10)%5;
    value = Stair[value];

    x1d = value + osc*sin(simPrm.real_time*PI/peroid);
    dotx1d = PI/peroid * osc * cos(simPrm.real_time*PI/peroid);
    ddotx1d = -PI/peroid * PI/peroid * osc * sin(simPrm.real_time*PI/peroid);
}

void CAN_BackStep::simulation(void)
{
    generate_Xd();

    err = x1d - x[0];

    x2d = dotx1d + k1*err;
    delta = x2d- x[1];

    u = m* err + m* ddotx1d + m*k1 *(dotx1d - x[1])+ alpha*x[0]*x[0]*x[0]+ m*k2*delta;

    this->rk4(dx,x,&u,simPrm.step_size,2);
}

void CAN_BackStep::wavePlot(void)
{
//    emit signal_appendWave("x1d","x1d",simPrm.real_time,x1d);
//    emit signal_appendWave("x1d","dotx1d",simPrm.real_time,dotx1d);
//    emit signal_appendWave("x1d","ddotx1d",simPrm.real_time,ddotx1d);

    emit signal_appendWave("x","x1d",simPrm.real_time,x1d);
    emit signal_appendWave("x","x1",simPrm.real_time,x[0]);
}

