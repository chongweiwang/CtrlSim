/**
 * @copyright   Copyright wangchongwei
 * @license:    GNU GPLv2 
 * @brief:      can_robust_ctrl_compare
 * @date        2022.08.04
 * @changelog:
 * date         author          notes
 * 2022.08.04   wangchongwei    Create a file   
 **/

#include "can_robust_ctrl_compare.hpp"
#include <random>
#include <ctime>

CAN_RobustCtrlCompare::CAN_RobustCtrlCompare(SimObj *parent):SimObj(parent)
{
    for (int i = 0; i < 5; i++)
    {
        dx[i] = 0;
        x[i]  = 0;
        u[i]  = 0;
    }

    a = 1;

    xd = 2;
    dotxd = 0;
    k = 1;
}

CAN_RobustCtrlCompare::~CAN_RobustCtrlCompare()
{

}


void CAN_RobustCtrlCompare::dynamic_ode(double *dx, double *x,double *u)
{
    /*sliding  gain:1*/
    dx[0] = a* x[0]* x[0] + u[0];

    /*high gain  gain:1, epsilo: 0.1*/
    dx[1] = a* x[1]* x[1] + u[1];

    /*high gain  gain:1, epsilo: 1*/
    dx[2] = a* x[2]* x[2] + u[2];

    /*high freq  gain:1, epsilo: 0.1*/
    dx[3] = a* x[3]* x[3] + u[3];

    /*high freq  gain:1, epsilo: 1*/
    dx[4] = a* x[4]* x[4] + u[4];
    
}

void CAN_RobustCtrlCompare::run(void)
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


void CAN_RobustCtrlCompare::sim_run(void)
{
    run();
}


#define MABS(a)  (a<0?-a:a)
void CAN_RobustCtrlCompare::simulation(void)
{

    double ms;
    static double last_ms =0;

    ms = simPrm.real_time*1000;

    // 0.1s a rand 
    if (ms - last_ms >= 100)
    {
        last_ms = ms;
        a = (rand() % 100)*0.01;

        if ((uint32_t)(a*10)%2 == 0)
        {

        }
        else
        {
            a = -a;
        }
    }


    /*sliding  gain:1*/
    err = xd - x[0];
    u[0] = 1*err + dotxd + (x[0]* x[0]+0.1) *(MABS(err)/err) ;

    /*high gain  gain:1, epsilo: 0.1*/
    err = xd - x[1];
    u[1] = 1*err + dotxd + (1/0.1) * (x[1]* x[1]+0.1)*(x[1]* x[1]+0.1) *err ;

    /*high gain  gain:1, epsilo: 1*/
    err = xd - x[2];
    u[2] = 1*err + dotxd + (1/1) * (x[2]* x[2]+0.1)*(x[2]* x[2]+0.1) *err ;

    /*high freq  gain:1, epsilo: 0.1*/
    err = xd - x[3];
    u[3] = 1*err + dotxd +  ((x[3]* x[3]+0.1)*(x[3]* x[3]+0.1) *err) /( (x[3]* x[3]+0.1)*MABS(err)+0.1)  ;

    /*high freq  gain:1, epsilo: 1*/
    err = xd - x[4];
    u[4] = 1*err + dotxd +  ((x[4]* x[4]+0.1)*(x[4]* x[4]+0.1) *err) /( (x[4]* x[4]+0.1)*MABS(err)+1)  ;

    this->rk4(dx,x,u,simPrm.step_size,5);

}

void CAN_RobustCtrlCompare::wavePlot(void)
{
    emit signal_appendWave("err","sliding",simPrm.real_time,xd-x[0]);
    emit signal_appendWave("err","high gain 0.1",simPrm.real_time,xd-x[1]);
    emit signal_appendWave("err","high gain 1",simPrm.real_time,xd-x[2]);
    emit signal_appendWave("err","high freq 0.1",simPrm.real_time,xd-x[3]);
    emit signal_appendWave("err","high freq 1",simPrm.real_time,xd-x[4]);

    emit signal_appendWave("u","u0",simPrm.real_time,u[0]);
    emit signal_appendWave("u","u1",simPrm.real_time,u[1]);
    emit signal_appendWave("u","u2",simPrm.real_time,u[2]);
    emit signal_appendWave("u","u3",simPrm.real_time,u[3]);
    emit signal_appendWave("u","u4",simPrm.real_time,u[4]);

    emit signal_appendWave("a","a",simPrm.real_time,a);

}

