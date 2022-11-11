/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2 
 * @brief:      CAN_Kalman_recursive
 * @date        2022.10.28
 * @changelog:
 * date         author          notes
 * 2022.10.28   wangchongwei    first version   
 **/

#include "can_kalman_recursive.hpp"
#include <random>
#include <ctime>

CAN_KalmanRecursive::CAN_KalmanRecursive(SimObj *parent):SimObj(parent)
{
    err_mea = 3;
    err_est = 5;
    x_hat   = 40;
    Zk      = 51;
}

CAN_KalmanRecursive::~CAN_KalmanRecursive()
{

}


void CAN_KalmanRecursive::dynamic_ode(double *dx, double *x,double *u)
{

}

void CAN_KalmanRecursive::run(void)
{
    while (1)
    {
        if (simPrm.real_time <= simPrm.end_time)
        {
            simPrm.real_time += 1;
            wavePlot();
            simulation();
        }
        else
        {
            emit signal_showAllGraph();
            break;
        }
    }
}



void CAN_KalmanRecursive::sim_run(void)
{
    run();
}

void CAN_KalmanRecursive::simulation(void)
{
    Kk =  err_est/(err_est+err_mea);

    x_hat = x_hat + Kk *(Zk - x_hat);

    err_est = (1 - Kk)*err_est;

    
    double a = (rand() % 30)*0.1;

    if ((uint32_t)(a*10)%2)
    {
        a = -a;
    }

    Zk = 50 + a;
}

void CAN_KalmanRecursive::wavePlot(void)
{
    emit signal_appendWave("x","x_hat",simPrm.real_time,x_hat);
    emit signal_appendWave("x","Zk",simPrm.real_time,Zk);
}
