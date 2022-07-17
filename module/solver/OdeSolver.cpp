/**
 * @copyright   Copyright  wangchongwei 
 * @license:    GNU GPLv2 
 * @brief:      soLver_ode  
 * @date        2022.06.30
 * @changelog:
 * date         author          notes
 * 2022.06.30   wangchongwei    first version   
 **/

#include "OdeSolver.hpp"

OdeSolver::OdeSolver(/* args */)
{


}

OdeSolver::~OdeSolver()
{


}

void OdeSolver::rk4(double *dx, double *x ,double *u,double step_size, uint32_t order)
{
    double k1[MAX_ORDER], k2[MAX_ORDER], k3[MAX_ORDER], k4[MAX_ORDER], xk[MAX_ORDER];
    uint32_t i;

    dynamic_ode(dx,x,u);
    for(i = 0; i < order; i++)
    {
        k1[i] = dx[i] * step_size;
        xk[i] =  x[i] + k1[i]*0.5;
    }

    dynamic_ode(dx,xk,u);
    for(i = 0; i < order; i++)
    {
        k2[i] = dx[i] * step_size;
        xk[i] =  x[i] + k2[i]*0.5;
    }

    dynamic_ode(dx,xk,u);
    for(i = 0; i < order; i++)
    {
        k3[i] = dx[i] * step_size;
        xk[i] = x[i] + k3[i];
    }

    dynamic_ode(dx,xk,u);
    for(i = 0; i < order; i++)
    {
        k4[i] = dx[i] * step_size;
        x[i] = x[i] + (k1[i] + 2*(k2[i] + k3[i]) + k4[i])/6.0;
    }   
}

void OdeSolver::euler(double *dx, double *x ,double *u,double step_size, uint32_t order)
{
    uint32_t i;

    dynamic_ode(dx,x,u);

    for(i = 0; i < order; i++)
    {
        x[i] =  x[i] + dx[i] * step_size;;
    }
}

