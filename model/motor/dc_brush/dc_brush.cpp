/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2
 * @brief:      dc_brush
 * @date        2022.07.17
 * @changelog:
 * date         author          notes
 * 2021.09.28   wangchongwei    first version
 * 2021.10.19   wangchongwei    Change the naming convention and add interfaces
 * 2022.05.05   wangchongwei    cpp
 * 2022.07.17   wangchongwei    new version
 **/

#include "dc_brush.hpp"

using namespace BDC_Space;

DcBrush::DcBrush()
{
    cfg.R = 0.5;
    cfg.L = 0.003;
    cfg.K = 0.1;
    cfg.B = 0.0001;
    cfg.J = 0.001;
}

DcBrush::~DcBrush()
{

}

void DcBrush::setConfigPrm(struct CfgPrm_t prm)
{
    cfg = prm;
}
void DcBrush::inputPrm(struct InPrm_t prm)
{
    in = prm;
}
struct OutPrm_t DcBrush::getOutPrm()
{
    return out;
}

/**
 * @brief: bdc mathematical model
 * @details: Differential equation:
 *  di/dt = (u-R*i-K*w)/L
 *  dw/dt = (K*i -TL-Bw)/J
 *  dtheta/dt = w
 *  dx[i,w,theta]  x[i,w,theta]
*/
void DcBrush::dynamic_ode(double *dx, double *x, double *u)
{
    dx[0] = (in.Ua - cfg.R*x[0] - cfg.K * x[1])/ cfg.L;
    out.Te = cfg.K * x[0];
    dx[1] = (out.Te - in.TL- cfg.B*x[1])/ cfg.J ;
    dx[2] = x[1];
}

#define pi 3.141591
void DcBrush::simulation(double step_size, enum ODE_SOLVE_TYPE type)
{
    switch (type)
    {
        case ODE_SOLVE_EULER:   this->euler(dx,x,&u,step_size,3);   break;
        case ODE_SOLVE_RK4:     this->rk4(dx,x,&u,step_size,3);     break;
    }

    /* 2. get output I, omega, rpm, theta  */
    out.Ia    = x[0];
    out.omega = x[1];
    //out.rpm   = (30/pi) * out.omega;
    out.rpm   = (30*0.31831005) * out.omega;

    if (x[2] > 2*pi)        x[2] = x[2] - 2*pi;
    else if (x[2] < 0)      x[2] = x[2] + 2*pi;

    out.theta = x[2];
}
