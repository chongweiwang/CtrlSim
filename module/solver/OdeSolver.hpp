/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2 
 * @brief:      OdeSolver
 * @date        2022.06.30
 * @changelog:
 * date         author          notes
 * 2022.06.30   wangchongwei    first version   
 **/

#ifndef _ODE_SOLVER_HPP_
#define _ODE_SOLVER_HPP_

#include <cstdint>
#define MAX_ORDER  20

class OdeSolver
{

public:
    OdeSolver(/* args */);
    ~OdeSolver();

    virtual void dynamic_ode(double *dx, double *x, double *u)  = 0;

    void rk4(double *dx,   double *x, double *u,double step_size, uint32_t order);
    void euler(double *dx, double *x ,double *u,double step_size, uint32_t order);

private:
    /* data */

};





#endif
