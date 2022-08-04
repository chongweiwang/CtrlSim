/**
 * @copyright   Copyright wangchongwei
 * @license:    GNU GPLv2 
 * @brief:      can_back_step
 * @date        2022.08.03
 * @changelog:
 * date         author          notes
 * 2022.08.03   wangchongwei    Create a file   
 **/

#ifndef _CAN_BACK_STEP_HPP_
#define _CAN_BACK_STEP_HPP_


#include <cstdint>
#include "../../../module/simobj/SimObj.hpp"


class CAN_BackStep: public SimObj
{
public:
    explicit CAN_BackStep(SimObj *parent = nullptr);
    ~CAN_BackStep();

    virtual void dynamic_ode(double *dx, double *x, double *u);
    virtual void run(void);
    virtual void sim_run(void);

private:
    void simulation(void);
    void generate_Xd(void);
    void wavePlot(void);

    double dx[2];
    double  x[2];
    double  u;
    double  a;

    double  k1;
    double  k2;
    double  m;
    double  alpha;

    double  x1d;
    double  dotx1d;
    double  ddotx1d;

    double  x2d;
    double  delta;

    double  err;
};

#endif
