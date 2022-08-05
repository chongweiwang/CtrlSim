/**
 * @copyright   Copyright wangchongwei
 * @license:    GNU GPLv2 
 * @brief:      can_robust_ctrl_compare
 * @date        2022.08.04
 * @changelog:
 * date         author          notes
 * 2022.08.04   wangchongwei    Create a file   
 **/

#ifndef _CAN_ROBUST_CTRL_COMPARE_HPP_
#define _CAN_ROBUST_CTRL_COMPARE_HPP_


#include <cstdint>
#include "../../../module/simobj/SimObj.hpp"


class CAN_RobustCtrlCompare: public SimObj
{
public:
    explicit CAN_RobustCtrlCompare(SimObj *parent = nullptr);
    ~CAN_RobustCtrlCompare();

    virtual void dynamic_ode(double *dx, double *x, double *u);
    virtual void run(void);
    virtual void sim_run(void);

private:
    void simulation(void);
    void wavePlot(void);

    double dx[5];
    double  x[5];
    double  u[5];
    double  a;

    double  xd;
    double  err;
    double  dotxd;
    double  k;
    double  a_bar;
};

#endif
