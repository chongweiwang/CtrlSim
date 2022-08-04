/**
 * @copyright   Copyright wangchongwei
 * @license:    GNU GPLv2 
 * @brief:      can_adaptive_ctrl
 * @date        2022.07.23
 * @changelog:
 * date         author          notes
 * 2022.07.23   wangchongwei    Create a file   
 **/

#ifndef _CAN_ADAPTIVE_CTRL_HPP_
#define _CAN_ADAPTIVE_CTRL_HPP_


#include <cstdint>
#include "../../../module/simobj/SimObj.hpp"


class CAN_AdaptiveCtrl: public SimObj
{
public:
    explicit CAN_AdaptiveCtrl(SimObj *parent = nullptr);
    ~CAN_AdaptiveCtrl();

    virtual void dynamic_ode(double *dx, double *x, double *u);
    virtual void run(void);
    virtual void sim_run(void);

private:
    void simulation(void);
    void wavePlot(void);

    double dx;
    double  x;
    double  u;
    double  a;

    double  xd;
    double  dotxd;
    double  k;
};

#endif
