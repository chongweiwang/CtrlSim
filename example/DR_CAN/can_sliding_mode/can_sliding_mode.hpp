/**
 * @copyright   Copyright wangchongwei
 * @license:    GNU GPLv2 
 * @brief:      can_sliding_mode
 * @date        2022.07.24
 * @changelog:
 * date         author          notes
 * 2022.07.24   wangchongwei    Create a file   
 **/

#ifndef _CAN_SLIDING_MODE_HPP_
#define _CAN_SLIDING_MODE_HPP_


#include <cstdint>
#include "../../../module/simobj/SimObj.hpp"


class CAN_SlidingMode: public SimObj
{
public:
    explicit CAN_SlidingMode(SimObj *parent = nullptr);
    ~CAN_SlidingMode();

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
    double  a_bar;
};

#endif
