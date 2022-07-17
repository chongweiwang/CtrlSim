/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2
 * @brief:      CAN_follow_desired
 * @date        2022.07.09
 * @changelog:
 * date         author          notes
 * 2022.07.09   wangchongwei    first version   
 **/

#ifndef _CAN_FOLLOW_DESIRED_HPP_
#define _CAN_FOLLOW_DESIRED_HPP_


#include <cstdint>
#include "../../../module/simobj/SimObj.hpp"




class CAN_FollowDesired: public SimObj
{
public:
    explicit CAN_FollowDesired(SimObj *parent = nullptr);
    ~CAN_FollowDesired();

    virtual void dynamic_ode(double *dx, double *x, double *u);
    virtual void run(void);
    virtual void sim_run(void);
private:
    void simulation(void);
    void wavePlot(void);

    double dx[2];
    double  x[2];
    double  u[2];

    double k1 = 11;
    double k2 = -2;
    double xd = 1;
};

#endif
