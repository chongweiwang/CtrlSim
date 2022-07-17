/**
 * @copyright   Copyright wangchongwei
 * @license:    GNU GPLv2 
 * @brief:      can_basic_feedback
 * @date        2022.07.16
 * @changelog:
 * date         author          notes
 * 2022.07.16   wangchongwei    first version   
 **/

#ifndef _CAN_BASIC_FEEDBACK_HPP_
#define _CAN_BASIC_FEEDBACK_HPP_


#include <cstdint>
#include "../../../module/simobj/SimObj.hpp"




class CAN_BasicFeedback: public SimObj
{
public:
    explicit CAN_BasicFeedback(SimObj *parent = nullptr);
    ~CAN_BasicFeedback();

    virtual void dynamic_ode(double *dx, double *x, double *u);
    virtual void run(void);
    virtual void sim_run(void);

private:
    void simulation(void);
    void wavePlot(void);

    double dx[3];
    double  x[3];
    double  u[3];
};

#endif
