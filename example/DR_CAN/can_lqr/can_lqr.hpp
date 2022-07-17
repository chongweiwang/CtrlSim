/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2 
 * @brief:      CAN_LQR
 * @date        2022.07.08
 * @changelog:
 * date         author          notes
 * 2022.07.08   wangchongwei    first version   
 **/

#ifndef _CAN_LQR_HPP_
#define _CAN_LQR_HPP_


#include <cstdint>
#include "../../../module/simobj/SimObj.hpp"

class CAN_Lqr: public SimObj
{
public:
    explicit CAN_Lqr(SimObj *parent = nullptr);
    ~CAN_Lqr();

    virtual void dynamic_ode(double *dx, double *x, double *u);
    virtual void run(void);
    virtual void sim_run(void);


private:
    void simulation(void);
    void wavePlot(void);

    double dx[2];
    double  x[2];
    double  u[2];
    double k1 = -110.4988;
    double k2 = -17.9164;

};





#endif
