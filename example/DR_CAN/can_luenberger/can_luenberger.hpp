/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2
 * @brief:      can_luenberger
 * @date        2022.07.10
 * @changelog:
 * date         author          notes
 * 2022.07.10   wangchongwei    first version   
 **/

#ifndef _CAN_LUENBERGER_HPP_
#define _CAN_LUENBERGER_HPP_


#include <cstdint>
#include "../../../module/simobj/SimObj.hpp"




class CAN_Luenberger: public SimObj
{
public:
    explicit CAN_Luenberger(SimObj *parent = nullptr);
    ~CAN_Luenberger();

    virtual void dynamic_ode(double *dx, double *x, double *u);
    virtual void run(void);
    virtual void sim_run(void);

private:
    void simulation(void);
    void observer(void);
    void wavePlot(void);

    double dx[2];
    double  x[2];
    double  u[2];

    // observer
    double dz[2];
    double z[2];
    double y;
};

#endif
