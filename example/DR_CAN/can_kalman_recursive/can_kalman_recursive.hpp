/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2 
 * @brief:      CAN_Kalman_recursive
 * @date        2022.10.28
 * @changelog:
 * date         author          notes
 * 2022.10.28   wangchongwei    first version   
 **/

#ifndef _CAN_KALMAN_RECURSIVE_HPP_
#define _CAN_KALMAN_RECURSIVE_HPP_


#include <cstdint>
#include "../../../module/simobj/SimObj.hpp"

class CAN_KalmanRecursive: public SimObj
{
public:
    explicit CAN_KalmanRecursive(SimObj *parent = nullptr);
    ~CAN_KalmanRecursive();

    virtual void dynamic_ode(double *dx, double *x, double *u);
    virtual void run(void);
    virtual void sim_run(void);


private:
    void simulation(void);
    void wavePlot(void);

    double  err = 0;
    double  err_est = 0;
    double  err_mea = 0;
    double  x_hat = 0;
    double  Zk = 0;
    double  Kk = 0;
};





#endif
