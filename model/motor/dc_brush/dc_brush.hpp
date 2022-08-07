/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2
 * @brief:      dc_brush
 * @date        2022.07.17
 * @changelog:
 * date         author          notes
 * 2021.09.28   wangchongwei    first version
 * 2021.10.19   wangchongwei    Change the naming convention and add interfaces
 * 2022.05.05   wangchongwei    cpp
 * 2022.07.17   wangchongwei    new version
 */

#ifndef _DC_BRUSH_HPP_
#define _DC_BRUSH_HPP_

#include "module/solver/OdeSolver.hpp"


namespace BDC_Space{


struct CfgPrm_t
{
    struct
    {
        double rated_cur;
        double rated_vol;
        double rated_tor;
        double rated_vel;
        double rated_pow;
    }nameplate;

    double  K;
    double  R;
    double  L;
    double  J;
    double  B;
};

struct InPrm_t
{
    double  TL;
    double  Ua;
};

struct OutPrm_t
{
    double  Ia;         /* A */
    double  Te;         /* N*m */
    double  omega;      /* rad/s */
    double  rpm;
    double  theta;      /* 0-2pi */
};

class DcBrush:public OdeSolver
{

public:
    DcBrush(/* args */);
    ~DcBrush();


    void simulation(double step_size, enum ODE_SOLVE_TYPE type);

    void setConfigPrm(struct CfgPrm_t prm);
    void inputPrm(struct InPrm_t prm);
    struct OutPrm_t getOutPrm();

    struct CfgPrm_t cfg;
    struct InPrm_t  in;
    struct OutPrm_t out;

private:
    virtual void dynamic_ode(double *dx, double *x, double *u);
    /* data */


    double dx[3];
    double x[3];
    double u;
};

}



#endif
