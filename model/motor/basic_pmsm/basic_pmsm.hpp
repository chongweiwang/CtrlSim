/**
 * @copyright   Copyright wangchongwei
 * @license:    GNU GPLv2
 * @brief:      BaiscPmsm
 * @date        2022.07.20
 * @changelog:
 * date         author          notes
 * 2021.10.14   wangchongwei    first version
 * 2021.11.11   wangchongwei    add config value rated cur vol tor vel pow
 * 2022.05.06   wangchongwei    cpp
 * 2022.07.20   wangchongwei    new version
 * 2022.07.20   wangchongwei    add back emf
 **/

#ifndef _BASIC_PMSM_HPP_
#define _BASIC_PMSM_HPP_

#include "module/solver/OdeSolver.hpp"
#include "lib/math/coordinate.h"
#include <math.h>


namespace Basic_Pmsm_Space{


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


    double  psi;
    double  Rs;
    double  Lq;
    double  Ld;
    double  J;
    double  B;
    uint32_t Pn;
};

struct InPrm_t
{
    double  Ua, Ub, Uc;
    double  TL;          // N*m
};

struct OutPrm_t
{
    double  Ia,Ib,Ic;         /* A */

    double  Te;

    double  omega_elec, omega_rotor;   /* rad/s */
    double  rpm;

    double  theta_elec, theta_rotor;

    double  emf_d, emf_q;
    double  emf_alpha, emf_beta;
    double  emf_a, emf_b, emf_c;

    double  Uq,Ud;
    double  Iq,Id;
};

class BaiscPmsm:public OdeSolver
{

public:
    BaiscPmsm(/* args */);
    ~BaiscPmsm();


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
    double m_dx[5];
    double m_x[5];
    double m_u;
};

}



#endif
