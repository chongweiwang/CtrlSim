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
 **/



#include "basic_pmsm.hpp"

using namespace Basic_Pmsm_Space;

BaiscPmsm::BaiscPmsm()
{
    cfg.nameplate.rated_cur = 20;
    cfg.nameplate.rated_pow = 4000;
    cfg.nameplate.rated_tor = 1.27;
    cfg.nameplate.rated_vol = 24;
    cfg.nameplate.rated_vel = 3000;

    cfg.psi = 0.0072;
    cfg.Rs  = 0.0525;
    cfg.Ld  = 0.0001275;
    cfg.Lq  = 0.0001275;
    cfg.B   = 0.00001;
    cfg.J   = 0.0058;
    cfg.Pn  = 5;

}

BaiscPmsm::~BaiscPmsm()
{

}

void BaiscPmsm::setConfigPrm(struct CfgPrm_t prm)
{
    cfg = prm;
}
void BaiscPmsm::inputPrm(struct InPrm_t prm)
{
    in = prm;
}
struct OutPrm_t BaiscPmsm::getOutPrm()
{
    return out;
}

/**
 * @brief: pmsm mathematical model 
 * @details: Differential equation:
 *  dId/dt = (Ud-RdId+we*LqIq)/Ld
 *  dIq/dt = (Uq-RqIq-we*LdId-we*Psi)/Lq
 *  Te = 3/2*Pn*Iq(Id(Ld-Lq)+Psi)
 *  dwr/dt = (Te-TL-Bwr)/J
 *  dthetaR/dt = wr
 *  we = Pn*wr
 *  dthetaE/dt = we
 *  rpm = (30/pi) *wr
 * 
 *  dx[id,iq,dwr,dthetaR,dthetaE]  x[id,iq,wr,thetaR,theta_elec]
*/
void BaiscPmsm::dynamic_ode(double *dx, double *x, double *u)
{
    dx[0] = (out.Ud - cfg.Rs*x[0] + out.omega_elec*x[1]*cfg.Lq)/ cfg.Ld;
    dx[1] = (out.Uq - cfg.Rs*x[1] - out.omega_elec*x[0]*cfg.Ld - out.omega_elec*cfg.psi)/ cfg.Lq;

    out.Te = 1.5 *cfg.Pn* x[1]* (x[0]*(cfg.Ld- cfg.Lq) + cfg.psi);

    dx[2] = (out.Te - in.TL - cfg.B*x[2])/(cfg.J);

    dx[3] = x[2];
    out.omega_elec = cfg.Pn*x[2];

    dx[4] = out.omega_elec;

}

#define pi 3.141591
void BaiscPmsm::simulation(double step_size, enum ODE_SOLVE_TYPE type)
{
    // 1.input vol abc->dq
    double st = sin(out.theta_elec);
    double ct = cos(out.theta_elec);

    abc2dq_amp(in.Ua, in.Ub, in.Uc,st,ct,&out.Ud,&out.Uq);

    switch (type) {
        case ODE_SOLVE_EULER:   this->euler(m_dx,m_x,&m_u,step_size,5);       break;
        case ODE_SOLVE_RK4:     this->rk4(m_dx,m_x,&m_u,step_size,5);     break;
    }

    // 3. get output I omega theta rpm
    out.Id = m_x[0];
    out.Iq = m_x[1];

    out.omega_rotor = m_x[2];
    out.rpm = (30/pi) * out.omega_rotor;

    if (m_x[3] > 2*pi)          m_x[3] = m_x[3] - 2*pi;
    else if (m_x[3] < 0)        m_x[3] = m_x[3] + 2*pi;

    out.theta_rotor = m_x[3];

    // 4. processing angle information
    if (m_x[4] > 2*pi)          m_x[4] = m_x[4] - 2*pi;
    else if (m_x[4] < 0)        m_x[4] = m_x[4] + 2*pi;

    out.theta_elec = m_x[4];

    // 5. cur dq->abc
    st = sin(out.theta_elec);
    ct = cos(out.theta_elec);

    dq2abc_amp(out.Id,out.Iq,st,ct,&out.Ia,&out.Ib,&out.Ic);

    // emf
}
