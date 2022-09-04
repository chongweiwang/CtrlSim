/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2
 * @brief:      http://cas.ensmp.fr/~praly/Telechargement/Journaux/2010-IEEE_TPEL-Lee-Hong-Nam-Ortega-Praly-Astolfi.pdf
 *              https://github.com/vedderb/bldc
 * @date        2022.08.10
 * @changelog:
 * date         author          notes
 * 2022.08.10   wangchongwei    create file 
 **/

#ifndef _MC_PMSM_FLUX_OBSERVER_HPP_
#define _MC_PMSM_FLUX_OBSERVER_HPP_


#include "../../../module/simobj/SimObj.hpp"
#include "../../../model/motor/basic_pmsm/basic_pmsm.hpp"
#include "../../../lib/controller/pid/dynamic_clamp_pid.hpp"




class McPmsmFluxObserver : public SimObj
{

public:
    explicit McPmsmFluxObserver(SimObj *parent = nullptr);

    ~McPmsmFluxObserver();
    virtual void sim_run(void);

private:
    virtual void dynamic_ode(double *dx, double *x, double *u);
    virtual void run(void);

    void simulation(void);
    void wavePlot(void);
    void foc_cur_loop(void);
    void vel_loop(void);
    void flux_observer(void);
    /* data */
    Basic_Pmsm_Space::BaiscPmsm  pmsm;

    DynClpPID  Iq_pid, Id_pid;
    DynClpPID  vel_pid;

    enum PMSM_BASIC_CTRL
    {
        PMSM_BASIC_CTRL_CUR,
        PMSM_BASIC_CTRL_VEL,
    };

    struct
    {
        enum PMSM_BASIC_CTRL ctrl_mode;
        double iq;
        double id;
        double vel;
    }Ctrl;

    struct
    {
        double i_Va, i_Vb;
        double i_Ia, i_Ib;
        double step_size;

        double x1, x2;
        double gamma;
        double theta;
        double o_theta;

        double pll_kp;
        double pll_ki;
        double pll_omega;
        double pll_dx[2];
        double pll_x[2];

        double rpm;
    }Observer;
    struct
    {
        double i_ele_angle;

        double i_cur_pu[3];    // a b c cur
        double i_ref_Iq_pu;
        double i_ref_Id_pu;

        double m_cur_alpha_pu;
        double m_cur_beta_pu;

        double m_Id_pu;
        double m_Iq_pu;

        double m_vol_alpha_pu;
        double m_vol_beta_pu;

        double o_Vq_pu;
        double o_Vd_pu;

        double o_Ua_pu;
        double o_Ub_pu;
        double o_Uc_pu;
    }foc;

    struct
    {
        double i_rpm_fb_pu;
        double i_ref_rmp_pu;    // a b c cur

        double o_tor_ctrl_pu;
    }Vel;
};

#endif
