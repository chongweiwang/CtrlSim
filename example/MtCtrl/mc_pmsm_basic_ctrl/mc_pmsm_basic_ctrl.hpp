/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2
 * @brief:      mc_pmsm_basic_ctrl
 * @date        2022.07.23
 * @changelog:
 * date         author          notes
 * 2021.11.07   wangchongwei    create file 
 * 2022.05.27   wangchongwei    cpp
 * 2022.07.23   wangchongwei    new version
 **/

#ifndef _MC_PMSM_BASIC_CTRL_HPP_
#define _MC_PMSM_BASIC_CTRL_HPP_


#include "../../../module/simobj/SimObj.hpp"
#include "../../../model/motor/basic_pmsm/basic_pmsm.hpp"
#include "../../../lib/controller/pid/dynamic_clamp_pid.hpp"




class McPmsmBasicCtrl : public SimObj
{



public:
    explicit McPmsmBasicCtrl(SimObj *parent = nullptr);

    ~McPmsmBasicCtrl();
    virtual void sim_run(void);

private:
    virtual void dynamic_ode(double *dx, double *x, double *u);
    virtual void run(void);

    void simulation(void);
    void wavePlot(void);
    void foc_cur_loop(void);
    void vel_loop(void);
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
