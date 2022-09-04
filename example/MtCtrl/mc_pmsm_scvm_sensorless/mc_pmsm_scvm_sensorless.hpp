/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2
 * @brief:      Synchronization at Startup and Stable Rotation Reversal of Sensorless Nonsalient PMSM Drives
 *              https://www.bilibili.com/video/BV1qf4y127uJ?spm_id_from=333.999.0.0&vd_source=1f88f15c4a8c95c1d720fa4c6218bc54
 * @date        2022.08.18
 * @changelog:
 * date         author          notes
 * 2022.08.18   wangchongwei    create file 
 **/

#ifndef _MC_PMSM_SCVM_SENSORLESS_HPP_
#define _MC_PMSM_SCVM_SENSORLESS_HPP_


#include "../../../module/simobj/SimObj.hpp"
#include "../../../model/motor/basic_pmsm/basic_pmsm.hpp"
#include "../../../lib/controller/pid/dynamic_clamp_pid.hpp"




class McPmsmScvmSensorless : public SimObj
{

public:
    explicit McPmsmScvmSensorless(SimObj *parent = nullptr);

    ~McPmsmScvmSensorless();
    virtual void sim_run(void);

private:
    virtual void dynamic_ode(double *dx, double *x, double *u);
    virtual void run(void);

    void simulation(void);
    void wavePlot(void);
    void foc_cur_loop(void);
    void vel_loop(void);
    void harnefos_scvm(void);

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
        double i_Vd, i_Vq;
        double i_Id, i_Iq;
        double i_step_size;


        double theta;
        double omega;
        double rpm;
    }scvm;
    
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
