/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2
 * @brief:      mc_pmsm_basic_ctrl
 * @date        2022.07.23
 * @changelog:
 * date         author          notes
 * 2021.10.24   wangchongwei    create file 
 * 2022.05.06   wangchongwei    cpp
 * 2022.07.23   wangchongwei    new version
 **/


#include "mc_pmsm_basic_ctrl.hpp"

McPmsmBasicCtrl::McPmsmBasicCtrl(SimObj *parent):SimObj(parent)
{
    pmsm.cfg.nameplate.rated_cur = 20;
    pmsm.cfg.nameplate.rated_pow = 4000;
    pmsm.cfg.nameplate.rated_tor = 1.27;
    pmsm.cfg.nameplate.rated_vol = 24;
    pmsm.cfg.nameplate.rated_vel = 3000;

    pmsm.cfg.psi = 0.0072;
    pmsm.cfg.Rs  = 0.0525;
    pmsm.cfg.Ld  = 0.0001275;
    pmsm.cfg.Lq  = 0.0001275;
    pmsm.cfg.B   = 0.00001;
    pmsm.cfg.J   = 0.0058;
    pmsm.cfg.Pn  = 5;

    double bandwidth = 2000;
    double cur_kPBase = pmsm.cfg.Lq * bandwidth * (pmsm.cfg.nameplate.rated_cur/pmsm.cfg.nameplate.rated_vol);
    double cur_kIBase = pmsm.cfg.Rs * bandwidth * (pmsm.cfg.nameplate.rated_cur/pmsm.cfg.nameplate.rated_vol) / 20000;

    /*set pid parm*/
    Iq_pid.setCtrlParm(cur_kPBase,cur_kIBase,0);
    Iq_pid.setClampParm(0.5,-0.5);

    Id_pid.setCtrlParm(cur_kPBase,cur_kIBase,0);
    Id_pid.setClampParm(0.5,-0.5);

    double K = 1.5*pmsm.cfg.Pn * pmsm.cfg.psi / pmsm.cfg.J;
    double delta = 5;
    double vel_kiBase = (bandwidth/(delta*delta));
    double vel_kpBase = (delta*vel_kiBase)/ K;
    vel_kiBase = vel_kiBase*vel_kpBase /20000;

    double vel2Cur = pmsm.cfg.nameplate.rated_vel * 0.10471976 / pmsm.cfg.nameplate.rated_cur;
    /*set pid parm*/
    vel_pid.setCtrlParm(vel_kpBase*vel2Cur,vel_kiBase*vel2Cur,0);
    vel_pid.setClampParm(1.0,-1.0);


}

McPmsmBasicCtrl::~McPmsmBasicCtrl()
{

}


void McPmsmBasicCtrl::dynamic_ode(double *dx, double *x,double *u)
{
    Q_UNUSED(dx);
    Q_UNUSED(x);
    Q_UNUSED(u);
}

void McPmsmBasicCtrl::run(void)
{
    while (1)
    {
        if (simPrm.real_time <= simPrm.end_time)
        {
            simPrm.real_time += simPrm.step_size;

            simulation();

            /*****wave data sample *******/
            if (simPrm.sim_cnt++ >= simPrm.sample_freq_div)
            {
                simPrm.sim_cnt = 0;
                wavePlot();
            }
        }
        else
        {
            emit signal_showAllGraph();
            break;
        }
    }
}


void McPmsmBasicCtrl::sim_run(void)
{
    run();
}

void McPmsmBasicCtrl::foc_cur_loop(void)
{
    double st = sin(foc.i_ele_angle);
    double ct = cos(foc.i_ele_angle);

    /*2. cur ab(c) -> alpha beta -> dq*/
    ab_clark_amp(foc.i_cur_pu[0], foc.i_cur_pu[1], &foc.m_cur_alpha_pu, &foc.m_cur_beta_pu);
    park(foc.m_cur_alpha_pu, foc.m_cur_beta_pu, st, ct, &foc.m_Id_pu, &foc.m_Iq_pu);

    /*3. control iq id=0 -> get Vq Vd*/
    foc.o_Vq_pu = Iq_pid.PI( foc.i_ref_Iq_pu - foc.m_Iq_pu);
    foc.o_Vd_pu = Id_pid.PI( foc.i_ref_Id_pu - foc.m_Id_pu);

    /*4. Vq Vd  -> abc pwm....... */
    inv_park(foc.o_Vd_pu, foc.o_Vq_pu,st, ct,&foc.m_vol_alpha_pu,&foc.m_vol_beta_pu);
    inv_clark_amp(foc.m_vol_alpha_pu,foc.m_vol_beta_pu,&foc.o_Ua_pu, &foc.o_Ub_pu, &foc.o_Uc_pu);
}

void McPmsmBasicCtrl::vel_loop(void)
{
    Vel.o_tor_ctrl_pu =  vel_pid.PI(Vel.i_ref_rmp_pu - Vel.i_rpm_fb_pu);
}

void McPmsmBasicCtrl::simulation(void)
{
    static  uint32_t last_us = 0;
    uint32_t us = simPrm.real_time*1000000;

    Ctrl.ctrl_mode = PMSM_BASIC_CTRL_VEL;
    Ctrl.id = 0;
    Ctrl.iq = 1 ;
    Ctrl.vel = -1000;

    /*50us ctrl period*/
    if (us-last_us >= 50)
    {
        last_us = us;

        /* 控制命令选择，环路输入输出选择*/
        switch(Ctrl.ctrl_mode)
        {
            case PMSM_BASIC_CTRL_CUR:
                foc.i_ref_Id_pu = Ctrl.id/pmsm.cfg.nameplate.rated_cur;            /* 目标D轴电流*/
                foc.i_ref_Iq_pu = Ctrl.iq/pmsm.cfg.nameplate.rated_cur;            /* 目标Q轴电流*/
            break;
            case PMSM_BASIC_CTRL_VEL:
                foc.i_ref_Id_pu = 0;
                foc.i_ref_Iq_pu = Vel.o_tor_ctrl_pu;                                /* 速度环控制量->电流环q轴目标*/
                Vel.i_ref_rmp_pu   = Ctrl.vel/pmsm.cfg.nameplate.rated_vel;         /* 速度指令*/
            break;
        }

        /* speed loop */
        Vel.i_rpm_fb_pu = pmsm.out.rpm / pmsm.cfg.nameplate.rated_vel;
        vel_loop();


        /*get pmsm feedback */
        foc.i_ele_angle = pmsm.out.theta_elec;
        foc.i_cur_pu[0] = pmsm.out.Ia / pmsm.cfg.nameplate.rated_cur;
        foc.i_cur_pu[1] = pmsm.out.Ib / pmsm.cfg.nameplate.rated_cur;
        foc.i_cur_pu[2] = pmsm.out.Ic / pmsm.cfg.nameplate.rated_cur;

        foc_cur_loop();

    }

    pmsm.in.Ua = foc.o_Ua_pu * pmsm.cfg.nameplate.rated_vol;
    pmsm.in.Ub = foc.o_Ub_pu * pmsm.cfg.nameplate.rated_vol;
    pmsm.in.Uc = foc.o_Uc_pu * pmsm.cfg.nameplate.rated_vol;

    pmsm.in.TL = 0.001;
    pmsm.simulation(simPrm.step_size,ODE_SOLVE_RK4);
}

void McPmsmBasicCtrl::wavePlot(void)
{
    emit signal_appendWave("theta","theta",simPrm.real_time,pmsm.out.theta_elec);
    emit signal_appendWave("RPM","RPM",simPrm.real_time,pmsm.out.rpm);

    emit signal_appendWave("udq","Uq",simPrm.real_time,pmsm.out.Uq);
    emit signal_appendWave("udq","Ud",simPrm.real_time,pmsm.out.Ud);

    emit signal_appendWave("Iq","Iq_fb",simPrm.real_time,foc.i_ref_Iq_pu * pmsm.cfg.nameplate.rated_cur);
    emit signal_appendWave("Iq","Iq_ref",simPrm.real_time,pmsm.out.Iq );

    emit signal_appendWave("Id","Id_fb",simPrm.real_time,pmsm.out.Id );
    emit signal_appendWave("Id","Id_ref",simPrm.real_time,foc.i_ref_Id_pu* pmsm.cfg.nameplate.rated_cur);
}
