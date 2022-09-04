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


#include "mc_pmsm_scvm_sensorless.hpp"
#include <cmath>

McPmsmScvmSensorless::McPmsmScvmSensorless(SimObj *parent):SimObj(parent)
{
    pmsm.cfg.nameplate.rated_cur = 5.9;
    pmsm.cfg.nameplate.rated_pow = 200;
    pmsm.cfg.nameplate.rated_tor = 0.64;
    pmsm.cfg.nameplate.rated_vol = 48;
    pmsm.cfg.nameplate.rated_vel = 3000;

    pmsm.cfg.psi = 0.024156;
    pmsm.cfg.Rs  = 0.2;
    pmsm.cfg.Ld  = 0.0005625;
    pmsm.cfg.Lq  = 0.0005625;
    pmsm.cfg.B   = 0.00001;
    pmsm.cfg.J   = 0.0000258;
    pmsm.cfg.Pn  = 3;

    double bandwidth = 1200;
    double cur_kPBase = pmsm.cfg.Lq * bandwidth * (pmsm.cfg.nameplate.rated_cur/pmsm.cfg.nameplate.rated_vol);
    double cur_kIBase = pmsm.cfg.Rs * bandwidth * (pmsm.cfg.nameplate.rated_cur/pmsm.cfg.nameplate.rated_vol) / 20000;

    /*set pid parm*/
    Iq_pid.setCtrlParm(cur_kPBase,cur_kIBase,0);
    Iq_pid.setClampParm(0.5,-0.5);

    Id_pid.setCtrlParm(cur_kPBase,cur_kIBase,0);
    Id_pid.setClampParm(0.5,-0.5);

    double K = 1.5*pmsm.cfg.Pn * pmsm.cfg.psi / pmsm.cfg.J;
    double delta = 14;
    double vel_kiBase = (bandwidth/(delta*delta));
    double vel_kpBase = (delta*vel_kiBase)/ K;
    vel_kiBase = vel_kiBase*vel_kpBase /20000;

    double vel2Cur = pmsm.cfg.nameplate.rated_vel * 0.10471976 / pmsm.cfg.nameplate.rated_cur;
    /*set pid parm*/
    vel_pid.setCtrlParm(vel_kpBase*vel2Cur,vel_kiBase*vel2Cur,0);
    vel_pid.setClampParm(1.0,-1.0);


}

McPmsmScvmSensorless::~McPmsmScvmSensorless()
{

}


void McPmsmScvmSensorless::dynamic_ode(double *dx, double *x,double *u)
{
    Q_UNUSED(dx);
    Q_UNUSED(x);
    Q_UNUSED(u);
}

void McPmsmScvmSensorless::run(void)
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


void McPmsmScvmSensorless::sim_run(void)
{
    run();
}

void McPmsmScvmSensorless::foc_cur_loop(void)
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

void McPmsmScvmSensorless::vel_loop(void)
{
    Vel.o_tor_ctrl_pu =  vel_pid.PI(Vel.i_ref_rmp_pu - Vel.i_rpm_fb_pu);
}


#define LAMBDA  2
#define TUNING_A 2.5
#define TUNING_B 2
#define RPM2RADS 0.1047
void McPmsmScvmSensorless::harnefos_scvm(void)
{
    #define MPI                 3.141592
    #define SQ(x)				((x) * (x))

    double Ed = 0, Eq = 0;

    double lambda_sign = LAMBDA *(scvm.omega<0?-1:1);
    double alpha_w = TUNING_A*(pmsm.cfg.nameplate.rated_vel *RPM2RADS) + TUNING_B*LAMBDA*(scvm.omega<0?-scvm.omega: scvm.omega);

    Ed = scvm.i_Vd - pmsm.cfg.Rs * scvm.i_Id + scvm.omega*pmsm.cfg.Lq* scvm.i_Iq;
    Eq = scvm.i_Vq - pmsm.cfg.Rs * scvm.i_Iq - scvm.omega*pmsm.cfg.Lq* scvm.i_Id;

    scvm.theta += scvm.i_step_size * scvm.omega;
    scvm.omega += scvm.i_step_size*alpha_w * ((Eq-lambda_sign*Ed)/pmsm.cfg.psi - scvm.omega);


    while(scvm.theta>2*M_PI) scvm.theta-=2*M_PI;
    while(scvm.theta<0) scvm.theta+=2*M_PI;
}


void McPmsmScvmSensorless::simulation(void)
{
    static  uint32_t last_us = 0;
    uint32_t us = simPrm.real_time*1000000;

    Ctrl.ctrl_mode = PMSM_BASIC_CTRL_VEL;
    Ctrl.id = 0;
    Ctrl.iq = 6 ;
    Ctrl.vel = 1000;


    pmsm.in.TL = 0.101;
    pmsm.simulation(simPrm.step_size,ODE_SOLVE_RK4);

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

        /***observer**/
        scvm.i_Id = foc.m_Id_pu * pmsm.cfg.nameplate.rated_cur;
        scvm.i_Iq = foc.m_Iq_pu * pmsm.cfg.nameplate.rated_cur;

        scvm.i_Vd = foc.o_Vd_pu * pmsm.cfg.nameplate.rated_vol;
        scvm.i_Vq = foc.o_Vq_pu  * pmsm.cfg.nameplate.rated_vol;

        scvm.i_step_size = 0.00005;


        harnefos_scvm();
        scvm.rpm = scvm.omega * 60 / (2*MPI * pmsm.cfg.Pn);


        if (us> 0)
        {
            /*get pmsm feedback */
            foc.i_ele_angle = scvm.theta;
            Vel.i_rpm_fb_pu = scvm.rpm / pmsm.cfg.nameplate.rated_vel;

        }
        else
        {
            foc.i_ele_angle = pmsm.out.theta_elec;
            Vel.i_rpm_fb_pu = pmsm.out.rpm / pmsm.cfg.nameplate.rated_vel;
        }

        foc.i_ele_angle = scvm.theta;


        /* speed loop */
        static int vc_cnt = 0;
        if (vc_cnt++ == 4)
        {
            vc_cnt = 0;
            vel_loop();
        }

        foc.i_cur_pu[0] = pmsm.out.Ia / pmsm.cfg.nameplate.rated_cur;
        foc.i_cur_pu[1] = pmsm.out.Ib / pmsm.cfg.nameplate.rated_cur;
        foc.i_cur_pu[2] = pmsm.out.Ic / pmsm.cfg.nameplate.rated_cur;

        foc_cur_loop();

    }

    pmsm.in.Ua = foc.o_Ua_pu * pmsm.cfg.nameplate.rated_vol;
    pmsm.in.Ub = foc.o_Ub_pu * pmsm.cfg.nameplate.rated_vol;
    pmsm.in.Uc = foc.o_Uc_pu * pmsm.cfg.nameplate.rated_vol;


}

void McPmsmScvmSensorless::wavePlot(void)
{
    emit signal_appendWave("theta","theta",simPrm.real_time,pmsm.out.theta_elec);
    emit signal_appendWave("theta","observer",simPrm.real_time,scvm.theta);

    emit signal_appendWave("rmp","rmp",simPrm.real_time, pmsm.out.rpm);
    emit signal_appendWave("rmp","observer",simPrm.real_time, scvm.rpm);


    emit signal_appendWave("udq","Uq",simPrm.real_time,pmsm.out.Uq);
    emit signal_appendWave("udq","Ud",simPrm.real_time,pmsm.out.Ud);

    emit signal_appendWave("Iq","Iq_ref",simPrm.real_time,foc.i_ref_Iq_pu * pmsm.cfg.nameplate.rated_cur);
    emit signal_appendWave("Iq","Iq_fb",simPrm.real_time,foc.m_Iq_pu * pmsm.cfg.nameplate.rated_cur);

    emit signal_appendWave("Id","Id_fb",simPrm.real_time,foc.m_Id_pu * pmsm.cfg.nameplate.rated_cur);
    emit signal_appendWave("Id","Id_ref",simPrm.real_time,foc.i_ref_Id_pu* pmsm.cfg.nameplate.rated_cur);
}
