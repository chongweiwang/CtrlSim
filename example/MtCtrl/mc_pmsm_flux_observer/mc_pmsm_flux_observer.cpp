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


#include "mc_pmsm_flux_observer.hpp"
#include <cmath>

McPmsmFluxObserver::McPmsmFluxObserver(SimObj *parent):SimObj(parent)
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
    double delta = 15;
    double vel_kiBase = (bandwidth/(delta*delta));
    double vel_kpBase = (delta*vel_kiBase)/ K;
    vel_kiBase = vel_kiBase*vel_kpBase /20000;

    double vel2Cur = pmsm.cfg.nameplate.rated_vel * 0.10471976 / pmsm.cfg.nameplate.rated_cur;
    /*set pid parm*/
    vel_pid.setCtrlParm(vel_kpBase*vel2Cur,vel_kiBase*vel2Cur,0);
    vel_pid.setClampParm(1.0,-1.0);


}

McPmsmFluxObserver::~McPmsmFluxObserver()
{

}


void McPmsmFluxObserver::dynamic_ode(double *dx, double *x,double *u)
{
    Q_UNUSED(dx);
    Q_UNUSED(x);
    Q_UNUSED(u);
}

void McPmsmFluxObserver::run(void)
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


void McPmsmFluxObserver::sim_run(void)
{
    run();
}

void McPmsmFluxObserver::foc_cur_loop(void)
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

void McPmsmFluxObserver::vel_loop(void)
{
    Vel.o_tor_ctrl_pu =  vel_pid.PI(Vel.i_ref_rmp_pu - Vel.i_rpm_fb_pu);
}


void McPmsmFluxObserver::flux_observer(void)
{
    #define MPI                 3.141592
    #define SQ(x)				((x) * (x))


    double err;

    double R = 1.5 * pmsm.cfg.Rs;
    double L = 1.5 * pmsm.cfg.Lq;

    double flux = pmsm.cfg.psi;

    double L_ia = L * Observer.i_Ia;
    double R_ia = R * Observer.i_Ia;
    double R_ib = R * Observer.i_Ib;
    double L_ib = L * Observer.i_Ib;


    err = SQ(flux) - (SQ( Observer.x1 - L_ia) + SQ(Observer.x2 - L_ib));
    double gamma_tmp = Observer.gamma * 0.5;

    if (err > 0.0) {
        err = 0.0;
    }

    double x1_dot = -R_ia + Observer.i_Va + gamma_tmp * (Observer.x1 - L_ia) * err;
    double x2_dot = -R_ib + Observer.i_Vb + gamma_tmp * (Observer.x2 - L_ib) * err;

    Observer.x1 += x1_dot * Observer.step_size ;
    Observer.x2 += x2_dot * Observer.step_size ;


    Observer.theta = atan2(Observer.x2 - L_ib, Observer.x1 - L_ia);
    Observer.o_theta = Observer.theta<0? 2*MPI +Observer.theta : Observer.theta;

    /*pll*/

    double delta_theta;
    delta_theta = Observer.theta - Observer.pll_x[0];

    if (delta_theta < -MPI) {
        delta_theta += 2.0f * MPI;
    }
    if (delta_theta >  MPI) {
        delta_theta -= 2.0f * MPI;
    }

    Observer.pll_x[0] += (Observer.pll_x[1]+ Observer.pll_kp *delta_theta)*Observer.step_size;

    if (Observer.pll_x[0] < -MPI) {
        Observer.pll_x[0] += 2.0f * MPI;
    }
    if (Observer.pll_x[0] >  MPI) {
        Observer.pll_x[0] -= 2.0f * MPI;
    }


    Observer.pll_x[1] += Observer.pll_ki* delta_theta *Observer.step_size;

    Observer.pll_omega = Observer.pll_x[1];

}


void McPmsmFluxObserver::simulation(void)
{
    static  uint32_t last_us = 0;
    uint32_t us = simPrm.real_time*1000000;

    Ctrl.ctrl_mode = PMSM_BASIC_CTRL_VEL;
    Ctrl.id = 0;
    Ctrl.iq = 6 ;
    Ctrl.vel = 1000;

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
        Observer.i_Ia = foc.m_cur_alpha_pu * pmsm.cfg.nameplate.rated_cur;
        Observer.i_Ib = foc.m_cur_beta_pu * pmsm.cfg.nameplate.rated_cur;

        Observer.i_Va = foc.m_vol_alpha_pu * pmsm.cfg.nameplate.rated_vol;
        Observer.i_Vb = foc.m_vol_beta_pu  * pmsm.cfg.nameplate.rated_vol;

        Observer.step_size = 0.00005;
        Observer.gamma = 20220810;

        Observer.pll_kp =  3000;
        Observer.pll_ki =  40000;

        flux_observer();
        Observer.rpm = Observer.pll_omega * 60 / (2*MPI * pmsm.cfg.Pn);

        if (us> 1000000)
        {
            /*get pmsm feedback */
            foc.i_ele_angle = Observer.o_theta;
            Vel.i_rpm_fb_pu = Observer.rpm / pmsm.cfg.nameplate.rated_vel;

        }
        else
        {
            foc.i_ele_angle = pmsm.out.theta_elec;
            Vel.i_rpm_fb_pu = pmsm.out.rpm / pmsm.cfg.nameplate.rated_vel;
        }

        /* speed loop */

        vel_loop();

        foc.i_cur_pu[0] = pmsm.out.Ia / pmsm.cfg.nameplate.rated_cur;
        foc.i_cur_pu[1] = pmsm.out.Ib / pmsm.cfg.nameplate.rated_cur;
        foc.i_cur_pu[2] = pmsm.out.Ic / pmsm.cfg.nameplate.rated_cur;

        foc_cur_loop();

    }

    pmsm.in.Ua = foc.o_Ua_pu * pmsm.cfg.nameplate.rated_vol;
    pmsm.in.Ub = foc.o_Ub_pu * pmsm.cfg.nameplate.rated_vol;
    pmsm.in.Uc = foc.o_Uc_pu * pmsm.cfg.nameplate.rated_vol;

    pmsm.in.TL = 0.101;
    pmsm.simulation(simPrm.step_size,ODE_SOLVE_RK4);
}

void McPmsmFluxObserver::wavePlot(void)
{
    emit signal_appendWave("theta","theta",simPrm.real_time,pmsm.out.theta_elec);
    emit signal_appendWave("theta","observer",simPrm.real_time,Observer.o_theta);

    emit signal_appendWave("rmp","rmp",simPrm.real_time,pmsm.out.rpm);
    emit signal_appendWave("rmp","observer",simPrm.real_time,Observer.rpm);


    emit signal_appendWave("udq","Uq",simPrm.real_time,pmsm.out.Uq);
    emit signal_appendWave("udq","Ud",simPrm.real_time,pmsm.out.Ud);

    emit signal_appendWave("Iq","Iq_ref",simPrm.real_time,foc.i_ref_Iq_pu * pmsm.cfg.nameplate.rated_cur);
    emit signal_appendWave("Iq","Iq_fb",simPrm.real_time,foc.m_Iq_pu * pmsm.cfg.nameplate.rated_cur);

    emit signal_appendWave("Id","Id_fb",simPrm.real_time,foc.m_Id_pu * pmsm.cfg.nameplate.rated_cur);
    emit signal_appendWave("Id","Id_ref",simPrm.real_time,foc.i_ref_Id_pu* pmsm.cfg.nameplate.rated_cur);
}
