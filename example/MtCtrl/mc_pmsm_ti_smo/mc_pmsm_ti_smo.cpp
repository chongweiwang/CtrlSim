/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2
 * @brief:      TI: Digital Motor Control Software Library: Target Independent Math Blocks         
 * @date        2022.09.30
 * @changelog:
 * date         author          notes
 * 2022.09.30   wangchongwei    create file 
 **/


#include "mc_pmsm_ti_smo.hpp"
#include <cmath>

McPmsmTiSmo::McPmsmTiSmo(SimObj *parent):SimObj(parent)
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
    double delta = 10;
    double vel_kiBase = (bandwidth/(delta*delta));
    double vel_kpBase = (delta*vel_kiBase)/ K;
    vel_kiBase = vel_kiBase*vel_kpBase /20000;

    double vel2Cur = pmsm.cfg.nameplate.rated_vel * 0.10471976 / pmsm.cfg.nameplate.rated_cur;
    /*set pid parm*/
    vel_pid.setCtrlParm(vel_kpBase*vel2Cur,vel_kiBase*vel2Cur,0);
    vel_pid.setClampParm(1.0,-1.0);


    smo.Fsmopos = exp((-pmsm.cfg.Rs/pmsm.cfg.Lq)*0.00005 );
    smo.Gsmopos = (pmsm.cfg.nameplate.rated_vol/pmsm.cfg.nameplate.rated_cur )*(1/pmsm.cfg.Rs) *(1-smo.Fsmopos);
    smo.Kslf = pmsm.cfg.nameplate.rated_vel/60 * pmsm.cfg.Pn *6.28 * 5 * 0.00005;
    smo.Kslide = 0.3;
    smo.E0 = 0.5;

    speed.K1 = 20000*60/pmsm.cfg.nameplate.rated_vel /pmsm.cfg.Pn ;
    speed.K2 = 0.05;
    speed.K3 = 0.95;
    speed.BaseRpm = pmsm.cfg.nameplate.rated_vel;

}

McPmsmTiSmo::~McPmsmTiSmo()
{

}


void McPmsmTiSmo::dynamic_ode(double *dx, double *x,double *u)
{
    Q_UNUSED(dx);
    Q_UNUSED(x);
    Q_UNUSED(u);
}

void McPmsmTiSmo::run(void)
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


void McPmsmTiSmo::sim_run(void)
{
    run();
}

void McPmsmTiSmo::foc_cur_loop(void)
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

void McPmsmTiSmo::vel_loop(void)
{
    Vel.o_tor_ctrl_pu =  vel_pid.PI(Vel.i_ref_rmp_pu - Vel.i_rpm_fb_pu);
}


double limit_value(double val, double up, double low)
{
    if (val > up)       return up;
    else if (val < low) return low;
    else                return val;

}

void McPmsmTiSmo::ti_smo(void)
{
    /*	Sliding mode current observer	*/	
    smo.EstIalpha = smo.Fsmopos * smo.EstIalpha + smo.Gsmopos*(smo.Valpha - smo.Ealpha - smo.Zalpha);
    smo.EstIbeta  = smo.Fsmopos * smo.EstIbeta  + smo.Gsmopos*(smo.Vbeta  - smo.Ebeta -  smo.Zbeta);

    /*	Current errors	*/	
    smo.IalphaError = smo.EstIalpha - smo.Ialpha;
    smo.IbetaError = smo.EstIbeta - smo.Ibeta;

    /*  Sliding control calculator	*/																	
	smo.IalphaError = limit_value (smo.IalphaError, smo.E0, -smo.E0);
    smo.IbetaError  = limit_value (smo.IbetaError, smo.E0, -smo.E0);
    smo.Zalpha = smo.IalphaError * 2 * smo.Kslide;
    smo.Zbeta  = smo.IbetaError  * 2 * smo.Kslide;

    /*	Sliding control filter -> back EMF calculator	*/
    smo.Ealpha = smo.Ealpha + smo.Kslf * (smo.Zalpha - smo.Ealpha);
    smo.Ebeta = smo.Ebeta + smo.Kslf * (smo.Zbeta - smo.Ebeta);

    /*	Rotor angle calculator -> Theta = atan(-Ealpha,Ebeta)	*/	
    smo.Theta = atan2(-smo.Ealpha,  smo.Ebeta);

    while(smo.Theta>2*M_PI) smo.Theta-=2*M_PI;
    while(smo.Theta<0) smo.Theta+=2*M_PI;

    
}

void McPmsmTiSmo::ti_speed_est(void)
{
    speed.Temp = speed.EstimatedTheta - speed.OldEstimatedTheta;

    if (speed.Temp < -0.5)
    {
        speed.Temp += 1.0;
    }
    else if (speed.Temp > 0.5)
    {
        speed.Temp -= 1.0;
    }

    speed.Temp = speed.K1 * speed.Temp;

    speed.Temp = speed.K2 * speed.EstimatedSpeed + speed.K3 *speed.Temp;

    speed.Temp = limit_value(speed.Temp, 1, -1);
    speed.EstimatedSpeed = speed.Temp;

    speed.OldEstimatedTheta = speed.EstimatedTheta;

    speed.EstimatedSpeedRpm = speed.EstimatedSpeed* speed.BaseRpm;
}


void McPmsmTiSmo::simulation(void)
{
    static  uint32_t last_us = 0;
    uint32_t us = simPrm.real_time*1000000;

    Ctrl.ctrl_mode = PMSM_BASIC_CTRL_VEL;
    Ctrl.id = 0;
    Ctrl.iq = 6 ;
    Ctrl.vel = 2000;


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
        smo.Valpha = foc.m_vol_alpha_pu;
        smo.Vbeta  = foc.m_vol_beta_pu;
        smo.Ialpha = foc.m_cur_alpha_pu;
        smo.Ibeta = foc.m_cur_beta_pu;

        ti_smo();

        speed.EstimatedTheta = smo.Theta/(2.0*3.141593);
        /*angle pu to rpm pu, hz*60/base_rpm/npp */
        ti_speed_est();



        if (us> 50000)
        {

            /*get pmsm feedback */
            foc.i_ele_angle = smo.Theta;
            Vel.i_rpm_fb_pu = speed.EstimatedSpeedRpm / pmsm.cfg.nameplate.rated_vel;

        }
        else
        {
            foc.i_ele_angle = pmsm.out.theta_elec;
            Vel.i_rpm_fb_pu = pmsm.out.rpm / pmsm.cfg.nameplate.rated_vel;
        }

        if (us> 1000000)
        {
            pmsm.in.TL = 0.4101;
        }
        else
        {
            pmsm.in.TL = 0.0;
        }

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

void McPmsmTiSmo::wavePlot(void)
{
    emit signal_appendWave("theta","theta",simPrm.real_time,pmsm.out.theta_elec);
    emit signal_appendWave("theta","observer",simPrm.real_time,smo.Theta);

    emit signal_appendWave("rmp","rmp",simPrm.real_time, pmsm.out.rpm);
    emit signal_appendWave("rmp","speed",simPrm.real_time, speed.EstimatedSpeedRpm);

    emit signal_appendWave("emf","alpha",simPrm.real_time,pmsm.out.emf_alpha);
    emit signal_appendWave("emf","Ealpha",simPrm.real_time,(smo.Ealpha )*pmsm.cfg.nameplate.rated_vol);
    emit signal_appendWave("emf","Zalpha",simPrm.real_time,(smo.Zalpha )*pmsm.cfg.nameplate.rated_vol);

    emit signal_appendWave("udq","Uq",simPrm.real_time,pmsm.out.Uq);
    emit signal_appendWave("udq","Ud",simPrm.real_time,pmsm.out.Ud);

    emit signal_appendWave("Iq","Iq_ref",simPrm.real_time,foc.i_ref_Iq_pu * pmsm.cfg.nameplate.rated_cur);
    emit signal_appendWave("Iq","Iq_fb",simPrm.real_time,foc.m_Iq_pu * pmsm.cfg.nameplate.rated_cur);

    emit signal_appendWave("Id","Id_fb",simPrm.real_time,foc.m_Id_pu * pmsm.cfg.nameplate.rated_cur);
    emit signal_appendWave("Id","Id_ref",simPrm.real_time,foc.i_ref_Id_pu* pmsm.cfg.nameplate.rated_cur);

    emit signal_appendWave("i","alpha",simPrm.real_time, smo.Ialpha);
    emit signal_appendWave("i","alpha_err",simPrm.real_time, smo.IalphaError);
    emit signal_appendWave("i","alpha_est",simPrm.real_time, smo.EstIalpha);
}
