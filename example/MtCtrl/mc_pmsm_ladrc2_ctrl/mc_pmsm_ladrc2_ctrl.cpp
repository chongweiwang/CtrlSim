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


#include "mc_pmsm_ladrc2_ctrl.hpp"

McPmsmLadrc2Ctrl::McPmsmLadrc2Ctrl(SimObj *parent):SimObj(parent)
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


    // double bandwidth = 2000;
    // double cur_kPBase = pmsm.cfg.Lq * bandwidth * (pmsm.cfg.nameplate.rated_cur/pmsm.cfg.nameplate.rated_vol);
    // double cur_kIBase = pmsm.cfg.Rs * bandwidth * (pmsm.cfg.nameplate.rated_cur/pmsm.cfg.nameplate.rated_vol) / 20000;

    // /*set pid parm*/
    // Iq_pid.setCtrlParm(cur_kPBase,cur_kIBase,0);
    // Iq_pid.setClampParm(0.5,-0.5);

    // Id_pid.setCtrlParm(cur_kPBase,cur_kIBase,0);
    // Id_pid.setClampParm(0.5,-0.5);

    // double K = 1.5*pmsm.cfg.Pn * pmsm.cfg.psi / pmsm.cfg.J;
    // double delta = 5;
    // double vel_kiBase = (bandwidth/(delta*delta));
    // double vel_kpBase = (delta*vel_kiBase)/ K;
    // vel_kiBase = vel_kiBase*vel_kpBase /20000;

    // double vel2Cur = pmsm.cfg.nameplate.rated_vel * 0.10471976 / pmsm.cfg.nameplate.rated_cur;
    // /*set pid parm*/
    // vel_pid.setCtrlParm(vel_kpBase*vel2Cur,vel_kiBase*vel2Cur,0);
    // vel_pid.setClampParm(1.0,-1.0);

//    double cw = 2000;
//    double cb0 = 20000;
//    Iq_ladrc.setCtrlParm(cw,5*cw,cb0,10*cw,0.00005);
//    Iq_ladrc.setClampParm(0.5,-0.5);

//    Id_ladrc.setCtrlParm(cw,5*cw,cb0,10*cw,0.00005);
//    Id_ladrc.setClampParm(0.5,-0.5);

//    double vw = 500;
//    double vb0 = 140;
//    vel_ladrc.setCtrlParm(vw,5*vw,vb0,10*vw,0.00005);
//    vel_ladrc.setClampParm(1,-1);

        double cw = 1000;
        double cb0 = 10*cw*pmsm.cfg.nameplate.rated_vol;
        Iq_ladrc.setCtrlParm(cw,cw,cb0,1,0.00005);
        Iq_ladrc.setClampParm(0.5,-0.5);

        Id_ladrc.setCtrlParm(cw,cw,cb0,1,0.00005);
        Id_ladrc.setClampParm(0.5,-0.5);

        double vw = 300;
        double vb0 = 10*vw*pmsm.cfg.nameplate.rated_cur;
        vel_ladrc.setCtrlParm(vw,2*vw,vb0,1,0.00005);
        vel_ladrc.setClampParm(1,-1);
}

McPmsmLadrc2Ctrl::~McPmsmLadrc2Ctrl()
{

}


void McPmsmLadrc2Ctrl::dynamic_ode(double *dx, double *x,double *u)
{
    Q_UNUSED(dx);
    Q_UNUSED(x);
    Q_UNUSED(u);
}

void McPmsmLadrc2Ctrl::run(void)
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


void McPmsmLadrc2Ctrl::sim_run(void)
{
    run();
}

void McPmsmLadrc2Ctrl::foc_cur_loop(void)
{
    double st = sin(foc.i_ele_angle);
    double ct = cos(foc.i_ele_angle);

    /*2. cur ab(c) -> alpha beta -> dq*/
    ab_clark_amp(foc.i_cur_pu[0], foc.i_cur_pu[1], &foc.m_cur_alpha_pu, &foc.m_cur_beta_pu);
    park(foc.m_cur_alpha_pu, foc.m_cur_beta_pu, st, ct, &foc.m_Id_pu, &foc.m_Iq_pu);

    /*3. control iq id=0 -> get Vq Vd*/
    //foc.o_Vq_pu = Iq_pid.PI( foc.i_ref_Iq_pu - foc.m_Iq_pu);
    //foc.o_Vd_pu = Id_pid.PI( foc.i_ref_Id_pu - foc.m_Id_pu);

    foc.o_Vq_pu = Iq_ladrc.ladrc_2st(foc.i_ref_Iq_pu,foc.m_Iq_pu);
    foc.o_Vd_pu = Id_ladrc.ladrc_2st(foc.i_ref_Id_pu,foc.m_Id_pu);

    /*4. Vq Vd  -> abc pwm....... */
    inv_park(foc.o_Vd_pu, foc.o_Vq_pu,st, ct,&foc.m_vol_alpha_pu,&foc.m_vol_beta_pu);
    inv_clark_amp(foc.m_vol_alpha_pu,foc.m_vol_beta_pu,&foc.o_Ua_pu, &foc.o_Ub_pu, &foc.o_Uc_pu);
}

void McPmsmLadrc2Ctrl::vel_loop(void)
{
    Vel.o_tor_ctrl_pu  = vel_ladrc.ladrc_2st(Vel.i_ref_rmp_pu,Vel.i_rpm_fb_pu);
    //Vel.o_tor_ctrl_pu =  vel_pid.PI(Vel.i_ref_rmp_pu - Vel.i_rpm_fb_pu);
}

void McPmsmLadrc2Ctrl::simulation(void)
{
    static  uint32_t last_us = 0;
    uint32_t us = simPrm.real_time*1000000;

    Ctrl.ctrl_mode = PMSM_BASIC_CTRL_VEL;
    Ctrl.id = 0;
    Ctrl.iq = 4 ;

    if (us < 500000)
    {
        Ctrl.vel = 1000;
    }
    else
    {
        Ctrl.vel = 2000;
    }


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

    if (us < 1000000)
    {
        pmsm.in.TL = 0.1;
    }
    else
    {
        pmsm.in.TL = 0.4;
    }

    pmsm.simulation(simPrm.step_size,ODE_SOLVE_RK4);
}

void McPmsmLadrc2Ctrl::wavePlot(void)
{
    emit signal_appendWave("RPM","RPM",simPrm.real_time,pmsm.out.rpm);
    emit signal_appendWave("RPM","ref_RPM",simPrm.real_time,Ctrl.vel);

    emit signal_appendWave("rpm","RPM",simPrm.real_time,pmsm.out.rpm);
    emit signal_appendWave("rpm","ESO_RPM",simPrm.real_time,vel_ladrc.z[0] *pmsm.cfg.nameplate.rated_vel );
    emit signal_appendWave("rpm","ESO_d",simPrm.real_time,vel_ladrc.z[1] );

    emit signal_appendWave("Iq","Iq_ref",simPrm.real_time,foc.i_ref_Iq_pu * pmsm.cfg.nameplate.rated_cur);
    emit signal_appendWave("Iq","Iq_fb",simPrm.real_time,pmsm.out.Iq );
    emit signal_appendWave("Iq","Iq_eso",simPrm.real_time,Iq_ladrc.z[0] *pmsm.cfg.nameplate.rated_cur);
    emit signal_appendWave("Iq","Iq_eso_d",simPrm.real_time, Iq_ladrc.z[1] / (pmsm.cfg.nameplate.rated_vol*200) );

    emit signal_appendWave("Id","Id_fb",simPrm.real_time,pmsm.out.Id );
    emit signal_appendWave("Id","Id_ref",simPrm.real_time,foc.i_ref_Id_pu* pmsm.cfg.nameplate.rated_cur);
}
