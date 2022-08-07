/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2
 * @brief:      mc_dc_brush_ctrl
 * @date        2022.07.17
 * @changelog:
 * date         author          notes
 * 2021.10.24   wangchongwei    dc brush motor ctrl test
 * 2022.05.06   wangchongwei    cpp
 * 2022.07.17   wangchongwei    new version
 **/

#include "mc_dc_brush_ctrl.hpp"

McDcBrushCtrl::McDcBrushCtrl(SimObj *parent):SimObj(parent)
{
    BDC_Space::CfgPrm_t cfg;

    cfg.nameplate.rated_cur = 6;
    cfg.nameplate.rated_vol = 24;
    cfg.nameplate.rated_vel = 2500;

    cfg.R = 0.5;
    cfg.L = 0.003;
    cfg.K = 0.08;
    cfg.B = 0.0001;
    cfg.J = 0.001;
    bdc.setConfigPrm(cfg);

    /*set pid parm*/
    vel_pid.setCtrlParm(11.111,0.030,0);
    vel_pid.setClampParm(1.0,-1.0);

    /*set pid parm*/
    cur_pid.setCtrlParm(0.6,0.005,0);
    cur_pid.setClampParm(1.0,-1.0);

    vol_dm_pu = 0;
}

McDcBrushCtrl::~McDcBrushCtrl()
{
}


void McDcBrushCtrl::dynamic_ode(double *dx, double *x,double *u)
{
    Q_UNUSED(dx);
    Q_UNUSED(x);
    Q_UNUSED(u);
}

void McDcBrushCtrl::run(void)
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


void McDcBrushCtrl::sim_run(void)
{
    run();
}


void McDcBrushCtrl::simulation(void)
{
    static  uint32_t last_us = 0;
    /*get ms*/
    uint32_t us = simPrm.real_time*1000000;


    uint32_t vel_rpm_cmd = 1000;

    vel_dm_pu = vel_rpm_cmd/bdc.cfg.nameplate.rated_vel;


    /*50us,ctrl*/
    if (us - last_us > 50)
    {
        last_us = us;

        double vel_fb_pu = bdc.out.rpm/bdc.cfg.nameplate.rated_vel;
        cur_dm_pu = vel_pid.PI(vel_dm_pu - vel_fb_pu);

        double cur_fb_pu =bdc.out.Ia/bdc.cfg.nameplate.rated_cur;
        vol_dm_pu = cur_pid.PI(cur_dm_pu - cur_fb_pu);
    }

    bdc.in.Ua = bdc.cfg.nameplate.rated_vol*vol_dm_pu;
    bdc.in.TL = 0.05;


    bdc.simulation(simPrm.step_size,ODE_SOLVE_RK4);
}

void McDcBrushCtrl::wavePlot(void)
{
    emit signal_appendWave("rpm","rpm",simPrm.real_time,bdc.out.rpm);

    emit signal_appendWave("Ia","ref",simPrm.real_time,cur_dm_pu*bdc.cfg.nameplate.rated_cur);
    emit signal_appendWave("Ia","Ia",simPrm.real_time,bdc.out.Ia);
    emit signal_appendWave("theta","theta",simPrm.real_time,bdc.out.theta);
    emit signal_appendWave("Ua","Ua",simPrm.real_time,bdc.in.Ua);
}
