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

#ifndef _MC_DC_BRUSH_CTRL_HPP_
#define _MC_DC_BRUSH_CTRL_HPP_


#include "../../../module/simobj/SimObj.hpp"
#include "../../../model/motor/dc_brush/dc_brush.hpp"
#include "../../../lib/controller/pid/dynamic_clamp_pid.hpp"

class McDcBrushCtrl : public SimObj
{
public:
    explicit McDcBrushCtrl(SimObj *parent = nullptr);

    ~McDcBrushCtrl();
    
    virtual void sim_run(void);
private:

    virtual void dynamic_ode(double *dx, double *x, double *u);
    virtual void run(void);

    void simulation(void);
    void wavePlot(void);
    /* data */

    BDC_Space::DcBrush  bdc;

    DynClpPID  vel_pid;
    DynClpPID  cur_pid;

    double vel_dm_pu;
    double cur_dm_pu;
    double vol_dm_pu;

};





#endif
