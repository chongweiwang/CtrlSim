/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2 
 * @brief:      sim_obj.h  
 * @date        2022.07.07
 * @changelog:
 * date         author          notes
 * 2022.07.07   wangchongwei    first version   
 **/


#ifndef _SIM_OBJ_
#define _SIM_OBJ_

#include <QThread>
#include <cstdint>
#include "../solver/OdeSolver.hpp"


struct sim_prm_t
{
    double  step_size;
    double  sample_freq_div;
    double  end_time;

    double  real_time;
    int     sim_cnt;
};

class SimObj: public QThread, public OdeSolver
{
    Q_OBJECT
public:
    explicit SimObj(QThread *parent = nullptr);
    ~SimObj();

    virtual void sim_run(void)  = 0;

    struct sim_prm_t simPrm;
signals:
    void signal_appendWave(QString wave, QString ch, double x, double y);
    void signal_showAllGraph(void);

private:
    /* data */


};






#endif


