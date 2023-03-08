/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2
 * @brief:      adrc 1st
 * @date        2023.03.05
 * @changelog:
 * date         author          notes
 * 2023.03.05   wangchongwei    create file 
 **/

#include "ladrc_1st.hpp"


LADRC_1st::LADRC_1st(/* args */)
{
    this->b0 = 0;
    this->wc = 0;
    this->wo = 0;
    this->out_up  = 0;
    this->out_low = 0;

    reset();
}

LADRC_1st::~LADRC_1st()
{

}

void LADRC_1st::reset(void)
{
    this->out = 0;

    this->z[0]  = 0;
    this->dz[0] = 0; 
    this->z[1]  = 0;
    this->dz[1] = 0; 
} 

void LADRC_1st::setCtrlParm(double wc, double wo, double b0,double ts)
{
    this->b0 = b0;
    this->wc = wc;
    this->wo = wo;
    this->ts = ts;
}
void LADRC_1st::setClampParm(double out_up, double out_low)
{
    this->out_up = out_up;
    this->out_low = out_low;
}

double LADRC_1st::ladrc_1st(double ref,double fb)
{
    // leso
    dz[0] = 2*wo*(fb-z[0])+z[1]+b0*out;
    dz[1] = wo*wo*(fb-z[0]);

    z[0] += ts*dz[0];
    z[1] += ts*dz[1];

    // controller
    out = (wc*(ref-z[0])-z[1])/b0;

    if (out > out_up) out = out_up;
    if (out < out_low) out = out_low;

    return  out;
}






