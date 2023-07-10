/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2
 * @brief:      adrc 2st
 * @date        2023.05.13
 * @changelog:
 * date         author          notes
 * 2023.05.13   wangchongwei    create file 
 **/

#include "ladrc_2st.hpp"


LADRC_2st::LADRC_2st(/* args */)
{
    this->b0 = 0;
    this->wc = 0;
    this->wo = 0;
    this->out_up  = 0;
    this->out_low = 0;

    reset();
}

LADRC_2st::~LADRC_2st()
{

}

void LADRC_2st::reset(void)
{
    this->ref = 0;
    this->fb  = 0;
    this->out = 0;


    this->z[0]  = 0;
    this->dz[0] = 0; 
    this->z[1]  = 0;
    this->dz[1] = 0; 
    this->z[2]  = 0;
    this->dz[2] = 0;
} 

void LADRC_2st::setCtrlParm(double wc, double wo, double b0,double zeta,double ts)
{
    this->b0 = b0;
    this->wc = wc;
    this->wo = wo;
    this->ts = ts;
    this->zeta = zeta;

    this->beta1 = 3 * wo;
    this->beta2 = 3 * wo*wo;
    this->beta3 =  wo*wo*wo;

    this->l1 = 2*zeta*wc;
    this->l2 = wc*wc;

}
void LADRC_2st::setClampParm(double out_up, double out_low)
{
    this->out_up = out_up;
    this->out_low = out_low;
}

double LADRC_2st::ladrc_2st(double ref,double fb)
{
  
    // leso
    dz[0] = beta1*(fb-z[0])+z[1];
    dz[1] = beta2*(fb-z[0])+z[2]+b0*out;
    dz[2] = beta3*(fb-z[0]);

    z[0] += ts*dz[0];
    z[1] += ts*dz[1];
    z[2] += ts*dz[2];

    // control
    out = (l2*(ref-z[0])-l1*z[1]-z[2])/b0;

    if (out > out_up) out = out_up;
    if (out < out_low) out = out_low;

    return  out;
}






