/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2
 * @brief:      adrc 2st
 * @date        2023.05.13
 * @changelog:
 * date         author          notes
 * 2023.05.13   wangchongwei    create file 
 **/

#ifndef  _LADRC_2ST_H_
#define  _LADRC_2ST_H_

class LADRC_2st
{

public:
    LADRC_2st(/* args */);
    ~LADRC_2st();

    void setCtrlParm(double wc, double wo, double b0,double zeta,double ts);
    void setClampParm(double out_up, double out_low);

    void reset(void);

    double ladrc_2st(double ref,double fb);


    // observer
    double z[3];
    double dz[3];

private:
    double ts;
    double wc;
    double wo;
    double zeta;
    double b0;
    double out_up;
    double out_low;

    double ref;
    double fb;
    double out;


    double beta1;
    double beta2;
    double beta3;
    double l1;
    double l2;
};


#endif
