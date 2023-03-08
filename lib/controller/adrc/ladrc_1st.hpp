/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2
 * @brief:      adrc 1st
 * @date        2023.03.05
 * @changelog:
 * date         author          notes
 * 2023.03.05   wangchongwei    create file 
 **/

#ifndef  _LADRC_1ST_H_
#define  _LADRC_1ST_H_

class LADRC_1st
{
public:
    LADRC_1st(/* args */);
    ~LADRC_1st();
    void setCtrlParm(double wc, double wo, double b0,double ts);
    void setClampParm(double out_up, double out_low);
    void reset(void);
    double ladrc_1st(double ref,double fb);

    // observer
    double z[2];
    double dz[2];
private:
    double ts;
    double wc;
    double wo;
    double b0;
    double out_up;
    double out_low;
    double out;
};

#endif
