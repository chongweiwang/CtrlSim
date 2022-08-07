/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2
 * @brief:      static clamping Integrator
 * @date        2021.11.20
 * @changelog:
 * date         author          notes
 * 2021.11.20   wangchongwei    create file 
 * 2022.07.31   wangchongwei    cpp, float->double
 **/
#ifndef  _STATIC_CLAMP_PID_H_
#define  _STATIC_CLAMP_PID_H_

class StClpPID
{
public:
    StClpPID(/* args */);
    ~StClpPID();
    
    void setCtrlParm(double kp, double ki, double kd);
    void setClampParm(double out_up, double out_low, double integral_up,double integral_low);

    double P  (double err);
    double PI (double err);
    double PID(double err);

private:
    double i_err;
    double o_out;

    double  m_kp;
    double  m_ki;
    double  m_kd;

    double  m_pre_out;

    double  m_out_up;
    double  m_out_low;

    double  m_integral_up;
    double  m_integral_low;

    double  m_sum;
    double  m_last_err;

};


#endif