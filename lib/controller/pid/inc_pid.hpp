/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2
 * @brief:      Incremental pid controller
 * @date        2021.11.21
 * @changelog:
 * date         author          notes
 * 2021.11.21   wangchongwei    create file 
 * 2022.07.31   wangchongwei    cpp, double->double
 **/

#ifndef  _INC_PID_H_
#define  _INC_PID_H_


class IncPID
{
public:
    IncPID(/* args */);
    ~IncPID();

    void setCtrlParm( double kp, double ki, double kd);
    void setClampParm( double out_up, double out_low);

    double P  ( double err);
    double PI ( double err);
    double PID( double err);
    
private:
    double  i_err;
    double  o_out;

    double  m_kp;
    double  m_ki;
    double  m_kd;

    double  m_out_up;
    double  m_out_low;

    double  m_delta_out;
    double  m_sum;

    double  m_last_pre_err;
    double  m_last_err;

    double  m_pre_out;

};




#endif