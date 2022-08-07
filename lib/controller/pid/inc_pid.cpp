/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2
 * @brief:      Incremental pid controller
 * @date        2021.11.21
 * @changelog:
 * date         author          notes
 * 2021.11.21   wangchongwei    create file 
 * 2022.07.31   wangchongwei    cpp, float->double
 **/

#include "inc_pid.hpp"


IncPID::IncPID(/* args */)
{

}

IncPID::~IncPID()
{
}

void IncPID::setCtrlParm( double kp, double ki, double kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}
void IncPID::setClampParm( double out_up, double out_low)
{
    m_out_up = out_up;
    m_out_low = out_low;    
}

double IncPID::P  ( double err)
{
    i_err = err;

    m_delta_out = m_kp*(i_err - m_last_err);

    m_pre_out = m_delta_out + m_sum;
    o_out = m_pre_out;


    /*output clamp*/
    if (o_out > m_out_up)
    {
        o_out = m_out_up;
    }
    else if (o_out < m_out_low)
    {
        o_out = m_out_low;
    }

    m_sum = o_out;

    m_last_err = i_err;

    return o_out;    
}
double IncPID::PI ( double err)
{
    i_err = err;

    m_delta_out = m_kp*(i_err - m_last_err) + m_ki*i_err;

    m_pre_out = m_delta_out + m_sum;
    o_out = m_pre_out;

    /*output clamp*/
    if (o_out > m_out_up)
    {
        o_out = m_out_up;
    }
    else if (o_out < m_out_low)
    {
        o_out = m_out_low;
    }
    
    m_sum = o_out;

    m_last_err = i_err;

    return o_out;    
}
double IncPID::PID( double err)
{
    i_err = err;

    m_delta_out =   m_kp*(i_err - m_last_err) + 
                        m_ki*i_err+
                        m_kd*(i_err -2*m_last_err + m_last_pre_err);

    m_pre_out = m_delta_out + m_sum;
    o_out = m_pre_out;

    /*output clamp*/
    if (o_out > m_out_up)
    {
        o_out = m_out_up;
    }
    else if (o_out < m_out_low)
    {
        o_out = m_out_low;
    }
    
    m_sum = o_out;

    m_last_err = i_err;
    m_last_pre_err = m_last_err;

    return o_out;    
}