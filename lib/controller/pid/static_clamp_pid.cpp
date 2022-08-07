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

#include "static_clamp_pid.hpp"

StClpPID::StClpPID(/* args */)
{
}

StClpPID::~StClpPID()
{
}


void StClpPID::setCtrlParm(double kp, double ki, double kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

void StClpPID::setClampParm(double out_up, double out_low, double integral_up,double integral_low)
{
    m_out_up = out_up;
    m_out_low = out_low;
    m_integral_up = integral_up;
    m_integral_low = integral_low;
}

double StClpPID::P  (double err)
{
    i_err = err;

    m_pre_out = i_err * m_kp;

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

    return o_out;
}

double StClpPID::PI (double err)
{
    i_err = err;

    m_pre_out = i_err * m_kp + m_sum;

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

    /*static Integrator clamp*/
    m_sum += i_err * m_ki;
    if (m_sum > m_integral_up)
    {
        m_sum = m_integral_up;
    }
    else if (m_sum < m_integral_low)
    {
        m_sum = m_integral_low;
    }

    return o_out;
}

double StClpPID::PID(double err)
{
    i_err = err;

    m_pre_out = i_err * m_kp + m_sum + m_kd*(i_err - m_last_err);;

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
    
    /*static Integrator clamp*/
    m_sum += i_err * m_ki;
    if (m_sum > m_integral_up)
    {
        m_sum = m_integral_up;
    }
    else if (m_sum < m_integral_low)
    {
        m_sum = m_integral_low;
    }

    m_last_err = i_err;

    return o_out;
}
