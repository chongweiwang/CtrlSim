/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2
 * @brief:      anti-windup pid, dynamic clamping Integrator
 * @date        2021.11.20
 * @changelog:
 * date         author          notes
 * 2021.11.20   wangchongwei    create file 
 * 2022.07.31   wangchongwei    cpp, float->double
 **/


#include "dynamic_clamp_pid.hpp"


DynClpPID::DynClpPID(/* args */)
{

}

DynClpPID::~DynClpPID()
{
    
}

void DynClpPID::setCtrlParm(double kp, double ki, double kd)
{
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

void DynClpPID::setClampParm(double out_up, double out_low)
{
    m_out_up = out_up;
    m_out_low = out_low;
}

double DynClpPID::P  (double err)
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

double DynClpPID::PI (double err)
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

    /*dynamic Integrator clamp*/
    m_sum += i_err * m_ki + o_out - m_pre_out ;

    return o_out;
}

double DynClpPID::PID(double err)
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

    /*dynamic Integrator clamp*/
    m_sum += i_err * m_ki + o_out - m_pre_out ;

    m_last_err = i_err;

    return o_out;
}
