/**
 * @copyright   Copyright wangchongwei 
 * @license:    GNU GPLv2
 * @brief:      TI: Digital Motor Control Software Library: Target Independent Math Blocks         
 * @date        2022.09.30
 * @changelog:
 * date         author          notes
 * 2022.09.30   wangchongwei    create file 
 **/

#ifndef _MC_PMSM_TI_SMO_HPP_
#define _MC_PMSM_TI_SMO_HPP_


#include "../../../module/simobj/SimObj.hpp"
#include "../../../model/motor/basic_pmsm/basic_pmsm.hpp"
#include "../../../lib/controller/pid/dynamic_clamp_pid.hpp"


typedef struct {  
    double  Valpha;   	// Input: Stationary alfa-axis stator voltage 
    double  Ealpha;   	// Variable: Stationary alfa-axis back EMF 
    double  Zalpha;      // Output: Stationary alfa-axis sliding control 
    double  Gsmopos;    	// Parameter: Motor dependent control gain 
    double  EstIalpha;   // Variable: Estimated stationary alfa-axis stator current 
    double  Fsmopos;    	// Parameter: Motor dependent plant matrix 
    double  Vbeta;   	// Input: Stationary beta-axis stator voltage 
    double  Ebeta;  		// Variable: Stationary beta-axis back EMF 
    double  Zbeta;      	// Output: Stationary beta-axis sliding control 
    double  EstIbeta;    // Variable: Estimated stationary beta-axis stator current 
    double  Ialpha;  	// Input: Stationary alfa-axis stator current 
    double  IalphaError; // Variable: Stationary alfa-axis current error                 
    double  Kslide;     	// Parameter: Sliding control gain 
    double  Ibeta;  		// Input: Stationary beta-axis stator current 
    double  IbetaError;  // Variable: Stationary beta-axis current error                 
    double  Kslf;       	// Parameter: Sliding control filter gain 
    double  Theta;     	// Output: Compensated rotor angle
    double  E0;			// Parameter: 0.5 	
    
} SMOPOS;	

typedef struct {
    double EstimatedTheta;  	// Input: Electrical angle (pu) 
    double OldEstimatedTheta;   // History: Electrical angle at previous step (pu)
    double EstimatedSpeed;      // Output: Estimated speed in per-unit  (pu)
    uint32_t BaseRpm;     		// Parameter: Base speed in rpm (Q0) - independently with global Q
    double K1;       			// Parameter: Constant for differentiator (Q21) - independently with global Q
    double K2;     				// Parameter: Constant for low-pass filter (pu)
    double K3;     				// Parameter: Constant for low-pass filter (pu)
    int32_t EstimatedSpeedRpm; // Output : Estimated speed in rpm  (Q0) - independently with global Q
    double Temp;				// Variable : Temp variable
} SPEED_ESTIMATION;  	// Data type created 

class McPmsmTiSmo : public SimObj
{

public:
    explicit McPmsmTiSmo(SimObj *parent = nullptr);

    ~McPmsmTiSmo();
    virtual void sim_run(void);

private:
    virtual void dynamic_ode(double *dx, double *x, double *u);
    virtual void run(void);

    void simulation(void);
    void wavePlot(void);
    void foc_cur_loop(void);
    void vel_loop(void);
    void ti_smo(void);
    void ti_speed_est(void);

    /* data */
    Basic_Pmsm_Space::BaiscPmsm  pmsm;

    DynClpPID  Iq_pid, Id_pid;
    DynClpPID  vel_pid;

    enum PMSM_BASIC_CTRL
    {
        PMSM_BASIC_CTRL_CUR,
        PMSM_BASIC_CTRL_VEL,
    };

    struct
    {
        enum PMSM_BASIC_CTRL ctrl_mode;
        double iq;
        double id;
        double vel;
    }Ctrl;

    SMOPOS smo;
    SPEED_ESTIMATION speed;
    
    struct
    {
        double i_ele_angle;

        double i_cur_pu[3];    // a b c cur
        double i_ref_Iq_pu;
        double i_ref_Id_pu;

        double m_cur_alpha_pu;
        double m_cur_beta_pu;

        double m_Id_pu;
        double m_Iq_pu;

        double m_vol_alpha_pu;
        double m_vol_beta_pu;

        double o_Vq_pu;
        double o_Vd_pu;

        double o_Ua_pu;
        double o_Ub_pu;
        double o_Uc_pu;
    }foc;

    struct
    {
        double i_rpm_fb_pu;
        double i_ref_rmp_pu;    // a b c cur

        double o_tor_ctrl_pu;
    }Vel;
};

#endif
