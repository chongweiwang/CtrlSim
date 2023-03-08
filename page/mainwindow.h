#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "module/wave/wave_widget.hpp"
#include "module/wave/plot_widget.hpp"


#include "example/DR_CAN/can_lqr/can_lqr.hpp"
#include "example/DR_CAN/can_follow_desired/can_follow_desired.hpp"
#include "example/DR_CAN/can_luenberger/can_luenberger.hpp"
#include "example/DR_CAN/can_basic_feedback/can_basic_feedback.hpp"
#include "example/DR_CAN/can_adaptive_ctrl/can_adaptive_ctrl.hpp"
#include "example/DR_CAN/can_sliding_mode/can_sliding_mode.hpp"
#include "example/DR_CAN/can_back_step/can_back_step.hpp"
#include "example/DR_CAN/can_robust_ctrl_compare/can_robust_ctrl_compare.hpp"
#include "example/DR_CAN/can_kalman_recursive/can_kalman_recursive.hpp"


#include "example/MtCtrl/mc_dc_brush_ctrl/mc_dc_brush_ctrl.hpp"
#include "example/MtCtrl/mc_pmsm_basic_ctrl/mc_pmsm_basic_ctrl.hpp"
#include "example/MtCtrl/mc_pmsm_flux_observer/mc_pmsm_flux_observer.hpp"
#include "example/MtCtrl/mc_pmsm_scvm_sensorless/mc_pmsm_scvm_sensorless.hpp"
#include "example/MtCtrl/mc_pmsm_ti_smo/mc_pmsm_ti_smo.hpp"
#include "example/MtCtrl/mc_pmsm_ladrc_ctrl/mc_pmsm_ladrc_ctrl.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    WaveWidget * PgWave;
    PlotWidget * PgPlot;

    SimObj *pSimObj;

    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
