#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "module/wave/wave_widget.hpp"
#include "module/wave/plot_widget.hpp"


#include "example/DR_CAN/can_lqr/can_lqr.hpp"
#include "example/DR_CAN/can_follow_desired/can_follow_desired.hpp"
#include "example/DR_CAN/can_luenberger/can_luenberger.hpp"
#include "example/DR_CAN/can_basic_feedback/can_basic_feedback.hpp"

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
