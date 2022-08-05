#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    PgWave = new WaveWidget(ui->widget);

    //pSimObj = new CAN_Lqr();
    //pSimObj = new CAN_FollowDesired();
    //pSimObj = new CAN_Luenberger();
    //pSimObj = new CAN_BasicFeedback();
    //pSimObj = new CAN_AdaptiveCtrl();
    //pSimObj = new CAN_SlidingMode();
    //pSimObj = new CAN_BackStep();
    pSimObj = new CAN_RobustCtrlCompare();

    pSimObj->simPrm.end_time = 5;
    pSimObj->simPrm.step_size = 0.000001;
    pSimObj->simPrm.sample_freq_div = 100;

    connect(pSimObj, SIGNAL(signal_appendWave(QString,QString, double, double)), PgWave, SLOT(appendWaveData(QString,QString, double, double)));
    connect(pSimObj, SIGNAL(signal_showAllGraph()), PgWave, SLOT(slot_showAllGraph()));

    pSimObj->start();

    //pSimObj->sim_run();

}

MainWindow::~MainWindow()
{
    delete ui;
}

