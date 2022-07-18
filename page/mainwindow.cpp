#include "mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    PgWave = new WaveWidget(ui->widget);

    //pSimObj = new CAN_Lqr();
    //pSimObj = new CAN_FollowDesired();
    pSimObj = new CAN_Luenberger();
    //pSimObj = new CAN_BasicFeedback();

    pSimObj->simPrm.end_time = 18;
    pSimObj->simPrm.step_size = 0.00001;
    pSimObj->simPrm.sample_freq_div = 10;

    connect(pSimObj, SIGNAL(signal_appendWave(QString,QString, double, double)), PgWave, SLOT(appendWaveData(QString,QString, double, double)));
    connect(pSimObj, SIGNAL(signal_showAllGraph()), PgWave, SLOT(slot_showAllGraph()));

    pSimObj->start();
    //pSimObj->sim_run();

}

MainWindow::~MainWindow()
{
    delete ui;
}

