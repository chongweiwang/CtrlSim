/**************************************************
 * @copyright (C) wangchongwei
 * @file       : plot_widget.hpp
 * @license:   : GNU GPLv2 
 * @author     : wangchongwei
 * @version    :
 * @date       : 2022/03/22
 * @description: 
 *
 * @log:
 *  date        author          notes
 *  2022.3.22   wangchongwei    first version
 ***********************************************/
#ifndef _PLOT_WIDGET_H_
#define _PLOT_WIDGET_H_

/**
 * 1. 缩放  鼠标   按钮接口
 * 2. xy 游标测量
 * 3. 示波器触发功能
 * 4. 波形的选择　屏蔽功能
 * 5. 保存，和历史曲线的读取机制
 * 6.　保存图片
 * */

#include <QObject>
#include <QWidget>
#include <QCheckBox>
#include "qcustomplot.h"
#include <QVBoxLayout>

struct curve_data_t
{
    QVector<double> x_vct;
    QVector<double> y_vct;
};

struct Curve_t
{
    curve_data_t *pData;
    QCPGraph     *pGraph;
    uint16_t      index;
    bool          show;
    QCheckBox     *pBox;
    QString       showName;
};


class PlotWidget :public QWidget
{
    Q_OBJECT
public:
    explicit PlotWidget(QWidget *parent= nullptr);
    ~PlotWidget();


    void appendCurveData(QString name, double x, double y);
    void addCurve(QString name);
    void showPlot(void);
    void setXRange(double);

    void setCurveShowName(QString curveName, QString showName);
    void setTitle(QString title);

    //void setAutoScroll(bool enable){autoScroll = enable;}

    void saveWaveCsv(QString file_path);
    void openWaveCsv(QString file_path);
public slots:
    void slot_showAllGraph(void);
    void slot_autoScrollSwitch(bool sw);
    void slot_tracerSwitch(bool sw);

    void slot_clearWave(void);

    void slot_saveWave(void);
    void slot_importWave(void);
    void slot_mouseMove(QMouseEvent *event);

    void slot_selectChange(void);
    void slot_legendDoubleClick(QCPLegend *legend, QCPAbstractLegendItem *item, QMouseEvent *event);


    void slot_CheckBox(bool checked);
protected:
    void timerEvent(QTimerEvent *event) Q_DECL_OVERRIDE;

    void mouseMoveEvent(QMouseEvent *event) override;
private:
    void re_layout(void);

    QGridLayout   *mainLayout;
    QCustomPlot   *mCstmPlot;

    QCPItemTracer   *mTracer;
    QCPGraph        *mTracerGraph;
    QCPItemText     *mTracerXText;  //用于实时显示游标X值
    QCPItemText     *mTracerYText;  //用于实时显示游标Y值

    QCPTextElement     *mTitle;
    bool  tracerSw;


    QAction *action_clear;          /*action: clear*/
    QAction *action_save;           /*action: save*/
    QAction *action_import;         /*action: import*/
    QAction *action_show_all;       /*action: save*/
    QAction *action_auto_scroll;    /*action: auto scroll*/
    QAction *action_tracer;         /*action: tracer*/

    QMap<QString, Curve_t> curve_map;
    QVector<QColor> curveColor;

    QList<QCheckBox *>  boxList;

    double x_max;
    double y_max;
    bool  autoScroll;

};
#endif /*_PLOT_WIDGET_H_*/
