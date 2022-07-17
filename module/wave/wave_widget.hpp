/**************************************************
 * @copyright (C)   wangchongwei
 * @file       : wave_widget.hpp
 * @author     : wangchongwei
 * @license:   : GNU GPLv2 
 * @date       : 2022/05/07
 * @description: 
 *      
 * @log:
 *  date        author          notes
 *  2022/05/07  wangchongwei    first version
 ***********************************************/

#ifndef _WAVE_WIDGET_H_
#define _WAVE_WIDGET_H_

#include <QObject>
#include <QWidget>
#include <QCheckBox>
#include <QGridLayout>
#include "plot_widget.hpp"

class WaveWidget:public QWidget
{
    Q_OBJECT
public:
    explicit WaveWidget(QWidget *parent= nullptr);
    ~WaveWidget();

    void addWave(QString wave);

public slots:
    void appendWaveData(QString wave, QString ch, double x, double y);
    void slot_showAllGraph(void);
protected:

private:
    uint32_t wave_size = 0;
    QMap<QString, PlotWidget*> wave_group;
    QGridLayout * mMainLayout;
    QGridLayout *mWaveLayout;

    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
};



#endif  /*_WAVE_WIDGET_H_*/
