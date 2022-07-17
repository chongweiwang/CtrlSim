/**************************************************
 * @copyright (C)   wangchongwei
 * @license:   : GNU GPLv2
 * @file       : wave_widget.hpp
 * @author     : wangchongwei
 * @version    :
 * @date       : 2022/05/07
 * @description: 
 *      
 * @log:
 *  date        author          notes
 *  2022/05/07  wangchongwei    first version
 ***********************************************/

#include "wave_widget.hpp"


WaveWidget::WaveWidget(QWidget *parent) : QWidget(parent)
{
    mMainLayout = new QGridLayout(parent);

    mWaveLayout = new QGridLayout(this);

    scrollArea = new QScrollArea(this);
    scrollArea->setObjectName(QString::fromUtf8("scrollArea"));
    scrollArea->setWidgetResizable(true);
    scrollAreaWidgetContents = new QWidget();
    scrollAreaWidgetContents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
    scrollAreaWidgetContents->setGeometry(QRect(0, 0, 438, 243));
    scrollArea->setWidget(scrollAreaWidgetContents);

    mMainLayout->addWidget(scrollArea);
    scrollAreaWidgetContents->setLayout(mWaveLayout);

}

WaveWidget::~WaveWidget()
{

}

void WaveWidget::addWave(QString wave)
{
    if (wave_group.contains(wave))   return;
    
    wave_group[wave] = new PlotWidget(this);
    wave_group[wave]->setMinimumSize(300, 200);
    wave_group[wave]->setTitle(wave);

    mWaveLayout->addWidget(wave_group[wave],wave_size/2,wave_size%2);
    wave_size++;

    mWaveLayout->setSpacing(1);

    this->adjustSize();
}

void WaveWidget::appendWaveData(QString wave, QString ch, double x, double y)
{
    addWave(wave);

    wave_group[wave]->appendCurveData(ch,x,y);
}

void WaveWidget::slot_showAllGraph(void)
{
    QMap<QString, PlotWidget*>::iterator iter = wave_group.begin();

    while(iter != wave_group.end())
    {
        iter.value()->slot_showAllGraph();
        iter++;
    }
}


