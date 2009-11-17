#ifndef _BASIC_PLOT_H
#define _BASIC_PLOT_H

#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include<iostream>

const int PLOT_SIZE = 100;      // 0 to 200

class BasicPlot : public QwtPlot
{
    Q_OBJECT

public:
    BasicPlot(QWidget* = NULL);
    int getPlotSize() { return PLOT_SIZE; }
    void setData(std::vector<double> data);

private:
    void alignScales();

    double d_x[PLOT_SIZE]; 
    double d_y[PLOT_SIZE]; 

    bool init;
    double Max_y;

    QwtPlotCurve *cRight;
};

#endif
