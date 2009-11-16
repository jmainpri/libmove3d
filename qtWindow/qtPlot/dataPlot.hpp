#ifndef _DATA_PLOT_H
#define _DATA_PLOT_H 1

#include <qwt_plot.h>

const int PLOT_SIZE = 201;      // 0 to 200

class DataPlot : public QwtPlot
{
    Q_OBJECT

public:
    DataPlot(QWidget* = NULL);

public slots:
	void setMax(double max);
    void setTimerInterval(double interval);

protected:
    virtual void timerEvent(QTimerEvent *e);

private:
    void alignScales();

    double d_x[PLOT_SIZE]; 
    double d_y[PLOT_SIZE]; 
    double d_z[PLOT_SIZE];

    bool init;
    double Max_y;
    double Max_z;

    int d_interval; // timer in ms
    int d_timerId;
};

#endif
