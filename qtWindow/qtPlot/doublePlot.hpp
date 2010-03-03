#ifndef DOUBLEPLOT_HPP
#define DOUBLEPLOT_HPP

#include "basicPlot.hpp"

#include<vector>

//const int PLOT_SIZE = 100;      // 0 to 200

/**
  * @ingroup qtPlot
  * @brief Qt simple plot relies on qwt
  */
class DoublePlot : public QwtPlot
{
    Q_OBJECT

public:
    DoublePlot(QWidget* = NULL);

    int getPlotSize() { return PLOT_SIZE; }
    void setData(std::vector<double> data1,std::vector<double> data2);
    void rescale();

private:
    void alignScales();

    double d_x[PLOT_SIZE];
    double d_y[PLOT_SIZE];
    double d_z[PLOT_SIZE];

    bool init;
    double Max_y;
    double Max_z;

    QwtPlotCurve *cData1;
    QwtPlotCurve *cData2;
};

#endif // DOUBLEPLOT_HPP
