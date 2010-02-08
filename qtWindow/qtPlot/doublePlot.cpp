#include <stdlib.h>
#include <qwt_painter.h>
#include <qwt_plot_canvas.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_curve.h>
#include <qwt_scale_widget.h>
#include <qwt_legend.h>
#include <qwt_scale_draw.h>
#include <qwt_math.h>
#include "doublePlot.hpp"
#include "p3d/env.hpp"
#include <iostream>
#include <algorithm>
//
//  Initialize main window
//

using namespace std;

DoublePlot::DoublePlot( QWidget *parent):
    QwtPlot(parent)

{
    // Disable polygon clipping
    QwtPainter::setDeviceClipping(false);

    // We don't need the cache here
//    canvas()->setPaintAttribute(QwtPlotCanvas::PaintCached, false);
//    canvas()->setPaintAttribute(QwtPlotCanvas::PaintPacked, false);

#if QT_VERSION >= 0x040000
#ifdef Q_WS_X11
    /*
       Qt::WA_PaintOnScreen is only supported for X11, but leads
       to substantial bugs with Qt 4.2.x/Windows
     */
    canvas()->setAttribute(Qt::WA_PaintOnScreen, true);
#endif
#endif

//    alignScales();

    //  Initialize data
    for (int i = 0; i< PLOT_SIZE; i++)
    {
        d_x[i] = i;     // time axis
        d_y[i] = i;
        d_z[i] = i;
    }

    // Assign a title
    setTitle("Cost along trajectory");
    insertLegend(new QwtLegend(), QwtPlot::BottomLegend);

    // Insert new curves
    cData1 = new QwtPlotCurve("Cost1");
    cData1->setPen(QPen(Qt::red));
    cData1->setRawData(d_x, d_y, PLOT_SIZE);
    cData1->attach(this);

    cData2 = new QwtPlotCurve("Cost2");
    cData2->setPen(QPen(Qt::blue));
    cData2->setRawData(d_x, d_z, PLOT_SIZE);
    cData2->attach(this);

    init = false;
    Max_y = 0.0;
    Max_z = 0.0;
    replot();
}

//
//  Set a plain canvas frame and align the scales to it
//
void DoublePlot::alignScales()
{
    // The code below shows how to align the scales to
    // the canvas frame, but is also a good example demonstrating
    // why the spreaded API needs polishing.

    canvas()->setFrameStyle(QFrame::Box | QFrame::Plain );
    canvas()->setLineWidth(1);

    for ( int i = 0; i < QwtPlot::axisCnt; i++ )
    {
        QwtScaleWidget *scaleWidget = (QwtScaleWidget *)axisWidget(i);

        if ( scaleWidget )
            scaleWidget->setMargin(0);

        QwtScaleDraw *scaleDraw = (QwtScaleDraw *)axisScaleDraw(i);

        if ( scaleDraw )
            scaleDraw->enableComponent(QwtAbstractScaleDraw::Backbone, false);
    }
}

void DoublePlot::rescale()
{
//    Max_y = *std::max_element(d_y,d_y+PLOT_SIZE);
//    setAxisScale(QwtPlot::yLeft, 0.0,Max_y*1.10);
}

//  Generate new values
void DoublePlot::setData(std::vector<double> data1,std::vector<double> data2)
{
    cout << "Adding data to plot" << endl;

    cout << "data1.size = "  << data1.size() << endl;
    cout << "data2.size = "  << data2.size() << endl;

    if(ENV.getBool(Env::initPlot) == false )
    {
        Max_y = *std::max_element(data1.begin(),data1.end());
        Max_z = *std::max_element(data2.begin(),data2.end());

        double max;
        if(Max_y > Max_z)
        {
            max = Max_y;
        }
        else
        {
            max = Max_z;
        }

        cout << "Setting fixed Axis"<< endl;
        setAxisScale(QwtPlot::yLeft, 0.0,max*1.10);

        ENV.setBool(Env::initPlot,true);
    }

    for ( int i = 0; i<PLOT_SIZE ; i++)
    {
        d_y[i] = data1[i];
        d_z[i] = data2[i];
//        cout << y[i] << endl;
    }

    replot();
//    updateLayout();
//    alignScales();

    cout << "Replot" << endl;
}
