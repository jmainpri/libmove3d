#ifndef PLOTWINDOW_HPP
#define PLOTWINDOW_HPP

#include "p3d_sys.h"
#include "../qtPlot/BasicPlot.hpp"

namespace Ui {
    class PlotWindow;
}

class PlotWindow : public QWidget {
    Q_OBJECT
public:
    PlotWindow(QWidget *parent = 0);
    ~PlotWindow();

    void setPlot(BasicPlot* plot);
    BasicPlot* getPlot();

protected:
    void changeEvent(QEvent *e);

private:
    Ui::PlotWindow *m_ui;
};

#endif // PLOTWINDOW_HPP
