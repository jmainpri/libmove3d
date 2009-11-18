#include "plotwindow.hpp"
#include "ui_plotwindow.hpp"

PlotWindow::PlotWindow(QWidget *parent) :
    QWidget(parent),
    m_ui(new Ui::PlotWindow)
{
    m_ui->setupUi(this);
}

PlotWindow::~PlotWindow()
{
    delete m_ui;
}

void PlotWindow::changeEvent(QEvent *e)
{
    QWidget::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        m_ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

void PlotWindow::setPlot(BasicPlot* plot)
{
    m_ui->plot = plot;
}

BasicPlot* PlotWindow::getPlot()
{
    return m_ui->plot;
}


#include "moc_plotwindow.cpp"
