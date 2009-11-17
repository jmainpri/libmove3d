#include "kcdpropertieswindow.hpp"
#include "ui_kcdpropertieswindow.hpp"
#include <iostream>

using namespace std;

KCDpropertiesWindow::KCDpropertiesWindow(QWidget *parent) :
    QDialog(parent),
    m_ui(new Ui::KCDpropertiesWindow)
{
    m_ui->setupUi(this);

    connect(m_ui->horizontalScrollTol,SIGNAL(valueChanged(int)),
            this,SLOT( setLineEditFromScrollBar() )
            );

}

KCDpropertiesWindow::~KCDpropertiesWindow()
{
    delete m_ui;
}

void KCDpropertiesWindow::changeEvent(QEvent *e)
{
    QDialog::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        m_ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

void KCDpropertiesWindow::setLineEditFromScrollBar()
{
    double newValue = (double)m_ui->horizontalScrollTol->value();

    double maximum = (double)m_ui->horizontalScrollTol->maximum();
    double minimum = (double)m_ui->horizontalScrollTol->minimum();

    double max = 100;
    double min = 0;

    double doubleValue = ((newValue - minimum) * (max - min)) / (maximum - minimum) + min ;

    m_ui->labelTol->setText(QString::number(doubleValue));
}

#include "moc_kcdpropertieswindow.cpp"