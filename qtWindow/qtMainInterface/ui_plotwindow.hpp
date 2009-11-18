/********************************************************************************
** Form generated from reading UI file 'plotwindow.ui'
**
** Created: Tue Nov 17 16:50:30 2009
**      by: Qt User Interface Compiler version 4.6.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef PLOTWINDOW_H
#define PLOTWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QPushButton>
#include <QtGui/QWidget>
#include "../qtPlot/BasicPlot.hpp"

QT_BEGIN_NAMESPACE

class Ui_PlotWindow
{
public:
    BasicPlot *plot;
    QPushButton *pushButton;

    void setupUi(QWidget *PlotWindow)
    {
        if (PlotWindow->objectName().isEmpty())
            PlotWindow->setObjectName(QString::fromUtf8("PlotWindow"));
        PlotWindow->resize(658, 381);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(PlotWindow->sizePolicy().hasHeightForWidth());
        PlotWindow->setSizePolicy(sizePolicy);
        plot = new BasicPlot(PlotWindow);
        plot->setObjectName(QString::fromUtf8("plot"));
        plot->setGeometry(QRect(20, 10, 621, 321));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(plot->sizePolicy().hasHeightForWidth());
        plot->setSizePolicy(sizePolicy1);
        pushButton = new QPushButton(PlotWindow);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(280, 340, 113, 32));

        retranslateUi(PlotWindow);

        QMetaObject::connectSlotsByName(PlotWindow);
    } // setupUi

    void retranslateUi(QWidget *PlotWindow)
    {
        PlotWindow->setWindowTitle(QApplication::translate("PlotWindow", "Form", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("PlotWindow", "Scale", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class PlotWindow: public Ui_PlotWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // PLOTWINDOW_H
