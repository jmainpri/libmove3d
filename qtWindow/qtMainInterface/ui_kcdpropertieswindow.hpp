/********************************************************************************
** Form generated from reading UI file 'kcdpropertieswindow.ui'
**
** Created: Mon Dec 14 17:55:11 2009
**      by: Qt User Interface Compiler version 4.6.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef KCDPROPERTIESWINDOW_H
#define KCDPROPERTIESWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QComboBox>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QScrollBar>

QT_BEGIN_NAMESPACE

class Ui_KCDpropertiesWindow
{
public:
    QDialogButtonBox *buttonBox;
    QScrollBar *horizontalScrollTol;
    QScrollBar *horizontalScrollVol;
    QScrollBar *horizontalScrollDmax;
    QLabel *labelTol;
    QLabel *labelVol;
    QLabel *labelDmax;
    QLineEdit *lineEditVol;
    QLineEdit *lineEditTol;
    QLineEdit *lineEditDmax;
    QGroupBox *groupBox;
    QComboBox *SlectionBox;
    QComboBox *ComputationBox;
    QLabel *SelectionLabel;
    QLabel *ComputationLabel;

    void setupUi(QDialog *KCDpropertiesWindow)
    {
        if (KCDpropertiesWindow->objectName().isEmpty())
            KCDpropertiesWindow->setObjectName(QString::fromUtf8("KCDpropertiesWindow"));
        KCDpropertiesWindow->resize(555, 355);
        buttonBox = new QDialogButtonBox(KCDpropertiesWindow);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setGeometry(QRect(200, 310, 341, 32));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
        horizontalScrollTol = new QScrollBar(KCDpropertiesWindow);
        horizontalScrollTol->setObjectName(QString::fromUtf8("horizontalScrollTol"));
        horizontalScrollTol->setGeometry(QRect(170, 270, 160, 16));
        horizontalScrollTol->setOrientation(Qt::Horizontal);
        horizontalScrollVol = new QScrollBar(KCDpropertiesWindow);
        horizontalScrollVol->setObjectName(QString::fromUtf8("horizontalScrollVol"));
        horizontalScrollVol->setGeometry(QRect(170, 300, 160, 16));
        horizontalScrollVol->setOrientation(Qt::Horizontal);
        horizontalScrollDmax = new QScrollBar(KCDpropertiesWindow);
        horizontalScrollDmax->setObjectName(QString::fromUtf8("horizontalScrollDmax"));
        horizontalScrollDmax->setGeometry(QRect(170, 330, 160, 16));
        horizontalScrollDmax->setOrientation(Qt::Horizontal);
        labelTol = new QLabel(KCDpropertiesWindow);
        labelTol->setObjectName(QString::fromUtf8("labelTol"));
        labelTol->setGeometry(QRect(20, 270, 59, 18));
        labelTol->setAlignment(Qt::AlignCenter);
        labelVol = new QLabel(KCDpropertiesWindow);
        labelVol->setObjectName(QString::fromUtf8("labelVol"));
        labelVol->setGeometry(QRect(20, 300, 59, 18));
        labelVol->setAlignment(Qt::AlignCenter);
        labelDmax = new QLabel(KCDpropertiesWindow);
        labelDmax->setObjectName(QString::fromUtf8("labelDmax"));
        labelDmax->setGeometry(QRect(20, 330, 59, 18));
        labelDmax->setAlignment(Qt::AlignCenter);
        lineEditVol = new QLineEdit(KCDpropertiesWindow);
        lineEditVol->setObjectName(QString::fromUtf8("lineEditVol"));
        lineEditVol->setGeometry(QRect(100, 290, 51, 28));
        lineEditTol = new QLineEdit(KCDpropertiesWindow);
        lineEditTol->setObjectName(QString::fromUtf8("lineEditTol"));
        lineEditTol->setGeometry(QRect(100, 260, 51, 28));
        lineEditDmax = new QLineEdit(KCDpropertiesWindow);
        lineEditDmax->setObjectName(QString::fromUtf8("lineEditDmax"));
        lineEditDmax->setGeometry(QRect(100, 320, 51, 28));
        groupBox = new QGroupBox(KCDpropertiesWindow);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(280, 10, 261, 131));
        SlectionBox = new QComboBox(groupBox);
        SlectionBox->setObjectName(QString::fromUtf8("SlectionBox"));
        SlectionBox->setGeometry(QRect(150, 40, 91, 26));
        ComputationBox = new QComboBox(groupBox);
        ComputationBox->setObjectName(QString::fromUtf8("ComputationBox"));
        ComputationBox->setGeometry(QRect(150, 80, 91, 26));
        SelectionLabel = new QLabel(groupBox);
        SelectionLabel->setObjectName(QString::fromUtf8("SelectionLabel"));
        SelectionLabel->setGeometry(QRect(70, 50, 59, 18));
        SelectionLabel->setAlignment(Qt::AlignCenter);
        ComputationLabel = new QLabel(groupBox);
        ComputationLabel->setObjectName(QString::fromUtf8("ComputationLabel"));
        ComputationLabel->setGeometry(QRect(48, 90, 81, 20));
        ComputationLabel->setAlignment(Qt::AlignCenter);

        retranslateUi(KCDpropertiesWindow);
        QObject::connect(buttonBox, SIGNAL(accepted()), KCDpropertiesWindow, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), KCDpropertiesWindow, SLOT(reject()));
        QObject::connect(buttonBox, SIGNAL(accepted()), KCDpropertiesWindow, SLOT(close()));
        QObject::connect(buttonBox, SIGNAL(rejected()), KCDpropertiesWindow, SLOT(close()));

        QMetaObject::connectSlotsByName(KCDpropertiesWindow);
    } // setupUi

    void retranslateUi(QDialog *KCDpropertiesWindow)
    {
        KCDpropertiesWindow->setWindowTitle(QApplication::translate("KCDpropertiesWindow", "Dialog", 0, QApplication::UnicodeUTF8));
        labelTol->setText(QApplication::translate("KCDpropertiesWindow", "Tol.", 0, QApplication::UnicodeUTF8));
        labelVol->setText(QApplication::translate("KCDpropertiesWindow", "Vol.", 0, QApplication::UnicodeUTF8));
        labelDmax->setText(QApplication::translate("KCDpropertiesWindow", "DMax.", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("KCDpropertiesWindow", "Bounding Box", 0, QApplication::UnicodeUTF8));
        SlectionBox->clear();
        SlectionBox->insertItems(0, QStringList()
         << QApplication::translate("KCDpropertiesWindow", "All", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("KCDpropertiesWindow", "None", 0, QApplication::UnicodeUTF8)
        );
        ComputationBox->clear();
        ComputationBox->insertItems(0, QStringList()
         << QApplication::translate("KCDpropertiesWindow", "Close", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("KCDpropertiesWindow", "Large", 0, QApplication::UnicodeUTF8)
         << QApplication::translate("KCDpropertiesWindow", "Col.", 0, QApplication::UnicodeUTF8)
        );
        SelectionLabel->setText(QApplication::translate("KCDpropertiesWindow", "Selection", 0, QApplication::UnicodeUTF8));
        ComputationLabel->setText(QApplication::translate("KCDpropertiesWindow", "Computation", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class KCDpropertiesWindow: public Ui_KCDpropertiesWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // KCDPROPERTIESWINDOW_H
