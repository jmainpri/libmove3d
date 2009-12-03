/********************************************************************************
** Form generated from reading UI file 'moverobot.ui'
**
** Created: Thu Dec 3 11:10:43 2009
**      by: Qt User Interface Compiler version 4.6.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef MOVEROBOT_H
#define MOVEROBOT_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MoveRobot
{
public:
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout;

    void setupUi(QWidget *MoveRobot)
    {
        if (MoveRobot->objectName().isEmpty())
            MoveRobot->setObjectName(QString::fromUtf8("MoveRobot"));
        MoveRobot->resize(400, 360);
        horizontalLayout = new QHBoxLayout(MoveRobot);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));

        horizontalLayout->addLayout(verticalLayout);


        retranslateUi(MoveRobot);

        QMetaObject::connectSlotsByName(MoveRobot);
    } // setupUi

    void retranslateUi(QWidget *MoveRobot)
    {
        MoveRobot->setWindowTitle(QApplication::translate("MoveRobot", "Form", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MoveRobot: public Ui_MoveRobot {};
} // namespace Ui

QT_END_NAMESPACE

#endif // MOVEROBOT_H
