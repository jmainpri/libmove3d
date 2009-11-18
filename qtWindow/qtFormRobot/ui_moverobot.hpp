/********************************************************************************
** Form generated from reading ui file 'moverobot.ui'
**
** Created: Mon Nov 16 18:57:09 2009
**      by: Qt User Interface Compiler version 4.5.3
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
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
        Q_UNUSED(MoveRobot);
    } // retranslateUi

};

namespace Ui {
    class MoveRobot: public Ui_MoveRobot {};
} // namespace Ui

QT_END_NAMESPACE

#endif // MOVEROBOT_H
