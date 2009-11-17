#ifndef MOVEROBOT_HPP
#define MOVEROBOT_HPP

#include "p3d_sys.h"
namespace Ui {
    class MoveRobot;
}

class JointSlider
{
    public:

    QLabel* label;
    QLineEdit* lineEdit;
    QScrollBar* horizontalScrollBar;

    double min;
    double max;
};

class MoveRobot : public QWidget {
    Q_OBJECT
public:
    MoveRobot(QWidget *parent = 0);
    ~MoveRobot();

protected:
    void changeEvent(QEvent *e);

private:
    Ui::MoveRobot *m_ui;

    void initSliders(void);
    JointSlider* makeSlider(QString& dofName);

    std::vector<JointSlider*> Sliders;
};

#endif // MOVEROBOT_HPP
