#ifndef KCDPROPERTIESWINDOW_HPP
#define KCDPROPERTIESWINDOW_HPP

#include "../qtLibrary.h"

namespace Ui {
    class KCDpropertiesWindow;
}

class KCDpropertiesWindow : public QDialog {
    Q_OBJECT
    Q_DISABLE_COPY(KCDpropertiesWindow)
public:
    explicit KCDpropertiesWindow(QWidget *parent = 0);
    virtual ~KCDpropertiesWindow();

protected:
    virtual void changeEvent(QEvent *e);

private:
    Ui::KCDpropertiesWindow *m_ui;

private slots:
    void setLineEditFromScrollBar();
};

#endif // KCDPROPERTIESWINDOW_H
