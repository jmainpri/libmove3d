#ifndef QT_MAIN_WIN
#define QT_MAIN_WIN

#include "../qtLibrary.h"

#include "../qtBase/qtBaseWindow.hpp"
#include "../../p3d/env.hpp"

/**
 * @ingroup qtWindow
 * @defgroup qtOldWidget
 This is the first module that was an atempt to implement Qt in move3d
 */

/**
 * @ingroup qtOldWidget
 * @brief Main Widget class
 */
class MainWidget : public QWidget {

  Q_OBJECT;

  private:

  QBoxLayout* layout;
  QBoxLayout* master_layout;
  QBoxLayout* left_layout;

  QStackedWidget *stackedWidget;
  QListWidget *pageList;
  QPushButton *resetButton;
  QPushButton *runButton;
  QPushButton *stopButton;
  QPushButton *miscButton;

  public:

  MainWidget(QWidget* parent = 0);
  ~MainWidget();

  QCheckBox* createCheckBox(QString s, Env::boolParameter p);

  void setAllWindows();
  void addWindow(qtBaseWindow* Frame);

  public slots:

  void waitThreads();

  void drawGraph();
  void drawTraj();
  void run();
  void stop();
  void reset();
  void misc();
};

#endif
