#ifndef QT_VISU_WIN
#define QT_VISU_WIN

#include "../qtBase/qtBaseWindow.hpp"

/**
 * @ingroup qtWidget
 * @brief Visualization Window
 */
class qtVisuWindow : public qtBaseWindow
{
	Q_OBJECT;

private:
	QWidget* spacer;

public:
	qtVisuWindow();
	void  init();

	~qtVisuWindow();


public slots:
	void saveStat();
	void showStat();
	void resetCounters();

};
#endif

