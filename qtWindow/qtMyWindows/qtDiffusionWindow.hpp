#ifndef QT_DIFF_WIN
#define QT_DIFF_WIN

#include "../qtBase/qtBaseWindow.hpp"
#include "../qtPlot/plotWin.hpp"

/**
 * @ingroup qtWidget
 * @brief Diffusion Window
 */
class qtDiffusionWindow : public qtBaseWindow
{

	Q_OBJECT;

private:
	QVGroupBox* expansionProcessBox;
	QVGroupBox* costSpacesBox;
	QWidget* spacer;
	PlotWindow* plotWin;

public slots:
	void showPlotWindow();
	void saveCostTemperature();
	void costEnv();

public:
	qtDiffusionWindow();
	void  init();
	~qtDiffusionWindow();

};


#endif
