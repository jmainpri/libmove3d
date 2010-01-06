#ifndef QT_DIFF_WIN
#define QT_DIFF_WIN

#include "../qtBase/qtBaseWindow.hpp"

#ifdef QWT
#include "../qtPlot/tempWin.hpp"
#endif
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
#ifdef QWT
        TempWin* plotWin;
#endif

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
