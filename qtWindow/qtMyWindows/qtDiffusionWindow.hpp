#ifndef QT_DIFF_WIN
#define QT_DIFF_WIN

#include "../qtBase/qtBaseWindow.hpp"


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

public slots:
	void saveCostTemperature();
	void costEnv();

public:
	qtDiffusionWindow();
	void  init();
	~qtDiffusionWindow();

};


#endif
