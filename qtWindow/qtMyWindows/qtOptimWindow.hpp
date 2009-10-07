#ifndef QT_OPTIM_WIN
#define QT_OPTIM_WIN

#include "../qtBase/qtBaseWindow.hpp"

/**
 * @ingroup qtWidget
 * @brief Optimization Window
 */
class qtOptimWindow : public qtBaseWindow
{
	Q_OBJECT;

private:
	QWidget* spacer;

public:
	qtOptimWindow();
	void  init();

	~qtOptimWindow();


public slots:
	void optimizeCost();
	void shortCutCost();
	void extractBestTraj();
	void computeGrid();
	void computeGridAndExtract();
	void removeRedundant();
	void graphSearchTest();
	void setCostCriterium(int choise);

};
#endif

