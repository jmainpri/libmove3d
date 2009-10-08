#ifndef QT_GREE_WIN
#define QT_GREE_WIN

#include "../qtBase/qtBaseWindow.hpp"

/**
 * @ingroup qtWidget
 * @brief Greedy Planner Window
 */
class qtGreedyWindow : public qtBaseWindow
{

	Q_OBJECT;

private:
/*	QVGroupBox* expansionProcessBox;
	QVGroupBox* costSpacesBox;*/
	QWidget* spacer;
public:
	qtGreedyWindow();
	void  init();
	~qtGreedyWindow();

public slots:
	void greedyPlan();
	void setCostCriterium(int choise);

};


#endif
