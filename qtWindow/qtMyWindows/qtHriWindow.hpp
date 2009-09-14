#ifndef QT_HRI_WIN
#define QT_HRI_WIN

#include "../qtBase/qtBaseWindow.hpp"

/**
 * @ingroup qtWidget
 * @brief HRI Cost Space Window
 */
class qtHriWindow : public qtBaseWindow
{
	Q_OBJECT;

private:
	QVGroupBox* akinBox;
	QVGroupBox* zoneBox;
	QVGroupBox* naturalBox;
	QWidget* spacer;

public:
	qtHriWindow();
	void  init();

	~qtHriWindow();

public slots:
	void setWhichTestSlot(int test);
	void computeFunctionGround(void);
	void computeCostTab(void);
	void computeCostGround(void);
	void changeColor(void);
	void enableHriSpace(void);
};
#endif

