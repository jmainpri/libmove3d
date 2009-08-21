#ifndef QT_EST_WIN
#define QT_EST_WIN
#include "../qtBase/qtBaseWindow.hpp"

/**
 * @ingroup qtWidget
 * @brief Expansive State space Window
 */
class qtESTWindow : public qtBaseWindow
{

	Q_OBJECT;

private:
/*	QVGroupBox* expansionProcessBox;
	QVGroupBox* costSpacesBox;*/
	QWidget* spacer;

public slots:
	void ESTPlan();

public:
	qtESTWindow();
	void  init();
	~qtESTWindow();

};


#endif
