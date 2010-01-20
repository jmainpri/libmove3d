#ifndef QT_PRM_WIN
#define QT_PRM_WIN

#include "../qtBase/qtBaseWindow.hpp"

/**
 * @ingroup qtOldWidget
 * @brief PRM Window
 */
class qtPRMWindow : public qtBaseWindow
{

	Q_OBJECT;

public slots:
	void runPRMAlgo();

public:
	qtPRMWindow();
	void  init();
	~qtPRMWindow();


};


#endif
