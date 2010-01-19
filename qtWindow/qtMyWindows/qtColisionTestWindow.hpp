#ifndef QT_COLISIONTEST_WIN
#define QT_COLISIONTEST_WIN

#include "../qtBase/qtBaseWindow.hpp"
#include "../../userappli/CppApi/testModel.hpp"

/**
 * @ingroup qtOldWidget
 * @brief Collision Test Window
 */
class qtColisionTestWindow : public qtBaseWindow
{
	Q_OBJECT;

private:
	QWidget* spacer;
	TestModel* tests;

public:
	qtColisionTestWindow();
	void  init();
	~qtColisionTestWindow();


public slots:
	void colisions();
	void localpaths();
	void allTests();
	void distance();


};
#endif

