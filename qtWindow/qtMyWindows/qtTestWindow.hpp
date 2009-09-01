#ifndef QT_TEST_WIN
#define QT_TEST_WIN

#include "../qtBase/qtBaseWindow.hpp"

#include <vector>

//#include "../userappli/testContext.hpp"

/**
 * @ingroup qtWidget
 * @brief Test Context Window
 */
class qtTestWindow : public qtBaseWindow
{
	Q_OBJECT;

private:
//	std::vector<testContext> context;
	QLineEdit* nameEdit;
	QListWidget* contextList;
	std::vector<QListWidgetItem *> itemList;

public:
	qtTestWindow();
	void  init();
	~qtTestWindow();

public slots:
	void saveContext();
	void printContext();
	void printAllContext();
	void resetContext();
	void setToSelected();

};

#endif
