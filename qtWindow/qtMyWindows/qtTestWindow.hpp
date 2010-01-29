#ifndef QT_TEST_WIN
#define QT_TEST_WIN

#include "../qtBase/qtBaseWindow.hpp"
#include "../util/CppApi/SaveContext.hpp"

#ifdef QWT
#include "../qtPlot/histoWin.hpp"
#endif

#include <vector>

//#include "../userappli/testContext.hpp"

/**
 * @ingroup qtOldWidget
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
	QPushButton* runAllRoundsRRT;
	QPushButton* runAllRoundsGreedy;
#ifdef QWT
	HistoWindow* histoWin;
#endif

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
	void runAllRRT();
	void runAllGreedy();
	void showHistoWindow();

};

#endif
