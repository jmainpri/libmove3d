/*
 * histoWin.hpp
 *
 *  Created on: Sep 14, 2009
 *      Author: jmainpri
 */

#ifndef HISTOWIN_HPP_
#define HISTOWIN_HPP_

#include <qwt_plot.h>
#include <qmainwindow.h>

class HistoWindow  {

public:
	HistoWindow();

	void startWindow();

private:
    QwtPlot* plot;
};

#endif /* HISTOWIN_HPP_ */
