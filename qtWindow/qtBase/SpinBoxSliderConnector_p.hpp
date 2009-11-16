/*
 *  Copyright (c) 2008 Cyrille Berger <cberger@cberger.net>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation;
 * version 2 of the license.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 */

#ifndef _SPINBOX_SLIDER_CONNECTOR_HPP_
#define _SPINBOX_SLIDER_CONNECTOR_HPP_

#include "p3d_sys.h"
#include "../../p3d/env.hpp"

class QDoubleSpinBox;
class QSlider;

namespace QtShiva
{
    class SpinBoxSliderConnector : public QObject
    {
        Q_OBJECT
    public:
        SpinBoxSliderConnector( QObject* _parent,
                                QDoubleSpinBox* _spinBox,
                                QSlider* _slider,
                                Env::doubleParameter p);

        ~SpinBoxSliderConnector();
        double value() const;
        void setValue( double _value );
    private slots:
        void spinBoxValueChanged( double _value );
        void sliderValueChanged( int _value );
    signals:
        void valueChanged( double _value );
    private:
        QDoubleSpinBox* m_spinBox;
        QSlider* m_slider;
    };
}

#endif
