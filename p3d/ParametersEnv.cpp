/*
 *  ParamtersEnv.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 21/09/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "ParametersEnv.hpp"

#include <iostream>

using namespace std;

//**********************************************************
//**********************************************************
boolContainer::boolContainer(bool v) :
    _Value(v) {
}

bool boolContainer::get() {
    return (_Value);
}

void boolContainer::set(bool v) {
    if (_Value != v) {
        _Value = v;
#ifdef QT_LIBRARY
        emit valueChanged(v);
#endif
    }
}

//----------------------------------------------------------
//----------------------------------------------------------
intContainer::intContainer(int v) :
    _Value(v) {
}

int intContainer::get() {
    return (_Value);
}

void intContainer::set(int v) {
    if (_Value != v) {
        _Value = v;
#ifdef QT_LIBRARY
        emit valueChanged(v);
#endif
    }
}

//----------------------------------------------------------
//----------------------------------------------------------
doubleContainer::doubleContainer(double v) :
    _Value(v) {
}

double doubleContainer::get() {
    return (_Value);
}

void doubleContainer::set(double v)
{
    //	cout << "set" << endl;

    if (_Value != v) {
        _Value = v;
#ifdef QT_LIBRARY
        emit valueChanged(v);
#endif
    }
}

//----------------------------------------------------------
//----------------------------------------------------------

stringContainer::stringContainer(std::string v) :
    _Value(v) {
}

std::string stringContainer::get() {
    return (_Value);
}

void stringContainer::set(std::string v) {
    if (_Value != v) {
        _Value = v;
#ifdef QT_LIBRARY
        emit valueChanged(v);
#endif
    }
}

//----------------------------------------------------------
//----------------------------------------------------------
vectorContainer::vectorContainer(std::vector<double> v) :
    _Value(v) {
}

vector<double> vectorContainer::get() {
    return (_Value);
}

void vectorContainer::set(std::vector<double> v) {
    if (_Value != v) {
        _Value = v;
#ifdef QT_LIBRARY
        emit valueChanged(v);
#endif
    }
}
