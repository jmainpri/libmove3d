/*
 *  ParamtersEnv.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 21/09/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef PARAMETERS_ENV_HPP
#define PARAMETERS_ENV_HPP

#ifdef QT_LIBRARY
#undef CursorShape
#include <QtCore/QObject>
#endif

#include <map>
#include <utility>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Bool Containers
// ---------------------------------------------------------------------------
class boolContainer
#ifdef QT_LIBRARY
: public QObject
#endif
{
#ifdef QT_LIBRARY
	Q_OBJECT;
#endif
	bool _Value;
	
public:
	boolContainer(bool v = false);
	bool get();
#ifdef QT_LIBRARY
	public slots:
#endif
	void set(bool v);
	
#ifdef QT_LIBRARY
signals:
#endif
	void valueChanged(bool v);
};

// ---------------------------------------------------------------------------
// Int Containers
// ---------------------------------------------------------------------------
class intContainer
#ifdef QT_LIBRARY
: public QObject
#endif
{
#ifdef QT_LIBRARY
	Q_OBJECT;
#endif
	int _Value;
	
public:
	intContainer(int v = false);
	int get();
	
#ifdef QT_LIBRARY
	public slots:
#endif
	void set(int v);
	
#ifdef QT_LIBRARY
signals:
#endif
	void valueChanged(int v);
};

// ---------------------------------------------------------------------------
// Double Containers
// ---------------------------------------------------------------------------
class doubleContainer
#ifdef QT_LIBRARY
: public QObject
#endif
{
#ifdef QT_LIBRARY
	Q_OBJECT;
#endif
	double _Value;
	
public:
	doubleContainer(double v = false);
	double get();
	
#ifdef QT_LIBRARY
	public slots:
#endif
	void set(double v);
	
#ifdef QT_LIBRARY
signals:
#endif
	void valueChanged(double v);
};

// ---------------------------------------------------------------------------
// String Containers
// ---------------------------------------------------------------------------
/**
 * @ingroup Interface
 * @brief String Container
 */
class stringContainer
#ifdef QT_LIBRARY
: public QObject
#endif
{
#ifdef QT_LIBRARY
	Q_OBJECT;
#endif
    std::string _Value;
	
public:
    stringContainer(std::string v = "");
    std::string get();
	
#ifdef QT_LIBRARY
	public slots:
#endif
    void set(std::string v);
	
#ifdef QT_LIBRARY
signals:
#endif
    void valueChanged(std::string v);

};

// ---------------------------------------------------------------------------
// Vector Containers
// ---------------------------------------------------------------------------
/**
 * @ingroup Interface
 * @brief String Container
 */
class vectorContainer
#ifdef QT_LIBRARY
: public QObject
#endif
{
#ifdef QT_LIBRARY
	Q_OBJECT;
#endif
	std::vector<double> _Value;
	
public:
	vectorContainer() { }
	vectorContainer(std::vector<double>);
	std::vector<double> get();
	
#ifdef QT_LIBRARY
	public slots:
#endif
	void set(std::vector<double> v);
	
#ifdef QT_LIBRARY
signals:
#endif
	void valueChanged(std::vector<double> v);
	
};

// ---------------------------------------------------------------------------
// \brief Classe permettant de se lier à l'interface graphique
// ---------------------------------------------------------------------------

template <typename enumBool,
					typename enumInt,
					typename enumDouble,
					typename enumString,
					typename enumVector>
class Parameters
#ifdef QT_LIBRARY
: public QObject
#endif
{
public:
	
//#ifdef QT_LIBRARY
//	Q_OBJECT;
//#endif
	
public:
	/**
   * Constructeur de la classe
   */
	Parameters(	std::map<enumBool,		boolContainer*>				valuesBool,
						 std::map<enumInt,			intContainer*>				valuesInt,
						 std::map<enumDouble,		doubleContainer*>			valuesDouble,
						 std::map<enumString,		stringContainer*>			valuesString,
						 std::map<enumVector,		vectorContainer*>			valuesVector )
						 :
						 mBoolMap(valuesBool),
						 mIntMap(valuesInt),
						 mDoubleMap(valuesDouble),
						 mStringMap(valuesString),
						 mVectorMap(valuesVector)
	{
		
	}
	
	/**
   * Destructeur de la classe
   */
	~Parameters() { }
	
	bool getBool(enumBool p) {
		return (mBoolMap[p]->get());
	}
	
	void setBool(enumBool p, bool v) {
		mBoolMap[p]->set(v);
	}
	
	int getInt(enumInt p) {
		return (mIntMap[p]->get());
	}
	
	void setInt(enumInt p, int v) {
		mIntMap[p]->set(v);
	}
	
	double getDouble(enumDouble p) {
		return (mDoubleMap[p]->get());
	}
	
	void setDouble(enumDouble p, double v) {
		mDoubleMap[p]->set(v);
	}
	
	std::vector<double> getVector(enumVector p) {
		return (mVectorMap[p]->get());
	}
	
	void setVector(enumVector p, std::vector<double> v) {
		mVectorMap[p]->set(v);
	}
	
    std::string getString(enumString p) {
		return (mStringMap[p]->get());
	}
	
    void setString(enumString p, std::string v) {
		mStringMap[p]->set(v);
	}
	
#ifdef QT_LIBRARY
	QObject* getObject(enumInt p) {
		return (mIntMap[p]);
	}
	
	QObject* getObject(enumBool p) {
		return (mBoolMap[p]);
	}
	
	QObject* getObject(enumDouble p) {
		return (mDoubleMap[p]);
	}
	
	QObject* getObject(enumVector p) {
		return (mVectorMap[p]);
	}
	
	QObject* getObject(enumString p) {
		return (mStringMap[p]);
	}
#endif
	
	/**
   * Maps Getters and Setters
   */
	typedef std::pair<enumBool,					boolContainer*>		boolMap_t;
	typedef std::pair<enumInt,					intContainer*>		intMap_t;
	typedef std::pair<enumDouble,				doubleContainer*> doubleMap_t;
	typedef std::pair<enumString,				stringContainer*> stringMap_t;
	typedef std::pair<enumVector,				vectorContainer*> vectorMap_t;
	
	std::map<enumBool, boolContainer*>				getBoolMap() {return mBoolMap; }
	std::map<enumInt, intContainer*>					getIntMap() {return mIntMap; }
	std::map<enumDouble, doubleContainer*>		getDoubleMap() {return mDoubleMap; }
	std::map<enumString, stringContainer*>		getStringMap() {return mStringMap; }
	std::map<enumVector, vectorContainer*>		getVectorMap() {return mVectorMap; }
	
private:
	std::map<enumBool,		boolContainer*>				mBoolMap;
	std::map<enumInt,			intContainer*>				mIntMap;
	std::map<enumDouble,	doubleContainer*>			mDoubleMap;
	std::map<enumString,	stringContainer*>			mStringMap;
	std::map<enumVector,	vectorContainer*>			mVectorMap;
	
};

#endif
