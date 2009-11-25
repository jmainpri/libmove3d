//
// C++ Implementation: workspace
//
// Description: 
//
//
// Author: Florian Pilardeau,B90,6349 <fpilarde@jolimont>, (C) 2009
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "project.h"

using namespace std;

Project::Project()
{
}

Project::Project(string nameMainEnv)
{
	if(XYZ_ROBOT == NULL)
	{
		cout << "Warning: XYZ_ROBOT doesn't exist" << endl;
	}

        Robot* xyz_Robot = new Robot(XYZ_ROBOT);
        Scene* BasicEnv = new Scene("MainEnv",xyz_Robot);
        insertScene(BasicEnv);
        setActivScene("MainEnv");
}


Project::~Project()
{
}

Scene* Project::getActivScene()
{
        for(uint i = 0; i < _Scenes.size(); i = i + 1)
	{
                if(_Scenes[i]->getName() == _activScene)
		{
                        return _Scenes[i];
		}
	}
        Scene* env = new Scene(_activScene);
        this->insertScene(env);
	return env;
}

void Project::setActivScene(string name)
{
        _activScene = name;
        for(uint i = 0; i < _Scenes.size(); i = i + 1)
	{
                if(_Scenes[i]->getName() == _activScene)
		{
			return;
		}
	}
        Scene* env = new Scene(_activScene);
        this->insertScene(env);
}

void Project::insertScene(Scene* E)
{
        _Scenes.push_back(E);
}

Project WS;

