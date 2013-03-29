/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Wim Meeussen */

/**
 * \file p3d_load_model.cpp
 * \brief Fonction qui charge un fichier Collada 1.5 dans les structures p3d.
 * \author F. Lancelot
 * \version 0.1
 * \date 26 août 2011
 *
 * Programme qui charge un fichier Collada 1.5 dans les structures p3d.
 *      - Le modèle COLLADA 1.5 est parsé par la fonction initFile de la classe URDFModel.
 *      - Le parsing retourne un modèle URDF (qui décrit sous forme d'un arbre les links et joints du modèle).
 *      - La fonction urdf_p3d_converter() crée à l'aide de ce modèle, un modèle p3d.
 *
 */

#include <boost/algorithm/string.hpp>
#include <vector>
#include <fstream>
#include <iostream>

#include "urdf_p3d_converter.h"
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

int p3d_load_model(char* filename, char* modelName, double scale)
{
    urdf::URDFModel model;
    std::cout << "Fichier " << filename << " en cours de parsing."<< std::endl;

    if(!model.initFile(filename))
    {
        std::cerr << "ERROR: Model Parsing the xml failed" << std::endl;
        return -1;
    }

    std::cout << "Fichier " << filename << " en cours de conversion."<< std::endl;
    urdf_p3d_converter( &model, modelName, scale );

    return 0;
}

namespace urdf{

bool URDFModel::initFile(const std::string& filename)
{

    // get the entire file
    std::string xml_string;
    std::fstream xml_file(filename.c_str(), std::fstream::in);
    if (xml_file.is_open())
    {
        while ( xml_file.good() )
        {
            std::string line;
            std::getline( xml_file, line);
            xml_string += (line + "\n");
        }
        xml_file.close();
        return URDFModel::initString(xml_string);
    }
    else
    {
        std::cout << "Could not open file " << filename.c_str() << " for parsing" << std::endl;
        return false;
    }

}

bool URDFModel::initString(const std::string& xml_string)
{
    boost::shared_ptr<ModelInterface> model;

    // std::cout << "Parsing robot urdf xml string" << std::endl;
    model = parseURDF(xml_string);

    // copy data from model into this object
    if (model){
        this->links_ = model->links_;
        this->joints_ = model->joints_;
        this->materials_ = model->materials_;
        this->name_ = model->name_;
        this->root_link_ = model->root_link_;
        return true;
    }
    else
        return false;
}

}// namespace
