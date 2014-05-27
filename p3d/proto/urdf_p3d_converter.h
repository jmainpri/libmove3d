/*
 * Copyright (c) 2001-2014 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 */
#include <urdf_model/model.h>

/**
 * \file urdf_p3d_converter.h
 * \brief Fonction qui parcourt une structure URDF C++ et crée un modèle p3d associé.
 * \author F. Lancelot
 * \version 0.1
 * \date 26 août 2011
 *
 * Programme qui parcourt une structure URDF C++ et crée un modèle p3d associé.
 * Le modèle URDF ne semble pas proposer de structures pour sauvegarder les vertices et indices.
 *
 */

namespace urdf
{
class URDFModel: public ModelInterface
{
public:
    /// \brief Load Model given a filename
    bool initFile(const std::string& filename);
private:
    /// \brief Load Model from a XML-string
    bool initString(const std::string& xmlstring);
};
}

int urdf_p3d_converter(urdf::URDFModel* model, char* modelName, double scale);

