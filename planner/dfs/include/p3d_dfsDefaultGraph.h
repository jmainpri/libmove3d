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
#ifndef __P3DDFSDEFAULTGRAPH_H__
#define __P3DDFSDEFAULTGRAPH_H__

#include "Planner-pkg.h"

class DfsDefaultGraph: public Dfs{
  public:
    DfsDefaultGraph();
    virtual ~DfsDefaultGraph();
    void* p3d_dfs(void* graph, void* node);
  protected:
    virtual void deleteNodeList(void);
    virtual void initGraph(void* graph);
    virtual void setDiscovered(void* node, bool value);
    virtual bool getDiscovered(void* node);
    virtual void setProceeded(void* node, bool value);
    virtual bool getProceeded(void* node);
    virtual void processNode(void* node);
    virtual void processEdge(void* edge, int id);
    virtual int getNodeNbEdges(void* node);
    virtual void* getEdgeNode(void* node, int id);
    virtual bool validEdge(void* edge, int id);
};

#endif
