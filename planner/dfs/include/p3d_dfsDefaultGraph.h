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
