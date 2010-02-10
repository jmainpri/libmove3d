#ifndef GP_VOL_INT_PROTO_H
#define GP_VOL_INT_PROTO_H

extern void readPolyhedron(p3d_polyhedre *poly, VOLINT_POLYHEDRON *p);
extern void compMassProperties(VOLINT_POLYHEDRON *p, Mass_properties *mass_prop);
extern void gpCompute_mass_properties(p3d_polyhedre *poly);

#endif
