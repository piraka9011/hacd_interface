#include <vector>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <HACD/hacdHACD.h>
#include <pcl_conversions/pcl_conversions.h>

#ifndef PROJECT_HACDINTERFACE_H
#define PROJECT_HACDINTERFACE_H

using HACD::Vec3;
using HACD::Real;
using namespace pcl;
using namespace std;

class HACDInterface {
public:
    HACDInterface() {};
    void polygonMeshFromPointsTriangles(pcl::PolygonMesh& mesh,
                                        const vector< Vec3< Real > >& points,
                                        const vector< Vec3<long> >& triangles);
    void pointsTrianglesFromPolygonMesh(const pcl::PolygonMesh& mesh,
                                        vector< Vec3<Real> >& points,
                                        vector< Vec3<long> >& triangles);
    vector<pcl::PolygonMesh::Ptr> ConvexDecompHACD(const pcl::PolygonMesh& mesh,
                                                   float concavity);

};
#endif //PROJECT_HACDINTERFACE_H
