/*
 *  HRICS_Distance.cpp
 *
 *
 *  Created by Jim Mainprice on 05/12/09.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#include "HRICS_Distance.h"

#define HUMANj_BODY 1
#define HUMANj_NECK_PAN 4
#define HUMANj_NECK_TILT 5
#define HUMANj_RHAND 29 /* or 30 or 31 */
#define HUMANj_LHAND 26 /* or 27 or 28 */

using namespace std;
using namespace tr1;

HRICS_Distance::HRICS_Distance()
{
    for (int i=0; i<XYZ_ENV->nr; i++)
    {
        string name(XYZ_ENV->robot[i]->name);

        if(name.find("ROBOT") != string::npos )
        {
            _Robot = new Robot(XYZ_ENV->robot[i]);
            cout << "Robot is " << name << endl;
        }

        if(name.find("HUMAN") != string::npos )
        {
            _Humans.push_back(new Robot(XYZ_ENV->robot[i]));
            cout << "Humans is " << name << endl;
        }
    }

    _SafeRadius = 0;
}

HRICS_Distance::HRICS_Distance(Robot* rob, vector<Robot*> humans) :
        _Robot(rob),
        _Humans(humans)
{
    //    cout << "New HRICS_Planner" << endl;
    _SafeRadius = 0;

}

HRICS_Distance::~HRICS_Distance()
{
    cout << "Delete Distance" << endl;
    for(int j=0; j<_Humans.size(); j++)
    {
        for(int i =0; i<_Humans[j]->getRobotStruct()->no; i++)
        {
            string body = _Humans[j]->getRobotStruct()->o[i]->name;

            if( body.find("safety_zone_graphic") != string::npos )
            {
                //                cout << "i = " << i << endl;
                for(int k=0;k< _Humans[j]->getRobotStruct()->o[i]->np ; k++)
                {
                    //                    cout << "_Humans[j]->getRobotStruct()->o[i]->np = " << _Humans[j]->getRobotStruct()->o[i]->np << endl;
                    /* the shape : 0,1,2=polyh,
                                 * 3=sphere, 4=cube,
                                 * 5=box, 6=cylinder,
                                 * 7=cone */

                    int shape = _Humans[j]->getRobotStruct()->o[i]->pol[k]->entity_type;

                    if( shape==0 || shape==1 || shape == 2)
                    {
                        //                        cout << "Shape is oval cylinder" << endl;
                        break;
                    }

                    offSetPrim(_Humans[j]->getRobotStruct()->o[i]->pol[k],-_SafeRadius);
                    //                    cout << "Set object " << i <<" with offset " << -_SafeRadius << endl;
                }
            }
        }
    }
}

void HRICS_Distance::parseHumans()
{

    string body;
    string b_name;
    string buffer;

    _SafetyZonesBodyId.clear();

    for(int i=0;i<_Humans.size();i++)
    {
        Robot* human = _Humans[i];
        vector<int> safetyZonesId;

        //        cout << "Looking for important zones on " << human->getName() << endl;
        for(int j=0;j<human->getRobotStruct()->no;j++)
        {
            body = human->getRobotStruct()->o[j]->name;
            size_t found;
            b_name = body;

            //            cout << "Looking for " << body << endl;

            //            buffer = b_name.substr(0,10);
            //            cout << "buffer = " << buffer << endl;

            if( body.find("safety_zone_ghost") != string::npos)
            {
                safetyZonesId.push_back(j);
                //                cout << "safetyZonesId += " << j << endl;
                continue;
            }

            if( body.find("safety_zone_graphic") != string::npos)
            {
                //                cout << "Found safety Zones" << endl;
                for(int k=0;k< human->getRobotStruct()->o[j]->np ; k++)
                {
                    //                    cout << "j = " << j << endl;
                    //                    cout << "human->getRobotStruct()->o[j]->np = " <<human->getRobotStruct()->o[j]->np << endl;
                    /* the shape : 0,1,2=polyh,
                                 * 3=sphere, 4=cube,
                                 * 5=box, 6=cylinder,
                                 * 7=cone */

                    int shape = human->getRobotStruct()->o[j]->pol[k]->entity_type;

                    if( shape==0 || shape==1 || shape == 2)
                    {
                        //                        cout << "Shape is oval cylinder" << endl;
                        break;
                    }

                    if(shape == 3){
                        //                        cout << "Shape is sphere" << endl;
                    }
                    if(shape==4){
                        //                        cout << "Shape is cube" << endl;
                    }
                    if(shape==5){
                        //                        cout << "Shape is box" << endl;
                    }
                    if(shape==6){
                        //                        cout << "Shape is cylinder" <<endl;
                    }
                    if(shape==7){
                        //                        cout << "Shape is cone" << endl;
                    }

                    _SafeOffset = ENV.getDouble(Env::zone_size) - _SafeRadius;
                    offSetPrim(human->getRobotStruct()->o[j]->pol[k],_SafeOffset);
                    //                    cout << "offSetPrim on " << body << endl;
                }
            }
        }
        _SafetyZonesBodyId.push_back(safetyZonesId);
    }
    _SafeRadius = ENV.getDouble(Env::zone_size);
    //    activateNormalMode();
}

/**
 * Changes dynamically the size of the zone shown
 * in the OpenGl display
 */
void HRICS_Distance::offSetPrim(p3d_poly* poly,double offset)
{
    // p3d_scale_poly(poly->poly, scale, scale, scale);
    // polyhedre.h
    //	p3d_poly *poly;

    if (poly->primitive_data)
    {
        poly->primitive_data->radius += offset;
        poly->primitive_data->other_radius += offset;
        poly->primitive_data->height += (2*offset);
        poly->primitive_data->x_length += (2*offset);
        poly->primitive_data->y_length += (2*offset);
        poly->primitive_data->z_length += (2*offset);
    }

    if(poly->box.x1 > 0)
        poly->box.x1 += offset;
    else
        poly->box.x1 -= offset;

    if(poly->box.x2 > 0)
        poly->box.x2 += offset;
    else
        poly->box.x2 -= offset;

    if(poly->box.y1 > 0)
        poly->box.y1 += offset;
    else
        poly->box.y1 -= offset;

    if(poly->box.y2 > 0)
        poly->box.y2 += offset;
    else
        poly->box.y2 -= offset;

    if(poly->box.z1 > 0)
        poly->box.z1 += offset;
    else
        poly->box.z1 -= offset;

    if(poly->box.z2 > 0)
        poly->box.z2 += offset;
    else
        poly->box.z2 -= offset;
}

double HRICS_Distance::getDistance()
{

}

/**
 * p3d_DeactivateHris
 */
void HRICS_Distance::activateSafetyZonesMode()
{
    //sort(zone_id.begin(), zone_id.end());

    for(int j=0; j<_Humans.size(); j++)
    {
        //        cout << _Robot->getRobotStruct() << endl;

        p3d_col_activate_rob_rob(_Robot->getRobotStruct(),_Humans[j]->getRobotStruct());

        for(int i =0; i<_Humans[j]->getRobotStruct()->no; i++)
        {
            if(binary_search(_SafetyZonesBodyId[j].begin(), _SafetyZonesBodyId[j].end(),i))
            {
                                p3d_col_activate_rob_obj(_Robot->getRobotStruct(),
                                                         _Humans[j]->getRobotStruct()->o[i]);
                //                                cout << "Activate " << _Robot->getName() << " to " << _Humans[j]->getRobotStruct()->o[i]->name << endl;
            }
            else
            {
                                p3d_col_deactivate_rob_obj(_Robot->getRobotStruct()
                                                           ,_Humans[j]->getRobotStruct()->o[i]);
            }
        }
    }
}

void HRICS_Distance::activateNormalMode()
{
    for(int j=0; j<_Humans.size(); j++)
    {
        p3d_col_activate_rob_rob(_Robot->getRobotStruct(),
                                 _Humans[j]->getRobotStruct());

        for(int i =0; i<_Humans[j]->getRobotStruct()->no; i++)
        {
            if(binary_search(_SafetyZonesBodyId[j].begin(),
                             _SafetyZonesBodyId[j].end(),i))
            {
                p3d_col_deactivate_rob_obj(_Robot->getRobotStruct(),
                                           _Humans[j]->getRobotStruct()->o[i]);

                p3d_col_deactivate_rob_obj(_Humans[j]->getRobotStruct(),
                                           _Humans[j]->getRobotStruct()->o[i]);
            }
        }
    }
}

vector<double> HRICS_Distance::getDistToZones()
{
    //    activateSafetyZonesMode();

    int nof_bodies = _Robot->getRobotStruct()->no;
    double* distances= MY_ALLOC(double,nof_bodies);

    p3d_vector3 *body= MY_ALLOC(p3d_vector3,nof_bodies);
    p3d_vector3 *other= MY_ALLOC(p3d_vector3,nof_bodies);

    int k=0;
    _PenetrationDist.resize(nof_bodies);

    if(ENV.getBool(Env::bbDist))
    {
        distances[k] = computeBBDist(body[k], other[k]);
        _PenetrationDist[k] = (_SafeRadius - distances[k])/_SafeRadius;
    }
    else
    {
        if(ENV.getBool(Env::isHriTS))
        {
            distances[k] = computeBoundingBalls(body[k], other[k]);
            _PenetrationDist[k] = (_SafeRadius - distances[k])/_SafeRadius;
        }
        else
        {
            activateSafetyZonesMode();

            switch (p3d_col_get_mode())
            {
            case p3d_col_mode_kcd:
                {
                    int settings = get_kcd_which_test();
                    set_kcd_which_test((p3d_type_col_choice)(20+3));
                    // 40 = KCD_ROB_ENV
                    // 3 = DISTANCE_EXACT

                    p3d_col_test_choice();
                    // Collision detection with other robots only

                    p3d_kcd_closest_points_between_bodies(_Robot->getRobotStruct(),body,other,distances);

                    double buffer = -numeric_limits<double>::max();
                    double radius = ENV.getDouble(Env::zone_size);

                    for(int i=0;i<nof_bodies;i++)
                    {
                        //        cout << "distance["<<i<<"] = "<< distances[i] << endl;
                        _PenetrationDist[i] = (_SafeRadius - distances[i])/_SafeRadius;

                        if(buffer<_PenetrationDist[i])
                        {
                            buffer = _PenetrationDist[i];
                            k = i;
                        }
                    }

                    set_kcd_which_test((p3d_type_col_choice)settings);// ROB_ALL + BOOL
                }
                break;

#ifdef PQP
            case p3d_col_mode_pqp:
                {
                    distances[k] =  pqp_robot_robot_distance(
                            _Robot->getRobotStruct(),
                            _Humans[k]->getRobotStruct(),
                            body[k],
                            other[k]);

                    _PenetrationDist[k] = (_SafeRadius - distances[k])/_SafeRadius;
                }
                break;
#endif
            }

            activateNormalMode();
        }
    }

    //    cout << " k = " << k << endl;

    /* ----------------------------------------------------
         * Vecteur de distance aux zones HRI
         **/
    vect_jim.clear();

    vect_jim.push_back(body[k][0]);
    vect_jim.push_back(body[k][1]);
    vect_jim.push_back(body[k][2]);
    vect_jim.push_back(other[k][0]);
    vect_jim.push_back(other[k][1]);
    vect_jim.push_back(other[k][2]);

    //    cout << "vect_jim[0] = " << vect_jim[0] << " vect_jim[1] = " << vect_jim[1] << " vect_jim[2] = " << vect_jim[2] << endl;
    //    cout << "vect_jim[3] = " << vect_jim[3] << " vect_jim[4] = " << vect_jim[4] << " vect_jim[5] = " << vect_jim[5] << endl;

    delete(distances);
    delete(body);
    delete(other);

    double _Cost;
    if( _PenetrationDist[k] < 0 )
    {
        _Cost = 0;
    }
    else
    {
        _Cost = _PenetrationDist[k];
    }
    vector<double> distCost;
    distCost.push_back( _Cost );
    return distCost;
}

double HRICS_Distance::computeBBDist(p3d_vector3 robot, p3d_vector3 human)
{
    double minDistance1Prev = numeric_limits<double>::max();
    double minDistance2Prev = numeric_limits<double>::max();
    int nbBBTests =0;

    //    double tu,ts;
    //    ChronoOn();

    for(int i=0; i<_Humans.size(); i++)
    {
        //        cout << _Robot->getRobotStruct() << endl;

        p3d_col_activate_rob_rob(_Robot->getRobotStruct(),_Humans[i]->getRobotStruct());

        for(int j =0; j<_Humans[i]->getRobotStruct()->no; j++)
        {
            if(binary_search(_SafetyZonesBodyId[i].begin(), _SafetyZonesBodyId[i].end(),j))
            {
                bool show_box = false;

                for(int k =0; k<_Robot->getRobotStruct()->no; k++)
                {
                    for(int l=0; l<_Robot->getRobotStruct()->o[k]->np;l++)
                    {
                        if(_Robot->getRobotStruct()->o[k]->pol[l]->TYPE!=P3D_GRAPHIC)
                        {
                            show_box = true;
                        }
                    }

                    if(show_box)
                    {
                        double minDistance2;
                        double minDistance1 = p3d_BB_obj_obj_extern_dist ( _Robot->getRobotStruct()->o[k],
                                                                           _Humans[i]->getRobotStruct()->o[j], &minDistance2 );

                        if(minDistance1 < minDistance1Prev)
                        {
                            minDistance1Prev = minDistance1;

                            p3d_obj* Obj= _Robot->getRobotStruct()->o[k];

                            robot[0] = (Obj->BB.xmax +  Obj->BB.xmin)/2;
                            robot[1] = (Obj->BB.ymax +  Obj->BB.ymin)/2;
                            robot[2] = (Obj->BB.zmax +  Obj->BB.zmin)/2;

                            Obj= _Humans[i]->getRobotStruct()->o[j];

                            human[0] = (Obj->BB.xmax +  Obj->BB.xmin)/2;
                            human[1] = (Obj->BB.ymax +  Obj->BB.ymin)/2;
                            human[2] = (Obj->BB.zmax +  Obj->BB.zmin)/2;
                        }
                    }
                }
            }
        }
    }

    //    ChronoMicroTimes(&tu, &ts);
    //    ChronoOff();
    //    cout << "tu = " << tu << endl;

    return minDistance1Prev;
}

double HRICS_Distance::computeBoundingBalls(p3d_vector3 robot, p3d_vector3 human)
{
    double pointbodydist;
    double pointneckdist;
    double mindist[3];

    Vector3d point = _Robot->getJointPos(ENV.getInt(Env::akinJntId));

    Vector3d hbody;
    hbody[0] = _Humans[0]->getRobotStruct()->joints[HUMANj_BODY]->abs_pos[0][3];
    hbody[1] = _Humans[0]->getRobotStruct()->joints[HUMANj_BODY]->abs_pos[1][3];
    hbody[2] = _Humans[0]->getRobotStruct()->joints[HUMANj_BODY]->abs_pos[2][3];

    Vector3d hneck;
    hneck[0] = _Humans[0]->getRobotStruct()->joints[HUMANj_NECK_TILT]->abs_pos[0][3];
    hneck[1] = _Humans[0]->getRobotStruct()->joints[HUMANj_NECK_TILT]->abs_pos[1][3];
    hneck[2] = _Humans[0]->getRobotStruct()->joints[HUMANj_NECK_TILT]->abs_pos[2][3];

    pointbodydist = ( point - hneck ).norm();
    pointneckdist = ( point - hbody ).norm();

    // Warning here

//    double human_max_reach_length = 1.5;

    if(pointneckdist < pointbodydist)
    {
        for(int i=0; i<3; i++)
        {
            mindist[i] = ABS(point[i] - hneck[i]);
            robot[i] = point[i];
            human[i] = hneck[i];
        }

        return pointneckdist;
    }
    else
    {
        for(int i=0; i<3; i++)
        {
            mindist[i] = ABS(point[i] - hbody[i]);
            robot[i] = point[i];
            human[i] = hbody[i];
        }

        return pointbodydist;
    }

//    if(pointneckdist > human_max_reach_length)
//    {
//        return 0;
//    }

//    return  cos((mindist[0]/human_max_reach_length)*M_PI_2)*
//            cos((mindist[1]/human_max_reach_length)*M_PI_2)*
//            cos((mindist[2]/human_max_reach_length)*M_PI_2);

}
