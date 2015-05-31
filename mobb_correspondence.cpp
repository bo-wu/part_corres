/*
 * =====================================================================================
 *
 *       Filename:  mobb_correspondence.cpp      Created:  05/31/2015 12:44:38 PM
 *
 *    Description:  mini volume obb correspondence
 *
 *         Author:  Wu Bo (Robert), wubo.gfkd@gmail.com
 *		Copyright:	Copyright (c) 2015, Wu Bo
 *   Organization:  National University of Defense Technology
 *
 * =====================================================================================
 */
#include <iostream>
#include <sstream>
#include <algorithm>
#include <pugixml.hpp>
#include <cfloat>
#include <QVector>
#include "mobb_correspondence.h"

OBBoxCorres::OBBoxCorres(char *s, char *t)
{
    read_boxes_xml(s, t);
    compute_fuzzy_correspond();
}

std::vector<Geom::Box> OBBoxCorres::parse_xml(char *fname)
{
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(fname);
    std::cout<<"Load "<<fname<<" results: "<<result.description()<<std::endl;
    std::vector<Geom::Box> model_boxes;
    //read boxes for model: should be fix format xml
    std::vector<double> param_value;
    for(pugi::xml_node box : doc.children("box"))
    {
        Eigen::Vector3d c;
        QVector<Eigen::Vector3d> axis;
        Eigen::Vector3d ext;
        param_value.clear();
        for(pugi::xml_node param : box.children())
        {
            std::string value_string(param.text().get());
            std::istringstream ss(value_string);
            double temp;
            while(ss>>temp)
            {
                param_value.push_back(temp);
            }
        }
        c << param_value.at(0), param_value.at(1), param_value.at(2);
        ext << param_value.at(12), param_value.at(13), param_value.at(14);
        Eigen::Vector3d temp;
        for(int i=1; i<=3; ++i)
        {
            temp << param_value.at(i*3+0), param_value.at(i*3+1), param_value.at(i*3+2);
            axis.push_back(temp);
        }
        model_boxes.push_back(Geom::Box(c, axis, ext));
    }
    return model_boxes;
}

void OBBoxCorres::read_boxes_xml(char *source_name, char *target_name)
{
    source = parse_xml(source_name);
    target = parse_xml(target_name);
}


void OBBoxCorres::compute_fuzzy_correspond()
{
    fuzzy_corres = Eigen::MatrixXd(source.size()-1, target.size()-1);
    int i = 0, j = 0;
    for(auto sit=source.begin()+1; sit!=source.end(); ++sit, ++i)
    {
        j = 0;
        for(auto tit=target.begin()+1; tit!=target.end(); ++tit, ++j)
        {
            fuzzy_corres(i, j) = box_hausdorff(*sit, *tit);
        }
    }
    double l = ((source[0].Extent*2).norm() + (target[0].Extent*2).norm() ) / 2;
    fuzzy_corres /= l;
    fuzzy_corres = 1 - fuzzy_corres.array();
}

Eigen::MatrixXd OBBoxCorres::get_fuzzy_correspond()
{
    return fuzzy_corres;
}

// hausdorff distance between two boxes, not sure if intersection affect or not
double OBBoxCorres::box_hausdorff(Geom::Box &source, Geom::Box &target)
{
    QVector<Eigen::Vector3d> source_box_points = source.getConnerPoints();
    QVector<Eigen::Vector3d> target_box_points = target.getConnerPoints();
    double st = FLT_MIN, ts = FLT_MIN;
    foreach(auto s, source_box_points)
    {
        double min_len = FLT_MAX;
        double dist; 
        foreach(auto t, target_box_points)
        {
            dist = (s - t).norm();
            if(min_len > dist)
                min_len = dist;
        }

        if(st < min_len)
            st = min_len;
    }

    foreach(auto t, target_box_points)
    {
        double min_len = FLT_MAX;
        double dist;
        foreach(auto s, source_box_points)
        {
            dist = (s - t).norm();
            if(min_len > dist)
                min_len = dist;
        }
        if(ts < min_len)
            ts = min_len;
    }
    return std::max(st, ts);
}

