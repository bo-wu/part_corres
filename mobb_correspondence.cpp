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
#include <fstream>
#include <set>
#include "mobb_correspondence.h"

OBBoxCorres::OBBoxCorres(char *s, char *t)
{
    read_boxes_xml(s, t);
    compute_fuzzy_correspond();
    save_result_xml(0.6);
}

void OBBoxCorres::read_boxes_xml(char *source_name, char *target_name)
{
    source_file_name = source_name;
    target_file_name = target_name;
    source = parse_xml(source_name);
    target = parse_xml(target_name);
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


void OBBoxCorres::compute_fuzzy_correspond()
{
    fuzzy_corres = Eigen::MatrixXd(source.size()-1, target.size()-1);
    int i = 0, j = 0;
    //first one is the overall obb
    for(auto sit=source.begin()+1; sit!=source.end(); ++sit, ++i)
    {
        j = 0;
        for(auto tit=target.begin()+1; tit!=target.end(); ++tit, ++j)
        {
            fuzzy_corres(i, j) = box_hausdorff(*sit, *tit);
        }
    }
    /*  
    std::ofstream file;
    file.open("hausdorff.txt");
    if(file.is_open())
    {
        file<<fuzzy_corres;
    }
    file.close();
    */
    double l = ((source[0].Extent*2).norm() + (target[0].Extent*2).norm()) * 0.3;
    //l = std::min(fuzzy_corres.maxCoeff(), l);
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
    /*
    std::ofstream source_file("source.txt", std::ios::out|std::ios::app);
    if(source_file.is_open())
    {
    for(auto s : source_box_points)
    {
        source_file<<s<<std::endl;
    }
    }
    source_file.close();
    std::ofstream target_file("target.txt", std::ios::out|std::ios::app);
    if(target_file.is_open())
    {
    for(auto t : target_box_points)
    {
        target_file<<t<<std::endl;
    }
    }
    target_file.close();
    */
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


/* 
 * ===  FUNCTION  ======================================================================
 *         Name:  save_result_xml
 *  Description:  save result  xml
 * =====================================================================================
 */

void OBBoxCorres::save_result_xml(double theta)
{
    pugi::xml_document source_doc, target_doc;
    source_doc.load_file(source_file_name.c_str());
    target_doc.load_file(target_file_name.c_str());
    auto box_node = source_doc.first_child().next_sibling();
    if( ! box_node.attribute("appearence") )
    {
        for(pugi::xml_node box=box_node; box; box=box.next_sibling())
        {
            box.append_attribute("appearence") = 0;
            box.append_attribute("total") = 0;
        }
    }
    box_node = target_doc.first_child().next_sibling();
    if( ! box_node.attribute("appearence") )
    {
        for(pugi::xml_node box=box_node; box; box=box.next_sibling())
        {
            box.append_attribute("appearence") = 0;
            box.append_attribute("total") = 0;
        }
    }

    Eigen::VectorXi source_index = Eigen::VectorXi::Zero(fuzzy_corres.rows());
    Eigen::VectorXi target_index = Eigen::VectorXi::Zero(fuzzy_corres.cols());
    for(int i=0; i<fuzzy_corres.rows(); ++i)
    {
        for(int j=0; j<fuzzy_corres.cols(); ++j)
        {
            if(fuzzy_corres(i, j) > theta)
            {
                source_index(i) = 1;
                target_index(j) = 1;
            }

        }
    }

    // count result
    int idx = -1;
    for(pugi::xml_node box : source_doc.children("box"))
    {
        if (idx < 0)
        {
            //first one is obb for the whole shape
            ++idx;
            continue;
        }
        int app = box.attribute("appearence").as_int();
        int tot = box.attribute("total").as_int();
        box.attribute("appearence").set_value(app+source_index(idx));
        box.attribute("total").set_value(tot+1);
        ++idx;
    }
    idx = -1;
    for(pugi::xml_node box : target_doc.children("box"))
    {
        if (idx < 0)
        {
            //first one is obb for the whole shape
            ++idx;
            continue;
        }
        int app = box.attribute("appearence").as_int();
        int tot = box.attribute("total").as_int();
        box.attribute("appearence").set_value(app+target_index(idx));
        box.attribute("total").set_value(tot+1);
        ++idx;
    }

    //overwrite origin xml file for later use
    source_doc.save_file(source_file_name.c_str());
    target_doc.save_file(target_file_name.c_str());
}

void OBBoxCorres::reset_result_xml()
{
    pugi::xml_document source_doc, target_doc;
    source_doc.load_file(source_file_name.c_str());
    target_doc.load_file(target_file_name.c_str());
    for(auto box : source_doc.children("box"))
    {
        box.attribute("appearence").set_value(0);
        box.attribute("total").set_value(0);
    }
    for(auto box : target_doc.children("box"))
    {
        box.attribute("appearence").set_value(0);
        box.attribute("total").set_value(0);
    }
    source_doc.save_file(source_file_name.c_str());
    target_doc.save_file(target_file_name.c_str());
}
