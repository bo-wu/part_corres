/*
 * =====================================================================================
 *
 *       Filename:  mobb_correspondence.h      Created:  05/31/2015 12:26:19 PM
 *
 *    Description:  compute mini oriented bounding box correspondence
 *
 *         Author:  Wu Bo (Robert), wubo.gfkd@gmail.com
 *		Copyright:	Copyright (c) 2015, Wu Bo
 *   Organization:  National University of Defense Technology
 *
 * =====================================================================================
 */
#ifndef MOBB_CORRESPONDENCE_H_
#define MOBB_CORRESPONDENCE_H_
#include <Box.h>
#include <vector>
#include <string>
#include <Eigen/Dense>
class OBBoxCorres
{
public:
    OBBoxCorres(){}
    OBBoxCorres(char*, char*);
    void read_boxes_xml(char*, char*);
    void compute_fuzzy_correspond();
    void save_result_xml(double theta=0.6);
    void reset_result_xml();
    Eigen::MatrixXd get_fuzzy_correspond();

    std::string source_file_name, target_file_name;
    Eigen::MatrixXd fuzzy_corres;

private:
    double box_hausdorff(Geom::Box&, Geom::Box&);
    std::vector<Geom::Box> parse_xml(char*);
    //according to our data the
    //first box is about the whole shape
    std::vector<Geom::Box> source; 
    std::vector<Geom::Box> target;
};

#endif

