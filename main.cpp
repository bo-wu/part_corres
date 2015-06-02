/*
 * =====================================================================================
 *
 *       Filename:  main.cpp      Created:  05/31/2015 01:31:02 AM
 *
 *    Description:  main 
 *
 *         Author:  Wu Bo (Robert), wubo.gfkd@gmail.com
 *		Copyright:	Copyright (c) 2015, Wu Bo
 *   Organization:  National University of Defense Technology
 *
 * =====================================================================================
 */
#include "mobb_correspondence.h"

int main(int argc, char** argv)
{
    if (argc < 3)
    {
        std::cerr<<"need two xml files as input\n";
        return -1;
    }
    OBBoxCorres obb_corres;
    obb_corres.read_boxes_xml(argv[1], argv[2]);
    obb_corres.compute_fuzzy_correspond();
    std::cout<<obb_corres.fuzzy_corres<<std::endl;
    obb_corres.save_result_xml(0.6);
    //obb_corres.reset_result_xml();
    return 0;
}
