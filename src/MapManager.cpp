/*!
 *  \file MapManager.cpp
 *
 *  Created on: Dec, 09 2015
 *      Author: daniel
 */

// ROS
#include <ros/ros.h>
#include <ros/package.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <assert.h>     /* assert */
#include <cmath>

#include "MapManager.h"
#include "params_MapManager.h"

MapManager::MapManager()
    : n_xy(0),
      n_x(0),
      n_y(0.),
      s_x(0.),
      s_y(0.),
      desired_s_x(ParamsMapManager::desired_s_x_param),
      desired_s_y(ParamsMapManager::desired_s_y_param),
      len_x(0),
      len_y(0),
      len_z(0),
      min_xr(std::numeric_limits<double>::max()),
      min_yr(std::numeric_limits<double>::max()),
      min_zr(std::numeric_limits<double>::max()),
      map_x(0.),
      map_y(0.),
      map_z(0.),
      map_xr(0.),
      map_yr(0.),
      map_zr(0.),
      octomap(new octomap::OcTree(ParamsMapManager::octomap_res_param)), octomap_res(ParamsMapManager::octomap_res_param), saveOctomap(ParamsMapManager::saveOctomap_param),
      airplane_bb{ParamsMapManager::airplane_bb_param[0],ParamsMapManager::airplane_bb_param[1],ParamsMapManager::airplane_bb_param[2]} {
}

MapManager::~MapManager() {
}

bool isOdd( int integer )
{
  if ( integer % 2 == 1 )
     return true;
  else
     return false;
}

void writeToFile(std::vector<double> x, std::vector<double> y, std::vector<double> z, std::string rel_path, std::string fileName) {

  std::string path = ros::package::getPath("testing");
  std::ofstream ofile(path + rel_path + fileName);
  ofile << "Header" << "\n";
  int ctr = 0;
  for(std::vector<double>::iterator it = x.begin(); it != x.end(); ++it) {
    ofile << *it << " " << y[ctr] << " " << z[ctr] << "\n";
    ctr += 1;
  }
  ofile.close();
}
void skipOneLine(std::ifstream& myfile) {
  // Skip header
  std::string dummyLine;
  std::getline(myfile, dummyLine);
}

std::tuple<int, double, double> findNxSxSy(std::string rel_path,
                                           std::string fileName) {

  // Read in .xyz file
  std::string path = ros::package::getPath("testing");
  std::ifstream myfile(path + rel_path + fileName);
  // Skip header
  skipOneLine(myfile);

  bool firstTime = true, foundNbr = false;
  double xr = 0., yr = 0., zr = 0., xr_old = 0., yr_old = 0.;
  int nbr_xr = 0, n_x = 0, line_nbr = 0;
  double s_x = 0., s_y = 0.;

  // Find n_x, s_x, s_y
  if (myfile.is_open()) {
    while (true) {
      nbr_xr += 1;
      std::string line;
      std::getline(myfile, line);
      std::stringstream ss_line(line);
      xr_old = xr;
      yr_old = yr;
      ss_line >> xr >> yr >> zr;

      if (myfile.eof())
        break;

      if (!firstTime && !foundNbr) {
        if (yr_old != yr) {
          n_x = nbr_xr - 1;
          s_y = fabs(yr - yr_old);
          foundNbr = true;
          break;
        }
      }
      if (firstTime) {
        firstTime = false;
      }
      if (line_nbr == 1) {
        s_x = fabs(xr - xr_old);
      }

      line_nbr += 1;
    }
  }
  myfile.close();

  if (s_x == 0)
    std::cout << "Warning: s_x == 0" << std::endl;
  if (s_y == 0)
    std::cout << "Warning: s_y == 0" << std::endl;

  return std::make_tuple(n_x, s_x, s_y);
}

void reshape(MapManager* mM) {
  std::vector<double> map_xr_new, map_yr_new, map_zr_new;

  for(int y_it = mM->n_xy - mM->n_x; y_it >= 0; y_it -= mM->n_x) {
    for(int x_it = 0; x_it < mM->n_x; ++x_it) {
      map_xr_new.push_back(mM->map_xr[x_it + y_it]);
      map_yr_new.push_back(mM->map_yr[x_it + y_it]);
      map_zr_new.push_back(mM->map_zr[x_it + y_it]);
    }
  }
  mM->map_xr.clear();
  mM->map_yr.clear();
  mM->map_zr.clear();

  mM->map_xr = map_xr_new;
  mM->map_yr = map_yr_new;
  mM->map_zr = map_zr_new;

}

void reshapeToFileFormat(std::vector<double>& map_xr_red, std::vector<double>& map_yr_red, std::vector<double>& map_zr_red, int n_x_red, int n_xy_red) {
  std::vector<double> map_xr_old = map_xr_red,
      map_yr_old = map_yr_red,
      map_zr_old = map_zr_red;

  map_xr_red.clear();
  map_yr_red.clear();
  map_zr_red.clear();
  for(int y_it = n_xy_red - n_x_red; y_it >= 0; y_it -= n_x_red) {
    for(int x_it = 0; x_it < n_x_red; ++x_it) {
      map_xr_red.push_back(map_xr_old[x_it + y_it]);
      map_yr_red.push_back(map_yr_old[x_it + y_it]);
      map_zr_red.push_back(map_zr_old[x_it + y_it]);
    }
  }
}

void shiftOrigin(MapManager* mM) {
  int ctr = 0;
  for (std::vector<double>::iterator it = mM->map_xr.begin(); it != mM->map_xr.end();
      ++it) {
    mM->map_x.push_back(*it - mM->min_xr);
    mM->map_y.push_back(mM->map_yr[ctr] - mM->min_yr);
    mM->map_z.push_back(mM->map_zr[ctr] - mM->min_zr);

    ctr += 1;
  }

}

void writeReducedFile(MapManager* mM, std::string path, std::string rel_path, std::string newFileName,
                      std::vector<double> map_xr_red, std::vector<double> map_yr_red, std::vector<double> map_zr_red, int skipPts_x, int skipPts_y) {
  // Write reduced file
  std::ofstream o_myfile(path + rel_path + newFileName);
  // Write header
  o_myfile << "Header" << "\n";

  /*
  std::ofstream tmp(path + rel_path + "tmp.xyz");
  tmp << "Header" << "\n";
  int ctrtmp = 0;
  for (std::vector<double>::iterator it = mM->map_xr.begin(); it != mM->map_xr.end();
      ++it) {
    tmp << *it << " " << mM->map_yr[ctrtmp] << " " << mM->map_zr[ctrtmp] << "\n";
    ctrtmp+=1;
  }
  tmp.close();

  std::ofstream tmp1(path + rel_path + "tmp1.xyz");
  tmp1 << "Header" << "\n";
  int ctrtmp1 = 0;
  for (std::vector<double>::iterator it = map_xr_red.begin(); it != map_xr_red.end();
      ++it) {
    tmp1 << *it << " " << map_yr_red[ctrtmp1] << " " << map_zr_red[ctrtmp1] << "\n";
    ctrtmp1+=1;
  }
  tmp1.close();
*/
  int ctr = 0;
  for (std::vector<double>::iterator it = map_zr_red.begin(); it != map_zr_red.end();
      ++it) {

    double xr_red = map_xr_red[ctr];
    double yr_red = map_yr_red[ctr];
    double x_red = map_xr_red[ctr] - mM->min_xr;
    double y_red = map_yr_red[ctr] - mM->min_yr;

    double xr_startSearch = xr_red - std::floor(skipPts_x/2.)*fabs(mM->s_x);
    double xr_endSearch = xr_red + std::floor(skipPts_x/2.)*fabs(mM->s_x);
    double yr_startSearch = yr_red - std::floor(skipPts_y/2.)*fabs(mM->s_y);
    double yr_endSearch = yr_red + std::floor(skipPts_y/2.)*fabs(mM->s_y);

    double max_zr = -std::numeric_limits<double>::max();
    for(double yr = yr_startSearch; yr <= yr_endSearch; yr = yr+fabs(mM->s_y)) {
      for(double xr = xr_startSearch; xr <= xr_endSearch; xr = xr+fabs(mM->s_x)) {

        if( xr < mM->min_xr || yr < mM->min_yr ) {
          continue;
        } else if( xr > mM->min_xr+mM->len_x || yr > mM->min_yr+mM->len_y ) {
          continue;
        }

//        int idx = mM->getIdx(xit, yit);

        double zr = mM->getZr(xr-mM->min_xr,yr-mM->min_yr);
/*        if (fabs(idx) > mM->map_zr.size()-1)
          zr = 0.;
        else if (idx < 0)
          zr = 0.;
        else
          zr = mM->map_zr[idx];
*/

        if( max_zr < zr ) {
          max_zr = zr;
        }
      }
    }
    //o_myfile << map_xr_red[ctr] << " " << map_yr_red[ctr] << " " << *it << "\n";
    o_myfile << map_xr_red[ctr] << " " << map_yr_red[ctr] << " " << max_zr << "\n";
    //o_myfile << xr_red << " " << yr_red << " " << max_zr << "\n";
    if (ctr % 500000 == 0)
      std::cout << ctr << std::endl;

    ctr += 1;
  }

  o_myfile.close();
}

void MapManager::readMapAndGenerateOctomap(std::string rel_path,
                                           std::string fileName) {

  this->readMap(rel_path, fileName);

  this->generateOctomap(rel_path, fileName);

}

void MapManager::readMapAndReduce(std::string rel_path, std::string fileName,
                                  std::string newFileName) {

  readMap(rel_path, fileName);

  std::vector<double> map_xr_red, map_yr_red, map_zr_red;
  std::vector<double> map_x_red, map_y_red, map_z_red;

  int skipPts_x = 1, skipPts_y = 1;
  if (fabs(s_x) < desired_s_x) {
    skipPts_x = int(ceil(desired_s_x / fabs(s_x))); /* every skipPts Pt is stored */
  } else {
    skipPts_x = 1;
  }
  if (fabs(s_y) < desired_s_y) {
    skipPts_y = int(ceil(desired_s_y / fabs(s_y))); /* every skipPts Pt is stored */
  } else {
    skipPts_y = 1;
  }

  int n_x_red = int(std::floor((n_x-1)/skipPts_x)+1);
  int n_y_red = int(std::floor((n_y-1)/skipPts_y)+1);
  int n_xy_red = n_x_red*n_y_red;


  std::cout << "Reduce amount of points and take into account highest value in neighborhood..." << std::endl;
  int ctr = 0;
  for(std::vector<double>::iterator it = map_xr.begin(); it != map_xr.end(); ++it) {

    if ( int(ctr-floor(ctr/n_x)*n_x) % skipPts_x == 0 /* get every skipPts Pt in x direction */
        && int(floor(ctr / n_x)) % skipPts_y
            == 0. /* get every skipPts Pt in y direction CORRECT */) {

      // Search for biggest value in neighborhood
      double x_startSearch = map_x[ctr] - floor(skipPts_x/2)*fabs(s_x);
      double x_endSearch = map_x[ctr] + floor(skipPts_x/2)*fabs(s_x);
      double y_startSearch = map_y[ctr] - floor(skipPts_y/2)*fabs(s_y);
      double y_endSearch = map_y[ctr] + floor(skipPts_y/2)*fabs(s_y);

      double max_zr = -std::numeric_limits<double>::max();
      for(double y = y_startSearch; y <= y_endSearch; y = y+fabs(s_y)) {
        for(double x = x_startSearch; x <= x_endSearch; x = x+fabs(s_x)) {

          if( x < 0 || y < 0 ) {
            continue;
          } else if( x > len_x || y > len_y ) {
            continue;
          }

          double zr = getZr(x, y);


          if( max_zr < zr ) {
            max_zr = zr;
          }
        }
      }

      map_xr_red.push_back(*it);
      map_yr_red.push_back(map_yr[ctr]);
      map_zr_red.push_back(max_zr);

    }
    if (ctr % 500000 == 0)
      std::cout << ctr << std::endl;

    ctr += 1;
  }
  std::cout << "Successfully reduced amount of points." << std::endl;

  reshapeToFileFormat(map_xr_red, map_yr_red, map_zr_red, n_x_red, n_xy_red);
  writeToFile(map_xr_red, map_yr_red, map_zr_red, rel_path, newFileName);


  std::string path = ros::package::getPath("testing");
  std::cout << "Successfully read and reduced " << rel_path + fileName << std::endl;
  std::cout << "to " << path + rel_path + newFileName << std::endl;
}

void MapManager::readMap(std::string rel_path, std::string fileName) {

  double xr = 0., yr = 0., zr = 0.;
  double max_zr = -std::numeric_limits<double>::max(), max_xr = -std::numeric_limits<double>::max(), max_yr = -std::numeric_limits<double>::max();
  double yr_old;
  bool firstTime = true, foundNbr = false;

  auto tuple = findNxSxSy(rel_path, fileName);
  n_x = std::get<0>(tuple);
  s_x = std::get<1>(tuple);
  s_y = std::get<2>(tuple);

  std::string path = ros::package::getPath("testing");
  std::ifstream myfile(path + rel_path + fileName);
  // Skip header
  skipOneLine(myfile);

  if (myfile.is_open()) {

    int line_nbr = 2; // file begins with line nbr = 1, skip header --> line nbr = 2
    while (true) {

      std::string line;
      std::getline(myfile, line);
      std::stringstream ss_line(line);
      ss_line >> xr >> yr >> zr;
      if(fabs(zr)>15000) // No data available for this point if height more than 15km
        zr = 0.;


      if (myfile.eof())
        break;

      map_xr.push_back(xr);
      map_yr.push_back(yr);
      map_zr.push_back(zr);

      // search min and max extents of map
      if (xr < min_xr)
        min_xr = xr;
      if (yr < min_yr)
        min_yr = yr;
      if (zr < min_zr)
        min_zr = zr;
      if (xr > max_xr)
        max_xr = xr;
      if (yr > max_yr)
        max_yr = yr;
      if (zr > max_zr)
        max_zr = zr;



      if ((line_nbr-2) % 500000 == 0)
        std::cout << "Reading line: " << line_nbr-2 << std::endl;

      line_nbr += 1;
    }
  } else {
    assert(myfile.is_open() && "File could not be found.");
  }
  myfile.close();

  // Copy extents of the map.
  len_x = max_xr - min_xr;
  len_y = max_yr - min_yr;
  len_z = max_zr - min_zr;

  n_xy = map_zr.size();
  double n_ytmp = n_xy / n_x;
  assert(
      (std::fmod(n_ytmp, 1.) == 0.)
          && "n_xy is not an integer. Check the data file. Assumption: File contains m*n data points.");
  n_y = n_ytmp;


  std::cout << "Successfully read " <<  path + rel_path + fileName << std::endl;

  reshape(this);
  shiftOrigin(this);

}

void MapManager::generateOctomap(std::string rel_path, std::string fileName) {

  // Make free octomap
  std::cout << "Generating octomap." << std::endl;
  int ctr = 0;
  double s_z = std::min(s_x,s_y);
  for (std::vector<double>::iterator it = map_z.begin(); it != map_z.end();
      ++it) {
    for(double z = 0.; z <= len_z; z += s_z) {
      // Fill in info in octomap.
      octomap->updateNode(map_x[ctr],map_y[ctr], z, false);
    }
    if (ctr % 100000 == 0)
      std::cout << ctr << std::endl;
    ctr += 1;

  }
  // Mark all below height map as occupied
  ctr = 0;
  for (std::vector<double>::iterator it = map_z.begin(); it != map_z.end();
      ++it) {
    for(double z = 0.; z <= *it; z += s_z) {
      // Fill in info in octomap.
      octomap->updateNode(map_x[ctr],map_y[ctr], z, true);
    }
    if (ctr % 100000 == 0)
      std::cout << ctr << std::endl;
    ctr += 1;
  }
  octomap->prune();
  std::cout << "Successfully generated octomap." << std::endl;

  // Save octomap
  bool writeOK = false;
  std::string path = ros::package::getPath("testing");
  std::string octreeName = fileName;
  std::string octreeNameEnd = "_octres" + std::to_string(int(octomap_res)) + "m.ot";
  if(saveOctomap) {
    octreeName.replace(octreeName.end()-4,octreeName.end(),octreeNameEnd);
    writeOK = octomap->write(path + rel_path + octreeName);
  }

  if(writeOK)
    std::cout << "Successfully saved octomap to " << path + rel_path + octreeName << std::endl;
  else
    std::cout << "Did not save octomap as a file." << std::endl;

}

void MapManager::readOctomap(std::string rel_path, std::string fileName) {

  std::string path = ros::package::getPath("fw_planning");
  octomap::AbstractOcTree* atree = octomap::AbstractOcTree::read(path + rel_path + fileName);
  octomap.reset(dynamic_cast<octomap::OcTree*>(atree));
//  octomap->read(path + rel_path + fileName);
  octomap->setResolution(octomap_res);
  octomap->prune();

}

void MapManager::reset() {
  n_x = 0;
  n_y = 0;
  n_xy = 0;

  s_x = 0.;
  s_y = 0.;

  map_xr.clear();
  map_yr.clear();
  map_zr.clear();
  map_x.clear();
  map_y.clear();
  map_z.clear();

  len_x = 0.;
  len_y = 0.;
  len_z = 0.;
  min_xr = std::numeric_limits<double>::max();
  min_yr = std::numeric_limits<double>::max();
  min_zr = std::numeric_limits<double>::max();

  octomap_res = ParamsMapManager::octomap_res_param;
  saveOctomap = ParamsMapManager::saveOctomap_param;
  airplane_bb = ParamsMapManager::airplane_bb_param;
  octomap->clear();
}

// TODO make sure this fcts works correctly, especially for airplane bb box > map resolution


bool MapManager::collide(double x, double y, double z) const {
  // TODO Currently, conservative implementation. For the case that n_cellsToCheck_x or n_cellsToCheck_y is even, there are to many checks.
  bool collision = false;
  int n_cellsToCheck_x = (std::ceil(airplane_bb[0]/s_x) + 1);
  int n_cellsToCheck_y = (std::ceil(airplane_bb[1]/s_y) + 1);
  //double z_map [4] = {0,0,0,0};
  std::cout << "n_cellsToCheck_x: " << n_cellsToCheck_x << " " << n_cellsToCheck_y << std::endl;

  auto xx_yy = mToPt(x,y);

  std::cout << "state to check (x,y,z) :" << x << " " << y << " " << z << std::endl;

  std::cout << "neighboring states :" << std::endl;
  if (isOdd(n_cellsToCheck_x) && isOdd(n_cellsToCheck_y)) { // x,y are odd

    std::cout << "x,y odd" << std::endl;

    for ( double xi = x - std::floor(n_cellsToCheck_x/2.)*s_x; xi <= x + std::floor(n_cellsToCheck_x/2.)*s_x; xi += s_x) {
      for ( double yi = y - std::floor(n_cellsToCheck_y/2.)*s_y; yi <= y + std::floor(n_cellsToCheck_y/2.)*s_y; yi += s_y) {
        std::cout << "x,y,z :" << xi << " " << yi << " " << getZ(xi,yi) << std::endl;
        //z_map.push_back(getZ(xi,yi));
        if( getZ(xi,yi) > z ) {
          collision = true;
        }
      }
    }

  } else if( isOdd(n_cellsToCheck_x) ) { // x is odd, y is even

    std::cout << "x odd" << std::endl;
    for ( double xi = x - std::floor(n_cellsToCheck_x/2.)*s_x; xi <= x + std::floor(n_cellsToCheck_x/2.)*s_x; xi += s_x) {
      for ( double yi = y - std::floor(n_cellsToCheck_y/2.)*s_y/2.; yi <= y + std::floor(n_cellsToCheck_y/2.)*s_y/2.; yi += s_y) {
        std::cout << "x,y,z :" << xi << " " << yi << " " << getZ(xi,yi) << std::endl;
        //z_map.push_back(getZ(xi,yi));
        if( getZ(xi,yi) > z ) {
          collision = true;
        }
      }
    }

  } else if( isOdd(n_cellsToCheck_y) ) { // x is even, y is odd

    std::cout << "y odd" << std::endl;
    for ( double xi = x - std::floor(n_cellsToCheck_x/2.)*s_x/2.; xi <= x + std::floor(n_cellsToCheck_x/2.)*s_x/2.; xi += s_x) {
      for ( double yi = y - std::floor(n_cellsToCheck_y/2.)*s_y; yi <= y + std::floor(n_cellsToCheck_y/2.)*s_y; yi += s_y) {
        std::cout << "x,y,z :" << xi << " " << yi << " " << getZ(xi,yi) << std::endl;
        //z_map.push_back(getZ(xi,yi));
        if( getZ(xi,yi) > z ) {
          collision = true;
        }
      }
    }

  } else { // x,y are even

    std::cout << "x,y even, s_x, s_y: " << s_x << " " << s_y << std::endl;
    for ( double xi = x - std::floor(n_cellsToCheck_x/2.)*s_x/2.; xi <= x + std::floor(n_cellsToCheck_x/2.)*s_x/2.; xi += s_x) {
      for ( double yi = y - std::floor(n_cellsToCheck_y/2.)*s_y/2.; yi <= y + std::floor(n_cellsToCheck_y/2.)*s_y/2.; yi += s_y) {
        std::cout << "x,y,z :" << xi << " " << yi << " " << getZ(xi,yi) << std::endl;
        //z_map.push_back(getZ(xi,yi));
        if( getZ(xi,yi) > z ) {
          collision = true;
        }
      }
    }

  }


  /*
  if ( xx_yy.first > x && xx_yy.second > y ) {

    //z_map [0] = getZ(x, y);
    //z_map [1] = getZ(x + desired_s_x, y);
    //z_map [2] = getZ(x, y + desired_s_y);
    //z_map [3] = getZ(x + desired_s_x, y + desired_s_y);

  } else if ( xx_yy.first > x && xx_yy.second < y) {
    //z_map [0] = getZ(x, y);
    //z_map [1] = getZ(x + desired_s_x, y);
    //z_map [2] = getZ(x, y - desired_s_y);
    //z_map [3] = getZ(x + desired_s_x, y - desired_s_y);

  } else if ( xx_yy.first < x && xx_yy.second > y) {
    //z_map [0] = getZ(x, y);
    //z_map [1] = getZ(x - desired_s_x, y);
    //z_map [2] = getZ(x, y + desired_s_y);
    //z_map [3] = getZ(x - desired_s_x, y + desired_s_y);

  } else {
    z_map [0] = getZ(x, y);
    z_map [1] = getZ(x - desired_s_x, y);
    z_map [2] = getZ(x, y - desired_s_y);
    z_map [3] = getZ(x - desired_s_x, y - desired_s_y);

  }
*/
  if(collision) {
    return true;
  } else {
    return false;
  }

}

double MapManager::getXr(double x, double y) const {
  double xt = x;
  double yt = y;
  int idx = getIdx(xt, yt);

  if (fabs(idx) > map_xr.size()-1)
    return 0;
  else if (idx < 0)
    return 0;
  else
    return map_xr[idx];
}
double MapManager::getYr(double x, double y) const {
  double xt = x;
  double yt = y;
  int idx = getIdx(xt, yt);

  if (fabs(idx) > map_yr.size()-1)
    return 0;
  else if (idx < 0)
    return 0;
  else
    return map_yr[idx];
}
double MapManager::getZr(double x, double y) const {
  double xt = x;
  double yt = y;
  int idx = getIdx(xt, yt);

  if (fabs(idx) > map_zr.size()-1)
    return 0;
  else if (idx < 0)
    return 0;
  else
    return map_zr[idx];
}
double MapManager::getX(double x, double y) const {
  double xt = x;
  double yt = y;
  int idx = getIdx(xt, yt);

  if (fabs(idx) > map_x.size()-1)
    return 0;
  else if (idx < 0)
    return 0;
  else
    return map_x[idx];
}
double MapManager::getY(double x, double y) const {
  double xt = x;
  double yt = y;
  int idx = getIdx(xt, yt);

  if (fabs(idx) > map_y.size()-1)
    return 0;
  else if (idx < 0)
    return 0;
  else
    return map_y[idx];

}
double MapManager::getZ(double x, double y) const {
  double xt = x;
  double yt = y;
  int idx = getIdx(xt, yt);

  if (fabs(idx) > map_z.size()-1)
    return 0;
  else if (idx < 0)
    return 0;
  else
    return map_z[idx];
}

int MapManager::getIdx(double x, double y) const {
  auto closestPt = mToPt(x, y);
  int idx = ptToIdx(closestPt.first, closestPt.second);
  return idx;
}

std::pair<double, double> MapManager::mToPt(double x, double y) const {
  double xx = std::round(x / fabs(s_x)) * fabs(s_x);
  double yy = std::round(y / fabs(s_y)) * fabs(s_y);

  return std::make_pair(xx, yy);
}

int MapManager::ptToIdx(double x, double y) const {
  assert(std::fmod(x/fabs(s_x),1.)==0 && "Error in ptToIdx");
  assert(std::fmod(y/fabs(s_y),1.)==0 && "Error in ptToIdx");

  if( x < 0 || y < 0 ) {
    return -1;
  } else {
    int xx = int(x / fabs(s_x));
    int yy = int(y / fabs(s_y));
    return xx + yy * n_x;
  }
}

void MapManager::print() {
  std::cout << "----------------------------------------" << std::endl;
  std::cout << "n_x: " << n_x << std::endl;
  std::cout << "n_y: " << n_y << std::endl;
  std::cout << "n_xy: " << n_xy << std::endl;

  std::cout << "desired_s_x: " << desired_s_x << std::endl;
  std::cout << "desired_s_y: " << desired_s_y << std::endl;

  std::cout << "s_x: " << s_x << std::endl;
  std::cout << "s_y: " << s_y << std::endl;

  std::cout << "map_z size: " << map_z.size() << std::endl;
  std::cout << "map_xr size: " << map_xr.size() << std::endl;
  std::cout << "map_yr size: " << map_yr.size() << std::endl;
  std::cout << "map_zr size: " << map_zr.size() << std::endl;

  std::cout << "len_x: " << len_x << std::endl;
  std::cout << "len_y: " << len_y << std::endl;
  std::cout << "len_z: " << len_z << std::endl;

  std::cout << "min_xr: " << min_xr << std::endl;
  std::cout << "min_yr: " << min_yr << std::endl;
  std::cout << "min_zr: " << min_zr << std::endl;
  std::cout << "----------------------------------------" << std::endl
            << std::endl;

}
