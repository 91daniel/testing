/*!
 *  \file MapManager.h
 *
 *  Created on: Dec, 09 2015
 *      Author: daniel
 */

#ifndef SRC_MAPMANAGER_H_
#define SRC_MAPMANAGER_H_

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// octomap
#include <octomap/octomap.h>
#include <octomap/math/Quaternion.h>
#include <octomap/OcTree.h>

#include <boost/shared_ptr.hpp>

#include <vector>
#include <string>
#include <iostream>



/**
 * \brief A class to read in and access a map and do collision checking.
 *
 * MapManager assumes that the map is stored in a .xyz file.
 * The first line of the file is the header and is skipped.
 * All the following lines contain the real coordinates of points in meters, xr,yr,zr.
 * As separator between each coordinate xr,yr,zr, space (" ") is used.
 *
 * For convenience, the points are transformed, such that the minimal xr,yr are shifted to 0,0.
 * Furthermore the points are transformed s.t. the minimal zr is shifted to 0 as well.
 * This results in transformed coordinates x,y,z.
 * The real coordinates are accessible through getXr(), getYr(), getZr().
 * The shifted coordinates are accessible through getX(), getY(), getZ().
 */
class MapManager {
 public:
  MapManager();
  ~MapManager();

  /**
   * \brief Read a map from an .xyz file. Furthermore, generate an octree octomap from the map for visualization.
   *
   * readMapAndGenerateOctomap assumes that the map is stored in a .xyz file. The first line of the file is the header and is skipped.
   * All the following lines contain the real coordinates of points in meters, xr,yr,zr.
   * As separator between each coordinate xr,yr,zr, space (" ") is used.
   */
  void readMapAndGenerateOctomap(std::string rel_path, std::string fileName);
  void readMap(std::string rel_path, std::string fileName);
  void readMapAndReduce(std::string rel_path, std::string fileName, std::string newFileName);
  void readOctomap(std::string rel_path, std::string fileName);
  void generateOctomap(std::string rel_path, std::string fileName);
  void reset();

  /**
   * \brief Return true if x,y,z collide with the map read in in the constructor. Otherwise, return false.
   */
  bool collide(double x, double y, double z) const;

  /**
   * \brief Get x coordinate of map from the two coordinates x,y, where [x]=[y]=meter
   */
  double getX(double x, double y) const;

  /**
   * \brief Get y coordinate of map from the two coordinates x,y, where [x]=[y]=meter
   */
  double getY(double x, double y) const;

  /**
   * \brief Get height z of map at the two coordinates x,y, where [x]=[y]=meter
   */
  double getZ(double x, double y) const;

  /**
   * \brief Get original xr coordinate of map from the two coordinates x,y, where [x]=[y]=meter
   */
  double getXr(double x, double y) const;

  /**
   * \brief Get original yr coordinate of map from the two coordinates x,y, where [x]=[y]=meter
   */
  double getYr(double x, double y) const;

  /**
   * \brief Get original height zr of map at the two coordinates x,y, where [x]=[y]=meter
   */
  double getZr(double x, double y) const;

  /**
   * \brief Returns idx for which in map_z[idx] the height of the value at x,y is stored.
   */
  int getIdx(double x, double y) const;

  /**
   * \brief From the two coordinates x,y ([x]=[y]=meter) get the closest point xx,yy ([xx]=[yy]=meter) with known height
   */
  std::pair<double,double> mToPt(double x, double y) const;

  /**
   * \brief From the coordinates of a point with known height, get the index to access the height,x,y in the map_z,x,y vectors
   */
  int ptToIdx(double x, double y) const;

  void print();
 public:
  int n_xy; /**< Number of data points. */
  int n_x; /**< Number of data points in x dimension. */
  int n_y; /**< Number of data points in y dimension. */

  double desired_s_x; /**< Desired distance between two points in x direction */
  double desired_s_y; /**< Desired distance between two points in x direction */
  double s_x; /**< Distance (positive) between two points in x direction */
  double s_y; /**< Distance (positive) between two points in x direction */

  // TODO think about changing this to std::map --> may be slower but indexing will get more intutive
  std::vector<double> map_x; /**< Contains the original x-values/ heights of the map as a 1D array. */
  std::vector<double> map_y; /**< Contains the original y-values/ heights of the map as a 1D array. */
  std::vector<double> map_z; /**< Contains the z-values/ heights of the map as a 1D array. */
  std::vector<double> map_xr; /**< Contains the original x-values/ heights of the map as a 1D array. */
  std::vector<double> map_yr; /**< Contains the original y-values/ heights of the map as a 1D array. */
  std::vector<double> map_zr; /**< Contains the original z-values/ heights of the map as a 1D array. */

  double len_x; /**< Length in x direction in meters */
  double len_y; /**< Length in y direction in meters */
  double len_z; /**< Length in z direction in meters */
  double min_xr, min_yr, min_zr;

  boost::shared_ptr<octomap::OcTree> octomap; /**< Octomap for visualizing the map. */
  double octomap_res;
  bool saveOctomap;

  std::vector<double> airplane_bb;



};

#endif /* SRC_DUBINSMOTIONVALIDATOR_H_ */
