//============================================================================
// Name        : test.cpp
// Author      : dani
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>
#include <octomap_msgs/conversions.h>

#include <iostream>
#include <chrono>
#include <ctime>
#include <Eigen/Dense>

#include "MapManager.h"

#include <boost/config.hpp>
#include <boost/shared_ptr.hpp>


using namespace std;

enum DubinsPathSegmentType {
  DUBINS_LEFT = 0,
  DUBINS_STRAIGHT = 1,
  DUBINS_RIGHT = 2
};

const DubinsPathSegmentType dubinsPathType[6][3] = { { DUBINS_LEFT,
    DUBINS_STRAIGHT, DUBINS_LEFT }, { DUBINS_RIGHT, DUBINS_STRAIGHT,
    DUBINS_RIGHT }, { DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_LEFT }, {
    DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_RIGHT }, { DUBINS_RIGHT, DUBINS_LEFT,
    DUBINS_RIGHT }, { DUBINS_LEFT, DUBINS_RIGHT, DUBINS_LEFT } };

class Ct {
 public:
  Ct(): f(0.),g(0.), duration(0) {}
 public:
  double f,g;
  std::chrono::duration<int,std::micro>::rep duration;
  double add(double a, double b);

};

double Ct::add(double a, double b) {
  double c = 0.;
  std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
  for(int i=0;i<10000;++i) {
    c=c+1;
  }
  c=0;
  c = a+b;

  std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
  duration = duration + std::chrono::duration_cast<std::chrono::microseconds>(t2- t1).count();

  return a+b;

}

int main(int argc, char** argv) {
  cout << "!!!Hello World!!!" << endl;  // prints !!!Hello World!!!

  std::cout << *dubinsPathType[0] << std::endl;
  const DubinsPathSegmentType* type_;

  type_ = dubinsPathType[0];
  std::cout << type_[0] << type_[1] << type_[2] << std::endl;

  type_ = dubinsPathType[1];
  std::cout << type_[0] << type_[1] << type_[2] << std::endl;

  type_ = dubinsPathType[3];
  std::cout << type_[0] << type_[1] << type_[2] << std::endl;

  type_ = dubinsPathType[5];
  std::cout << type_[0] << type_[1] << type_[2] << std::endl;

  Ct i;
  std::cout << "time: " << i.duration << std::endl;

  double h = i.add(5.,2.);
  std::cout << "h: " << h << " time: " << i.duration << std::endl;

  h = i.add(3.,8.);
  std::cout << "h: " << h << " time: " << i.duration << std::endl;

  h = i.add(3.,122.);

  std::cout << "h: " << h << " time: " << i.duration << std::endl;

  Eigen::Vector4d a(10,11,12,2.);
  double f=2,g=3;
  double deviation_xy = Eigen::Vector2d(a.block(0,0,2,1)-Eigen::Vector2d(f,g)).norm();
  std::cout << "dev_xy: " << deviation_xy << std::endl;
  std::cout << "ceil 2.: " << ceil(2.) << std::endl;
  std::cout << "ceil 2.0: " << ceil(2.0) << std::endl;
  std::cout << "ceil 2.01: " << ceil(2.01) << std::endl;
  std::cout << "ceil 2.5: " << ceil(2.5) << std::endl;
  std::cout << "round 2.: " << round(2.) << std::endl;
  std::cout << "ceil 2.0: " << round(2.0) << std::endl;
  std::cout << "ceil 2.01: " << round(2.01) << std::endl;
  std::cout << "ceil 2.5: " << round(2.5) << std::endl;


  int ctr = 0;
  std::ofstream ofile("/home/daniel/catkin_ws/src/testing/maps/customMap.xyz");
  ofile << "Header" << "\n";
  for(double y = 155.; y >= 0; y -= 2) {
    for(double x = 5.; x <= 160; x += 2) {
      ofile << x << " " << y << " " << ctr << "\n";
    }
    ctr +=3;
  }
  ofile.close();

  boost::shared_ptr<MapManager> mM(new MapManager());
  mM->readMapAndReduce("/maps/","TicinoValley_3km2_2m.xyz", "TicinoValley_3km2_10m.xyz");
  mM->reset();
  mM->readMapAndGenerateOctomap("/maps/","TicinoValley_3km2_2m.xyz");
//  mM->print();


/*
  std::cout << "0, 0" << std::endl;
  std::cout << mM->getX(0.,0.) << std::endl; // 0
  std::cout << mM->getXr(0.,0.) << std::endl; // 18
  std::cout << mM->getY(0.,0.) << std::endl; // 0
  std::cout << mM->getYr(0.,0.) << std::endl; // -40
  std::cout << mM->getZ(0.,0.) << std::endl; // 955-955
  std::cout << mM->getZr(0.,0.) << std::endl; // 955
  std::cout << std::endl;

  std::cout << "13, 0" << std::endl;
  std::cout << mM->getX(13.,0.) << std::endl; // 13 (!!!12
  std::cout << mM->getXr(13.,0.) << std::endl; // 31 (!!!30
  std::cout << mM->getY(13.,0.) << std::endl; // 0
  std::cout << mM->getYr(13.,0.) << std::endl; // -40
  std::cout << mM->getZ(13.,0.) << std::endl; // 0
  std::cout << mM->getZr(13.,0.) << std::endl; // 955
  std::cout << std::endl;

  std::cout << "0, 22" << std::endl;
  std::cout << mM->getX(0.,22.) << std::endl; // 0
  std::cout << mM->getXr(0.,22.) << std::endl; // 18
  std::cout << mM->getY(0.,22.) << std::endl; // 22
  std::cout << mM->getYr(0.,22.) << std::endl; // -40+22=-18
  std::cout << mM->getZ(0.,22.) << std::endl; //
  std::cout << mM->getZr(0.,22.) << std::endl;
  std::cout << std::endl;

  std::cout << "0, 21" << std::endl;
  std::cout << mM->getX(0.,21.) << std::endl; // 0
  std::cout << mM->getXr(0.,21.) << std::endl; // 18
  std::cout << mM->getY(0.,21.) << std::endl; // 22
  std::cout << mM->getYr(0.,21.) << std::endl; // -40+22=-18
  std::cout << mM->getZ(0.,21.) << std::endl; //
  std::cout << mM->getZr(0.,21.) << std::endl;
  std::cout << std::endl;

  std::cout << "5, 18" << std::endl;
  std::cout << mM->getX(5, 18) << std::endl;
  std::cout << mM->getXr(5, 18) << std::endl;
  std::cout << mM->getY(5, 18) << std::endl;
  std::cout << mM->getYr(5, 18) << std::endl;
  std::cout << mM->getZ(5, 18) << std::endl;
  std::cout << mM->getZr(5, 18) << std::endl;
  std::cout << "1" << std::endl;
  std::cout << std::endl;
*/


  octomap::OcTree octomap(10.);
  octomap::Pointcloud pcl;
  octomap::point3d origin(0,0,0);

  double ctr1 = 0;

  for(double x = 5.; x <= 160; x += 2) {
    for(double y = 5.; y <= 160; y += 2) {
      //for(double z = 1.; z <= 7; z += 1) {
        octomap::point3d pt(x, y, ctr1);
        //octomap->insertRay(origin,pt);
        octomap.updateNode(pt, true);
        //pcl.push_back(x,y, z);

        //std::cout << "x,y: " << map_x[ctr] << " " << map_y[ctr] << " z: " << z << std::endl;
      //}
    }
    ctr1 +=3;
  }
  for(double x = -15.; x <= -1; x += 2) {
    for(double y = 1.; y <= 7; y += 2) {
      for(double z = 1.; z <= 4; z += 2) {
        octomap::point3d pt(x, y, z);
        //octomap.updateNode(pt, false);

      }
      ctr1 +=1;
    }
  }
  for(double x = -7.; x <= 8; x += 2) {
    for(double y = -7.; y <= -4; y += 2) {
      for(double z = 1.; z <= 4; z += 2) {
        octomap::point3d pt(x, y, z);
        //octomap.updateNode(pt, true);
      }
      ctr1 +=1;
    }
  }
  for(double x = -7.; x <= -4; x += 2) {
    for(double y = -3.; y <= 8; y += 2) {
      for(double z = 1.; z <= 4; z += 2) {
        octomap::point3d pt(x, y, z);
        //octomap.updateNode(pt, true);
      }
      ctr1 +=1;
    }
  }
  std::cout << "collide: " << mM->collide(0,0,0)<<std::endl;
  std::cout << "collide: " << mM->collide(40,100,0)<<std::endl;

  //octomap.updateNode(pt, false);
/*
  if(!octomap.search(-5,0,1)) {
    std::cout << "  This cell is unknown" << std::endl;
  } else {
    std::cout << "is occupied?: " << octomap.isNodeOccupied(octomap.search(-5,0,1)) << std::endl;
  }
  if(!octomap.search(-20,-20,1)) {
    std::cout << "  This cell is unknown" << std::endl;
  } else {
    std::cout << "is occupied?: " << octomap.isNodeOccupied(octomap.search(-20,-20,1)) << std::endl;
  }
  if(!octomap.search(7.9,5,1)) {
    std::cout << "  This cell is unknown" << std::endl;
  } else {
    std::cout << "is occupied?: " << octomap.isNodeOccupied(octomap.search(7.9,5,1)) << std::endl;
  }
  if(!octomap.search(6,5,1)) {
    std::cout << "  This cell is unknown" << std::endl;
  } else {
    std::cout << "is occupied?: " << octomap.isNodeOccupied(octomap.search(6,5,1)) << std::endl;
  }
  if(!octomap.search(6,5,0)) {
    std::cout << "  This cell is unknown" << std::endl;
  } else {
    std::cout << "is occupied?: " << octomap.isNodeOccupied(octomap.search(6,5,0)) << std::endl;
  }
  if(!octomap.search(6,5,3.99)) {
    std::cout << "  This cell is unknown" << std::endl;
  } else {
    std::cout << "is occupied?: " << octomap.isNodeOccupied(octomap.search(6,5,3.99)) << std::endl;
  }
  if(!octomap.search(-1,-1,1)) {
    std::cout << "  This cell is unknown" << std::endl;
  } else {
    std::cout << "is occupied?: " << octomap.isNodeOccupied(octomap.search(-1,-1,1)) << std::endl;
  }
  if(!octomap.search(-14.4,1,1)) {
    std::cout << "  This cell is unknown" << std::endl;
  } else {
    std::cout << "is occupied?: " << octomap.isNodeOccupied(octomap.search(-14.4,1,1)) << std::endl;
  }
  std::cout << "volume: " << octomap.volume() << std::endl;
  //octomap.updateInnerOccupancy();

  //octomap.updateInnerOccupancy();
  //octomap.insertPointCloud(pcl, origin);
  octomap.prune();
*/
//  octomap.write("/home/daniel/Desktop/tmp.ot");
  std::cout << "Done" << std::endl;
  // ROS
  ros::init(argc, argv, "testing");
  ros::NodeHandle n;

  ros::Publisher octomap_pub = n.advertise<octomap_msgs::Octomap>("octomap_msg",1);

  octomap_msgs::Octomap octomap_msg;
  octomap_msg.binary = 1;
  octomap_msg.id = 1;
  octomap_msg.header.frame_id = "/base_link";
  octomap_msg.header.stamp = ros::Time::now();

  bool octomap_ok = octomap_msgs::fullMapToMsg(*(mM->octomap),octomap_msg);
//  bool octomap_ok = octomap_msgs::fullMapToMsg(octomap,octomap_msg);

  ros::Rate r(10);

  while (ros::ok()) {
    // Get updates
    ros::spinOnce();
    if (octomap_ok)
      octomap_pub.publish(octomap_msg);
    else
      ROS_WARN("OCT Map serialization failed!");

    r.sleep();
  }

  return 0;
}
