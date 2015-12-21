/*!
 * \file consts_and_params_MapManager.h
 *
 *  Created on: Apr 27, 2015
 *      Author: philipp
 */

#ifndef PARAMS_MAPMANAGER_H_
#define PARAMS_MAPMANAGER_H_

#include <boost/math/constants/constants.hpp>

namespace ParamsMapManager {

  // MapManager
  const double desired_s_x_param = 2;
  const double desired_s_y_param = 2;

  const double octomap_res_param = 2;

  std::vector<double> airplane_bb_param = {2.,2.,0.5};

  const bool saveOctomap_param = true;

}



#endif
