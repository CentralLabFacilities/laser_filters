/* 
 * File:   global.h
 * Author: jkummert
 *
 * Created on November 24, 2016, 10:44 AM
 */

#ifdef leg_glob
  #define EXTERN
#else
  #define EXTERN extern
#endif

EXTERN std::vector<double> leg_distances;
EXTERN std::vector<double> leg_angles;
EXTERN bool isInit;
EXTERN boost::mutex leg_lock;