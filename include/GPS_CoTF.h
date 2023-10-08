/* 该文件实现ENU坐标系和给定任务坐标系之间坐标转换 */
#ifndef GPS_COTF_H
#define GPS_COTF_H
#include "WGS84toCartesian.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Eigen>

#define PAI (3.1415926)

using namespace std;

class GPS_CoTF
{
  private:
  // MSN坐标系相对正北正东的偏移角，逆时针为正方向
    double MSN_head_rad;
  // 上电建立的ENU坐标系相对正北正东的偏移角，逆时针为正方向
    double ENU_head_rad;
  // 任务坐标系原点相对于GPS原点的偏移，GPS坐标系方向沿正北正东
    Eigen::Vector2d MSN_to_GPS_offset;
  // MSN相对正北正东的旋转矩阵
    Eigen::Matrix2d MSN_ref_GPS;
  // ENU相对正北正东的旋转矩阵
    Eigen::Matrix2d ENU_ref_GPS;
  public:
  // 构造函数，计算偏移量
  /* _ENU_head_deg: ENU坐标系建立时，相对正北正东的偏移角，逆时针为正方向；可从电子罗盘处获取
   * ENU_LALO：ENU原点的纬度和经度
   * MSN_LALO：参考点的纬度和经度
   * MSN_XY：参考点在任务坐标系下的坐标
   */
    GPS_CoTF(const double &_ENU_head_deg, const Eigen::Vector2d &ENU_LALO, const vector<Eigen::Vector2d> &MSN_LALO, const vector<Eigen::Vector2d> &MSN_XY);
  // 任务坐标系坐标 -> ENU坐标系坐标
    Eigen::Vector2d MSN_to_ENU(const Eigen::Vector2d &MSN_XY);
    Eigen::Vector2d MSN_to_ENU(const double &MSN_X, const double &MSN_Y);
    void MSN_to_ENU(const double &MSN_X, const double &MSN_Y, double &ENU_X, double &ENU_Y);
  // ENU坐标系坐标 -> 任务坐标系坐标
    Eigen::Vector2d ENU_to_MSN(const Eigen::Vector2d &ENU_XY);
    Eigen::Vector2d ENU_to_MSN(const double &ENU_X, const double &ENU_Y);
    void ENU_to_MSN(const double &ENU_X, const double &ENU_Y, double &MSN_X, double &MSN_Y);
};
#endif
