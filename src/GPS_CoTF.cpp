#include "GPS_CoTF.h"

GPS_CoTF::GPS_CoTF(const double &_ENU_head_deg, const Eigen::Vector2d &ENU_LALO, const vector<Eigen::Vector2d> &MSN_LALO, const vector<Eigen::Vector2d> &MSN_XY)
{
  ENU_head_rad = _ENU_head_deg*PAI/180.0;
  std::array<double, 2> delta_EN;
  int LALO_num = MSN_LALO.size();
  int XY_num = MSN_XY.size();
  if(LALO_num != XY_num)
  {
    cout << "Error: Mismatch in the number of coordiates!" << endl;
    return;
  }
  Eigen::MatrixXd H_mat = Eigen::MatrixXd::Zero(2*LALO_num, 4);
  Eigen::VectorXd y = Eigen::VectorXd::Zero(2*LALO_num);
  for(int i=0; i<LALO_num; i++)
  {
    // MSN参考点在GPS坐标系下的坐标
    delta_EN = wgs84::toCartesian({ENU_LALO(0), ENU_LALO(1)}, {MSN_LALO.at(i)(0), MSN_LALO.at(i)(1)});
    H_mat(2*i, 0) = MSN_XY.at(i)(0);
    H_mat(2*i, 1) = -MSN_XY.at(i)(1);
    H_mat(2*i, 2) = 1;
    H_mat(2*i+1, 0) = MSN_XY.at(i)(1);
    H_mat(2*i+1, 1) = MSN_XY.at(i)(0);
    H_mat(2*i+1, 3) = 1;

    y(2*i) = delta_EN[0];
    y(2*i+1) = delta_EN[1];
  }
  Eigen::Vector4d est = (H_mat.transpose()*H_mat).inverse()*H_mat.transpose()*y;

  MSN_head_rad = atan2(est(1), est(0));

  MSN_to_GPS_offset(0) = est(2);
  MSN_to_GPS_offset(1) = est(3);

  MSN_ref_GPS << cos(MSN_head_rad), -sin(MSN_head_rad), sin(MSN_head_rad), cos(MSN_head_rad);
  ENU_ref_GPS << cos(ENU_head_rad), -sin(ENU_head_rad), sin(ENU_head_rad), cos(ENU_head_rad);
}

Eigen::Vector2d GPS_CoTF::MSN_to_ENU(const Eigen::Vector2d &MSN_XY)
{
  Eigen::Vector2d ENU_XY(0.0, 0.0);
  ENU_XY = ENU_ref_GPS.inverse()*MSN_ref_GPS*MSN_XY+ENU_ref_GPS.inverse()*MSN_to_GPS_offset;
  return ENU_XY;
}

Eigen::Vector2d GPS_CoTF::MSN_to_ENU(const double &MSN_X, const double &MSN_Y)
{
  Eigen::Vector2d ENU_XY(0.0, 0.0);
  Eigen::Vector2d MSN_XY(MSN_X, MSN_Y);
  ENU_XY = ENU_ref_GPS.inverse()*MSN_ref_GPS*MSN_XY+ENU_ref_GPS.inverse()*MSN_to_GPS_offset;
  return ENU_XY;
}

void GPS_CoTF::MSN_to_ENU(const double &MSN_X, const double &MSN_Y, double &ENU_X, double &ENU_Y)
{
  Eigen::Vector2d ENU_XY(0.0, 0.0);
  Eigen::Vector2d MSN_XY(MSN_X, MSN_Y);
  ENU_XY = ENU_ref_GPS.inverse()*MSN_ref_GPS*MSN_XY+ENU_ref_GPS.inverse()*MSN_to_GPS_offset;
  ENU_X = ENU_XY(0);
  ENU_Y = ENU_XY(1);
  return;
}

Eigen::Vector2d GPS_CoTF::ENU_to_MSN(const Eigen::Vector2d &ENU_XY)
{
  Eigen::Vector2d MSN_XY(0.0, 0.0);
  MSN_XY = MSN_ref_GPS.inverse()*ENU_ref_GPS*(ENU_XY - ENU_ref_GPS.inverse()*MSN_to_GPS_offset);
  return MSN_XY;
}

Eigen::Vector2d GPS_CoTF::ENU_to_MSN(const double &ENU_X, const double &ENU_Y)
{
  Eigen::Vector2d MSN_XY(0.0, 0.0);
  Eigen::Vector2d ENU_XY(ENU_X, ENU_Y);
  MSN_XY = MSN_ref_GPS.inverse()*ENU_ref_GPS*(ENU_XY - ENU_ref_GPS.inverse()*MSN_to_GPS_offset);
  return MSN_XY;
}

void GPS_CoTF::ENU_to_MSN(const double &ENU_X, const double &ENU_Y, double &MSN_X, double &MSN_Y)
{
  Eigen::Vector2d MSN_XY(0.0, 0.0);
  Eigen::Vector2d ENU_XY(ENU_X, ENU_Y);
  MSN_XY = MSN_ref_GPS.inverse()*ENU_ref_GPS*(ENU_XY - ENU_ref_GPS.inverse()*MSN_to_GPS_offset);
  MSN_X = MSN_XY(0);
  MSN_Y = MSN_XY(1);
  return;
}
