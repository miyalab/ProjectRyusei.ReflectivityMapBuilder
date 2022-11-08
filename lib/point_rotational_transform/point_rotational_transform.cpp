//-----------------------------
// include
//-----------------------------
// STL
#include <iostream>
#include <cmath>
#include <vector>

#include "point_rotational_transform.hpp"

//-----------------------------
// Symbol
//-----------------------------
//#define PROCESS_TIME_SHOW

//-----------------------------
// Namespace & using
//-----------------------------


//-----------------------------
// Methods
//-----------------------------
/**
 * @brief Project name
 * 
 */
namespace project_ryusei{
void PointRotationalTransform::setAngle(const double &roll, const double &pitch, const double &yaw)
{
    m_cos_roll  = std::cos(roll);
    m_cos_pitch = std::cos(pitch);
    m_cos_yaw   = std::cos(yaw);
    m_sin_roll  = std::sin(roll);
    m_sin_pitch = std::sin(pitch);
    m_sin_yaw   = std::sin(yaw);
}

Point32 PointRotationalTransform::transPoint(const Point32 &point) const 
{
    // IMUを原点に合わせる
    const double &x = point.x - m_offset_x;
    const double &y = point.y - m_offset_y;
    const double &z = point.z - m_offset_z;

    // 三次元回転行列　          
    const double &m11 =  m_cos_pitch * m_cos_yaw;
    const double &m12 = -m_cos_pitch * m_sin_yaw;
    const double &m13 = -m_sin_pitch;
    const double &m21 =  m_sin_roll * m_sin_pitch * m_sin_yaw + m_cos_roll * m_sin_yaw;
    const double &m22 = -m_sin_roll * m_sin_pitch * m_sin_yaw + m_cos_roll * m_cos_yaw;
    const double &m23 =  m_sin_roll * m_cos_pitch;
    const double &m31 =  m_cos_roll * m_sin_pitch * m_cos_yaw + m_sin_roll * m_sin_yaw;
    const double &m32 = -m_cos_roll * m_sin_pitch * m_sin_yaw - m_sin_roll * m_cos_yaw;
    const double &m33 =  m_cos_roll * m_cos_pitch;

    Point32 ret;
    ret.x = m11 * x + m12 * y + m13 * z + m_offset_x;
    ret.y = m21 * x + m22 * y + m23 * z + m_offset_y;
    ret.z = m31 * x + m32 * y + m33 * z + m_offset_z;

    return ret;
}
}

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------