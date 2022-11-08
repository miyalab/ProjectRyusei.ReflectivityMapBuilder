#ifndef _PROJECT_RYUSEI_LIB_POINT_ROTATION_HPP_
#define _PROJECT_RYUSEI_LIB_POINT_ROTATION_HPP_

//-----------------------------
// include
//-----------------------------
// ROS2
#include <geometry_msgs/msg/point32.hpp>

//-----------------------------
// Namespace & using
//-----------------------------
using geometry_msgs::msg::Point32;

/**
 * @brief Project Ryusei
 * 
 */
namespace project_ryusei {
//-----------------------------
// Class
//-----------------------------
class PointRotationalTransform{
public:
    PointRotationalTransform(){}
    virtual ~PointRotationalTransform(){}
    void setOffset(const double &_x, const double &_y, const double &_z){m_offset_x = _x; m_offset_y = _y; m_offset_z = _z;}
    void setAngle(const double &_roll, const double &_pitch, const double &_yaw);
    Point32 transPoint(const Point32 &point) const;
private:
    double m_offset_x = 0;
    double m_offset_y = 0;
    double m_offset_z = 0;
    
    double m_cos_roll = 1;
    double m_sin_roll = 0;
    double m_cos_pitch = 1;
    double m_sin_pitch = 0;
    double m_cos_yaw = 1;
    double m_sin_yaw = 0;
};
}

#endif // _PROJECT_RYUSEI_LIB_POINT_ROTATION_HPP_

//-----------------------------------------------------------------------------------
// end of file
//-----------------------------------------------------------------------------------