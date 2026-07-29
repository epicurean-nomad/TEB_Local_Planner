/*
    Swaayatt Robots Pvt. Ltd.
    Author              :   Ishita Kamboj, Abhigyan Ganguly
    Date Last Modified  :   06 Feb 2025
    Description         :   Contains all the operations required for 2D points
*/




#ifndef GEOMETRY_2D_UTILS_HPP_
#define GEOMETRY_2D_UTILS_HPP_

#include <iostream>
#include <vector>
#include <cmath>
#include <limits>


namespace utils_2d_geometry{

constexpr double PI = 3.14159265;

/*
    for storing any 2d points or vectors
*/



struct Point {
    double x;
    double y;
    Point() {}
    Point( double x_, double y_)
        : x(x_), y(y_) {}
};

struct PointTheta {
    double x;
    double y;
    double theta;
    PointTheta() {}
    PointTheta( double x_, double y_, double theta_)

        : x(x_), y(y_), theta(theta_) {}

};

struct PathPoint{
    utils_2d_geometry::PointTheta       point;                                          // point in the path (x,y,theta)
    double                              k;                                              // curvature at this point 
    int                                 direction;                                      // direction on the path to this node (1: forward, -1: backward)



    
    PathPoint(double x_, double y_, double yaw_) {
        point.x     = x_;
        point.y     = y_;
        point.theta = yaw_;
    } 

    PathPoint(utils_2d_geometry::PointTheta pt) {
        point.x     = pt.x;
        point.y     = pt.y;
        point.theta = pt.theta;
    }

    PathPoint(utils_2d_geometry::PointTheta pt, double k_, int direction_) 
        : k(k_), direction(direction_) {
        point.x     = pt.x;
        point.y     = pt.y;
        point.theta = pt.theta;
    }

    PathPoint(double x_, double y_, double yaw_, double k_, int direction_) 
        : k(k_), direction(direction_) {
        point.x     = x_;
        point.y     = y_;
        point.theta = yaw_;
    } 
    
    PathPoint() : k(0.0), direction(1) {
        point.x     = 0.0;
        point.y     = 0.0;
        point.theta = 0.0;
    } 
    
};


class Path{
    
    
public: 
    std::vector<utils_2d_geometry::PathPoint> traj;                          
    double cost;
    Path() {
        traj.clear();
    }
    Path(std::vector<utils_2d_geometry::PathPoint>& points, double cost_)
        : traj(points), cost(cost_) {}
    
    int numPaths(){
        return traj.size()-1;
    }
                                                                                                                                         
};

/*
    gives the magnitude of a vector
*/
double magnitude (const Point& vec) ;

/*
    gives dot product of 2 vec tors
*/
double dotProduct (const Point& v1, const Point& v2) ;

/*
    returns unit vector along the given vector
*/
Point normalize (const Point& v) ;

/*
    Brings angle b/w [-PI,+PI)
*/
void normalize_angle (double& angle) ;

/*
    intersection point of 2 lines given in point slope form
*/
Point findIntersection (const double& m1, const Point& x1, 
                        const double& m2, const Point& x2) ;

/*
    gives unit vector perpendicular to a line given in 2 point form
*/
Point computePerpendicularUnitVector (const Point& x1, const Point& x2) ;

/*
    Calculate the angle bisector b/w two given vectors
*/
Point calculateAngleBisector (const Point& pt1, const Point& pt2, const Point& pt3) ;

/*
    the angle from vector v1 to vector v2
    (anticlockwize as +ve)
*/
double angleBetweenVectors (const Point& v1, const Point& v2) ;

/*
    from degrees to radians
*/
double deg_to_rad (double angle) ;

/*
    changes frame of ref of point B from global frame to point A's frame of Reference 
*/
void change_Frame_of_Ref (Point A, double angle, Point& B) ;

/* 
    Utility function to check if point q lies on the line segment pr 
    when points are collinear
*/
// doesn't consider intersection at the last or the first point of segment as well
bool onSegment (const Point& p, const Point& q, const Point& r) ;
// considers intersection at the last or the first point of segment as well
bool onSegment_1 (const Point& p, const Point& q, const Point& r) ;

/* 
    Function to compute the orientation of the triplet (p, q, r)
    0 -> p, q, r are collinear
    1 -> clockwise
    -1 -> counterclockwise
*/
int points_orientation (const Point& p, const Point& q, const Point& r) ;

/*
    Function to check if two line segments (p1, p2) and (q1, q2) intersect
*/
// doesn't consider line touching at the end points as collision as well
bool Line_segment_intersectionCheck (const Point& p1, 
                                     const Point& p2, 
                                     const Point& q1, 
                                     const Point& q2);
// considers line touching at the end points as collision as well
bool Line_segment_intersectionCheck_1 (const Point& p1, 
                                     const Point& p2, 
                                     const Point& q1, 
                                     const Point& q2);

/*
    Function returning the perpendicular distance from a line given in 2 point form
    + : point on the left side from x1 to x2
    - : point on the left side from x1 to x2
*/
double perpendicularDistance (const Point& point, 
                              const Point& x1, 
                              const Point& x2) ;


}  // namespace utils_2d_geometry



double utils_2d_geometry::magnitude (const Point& vec) {
    return std::sqrt(vec.x * vec.x + vec.y * vec.y);
}

double utils_2d_geometry::dotProduct (const Point& v1, const Point& v2) {
    return v1.x * v2.x + v1.y * v2.y;
}

utils_2d_geometry::Point utils_2d_geometry::normalize (const Point& v) {
    double mag = magnitude(v);
    // handling case for zero vector
    if(mag==0.0)
        return v;
    return Point(v.x/mag , v.y/mag);
}

void utils_2d_geometry::normalize_angle (double& angle) {
    angle = std::fmod(angle, 2 * PI);   // brings the angle in [0,2PI)

    if (angle < -PI) {
        angle += 2*PI;
    } 
    else if (angle >= PI) {
        angle -= 2* PI;
    }
}
 
utils_2d_geometry::Point utils_2d_geometry::findIntersection (const double& m1, const Point& x1, 
                                                              const double& m2, const Point& x2
                                                             )  {
    // Check if the lines are parallel (slopes are equal)
    const double epsilon = 1e-6;
    if (std::abs(m1 - m2) < epsilon) {
        std::cout <<"The lines are parallel and do not intersect. (or are coinscident) \n";
        return {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    }

    double x_intersect, y_intersect;

    // in case the first line equation given is a line parallel to y axis - making slope infinite
    if (m1 > 1e+5 || m1 < -1*1e+5) {
        // Calculate the x-coordinate of the intersection
        x_intersect = x1.x;
        // Calculate the y-coordinate of the intersection
        y_intersect = m2 * (x_intersect - x2.x) + x2.y;
    }
    // in case the second line equation given is a line parallel to y axis - making slope infinite
    else if (m2 > 1e+5 || m2 < -1*1e+5) {
        // Calculate the x-coordinate of the intersection
        x_intersect = x2.x;
        // Calculate the y-coordinate of the intersection
        y_intersect = m1 * (x_intersect - x1.x) + x1.y;
    }
    else {
        // Calculate the x-coordinate of the intersection
        double x_intersect = (  m1*x1.x - m2*x2.x - x1.y + x2.y  ) / (m1 - m2);
        // Calculate the y-coordinate of the intersection using the first line equation
        double y_intersect = m1 * (x_intersect - x1.x) + x1.y;
    }

    return Point(x_intersect,y_intersect);
}

utils_2d_geometry::Point utils_2d_geometry::computePerpendicularUnitVector (const Point& x1, const Point& x2) {
    // Calculate the difference between the two points (vector of the line)
    double dx = x2.x - x1.x;
    double dy = x2.y - x1.y;
    // Normalize the line vector (to make it a unit vector)
    double magnitude = std::sqrt(dx * dx + dy * dy);

    if (magnitude == 0.0) {
        std::cout <<"Two consecutive points are the same. \n";
        return {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
    }

    // Perpendicular vector to (dx, dy) can be (-dy, dx) or (dy, -dx)
    double perp_x = -dy;
    double perp_y = dx;
    
    // Normalize the perpendicular vector
    double perp_magnitude = std::sqrt(perp_x * perp_x + perp_y * perp_y);
    perp_x /= perp_magnitude;
    perp_y /= perp_magnitude;

    // ensure the perpendicular points upwards always (has a slope of 0 to pi)
    if (perp_y < 0) {
        // If the y is negative, reflect the vector to ensure it's between 0 and pi
        perp_x = -perp_x;
        perp_y = -perp_y;
    }

    return utils_2d_geometry::Point(perp_x, perp_y);
}

utils_2d_geometry::Point utils_2d_geometry::calculateAngleBisector (const Point& pt1,
                                                                    const Point& pt2,
                                                                    const Point& pt3
                                                                   ) {
    // Calculate vectors v1 and v2
    Point v1 = { pt1.x - pt2.x, pt1.y - pt2.y };
    Point v2 = { pt3.x - pt2.x, pt3.y - pt2.y };

    // Normalize the vectors
    Point v1_normalized = normalize(v1);
    Point v2_normalized = normalize(v2);

    // Calculate the angle bisector by adding the normalized vectors
    Point bisector( v1_normalized.x + v2_normalized.x, v1_normalized.y + v2_normalized.y );

    // Normalize the bisector vector to get the unit angle bisector
    bisector = normalize(bisector);

    // Ensure the bisector is in the right half-plane (between slope 0 and pi)
    // if (bisector.y < 0) {
    //     // Flip the bisector to ensure it's pointing in the positive x direction
    //     bisector.x *= -1;
    //     bisector.y *= -1;
    // }

    return bisector;
}

double utils_2d_geometry::angleBetweenVectors (const Point& v1, const Point& v2) {
    double mag1 = magnitude(v1);
    double mag2 = magnitude(v2);
    
    // Handle case where one or both vectors have zero length
    if (mag1 == 0 || mag2 == 0) {
        std::cerr << "One or both vectors have zero length.\n";
        return 0.0;
    }

    // Calculate angles using atan2 for both vectors
    double angle1 = std::atan2(v1.y, v1.x);  // Angle of v1
    double angle2 = std::atan2(v2.y, v2.x);  // Angle of v2

    // Compute the difference in angles
    double angle_diff = angle2 - angle1;

    // Adjust to ensure the angle is within the range [-pi, pi]
    normalize_angle(angle_diff);

    return angle_diff;
}

double utils_2d_geometry::deg_to_rad (double angle) {
    angle = angle * PI / 180;
    normalize_angle(angle);
    return angle;
}

void utils_2d_geometry::change_Frame_of_Ref (Point A, double angle, Point& B) {
    B.x =   ((B.x - A.x) * cos(angle)) + ((B.y - A.y) * sin(angle));
    B.y =   ((B.y - A.y) * cos(angle)) - ((B.x - A.x) * sin(angle));
}

bool utils_2d_geometry::onSegment (const Point& p, const Point& q, const Point& r) {

    return q.x < std::max(p.x, r.x)   &&   q.x > std::min(p.x, r.x) &&
           q.y < std::max(p.y, r.y)   &&   q.y > std::min(p.y, r.y);
}
bool utils_2d_geometry::onSegment_1 (const Point& p, const Point& q, const Point& r) {

    return q.x <= std::max(p.x, r.x)   &&   q.x >= std::min(p.x, r.x) &&
           q.y <= std::max(p.y, r.y)   &&   q.y >= std::min(p.y, r.y);
}

int utils_2d_geometry::points_orientation (const Point& p, const Point& q, const Point& r) {
    double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (val == 0) return 0;                 // Collinear
    return (val > 0) ? 1 : -1;              // Clock or counterclockwise
}

bool utils_2d_geometry::Line_segment_intersectionCheck (const Point& p1, const Point& p2, 
                                                        const Point& q1, const Point& q2
                                                       ) {
    // std::cout << "check line segment intersection \n";

    int o1 = points_orientation(p1, p2, q1);
    int o2 = points_orientation(p1, p2, q2);
    int o3 = points_orientation(q1, q2, p1);
    int o4 = points_orientation(q1, q2, p2);

    // General case
    // std::cout <<" o1 = " << o1<<" o2 = "<<o2<<" o3 = "<< o3<< " o4 = "<<o4<<"\n";
    if (o1 == -o2 && o3 == -o4 && o1!=0 && o2!=0 && o3!=0 && o4!=0 ) return true;

    // std::cout << "going to special cases----------------------- \n";

    // Special cases
    // std::cout << "case 1  \n";
    if (o1 == 0 && onSegment(p1, q1, p2)) return true;
    // std::cout << "case 2  \n";
    if (o2 == 0 && onSegment(p1, q2, p2)) return true;
    // std::cout << "case 3  \n";
    if (o3 == 0 && onSegment(q1, p1, q2)) return true;
    // std::cout << "case 4  \n";
    if (o4 == 0 && onSegment(q1, p2, q2)) return true;

    return false;
}

bool utils_2d_geometry::Line_segment_intersectionCheck_1 (const Point& p1, const Point& p2, 
                                                        const Point& q1, const Point& q2
                                                       ) {
    // std::cout << "check line segment intersection \n";

    int o1 = points_orientation(p1, p2, q1);
    int o2 = points_orientation(p1, p2, q2);
    int o3 = points_orientation(q1, q2, p1);
    int o4 = points_orientation(q1, q2, p2);

    // General case
    // std::cout <<" o1 = " << o1<<" o2 = "<<o2<<" o3 = "<< o3<< " o4 = "<<o4<<"\n";
    if (o1 == -o2 && o3 == -o4 && o1!=0 && o2!=0 && o3!=0 && o4!=0 ) return true;

    // std::cout << "going to special cases----------------------- \n";

    // Special cases
    // std::cout << "case 1  \n";
    if (o1 == 0 && onSegment_1(p1, q1, p2)) return true;
    // std::cout << "case 2  \n";
    if (o2 == 0 && onSegment_1(p1, q2, p2)) return true;
    // std::cout << "case 3  \n";
    if (o3 == 0 && onSegment_1(q1, p1, q2)) return true;
    // std::cout << "case 4  \n";
    if (o4 == 0 && onSegment_1(q1, p2, q2)) return true;

    return false;
}

double utils_2d_geometry::perpendicularDistance (const Point& point, 
                                                 const Point& x1, 
                                                 const Point& x2
                                                ) {
    // Calculate the numerator of the distance formula (signed)
    double numerator = (x2.y - x1.y) * point.x - (x2.x - x1.x) * point.y + x2.x * x1.y - x2.y * x1.x;
    // Calculate the denominator of the perpendicular distance formula
    double denominator = std::sqrt(std::pow(x2.y - x1.y, 2) + std::pow(x2.x - x1.x, 2));
    // Return the signed perpendicular distance
    return numerator / denominator;
}



#endif  // ends GEOMETRY_2D_UTILS_HPP_