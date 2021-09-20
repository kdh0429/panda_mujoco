#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/select.h>
#include <termios.h>

#define EIGEN_STACK_ALLOCATION_LIMIT 0
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/SVD>


static struct termios initial_settings, new_settings;
 
static int peek_character = -1;
 
void init_keyboard()
{
    tcgetattr(0,&initial_settings);
    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
}
 
void close_keyboard()
{
    tcsetattr(0, TCSANOW, &initial_settings);
}
 
int _kbhit()
{
    unsigned char ch;
    int nread;
 
    if (peek_character != -1) 
        return 1;
    new_settings.c_cc[VMIN]=0;
    tcsetattr(0, TCSANOW, &new_settings);
    nread = read(0,&ch,1);
    new_settings.c_cc[VMIN]=1;
    tcsetattr(0, TCSANOW, &new_settings);
    if(nread == 1)
    {
        peek_character = ch;
        return 1;
    }
    return 0;
}
 
int _getch()
{
    char ch;
 
    if(peek_character != -1)
    {
        ch = peek_character;
        peek_character = -1;
        return ch;
    }
    read(0,&ch,1);
    return ch;
}
 
int _putch(int c) {
    putchar(c);
    fflush(stdout);
    return c;
}
namespace Eigen
{
// Eigen default type definition
#define EIGEN_MAKE_TYPEDEFS(Type, TypeSuffix, Size, SizeSuffix)    \
  typedef Matrix<Type, Size, Size> Matrix##SizeSuffix##TypeSuffix; \
  typedef Matrix<Type, Size, 1> Vector##SizeSuffix##TypeSuffix;    \
  typedef Matrix<Type, 1, Size> RowVector##SizeSuffix##TypeSuffix;

  typedef double rScalar;

  EIGEN_MAKE_TYPEDEFS(rScalar, d, 6, 6)
  EIGEN_MAKE_TYPEDEFS(rScalar, d, 7, 7)

} // namespace Eigen

Eigen::Vector3d quintic_spline(
    double time,       // Current time
    double time_0,     // Start time
    double time_f,     // End time
    double x_0,        // Start state
    double x_dot_0,    // Start state dot
    double x_ddot_0,   // Start state ddot
    double x_f,        // End state
    double x_dot_f,    // End state
    double x_ddot_f    // End state ddot
)
{
    double a1, a2, a3, a4, a5, a6;
    double time_s;

    Eigen::Vector3d result;

    if (time < time_0)
    {
        result << x_0, x_dot_0, x_ddot_0;
        return result;
    }
    else if (time > time_f)
    {
        result << x_f, x_dot_f, x_ddot_f;
        return result;
    }

    time_s = time_f - time_0;
    a1 = x_0; a2 = x_dot_0; a3 = (x_ddot_0 / 2.0);

    Eigen::Matrix3d Temp;
    Temp << pow(time_s, 3), pow(time_s, 4), pow(time_s, 5),
        (3.0 * pow(time_s, 2)), (4.0 * pow(time_s, 3)), (5.0 * pow(time_s, 4)),
        (6.0 * time_s), (12.0 * pow(time_s, 2)), (20.0 * pow(time_s, 3));

    Eigen::Vector3d R_temp;
    R_temp << (x_f - x_0 - x_dot_0 * time_s - x_ddot_0 * pow(time_s, 2) / 2.0),
        (x_dot_f - x_dot_0 - x_ddot_0 * time_s),
        (x_ddot_f - x_ddot_0);

    Eigen::Vector3d RES;

    RES = (Temp.inverse() * R_temp);
    a4 = RES(0); a5 = RES(1); a6 = RES(2);

    double time_fs = (time - time_0);
    double position = (a1 + a2 * pow(time_fs, 1) + a3 * pow(time_fs, 2) + a4 * pow(time_fs, 3) + a5 * pow(time_fs, 4) + a6 * pow(time_fs, 5));
    double velocity = (a2 + 2.0 * a3 * pow(time_fs, 1) + 3.0 * a4 * pow(time_fs, 2) + 4.0 * a5 * pow(time_fs, 3) + 5.0 * a6 * pow(time_fs, 4));
    double acceleration = (2.0 * a3 + 6.0 * a4 * pow(time_fs, 1) + 12.0 * a5 * pow(time_fs, 2) + 20.0 * a6 * pow(time_fs, 3));
    result << position, velocity, acceleration;

    return result;
}

static double cubic(double time,    ///< Current time
                      double time_0,  ///< Start time
                      double time_f,  ///< End time
                      double x_0,     ///< Start state
                      double x_f,     ///< End state
                      double x_dot_0, ///< Start state dot
                      double x_dot_f  ///< End state dot
  )
  {
    double x_t;

    if (time < time_0)
    {
      x_t = x_0;
    }
    else if (time > time_f)
    {
      x_t = x_f;
    }
    else
    {
      double elapsed_time = time - time_0;
      double total_time = time_f - time_0;
      double total_time2 = total_time * total_time;  // pow(t,2)
      double total_time3 = total_time2 * total_time; // pow(t,3)
      double total_x = x_f - x_0;

      x_t = x_0 + x_dot_0 * elapsed_time

            + (3 * total_x / total_time2 - 2 * x_dot_0 / total_time - x_dot_f / total_time) * elapsed_time * elapsed_time

            + (-2 * total_x / total_time3 +
               (x_dot_0 + x_dot_f) / total_time2) *
                  elapsed_time * elapsed_time * elapsed_time;
    }

    return x_t;
  }

  static double cubicDot(double time,    ///< Current time
                         double time_0,  ///< Start time
                         double time_f,  ///< End time
                         double x_0,     ///< Start state
                         double x_f,     ///< End state
                         double x_dot_0, ///< Start state dot
                         double x_dot_f ///< End state dot
  )
{
    double x_t;

    if (time < time_0)
    {
        x_t = x_dot_0;
    }
    else if (time > time_f)
    {
        x_t = x_dot_f;
    }
    else
    {
        double elapsed_time = time - time_0;
        double total_time = time_f - time_0;
        double total_time2 = total_time * total_time;  // pow(t,2)
        double total_time3 = total_time2 * total_time; // pow(t,3)
        double total_x = x_f - x_0;

        x_t = x_dot_0

            + 2 * (3 * total_x / total_time2 - 2 * x_dot_0 / total_time - x_dot_f / total_time) * elapsed_time

            + 3 * (-2 * total_x / total_time3 + (x_dot_0 + x_dot_f) / total_time2) * elapsed_time * elapsed_time;
    }

    return x_t;
}

static Eigen::Vector3d getPhi(Eigen::Matrix3d current_rotation,
                            Eigen::Matrix3d desired_rotation)
{
Eigen::Vector3d phi;
Eigen::Vector3d s[3], v[3], w[3];

for (int i = 0; i < 3; i++)
{
    v[i] = current_rotation.block<3, 1>(0, i);
    w[i] = desired_rotation.block<3, 1>(0, i);
    s[i] = v[i].cross(w[i]);
}
phi = s[0] + s[1] + s[2];
phi = -0.5 * phi;

return phi;
}

const static Eigen::Matrix3d rotationCubic(double time,
    double time_0,
    double time_f,
    const Eigen::Matrix3d &rotation_0,
    const Eigen::Matrix3d &rotation_f)
{
    if (time >= time_f)
    {
        return rotation_f;
    }
    else if (time < time_0)
    {
        return rotation_0;
    }
    double tau = cubic(time, time_0, time_f, 0, 1, 0, 0);
    Eigen::Matrix3d rot_scaler_skew;
    rot_scaler_skew = (rotation_f* rotation_0.transpose()).log();

    Eigen::Matrix3d result =  (rot_scaler_skew * tau).exp()*rotation_0;

    return result;
}

const static Eigen::Vector3d rotationCubicDot(double time,
    double time_0,
    double time_f,
    const Eigen::Matrix3d &rotation_0, const Eigen::Matrix3d &rotation_f)
{
    Eigen::Vector3d result;
    result << 0, 0, 0;

    if (time >= time_f)
    {
        return result;
    }
    else if (time < time_0)
    {
        return result;
    }

    Eigen::Matrix3d rotation_d = rotation_f * rotation_0.transpose();
    double theta = acos((rotation_d(0, 0) + rotation_d(1, 1) + rotation_d(2, 2) - 1) / 2);
    double theta_dot = cubicDot(time, time_0, time_f, 0, theta, 0, 0);
    Eigen::Vector3d w;
    w << 1 / (2 * sin(theta))* (rotation_d(2, 1) - rotation_d(1, 2)), 1 / (2 * sin(theta))* (rotation_d(0, 2) - rotation_d(2, 0)), 1 / (2 * sin(theta))* (rotation_d(1, 0) - rotation_d(0, 1));

    result = theta_dot * w;

    return result;
}

  static inline double lowPassFilter(double input, double prev, double dt, double cut_off_frequency)
  {
    double alpha = (2*3.1415*dt*cut_off_frequency) / (2*3.1415*dt*cut_off_frequency + 1);
    return alpha*input + (1-alpha)*prev;
  }