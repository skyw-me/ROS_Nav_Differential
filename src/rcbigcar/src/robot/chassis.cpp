#include "chassis.h"

double YawFromQuaternion(const geometry_msgs::Quaternion &quat)
{
  tf::Quaternion q_orient(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3 m_orient(q_orient);

  double roll, pitch, yaw;
  m_orient.getRPY(roll, pitch, yaw);

  return yaw;
}
double YawFromQuaternion(const tf::Quaternion &quat)
{
  double roll, pitch, yaw;
  tf::Matrix3x3 m(quat);
  m.getEulerYPR(yaw, pitch, roll);
  return yaw;
}

Chassis::Chassis()
{
  ros::NodeHandle node_priv;

  // Setup Variables
  // Setup Motors
  for (int i = 0; i < 2; i++)
  {
    motors[i] = new Motor(MOTOR_CHASSIS_ID_START + i, &MOTOR_CHASSIS, MOTOR_CHASSIS_PARAMTER);
  }

  // Setup Paramters
  node_priv.param<bool>("IsDebug", Config_IsDebug, true);

  // Setup Reconfigurable Paramters
  static ros::NodeHandle DynamicParamNodeHandle("~/chassis");
  static dynamic_reconfigure::Server<rcbigcar::ChassisConfig> DynamicParamServer(DynamicParamNodeHandle);
  DynamicParamServer.setCallback(boost::bind(&Chassis::CallbackDynamicParam, this, _1, _2));

  // Setup Comm
  twist_sub = node_priv.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &Chassis::CallbackVelocity, this);
  odom_pub  = node_priv.advertise<nav_msgs::Odometry>("odom", 10);

  // Setup Odometry
  x = y = theta = 0;
  for (int i = 0; i < 2; i++)
  {
    last_position[i] = motors[i]->getPosition();
  }

  odom_last_time = ros::Time::now();
  vx_local = vtheta_local = 0;

  odom_seq = 0;

  // Setup Watchdog
  motorWatchdog = ros::Time::now();

  // Setup Debug
  if (Config_IsDebug)
  {
    dbg_spd_setpoint_pub = node_priv.advertise<std_msgs::Float64MultiArray>("dbg_set_spd", 10);
    dbg_spd_real_pub = node_priv.advertise<std_msgs::Float64MultiArray>("dbg_real_spd", 10);
  }
}

Chassis::~Chassis()
{
  // Free Motors
  for (int i = 0; i < 2; i++)
  {
    delete motors[i];
  }
}

void Chassis::update()
{
  // Update Motors
  for (int i = 0; i < 2; i++)
  {
    motors[i]->update();
  }

  // Update Modules
  UpdateWatchdog();
  UpdateOdometry();
  UpdateDebug();
}

void Chassis::UpdateOdometry()
{
  // Must get initial position
  double d[2];

  // calculate delta
  for (int id = 0; id < 2; id++)
  {
    d[id] = motors[id]->getPosition() - last_position[id];
    last_position[id] = motors[id]->getPosition();
  }

  // differential drive
  // L: 0, R: 1
  double dx = CHASSIS_WHEEL_RADIUS * (d[1] + (-d[0])) / 2;
  double dtheta = CHASSIS_WHEEL_RADIUS * (d[1] - (-d[0])) / CHASSIS_D;

  x += dx * cos(theta);
  y += dx * sin(theta);

  theta += dtheta;
  theta = fmod(theta, 2 * M_PI);

  // update velocity
  double dt = (ros::Time::now() - odom_last_time).toSec();
  odom_last_time = ros::Time::now();

  vx_local = dx / dt;
  vtheta_local = dtheta / dt;

  PublishPosition();
}

void Chassis::PublishPosition()
{
  // create quaternion from theta
  tf::Quaternion yaw_q = tf::createQuaternionFromYaw(theta);

  // publish odom message
  nav_msgs::Odometry odom;
  // init header
  odom.header.frame_id = "odom";
  odom.header.seq = ++odom_seq;
  odom.header.stamp = ros::Time::now();
  odom.child_frame_id = "base_link";
  // fill pose
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.orientation.x = yaw_q.x();
  odom.pose.pose.orientation.y = yaw_q.y();
  odom.pose.pose.orientation.z = yaw_q.z();
  odom.pose.pose.orientation.w = yaw_q.w();
  // fill twist
  odom.twist.twist.linear.x = vx_local;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.angular.z = vtheta_local;
  odom_pub.publish(odom);

  // publish tf message
  tf::Transform odom_base_tf;

  odom_base_tf.setOrigin(tf::Vector3(x, y, 0));
  odom_base_tf.setRotation(yaw_q);

  tf_odom_pub.sendTransform(tf::StampedTransform(odom_base_tf, ros::Time::now(), "odom", "base_link"));
}

void Chassis::CallbackVelocity(const geometry_msgs::Twist::ConstPtr &twist)
{
  // reset watchdog
  motorWatchdog = ros::Time::now();

  // set motor power
  double vx = twist->linear.x;
  double vw = twist->angular.z;

  // L: 0, R: 1
  double w[2] = {
    -(vx - vw * CHASSIS_D / 2) / CHASSIS_WHEEL_RADIUS, 
    (vx + vw * CHASSIS_D / 2) / CHASSIS_WHEEL_RADIUS
  };

  // Velocity Limitation
  double maxVel = 0.0;
  for (int i = 0; i < 2; i++)
    maxVel = std::max(maxVel, std::abs(w[i]));

  if (maxVel > Dyn_Config_MaxVel)
  {
    double factor = Dyn_Config_MaxVel / maxVel;
    for (int i = 0; i < 2; i++)
      w[i] *= factor;
  }

  // Send Velocity
  for (int i = 0; i < 2; i++)
    motors[i]->Setpoint = w[i];
}

void Chassis::UpdateWatchdog()
{
  // Check timeout
  if ((ros::Time::now() - motorWatchdog).toSec() > CHASSIS_WATCHDOG_TIMEOUT)
  {
    // Zero motor powers
    for (int i = 0; i < 2; i++)
    {
      motors[i]->Setpoint = 0;
    }
  }
}

void Chassis::CallbackDynamicParam(rcbigcar::ChassisConfig &config, uint32_t level)
{
  (void) level;

  // Dynamic Params
  Dyn_Config_MaxVel = config.MaxVel;

  // Dynamic Motor Params
  for (int i = 0; i < 2; i++)
  {
    motors[i]->setCoefficients(config.Kp, config.Ki, config.Kd, config.Kf, config.KmaxI, 1.0);
  }

  ROS_INFO("Chassis Reconfigure: [Kp = %lf, Ki = %lf, Kd = %lf, Kf = %lf, KmaxI = %lf, MaxVel = %lf]",
           config.Kp, config.Ki, config.Kd, config.Kf, config.KmaxI, config.MaxVel);
}

void Chassis::UpdateDebug()
{
  if (!Config_IsDebug)
    return;

  std_msgs::Float64MultiArray motorSetpoint;
  std_msgs::Float64MultiArray motorReal;

  for (int i = 0; i < 2; i++)
  {
    motorSetpoint.data.push_back(motors[i]->Setpoint);
    motorReal.data.push_back(motors[i]->getVelocity());
  }

  dbg_spd_setpoint_pub.publish(motorSetpoint);
  dbg_spd_real_pub.publish(motorReal);
}
