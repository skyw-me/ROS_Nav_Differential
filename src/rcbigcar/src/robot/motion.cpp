#include "motion.h"

Motion::Motion()
{
	ros::NodeHandle node_priv;

	//Setup Variables
	//Setup Motors
	for (int i = 0; i < MOTION_MOTOR_COUNT; i++)
	{
		motors[i] = new Motor(MOTION_MOTOR_PRESET[i].ID, MOTION_MOTOR_PRESET[i].Preset, MOTION_MOTOR_PRESET[i].Paramter);
	}

	//Setup Reconfigurable Paramters
	static ros::NodeHandle DynamicParamNodeHandle("~/motion");
	static dynamic_reconfigure::Server<rcbigcar::MotionConfig> DynamicParamServer(DynamicParamNodeHandle);
	DynamicParamServer.setCallback(boost::bind(&Motion::CallbackDynamicParam, this, _1, _2));

    //Setup Comm
	motor_sub   = node_priv.subscribe<std_msgs::Float64MultiArray>("motor", 10, &Motion::CallbackSetpoint, this);
	status_pub	= node_priv.advertise<std_msgs::Float64MultiArray>("motorstatus", 10);

	//Setup Watchdog
	motionWatchdog = ros::Time::now();
}

Motion::~Motion()
{
	//Free Motors
	for (int i = 0; i < MOTION_MOTOR_COUNT; i++)
	{
		delete motors[i];
	}
}

void Motion::update()
{
	UpdateWatchdog();
	UpdateMotors();
	UpdatePublisher();
}

void Motion::UpdateWatchdog()
{
	//Check timeout
	if ((ros::Time::now() - motionWatchdog).toSec() > MOTION_WATCHDOG_TIMEOUT)
	{
		//Zero motor powers
		for (int i = 0; i < MOTION_MOTOR_COUNT; i++)
		{
			if (motors[i]->Paramter.CloseloopType == CLOSELOOP_VELOCITY)
				motors[i]->Setpoint = 0;
		}
	}
}

void Motion::UpdateMotors()
{
	for (int i = 0; i < MOTION_MOTOR_COUNT; i++)
	{
		motors[i]->update();
	}
}

void Motion::UpdatePublisher()
{
	std_msgs::Float64MultiArray data;

	for (int i = 0; i < MOTION_MOTOR_COUNT; i++)
	{
		data.data.push_back(motors[i]->getPosition());
		data.data.push_back(motors[i]->getVelocity());
		data.data.push_back(motors[i]->Setpoint);
	}

	status_pub.publish(data);
}

void Motion::CallbackSetpoint(const std_msgs::Float64MultiArray::ConstPtr &setpoint)
{
	//reset watchdog
	motionWatchdog = ros::Time::now();

	//set setpoint
	for (int i = 0; i < std::min((int)setpoint->data.size(), MOTION_MOTOR_COUNT); i++)
	{
		motors[i]->Setpoint = setpoint->data[i];
	}
}

#define DynamicParamSet(id)                                                                                             \
	if (id < MOTION_MOTOR_COUNT)                                                                                        \
	{                                                                                                                   \
		ROS_INFO("Motion #%d Reconfigure: [Kp = %lf, Ki = %lf, Kd = %lf, Kf = %lf, KmaxI = %lf, KmaxO = %lf]",                       \
				 id, config.Kp_##id, config.Ki_##id, config.Kd_##id, config.Kf_##id, config.KmaxI_##id, config.KmaxO_##id);                \
		motors[id]->setCoefficients(config.Kp_##id, config.Ki_##id, config.Kd_##id, config.Kf_##id, config.KmaxI_##id, config.KmaxO_##id); \
	}

void Motion::CallbackDynamicParam(rcbigcar::MotionConfig &config, uint32_t level)
{
	(void) config;
	(void) level;
}