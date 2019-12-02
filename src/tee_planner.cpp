#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

#include <sstream>

#include <err.h>
#include <stdio.h>
#include <string.h>

/* OP-TEE TEE client API (built by optee_client) */
#include <tee_client_api.h>

/* To the the UUID (found the the TA's h-file(s)) */
#include <hello_world_ta.h>
#include "sensor_msgs/LaserScan.h"

TEEC_Result res;
TEEC_Context ctx;
TEEC_Session sess;
TEEC_UUID uuid = TA_HELLO_WORLD_UUID;
uint32_t err_origin;

float l_step = 0.5;
float a_step = 0.5;
int l_dir = 1;
int a_dir = 1;
int threshold = 1;


state s;

void tee_init() {
	res = TEEC_InitializeContext(NULL, &ctx);
	if (res != TEEC_SUCCESS)
		errx(1, "TEEC_InitializeContext failed with code 0x%x", res);

	res = TEEC_OpenSession(&ctx, &sess, &uuid,
			       TEEC_LOGIN_PUBLIC, NULL, NULL, &err_origin);
	if (res != TEEC_SUCCESS)
		errx(1, "TEEC_Opensession failed with code 0x%x origin 0x%x",
			res, err_origin);
}

void tee_get_dir(const sensor_msgs::LaserScan::ConstPtr& data)
{
  	TEEC_Operation op;
	memset(&op, 0, sizeof(op));
	op.paramTypes = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_INPUT, TEEC_VALUE_INPUT, TEEC_VALUE_OUTPUT, TEEC_NONE);
	
	float ranges[1000];
	for (int i = 0; i < data->ranges.size(); i++) {
		ranges[i] = data->ranges[i];
	}

	op.params[0].tmpref.buffer = ranges;
	op.params[0].tmpref.size = sizeof(float[1000]);
	
	op.params[1].value.a = threshold;
	res = TEEC_InvokeCommand(&sess, TA_HELLO_WORLD_CMD_GET_DIR, &op,
				 &err_origin);
	if (res != TEEC_SUCCESS)
		errx(1, "TEEC_InvokeCommand failed with code 0x%x origin 0x%x",
			res, err_origin);
	
	l_dir = op.params[2].value.a;
	printf("Sensor reading: %f -> Dir: %d\n", ranges[359], l_dir);
}


int tee_get_twist() {
	TEEC_Operation op;
	memset(&op, 0, sizeof(op));
	op.paramTypes = TEEC_PARAM_TYPES(TEEC_MEMREF_TEMP_INPUT, TEEC_VALUE_OUTPUT, TEEC_NONE, TEEC_NONE);

	op.params[0].tmpref.buffer = &s;
	op.params[0].tmpref.size = sizeof(s);

	res = TEEC_InvokeCommand(&sess, TA_HELLO_WORLD_CMD_GET_TWIST, &op,
				 &err_origin);
	if (res != TEEC_SUCCESS)
		errx(1, "TEEC_InvokeCommand failed with code 0x%x origin 0x%x",
			res, err_origin);
	return op.params[1].value.a;
}

void tee_destroy() {
	TEEC_CloseSession(&sess);
	TEEC_FinalizeContext(&ctx);
}

void robot_init() {
	s.x = 0;
	s.y = 0;
	s.pX = 0;
	s.pY = -1;
	s.tX = 6;
	s.tY = 9;
}

int main(int argc, char **argv)
{
  tee_init();

  ros::init(argc, argv, "tee_planner");
  ros::NodeHandle n;

  ros::Publisher commander = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  ros::Subscriber senser = n.subscribe("/front/scan/", 1, tee_get_dir);
  // ros::Subscriber goal = n.subscribe("/set_goal/", 1, set_goal);

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    geometry_msgs::Twist cmd;
    
    a_dir = tee_get_twist();

    cmd.linear.x = l_step * l_dir; 
    cmd.linear.y = 0;
    cmd.linear.z = 0;

    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = a_step * a_dir;
    
    commander.publish(cmd);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

