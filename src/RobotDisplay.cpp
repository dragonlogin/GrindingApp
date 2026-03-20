#include "RobotDisplay.h"
#include <cmath>

static constexpr double kDeg = M_PI / 180.0;

gp_Trsf DhTrsf(double theta_deg, double d, double a, double alpha_deg)
{
	double t = theta_deg * kDeg;
	double al = alpha_deg * kDeg;
	double ct = cos(t), st = sin(t);
	double ca = cos(al), sa = sin(al);

	gp_Trsf trsf;
	trsf.SetValues(
		ct, -st * ca, st * sa, a * ct,
		st, ct * ca, -ct * sa, a * st,
		0.0, sa, ca, d
	);
	return trsf;
}

gp_Trsf RpyPosTrsf(const double rpy[3], const double pos[3])
{
	double cr = cos(rpy[0] * kDeg), sr = sin(rpy[0] * kDeg);
	double cp = cos(rpy[1] * kDeg), sp = sin(rpy[1] * kDeg);
	double cy = cos(rpy[2] * kDeg), sy = sin(rpy[2] * kDeg);

	// R = Rz(yaw) * Ry(pitch) * Rx(roll)
	gp_Trsf trsf;
	trsf.SetValues(
		cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, pos[0],
		sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, pos[1],
		-sp, cp * sr, cp * cr, pos[2]
	);
	return trsf;
}

QVector<gp_Trsf> ComputeFkHome(const RbRobot& robot)
{
	QVector<gp_Trsf> fk(robot.joints.size());
	for (int i = 0; i < robot.joints.size(); ++i) {
		const RbJoint& j = robot.joints[i];
		gp_Trsf dh = DhTrsf(j.offset_deg, j.d, j.a, j.alpha_deg);
		if (i == 0)
			fk[i] = dh;
		else {
			fk[i] = fk[i - 1];
			fk[i].Multiply(dh);
		}
	}
	return fk;
}
