#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <libplayerc++/playerc++.h>

#include <fstream>
#include <string>

using namespace PlayerCc;

const double AVOIDANCE_DISTANCE = .4;
const double AVOID_TURN_SPEED = .5;
const double WANDER_SPEED = .15;
const double PI = 3.1415926;

int port = 6665;

/**
 * log_init
 */
void log_init() {

	std::ostringstream fn;
    fn << "../log/range_" << port << ".txt";

	std::ofstream file(fn.str().c_str());
    file << "PORT,R0,R1,R2,R3,R4,R5,R6,R7,SX,SY" << std::endl;
    file.close();
}

/**
 * log_range
 * @param xspeed
 * @param yawspeed
 * @param range
 */
void log_range(double xspeed, double yawspeed, RangerProxy &sp) {

	std::ostringstream fn;
    fn << "../log/range_" << port << ".txt";

    std::fstream file;
    file.open(fn.str().c_str(), std::fstream::in | std::fstream::out | std::fstream::app);
    file 
    << port << ','
    << sp.GetRange(0) << ',' 
    << sp.GetRange(1) << ',' 
    << sp.GetRange(2) << ',' 
    << sp.GetRange(3) << ',' 
    << sp.GetRange(4) << ',' 
    << sp.GetRange(5) << ',' 
    << sp.GetRange(6) << ',' 
    << sp.GetRange(7) << ','
    << xspeed << ','
    << yawspeed
    << std::endl;
    file.close();
}
	
/**
 * nondecreasing function that converts the distance from the ranger sensors
 * to a speed of the wheel
 * @param distance
 */
double MotorForce(double distance) {
	return 0.2 * distance;
}

/**
 * converts differential drive wheel speeds into a forward speed and turn rate
 * that Player understands
 * http://player-stage-gazebo.10965.n7.nabble.com/Differential-wheels-gt-Speed-amp-Turn-Rate-td7562.html
 * @param left speed of left wheel in metres/second
 * @param right speed of right wheel in metres/second
 */
void SetMotors(double left, double right, double *x, double *y) {
	const double separation = 40*0.001;
	*x = (left + right)/2;
	*y = (left - right)/separation;
}

/**
 * seek
 * @param xspeed
 * @param yawspeed
 * @param bp
 */
void Seek(double *xspeed, double *yawspeed, BlobfinderProxy &bp) {

	int noBlobs = bp.GetCount();
	int biggestBlobArea = 0;
	int biggestBlob = 0;

	for (int i=0; i<noBlobs; i++) {
		playerc_blobfinder_blob_t currBlob =  bp[i];
		if (abs((int)currBlob.area) > biggestBlobArea) {
			biggestBlob = i;
			biggestBlobArea = currBlob.area;
		}
	}

	playerc_blobfinder_blob_t blob;
	blob = bp[biggestBlob];

	double margin = 10;
	uint32_t centre;
	int turningSpeed = 10;

	centre = bp.GetWidth()/2;

	if (blob.x < centre - margin) {
		SetMotors(WANDER_SPEED, WANDER_SPEED*.9, xspeed, yawspeed);
	} else if (blob.x > centre + margin) {
		SetMotors(WANDER_SPEED*.9, WANDER_SPEED, xspeed, yawspeed);
	} else {
		SetMotors(WANDER_SPEED, WANDER_SPEED, xspeed, yawspeed);
	}

	return;
}

/**
 * wander
 * @param xspeed
 * @param yawspeed
 * @param sp
 */
void Wander(double *xspeed, double *yawspeed, RangerProxy &sp) {

	*xspeed = WANDER_SPEED;
	*yawspeed = 0;
	return;
}

/**
 * avoid obstacles
 * @param xspeed
 * @param yawspeed
 * @param sp
 * @param lvalues
 * @param rvalues
 */
void AvoidObstacles(double *xspeed, double *yawspeed, RangerProxy &sp, double lvalues, double rvalues) {

	srand(time(NULL));
	int turn_degrees = rand() % 180;

	*xspeed = 0;
	*yawspeed = turn_degrees * PI/360;

	if (lvalues < AVOIDANCE_DISTANCE) {
		*yawspeed *= -1;
	}	

	return;
}

/**
 * behave
 * @param xspeed
 * @param yawspeed
 * @param sp
 * @param bp
 */
void Behave(double *xspeed, double *yawspeed, RangerProxy &sp, BlobfinderProxy &bp) {

	double lvalues = 0;
	lvalues += sp.GetRange(6);
	lvalues += sp.GetRange(7);

	double rvalues = 0;
	rvalues += sp.GetRange(0);
	rvalues += sp.GetRange(1);

	if (lvalues < AVOIDANCE_DISTANCE || rvalues < AVOIDANCE_DISTANCE) {
		AvoidObstacles(xspeed, yawspeed, sp, lvalues, rvalues);
	} else if (bp.GetCount() > 0) {
		Seek(xspeed, yawspeed, bp);
	} else {
		Wander(xspeed, yawspeed, sp);
	}

	log_range(*xspeed, *yawspeed, sp);

	return;
}

int main(int argc, char *argv[]) {

	double xspeed, yawspeed;

	if (argc >= 2) {
		port = atoi(argv[1]);
	}

	log_init();

	PlayerClient robot("localhost", port);
	Position2dProxy pp(&robot, 0);
	RangerProxy sp(&robot, 0);
	BlobfinderProxy bp(&robot, 0);

	pp.SetMotorEnable(1);
	pp.RequestGeom();
	sp.RequestGeom();

	while (!sp.IsValid()) robot.Read();
	while (true) {
		robot.Read();
		Behave(&xspeed, &yawspeed, sp, bp);
		pp.SetSpeed(xspeed, yawspeed);			
	}

	return 0;
}