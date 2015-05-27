#include <RLinks/Controller.hpp>
#include <Pacer/controller.h>

std::string plugin_namespace;

void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t) {

	//instantiate controller object
    static boost::shared_ptr<Controller> control = boost::shared_ptr<Controller>(new Controller());

    //create pointers to hold IMU data
    float *mag;
    float *accel;

	//update the components -- can also use control->update_all()
	control->update("imu");

	//get value from imu handler. values can be :
	//gyro, gyro_raw, accel, accel_raw, mag, mag_raw, delta_theta, delta_velocity
	mag = control->imuh->get_data("mag");
	accel = control->imuh->get_data("accel");

	//build ravelin vector from magnetometer values
	Ravelin::Vector3d M(mag[0], mag[1], mag[2]);

	//build ravelin vector from magnetometer values
	Ravelin::Vector3d A(accel[0], accel[1], accel[2]);

	//assign Z as the "up" coordinate (opposite of accelerometer when stationary)
	Ravelin::Vector3d Z = -A;

	Z.normalize();

	//create y coordinate from z x m
	Ravelin::Vector3d Y = Ravelin::Vector3d::cross(Z, M);

	//create x coordinate from y x z
	Ravelin::Vector3d X = Ravelin::Vector3d::cross(Y, Z);

	//normalize values
	Y.normalize();
	X.normalize();

	//build camera frame from values
	Ravelin::Matrix3d c_frame_matrix = Ravelin::Matrix3d(X[0], X[1], X[2], Y[0], Y[1], Y[2],  Z[0], Z[1], Z[2]);

	//c_frame_matrix.transpose();

	//set the origin at zero
	control->pclh->camera_frame->x = Ravelin::Origin3d(0, 0, 0 );

	//set the camera frame of the device
	control->pclh->camera_frame->q = c_frame_matrix;

	//update the kinect/xtion pro sensor
	control->update("vision");

	int i,j;
    int h_x_size = 10;
    int h_y_size = 10;
    double scaling_factor = 100;

    int rows = control->pclh->Heightmap.rows();
    int cols = control->pclh->Heightmap.columns();

	for (i = 0; i < rows; i++) {
		for (j = 0; j < cols; j++) {
			Ravelin::Vector3d p((double)(i - rows/2 ) / scaling_factor, (double)(j - cols/2 ) / scaling_factor, control->pclh->Heightmap(i, j));
			Utility::visualize.push_back( Pacer::VisualizablePtr( new Pacer::Point( p,   Ravelin::Vector3d(0, 0, 1), 0.1)));
		}
	}


}

/** This is a quick way to register your plugin function of the form:
  * void Update(const boost::shared_ptr<Pacer::Controller>& ctrl, double t)
  */
#include "register-plugin"
