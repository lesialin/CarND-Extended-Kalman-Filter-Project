#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <cfloat>
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

//#define ONLYRASER
//#define ONLYLASER
//avoid to "only " test x both laser and lidar
#ifdef ONLYLASER
#undef ONLYRASER
#endif
#ifdef ONLYRASER
#undef ONLYLASER
#endif

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
           0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
           0, 0.0009, 0,
           0, 0, 0.09;

  H_laser_ << 1, 0, 0, 0,
           0, 1, 0, 0;
  Hj_ << 1, 1, 0, 0,
      1, 1, 0, 0,
      1, 1, 1, 1;
  /**
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  //transition matrix
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
          0, 1, 0, 1,
          0, 0, 1, 0,
          0, 0, 0, 1;
  //covariance matrix
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      //Range (rho) : radial distance from origin
      double rho = measurement_pack.raw_measurements_[0];
      //Bearing (phi) : angel between rho and x
      double phi = measurement_pack.raw_measurements_[1];
      //Radial velocity (rho_dot) : change of rho (range rate)
      double rho_dot = measurement_pack.raw_measurements_[2];
      //calculate position and velocity in cartesian coordinates
      double px = rho * cos(phi);
      double py = rho * sin(phi);
      double vx = rho_dot * cos(phi);
      double vy = rho_dot * sin(phi);
      //avoid dividing zero
      if (px < DBL_MIN) {
        px = DBL_MIN;
      }
      if (py < DBL_MIN) {
        py = DBL_MIN;
      }
      ekf_.x_ << px, py, vx, vy;
      previous_timestamp_ = measurement_pack.timestamp_;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      double px = measurement_pack.raw_measurements_[0];
      double py = measurement_pack.raw_measurements_[1];
      double vx = 0.0;
      double vy = 0.0;
      if (px < DBL_MIN) {
        px = DBL_MIN;
      }
      if (py < DBL_MIN) {
        py = DBL_MIN;
      }
      ekf_.x_ << px, py, vx, vy;
      previous_timestamp_ = measurement_pack.timestamp_;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  //generate process covariance matrix Q
  float  dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  float ax = 9.0;
  float ay = 9.0;
  float dt_2 = dt * dt;
  float dt_3 = dt * dt_2;
  float dt_4 = dt * dt_3;
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4 / 4 * ax, 0, dt_3 / 2 * ax, 0,
          0, dt_4 / 4 * ay, 0, dt_3 / 2 * ay,
          dt_3 / 2 * ax, 0, dt_2*ax, 0,
          0, dt_3 / 2 * ay, 0, dt_2*ay;
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
#ifdef ONLYLASER
  cout << "only processes lidar sensor" << endl;
  if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);

  }
#endif

#ifdef ONLYRASER
  cout << "only processes raser sensor" << endl;
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
#endif

#if !defined(ONLYRASER) && !defined(ONLYLASER)
  cout << "process raser and lidar sensor";
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }
#endif


  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
