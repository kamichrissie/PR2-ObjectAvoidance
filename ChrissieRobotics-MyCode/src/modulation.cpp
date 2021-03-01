#include "laser_line_extraction/modulation.h"
#include "laser_line_extraction/base_controller.h"
#include "laser_line_extraction/utilities.h"

using namespace std;

namespace line_extraction{
Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");

 Modulation::Modulation(Eigen::Vector3d& curr_position, Eigen::VectorXf& curr_speed) :
 modulation_(2,2),
 speed_(curr_speed),
 position_(curr_position),
 {
   modulation_ << 1,0,0,1;
 }
 Modulation::Modulation() :
 modulation_(2,2),
 speed_(3),
 position_(3),
 {
   speed_ << 1, 1, 1;
   position_ << 1, 1, 1;
   modulation_ << 1,0,0,1;

/*   do_ir_modulator_ = false;
   if (do_ir_modulator_)
     first_ellipse_ = 0;
   else
     first_ellipse_ = 2;*/
 }

 Modulation::~Modulation()
 {

 }



std::vector<ellipse_extraction::Ellipse>& Modulation::getEllipses() {

   //include ellipses extracted from the measured lines in line_extraction_ros.cpp

   return ellipses_;
 }



 void Modulation::computeXiWave() {
   xi_wave_.clear();
   for (int i = 0; i < ellipses_.size(); i++) {
     Eigen::Vector2f pos_ell_frame;
     //Xi Wave = Current position - Ellipse Center
     pos_ell_frame << position_[0] - ellipse.getPPoint()[0], position_[1] - ellipses.getPPoint()[1];
     //What does this calculation do? (Multiplication with the rotation matrix?)
     pos_ell_frame = ellipses[i].getR().transpose() * pos_ell_frame;
       /*pos_ell_frame << position_[0] -ellipses_[i].getPPoint()[0] , position_[1] -ellipses_[i].getPPoint()[1];
       pos_ell_frame = ellipses_[i].getR().transpose()*pos_ell_frame;*/
     std::vector<double> xi_wave_i = {pos_ell_frame[0] , pos_ell_frame[1]};
     xi_wave_.push_back(xi_wave_i);
   }
 }

 /*
 void Modulation::computeGammaAlpha(int ellipseNr){
   int ellipseNr = 1;
   //robot base orientation
   Eigen::Matrix2f R;
   R << std::cos(position_[2]),-std::sin(position_[2]),std::sin(position_[2]),std::cos(position_[2]);
   Eigen::Vector2f or_x;
   or_x << 1.0,0.0;
   or_x = ellipses_[ellipseNr].getR().transpose()*R*or_x; //
   // angle between robot base orientation and connection to gripper
   double dot = -xi_wave_[ellipseNr][0]*or_x[0] - xi_wave_[ellipseNr][1]*or_x[1];     // dot product between [x1, y1] and [x2, y2]
   double det = -xi_wave_[ellipseNr][1]*or_x[0] + xi_wave_[ellipseNr][0]*or_x[1];     //x1*y2 - y1*x2      # determinant, angle = atan2(det, dot)
   double current_alpha = atan2(det, dot);

   //base orientation in ellipseframe
   double base_angle = position_[2]+ellipses_[ellipseNr].getAlpha();// atan2(or_x[1], or_x[0]);
   double alpha_dist = ellipses_[ellipseNr].getPPointAlpha() - base_angle;

   if (alpha_dist < -M_PI)
   {
     alpha_dist += 2.0*M_PI;
   }
   else if (alpha_dist > M_PI)
     alpha_dist -= 2.0*M_PI;

   double powerF = 2.0;
   gamma_alpha_ = pow((alpha_dist/ellipses_[ellipseNr].getAlphaAp()/2.0), powerF);
   double speed_before = speed_(12);//-gripper_yaw_speed_;

   if (gamma_alpha_ >= 0.1)
   {
     // ROS_INFO("Outside Angle Bound. Angledist to opt: %g, measured angle: %g and speed: %g",alpha_dist*180.0/M_PI,position_[2]*180.0/M_PI,speed_(12));
     if(alpha_dist*speed_before>=0.0)
       speed_before = speed_before *gamma_alpha_*10;
     else
       speed_before = -speed_before *gamma_alpha_*10;
   }
   else void Modulation::updateSpeedAndPosition(Eigen::Vector3d& curr_pose, Eigen::VectorXf& curr_speed,Eigen::VectorXd& curr_gripper_pose,double dt) {
   position_ = curr_pose;
   speed_ = curr_speed;
   Eigen::Matrix2f R(2,2);

   for (int k = 0 ; k < ellipses_.size(); k++) {

       Eigen::Vector3d radial_velocity;
       Eigen::Vector3d angle_velocity;
       angle_velocity << curr_speed[3],curr_speed[4],curr_speed[5];
       std::vector<double> ellipse_speed;

       // update orientation part of positioning for irm ellipses
       int nr_neighbors = 9;//19;
       Eigen::Vector2f pos_ell_frame;
       pos_ell_frame << position_[0] -ellipses_[k].getPPoint()[0] , position_[1] -ellipses_[k].getPPoint()[1];
       pos_ell_frame = ellipses_[k].getR().transpose()*pos_ell_frame;
       float _sample[4];
       CvMat sample_beta0 = cvMat( 1, 4, CV_32FC1, _sample );
       sample_beta0.data.fl[0] = (float)curr_gripper_pose(2);
       sample_beta0.data.fl[1] = (float)Q.toRotationMatrix().eulerAngles(2, 1, 0)[1];//0.0;
       if(sample_beta0.data.fl[1] >M_PI/2)
         sample_beta0.data.fl[1] = M_PI - sample_beta0.data.fl[1];
       else if(sample_beta0.data.fl[1] < - M_PI/2)
         sample_beta0.data.fl[1] = -M_PI - sample_beta0.data.fl[1];
       sample_beta0.data.fl[2] = (float)pos_ell_frame[0];
       sample_beta0.data.fl[3] = (float)pos_ell_frame[1];
       float _response[nr_neighbors];
       float _neighbors[1];
       CvMat resultMat = cvMat(1,1,CV_32FC1,_neighbors);
       CvMat neighborResponses = cvMat(1,nr_neighbors,CV_32F,_response);
       const float **neighbors=0;
       float result_beta0 = knnAngle_.find_nearest(&sample_beta0, nr_neighbors,&resultMat,neighbors,&neighborResponses);
       double sum_sin = 0.0;
       double sum_cos = 0.0;
       for (int s = 0; s < nr_neighbors; s++)
       {
         double neighbor_i = (double) neighborResponses.data.fl[s];
         sum_sin += sin(neighbor_i);
         sum_cos += cos(neighbor_i);
       }
       result_beta0 = atan2(sum_sin,sum_cos);

       // find aperture for leagal orientation with knn regression
       float result_beta_ap = knnAperture_.find_nearest(&sample_beta0, nr_neighbors);

       ellipses_[k].setPPointAlpha(result_beta0);
       ellipses_[k].setAlphaAp(result_beta_ap);
     }
     ellipses_[k].setSpeed(ellipse_speed);

  }


   {
     // ROS_INFO("Inside  Angle Bound. Angledist to opt: %g, measured angle: %g and speed: %g",alpha_dist*180.0/M_PI,position_[2]*180.0/M_PI,speed_(12));
     if(alpha_dist*speed_before<0.0)
       speed_before = speed_before *(1.0-gamma_alpha_);
   }
   if(do_ir_modulation_)
     speed_(12) = speed_before;// + gripper_yaw_speed_;
 }
*/

 void Modulation::computeGamma() {
   gamma_.clear();
   computeXiWave();
   int i = 0;
   collision_ = false;
   for (ellipse_extraction::Ellipse ellipse : ellipses_) {
     double gamma_i = pow(pow((xi_wave_[i][0]/ellipse.getHeight()), 2*ellipse.getP1()) + pow((xi_wave_[i][1]/ellipse.getWidth()), 2*ellipse.getP2()),1.0/ellipse.getP2());
     ellipses_[i].setInCollision(false);
/*
//What is "real_gamma" ?
     real_gamma_.push_back(gamma_i);
     if(gamma_i < 1.0) {//1.00015) {//
       ellipses_[i].setInCollision(true);
       if (ellipse.getType() == "inner")
       {
         gamma_i = 1.0;//1.00015;//
         collision_ = true;
       }
       else
         gamma_i = 1.0;
     }
*/
     gamma_.push_back(gamma_i);

     ellipse.setGamma(gamma_i);
     i++;
   }
 }

//Only needed for multiple Ellipses
 /*
 double Modulation::computeWeight(int k)
 {
   double w = 1;
   for (int i = first_ellipse_; i < ellipses_.size(); i++) {
     if (i != k) {
       w = w * ((gamma_[i] - 1)/((gamma_[k] - 1) + (gamma_[i] - 1)));
     }
   }
   if(w != w)
   {
     w = 1.0;
     for (int i = first_ellipse_; i < ellipses_.size(); i++) {
       if (i != k) {
         w = w * ((real_gamma_[i] - 1)/((real_gamma_[k] - 1) + (real_gamma_[i] - 1)));
       }
     }
   }
   if(!do_ir_modulation_ & first_ellipse_>k)
     w = 0;
   return w;
 }*/


 std::vector<double> Modulation::computeEigenvalue(int k) {
   std::vector<double> lambda;

   //Set to standard value?  double w = computeWeight(k);
   double w = 1.0;
   double collision_repulsion = -50.0;

   Eigen::Vector2f speed;
   speed << speed_(7)-ellipses_[k].getSpeed()[0],speed_(8)-ellipses_[k].getSpeed()[1];
   Eigen::Vector2f e_k1;
   e_k1 << assembleE_k(k)(0,0),assembleE_k(k)(0,1);
   bool passed_object = false;
   if(speed.transpose().dot(ellipses_[k].getR()*e_k1) > 0.0)
     passed_object = true; // use this to conmtroll tail effekt, stop modulation if object already passed


//General Obstacles
   lambda = {1.0 - (w / pow(gamma_[k],1.0/ellipses_[k].getRho())), 1.0 + (w / pow(gamma_[k],1.0/ellipses_[k].getRho()))};
   if(passed_object && !ellipses_[k].getInCollision())
   {
     lambda[0] = 1.0;
     lambda[1] = 1.0;
   }
   else if (ellipses_[k].getInCollision())
   {
     if (passed_object)
       lambda[0] = 2 - lambda[0];
     else
       lambda[0] = collision_repulsion;
   }
   else if(gamma_[k]<1.01 && !ellipses_[k].getInCollision())
   {
     speed_(0) = speed_(0)*(gamma_[k]-1.0);
     speed_(1) = speed_(1)*(gamma_[k]-1.0);
     speed_(2) = speed_(2)*(gamma_[k]-1.0);
   }

   return lambda;
 }

 Eigen::MatrixXf Modulation::assembleD_k(int k) {
   Eigen::MatrixXf d_k(2,2);
   d_k.setIdentity();
   std::vector<double> lambda = computeEigenvalue(k);

   for(int i = 0; i < 2; i++) {
     d_k(i, i) = lambda[i];
   }
   return d_k;
 }

 std::vector<double> Modulation::computeHyperplane(int k) {
   //Derivation of Gamma in ~Xi_i direction
   std::vector<double> n = {(pow(xi_wave_[k][0] / ellipses_[k].getHeight(), 2.0*ellipses_[k].getP1() -1))*2*ellipses_[k].getP1() /ellipses_[k].getHeight(),
     (pow(xi_wave_[k][1] / ellipses_[k].getWidth(), 2.0*ellipses_[k].getP2() -1)) *2*ellipses_[k].getP2() /ellipses_[k].getWidth()};

   Eigen::Vector2f n_rot;
   n_rot << n[0],n[1];
   n_rot = ellipses_[k].getR()*n_rot;
   ellipses_[k].setHyperNormal(std::vector<double> {n_rot(0),n_rot(1)} );
   return n;
 };


 std::vector<std::vector<double> > Modulation::computeEBase(int k, std::vector<double> normal) {
   int d = 2;
   std::vector<std::vector<double> > base = {{0, 0}};
   for (int i = 1; i <= d - 1; i++) {
     for (int j = 1; j <= d; j++) {
       if (j == 1) {
         base[i-1][j-1] = -normal[i - 1];
       } else if (j == i && i != 1) {
         base[i-1][j-1] = normal[0];
       } else {
         base[i-1][j-1] = 0;
       }
     }
   }
   return base;
 };


 double computeL2Norm(std::vector<double> v) {
   double res = 0;
   for(double entry : v) {
     res += entry * entry;
   }
   return sqrt(res);
 }


 Eigen::MatrixXf Modulation::assembleE_k(int k) {
   Eigen::MatrixXf e_k(2,2);
   std::vector<double> norm = computeHyperplane(k);
   std::vector<std::vector<double> > base = computeEBase(k, norm);
   for (int i = 0; i < 2; i++) {
     e_k(i, 0) = norm[i];
     if(i==0)
       e_k(i, 1) = norm[1];
     else
       e_k(i, 1) = -norm[0];
   }
   return e_k;
 }

 /* Robot rotation

 std::vector<std::vector<double> > Modulation::computeS(int k) {
   counter << position_[0];
   denom << ellipses_.getPPoint()[0][1] - ellipses_.getPPoint()[0][2] - modulation_;

   return counter/denom;
 }

 std::vector<std::vector<double> > Modulation::computealpha(int k) {
   psi1 << (- position_[0] + computeS(k) ) / (ellipses_.getPPoint()[0][1] - position_[0]);
   psi2 << (- position_[0] + computeS(k) ) / (ellipses_.getPPoint()[0][2] - position_[0]);

   alpha1 = acos(psi1);
   alpha2 = acos(psi2);

   alpha = (alpha1 + alpha2)/2.;

   return alpha;

 }

 void Modulation::computeRotationMatrix() {
   rotation_ << 1, 0,
                  0, 1;


   std::stringstream mes;
   computeGamma();
   bool out_mod = false;

   Eigen::MatrixXf d_k = assembleD_k(k);
   Eigen::MatrixXf e_k = assembleE_k(k);
   Eigen::MatrixXf res = (ellipses_.getR()*e_k * d_k * e_k.inverse()*ellipses_.getR().transpose());
   modulation_ = (res * modulation_);

 }


 */

 void Modulation::setEllipses(const std::vector<Ellipse> ellipses)
 {
   ellipses_ = ellipses;
 }

 void Modulation::computeModulationMatrix() {
   modulation_ << 1, 0,
                  0, 1;
   std::stringstream mes;
   computeGamma();
   bool out_mod = false;

   Eigen::MatrixXf d_k = assembleD_k(k);
   Eigen::MatrixXf e_k = assembleE_k(k);
   Eigen::MatrixXf res = (ellipses_.getR()*e_k * d_k * e_k.inverse()*ellipses_.getR().transpose());
   modulation_ = (res * modulation_);

   //Only for multiple ellipses
   /*for (int k = 0; k < ellipses_.size(); k++) {
     Eigen::MatrixXf d_k = assembleD_k(k);
     Eigen::MatrixXf e_k = assembleE_k(k);
     Eigen::MatrixXf res = (ellipses_[k].getR()*e_k * d_k * e_k.inverse()*ellipses_[k].getR().transpose());
     modulation_ = (res * modulation_);
   }*/

   //No return needed?
   //return modulation_;

 }

 Eigen::VectorXf Modulation::compModulation() {
   if (ellipses_.size() == 0)
     return speed_;
   computeModulationMatrix();
   Eigen::VectorXf d2(2);
   // find weighted relative speed with respect to obstacles
   double meanVelX = 0.0;
   double meanVelY = 0.0;
   double weightSum = 0.0;
   //Delete this line for multiple ellipses:
   ellipses_.size() = 1;
   for (int k = 0; k <ellipses_.size(); k++) {
     double weight_k = computeWeight(k);
     // if (weight_k < 0.1)
     //   continue;
     weightSum += weight_k;
     meanVelX += weight_k*ellipses_.getSpeed()[0];
     meanVelY += weight_k*ellipses_.getSpeed()[1];
     // ROS_INFO("Type %s, weight: %g, Speed(%g, %g)",ellipses_[k].getType().c_str(),computeWeight(k),ellipses_[k].getSpeed()[0],ellipses_[k].getSpeed()[1]);
   }
   d2 << speed_[7]-meanVelX/weightSum, speed_[8]-meanVelY/weightSum;
   d2 = modulation_ * d2;
   speed_(7) = d2[0]+meanVelX/weightSum;
   speed_(8) = d2[1]+meanVelY/weightSum;

   return speed_;
 }

 Eigen::VectorXf Modulation::apply(Eigen::VectorXf& curr_pose, Eigen::VectorXf& curr_speed,double dt,const Eigen::VectorXf& currF) {

   Eigen::Vector3d trans;
   trans[0] = curr_pose(7);
   trans[1] = curr_pose(8);

   Eigen::Quaterniond Q2 = Eigen::Quaterniond(curr_pose(13),curr_pose(10),curr_pose(11),curr_pose(12));
   auto euler = Q2.toRotationMatrix().eulerAngles(0,1,2);
   trans[2] = euler[2];



   // compute and return modulated velocity
   return compModulation();;
 }


}
