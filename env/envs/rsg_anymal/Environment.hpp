//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#pragma once

#include <stdlib.h>
#include <set>
#include "../../RaisimGymEnv.hpp"
#include <iostream>
#include <cmath>


namespace raisim {

class ENVIRONMENT : public RaisimGymEnv {

 public:

  explicit ENVIRONMENT(const std::string& resourceDir, const Yaml::Node& cfg, bool visualizable) :
      RaisimGymEnv(resourceDir, cfg), visualizable_(visualizable) {

    /// create world
    world_ = std::make_unique<raisim::World>();

    /// add objects
    anymal_ = world_->addArticulatedSystem(resourceDir_+"/test/anymal_b_simple_description/robots/anymal-kinova-collision.urdf");
    anymal_->setName("anymal");
    anymal_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    world_->addGround();

    /// get robot data
    gcDim_ = anymal_->getGeneralizedCoordinateDim();
    gvDim_ = anymal_->getDOF();
//    std::cout << gvDim_ << std::endl;
    nJoints_ = gvDim_ - 6;

    /// initialize containers
    gc_.setZero(gcDim_); gc_init_.setZero(gcDim_);
    gv_.setZero(gvDim_); gv_init_.setZero(gvDim_);
    pTarget_.setZero(gcDim_); vTarget_.setZero(gvDim_); pTarget12_.setZero(nJoints_),prevTarget_.setZero(gcDim_),prevPrevTarget_.setZero(gcDim_);

    /// this is nominal configuration of anymal
    gc_init_ << 0, 0, 0.50, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8,0.0, 2.62, -1.57, 0.0, 2.62, 0.0;

    /// set pd gains
    Eigen::VectorXd jointPgain(gvDim_), jointDgain(gvDim_);
    jointPgain.setZero(); jointPgain.tail(nJoints_).setConstant(50.0);jointPgain.segment(6,12).setConstant(100.0);
//          std::cout << jointPgain.transpose() << std::endl;
    jointDgain.setZero(); jointDgain.tail(nJoints_).setConstant(0.2); jointDgain.segment(6,12).setConstant(0.2);
    anymal_->setPdGains(jointPgain, jointDgain);
    anymal_->setGeneralizedForce(Eigen::VectorXd::Zero(gvDim_));

    /// MUST BE DONE FOR ALL ENVIRONMENTS
    obDim_ = 51;
    actionDim_ = nJoints_; actionMean_.setZero(actionDim_); actionStd_.setZero(actionDim_);
    obDouble_.setZero(obDim_);

    /// action scaling
    actionMean_ = gc_init_.tail(nJoints_);
    double action_std;
    READ_YAML(double, action_std, cfg_["action_std"]) /// example of reading params from the config
    actionStd_.setConstant(action_std);

    /// Reward coefficients
    rewards_.initializeFromConfigurationFile (cfg["reward"]);

    /// indices of links that should not make contact with ground
    footIndices_.insert(anymal_->getBodyIdx("LF_SHANK"));
    footIndices_.insert(anymal_->getBodyIdx("RF_SHANK"));
    footIndices_.insert(anymal_->getBodyIdx("LH_SHANK"));
    footIndices_.insert(anymal_->getBodyIdx("RH_SHANK"));
    footIndices_.insert(anymal_->getBodyIdx("kinova_link_6"));

    RR_footIndex = anymal_->getBodyIdx("RH_SHANK");
    RL_footIndex = anymal_->getBodyIdx("LH_SHANK");
    FR_footIndex = anymal_->getBodyIdx("RF_SHANK");
    FL_footIndex = anymal_->getBodyIdx("LF_SHANK");


      posError_.setZero();
      TEEpos_.setZero();
      PEEpos_.setZero();
      baseError_.setZero(3);

      phaseSin_.setZero(2);
      footContactDouble_.setZero(4);

      /// visualize if it is the first environment
    if (visualizable_) {
      server_ = std::make_unique<raisim::RaisimServer>(world_.get());
      server_->launchServer();
      server_->focusOn(anymal_);
      visual_target = server_->addVisualSphere("visual_target",0.05,1,0,0,0.4);
//      visual_target2 = server_->addVisualSphere("visual_target2",0.05,0,1,0,0.4);
//        visual_target2 = server_->addVisualBox("visual_target2",0.1,0.1,0.1,0,1,0,0.4);
        visual_EEpos = server_->addVisualSphere("visual_EEpos",0.05,0,0,1,0.4);
    }
  }

  void init() final { }


  void reset() final {
    anymal_->setState(gc_init_, gv_init_);
    updateObservation();
    if (visualizable_) {
      Eigen::Vector3d des_pos(0.2*uniDist_(gen_)+2.8,0.2*uniDist_(gen_)-1.5,0.2*uniDist_(gen_)+0.6);
      visual_target->setPosition(des_pos);
      TEEpos_ = visual_target->getPosition();
    }
  }

  float step(const Eigen::Ref<EigenVec>& action) final {
      /// action scaling
      prevPrevTarget_ = prevTarget_;
      prevTarget_ = pTarget_;
      pTarget12_ = action.cast<double>();
      pTarget12_ = pTarget12_.cwiseProduct(actionStd_);
      pTarget12_ += actionMean_;
      pTarget_.tail(nJoints_) = pTarget12_;
//    pTarget_[22] = gc_init_[22];
//    pTarget_[24] = gc_init_[24];

//    pTarget_.segment(7,12) = gc_init_.segment(7,12);
      anymal_->setPdTarget(pTarget_, vTarget_);

//      /// for gait enforcing & foot clearance
//      true_contact.setZero(4);
//
//      for (auto &contact: anymal_->getContacts()) {
//          if (contact.skip()) continue; /// if the contact is internal, one contact point is set to 'skip'
//          if (FR_footIndex == contact.getlocalBodyIndex()) { //FR_footIndex
//              true_contact(0) = true;
//          }
//          if (FL_footIndex == contact.getlocalBodyIndex()) { //FL_footIndex
//              true_contact(1) = true;
//          }
//          if (RR_footIndex == contact.getlocalBodyIndex()) { //RR_footIndex
//              true_contact(2) = true;
//          }
//          if (RL_footIndex == contact.getlocalBodyIndex()) { //RL_footIndex
//              true_contact(3) = true;
//          }
//      }

//      float gait_hz_ = 0.8;
//      Eigen::VectorXd footContactPhase_;
//      footContactPhase_.setZero(4);
//
//      phase_ += simulation_dt_;
//      footContactPhase_(0) = sin(phase_ / gait_hz_ * 2 * 3.141592); // RR
//      footContactPhase_(1) = -footContactPhase_(0); // RL
//      footContactPhase_(2) = -footContactPhase_(0); // FR
//      footContactPhase_(3) = footContactPhase_(0); // FL
//
//
//      phaseSin_(0) = sin(phase_ / gait_hz_ * 2 * 3.141592); // for observation
//      phaseSin_(1) = cos(phase_ / gait_hz_ * 2 * 3.141592); // for observation


    for(int i=0; i< int(control_dt_ / simulation_dt_ + 1e-10); i++){
      if(server_) server_->lockVisualizationServerMutex();
      world_->integrate();
      if(server_) server_->unlockVisualizationServerMutex();
    }

    updateObservation();

    if (visualizable_) {
    //          visual_target2 ->setPosition(TEEpos_);
          visual_EEpos->setPosition(PEEpos_.e());
      }


   double distance = 0.6;
//   if (posError_.norm() > distance){ /// walking
//      // footContactDouble_ -> limit_foot_contact 에 있도록 (-0.3,3) -> Gait Enforcing (요 -0.3 이 벗어나도 되는 범위)
//      for(int i=0; i<4; i++) {
//          if ((bool)true_contact(i))
//          { footContactDouble_(i) = 1.0 * footContactPhase_(i);}
//          else { footContactDouble_(i) = -1.0 * footContactPhase_(i); }
//      }
//
//        // footClearance_ -> limit_foot_clearance 에 있도록 (-0.12,0.12) -> foot 드는 거 enforcing
////        double desiredFootZPosition = 0.15;
////        for (int i=0; i<4; i++){
////              if (footContactPhase_(i) < -0.6) /// during swing, 전체시간의 33 %
////              {footClearance_(i) = footToTerrain_.segment(i * 5, 5).minCoeff() - desiredFootZPosition;} // 대략, 0.17 sec, 0 보다 크거나 같으면 됨 (enforcing clearance)
////              else{ footClearance_(i) = 0.0; } // max reward (not enforcing clearance)
////        }
//  }
//  else { /// under standingMode_
//      /// standingMode_ 는 zero command 로 부터 유추 가능, command 는 obs 이기 때문에, robot 은 standingMode_인지 아닌지 충분히 알 수 있음
//      for (int i=0; i<4; i++){
//          footContactDouble_(i) = 1.0; // around max reward, where this value should go under (-0.3,3)
////              footClearance_(i) = 0.0; // max reward (not enforcing clearance)
//      }
//  }
  double inclination = 10.0;
  double rewardgap = 10.0;
  double rinclination = 1.0;
  double rrewardgap = 1.0;
  double Jointsm = rewardgap*(1/(1+std::exp(-inclination*(baseError_.norm()-distance))))+1;
  double Legsm = rewardgap*(1/(1+std::exp(inclination*(baseError_.norm()-distance))))+1;
  double Eerror = rrewardgap*(1/(1+std::exp(rinclination*(baseError_.norm()-distance))))+0.1;
  double baerror = rrewardgap*(1/(1+std::exp(-rinclination*(baseError_.norm()-distance))))+0.1;

    Eigen::VectorXd jointPosTemp(12), jointPosWeight(12);
    jointPosWeight << 1.0, 0.,0.,1.,0.,0.,1.,0.,0.,1.,0.,0.;
    jointPosTemp = gc_.segment(7,12) - gc_init_.segment(7,12);
    jointPosTemp = jointPosWeight.cwiseProduct(jointPosTemp.eval());

//    getLogBarReward();
    //      rewards_.record("footSlip", footSlip_.sum());
    rewards_.record("EEpos", Eerror*std::exp(-posError_.norm()));
    rewards_.record("basepos", baerror*std::exp(-baseError_.head(2).norm()));
//    rewards_.record("forwardVel", std::min(1.0, bodyLinearVel_[0]));
//    rewards_.record("Height", std::exp(-(gc_[2]-0.46)*(gc_[2]-0.46)));

    rewards_.record("Lsmoothness1",(Legsm * pTarget_.segment(7,12) - Legsm * prevTarget_.segment(7,12)).squaredNorm());
    rewards_.record("Jsmoothness1",(Jointsm * pTarget_.tail(6) - Jointsm *  prevTarget_.tail(6)).squaredNorm());
//    rewards_.record("smoothness2", ((pTarget_ - 2 * prevTarget_ + prevPrevTarget_).squaredNorm()));
    rewards_.record("jointPos", jointPosTemp.squaredNorm());
//    rewards_.record("pTarget", (pTarget_-actionMean_).squaredNorm());
    rewards_.record("torque", anymal_->getGeneralizedForce().squaredNorm());

    return rewards_.sum();
  }

  void updateObservation() {
    anymal_->getState(gc_, gv_);
    raisim::Vec<4> quat;
    raisim::Mat<3,3> rot;
    quat[0] = gc_[3]; quat[1] = gc_[4]; quat[2] = gc_[5]; quat[3] = gc_[6];
    raisim::quatToRotMat(quat, rot);
    bodyLinearVel_ = rot.e().transpose() * gv_.segment(0, 3);
    bodyAngularVel_ = rot.e().transpose() * gv_.segment(3, 3);
    bodyOri_ = rot.e()(8);



    auto EEFrameIndex_ = anymal_->getFrameIdxByName("kinova_joint_end_effector");
    anymal_->getFramePosition(EEFrameIndex_, PEEpos_);
    posError_ = TEEpos_-PEEpos_.e();
    baseError_ = TEEpos_- gc_.head(3);
    Eigen::Vector3d posError = rot.e().transpose() * (posError_);
    Eigen::Vector3d baseError = rot.e().transpose() * (baseError_);
//      raisim::Mat<3,3> PEErot_;
//      Eigen::Vector3d PEEori_;
//      anymal_->getFrameOrientation(EEFrameIndex_,PEErot_);
//      PEEori_ = PEErot_.e().col(2);



      obDouble_ << gc_[2], /// body height : 1
        rot.e().row(2).transpose(), /// body orientation : 3
        gc_.tail(18), /// joint angles : 12
        bodyLinearVel_, bodyAngularVel_, /// body linear&angular velocity : 6
        gv_.tail(18),
        posError,
        baseError.head(2);//rot.e().transpose()* PEEori_,phaseSin_(0),phaseSin_(1)
  }

  void observe(Eigen::Ref<EigenVec> ob) final {
    /// convert it to float
    ob = obDouble_.cast<float>();
  }

  bool isTerminalState(float& terminalReward) final {
    terminalReward = float(terminalRewardCoeff_);

    /// if the contact body is not feet
    for(auto& contact: anymal_->getContacts())
      if(footIndices_.find(contact.getlocalBodyIndex()) == footIndices_.end())
        return true;

    terminalReward = 0.f;

    return false;
  }

  void curriculumUpdate() { };

  void getLogBarReward(){
      /// compute barrier reward
      double barrierJointPos = 0.0, barrierBodyHeight = 0.0, barrierBaseMotion = 0.0, barrierJointVel = 0.0, barrierTargetVel = 0.0, barrierFootContact = 0.0, barrierFootClearance = 0.0;
      double tempReward = 0.0;

      Eigen::Vector2d limitFootContact_;
      limitFootContact_<< -0.3, 3;
//      /// Log Barrier - limit_joint_pos
//      for (int i = 0; i < 4; i++) {
//          for (int j = 0; j < 3; j++) {
//              int index_joint = i * 3 + j;
//              //         relaxedLogBarrier(0.09,limitJointPos_(index_leg,0),limitJointPos_(index_leg,1),gc_(7+index_leg),tempReward);
//              relaxedLogBarrier(0.08, limitJointPos_(index_joint, 0), limitJointPos_(index_joint, 1),
//                                gc_(7 + index_joint), tempReward);
//              barrierJointPos += tempReward;
//              //        std::cout << index_leg<<" th joint : " << tempReward << std::endl;
//          }
//      }
//      // Log Barrier - limit_body_height
//      for (int i = 0; i < 2; i++) {
//          relaxedLogBarrier(0.04, limitBodyHeight_(0), limitBodyHeight_(1), bodyFrameHeight_(i), tempReward);
//          barrierBodyHeight += tempReward;
//      }
//
//      // Log Barrier - limit_base_motion
//      relaxedLogBarrier(0.2, limitBaseMotion_(0, 0), limitBaseMotion_(0, 1), bodyLinearVel_(2), tempReward);
//      barrierBaseMotion += tempReward;
//
//      for (int i = 0; i < 2; i++) {
//          relaxedLogBarrier(0.3, limitBaseMotion_(1, 0), limitBaseMotion_(1, 1), bodyAngularVel_(i), tempReward);
//          barrierBaseMotion += tempReward;
//      }
//      // Log Barrier - limit_joint_vel
//      for (int i = 0; i < 12; i++) {
//          relaxedLogBarrier(2.0, limitJointVel_(0), limitJointVel_(1), gv_(6 + i), tempReward);
//          barrierJointVel += tempReward;
//      }
//      // Log Barrier - limit_target_vel
//      relaxedLogBarrier(0.2, limitTargetVel_(0), limitTargetVel_(1), bodyLinearVel_(0) - command_(0), tempReward);
//      barrierTargetVel += tempReward;
//      relaxedLogBarrier(0.2, limitTargetVel_(0), limitTargetVel_(1), bodyLinearVel_(1) - command_(1), tempReward);
//      barrierTargetVel += tempReward;
//      relaxedLogBarrier(0.2, limitTargetVel_(0), limitTargetVel_(1), bodyAngularVel_(2) - command_(2), tempReward);
//      barrierTargetVel += tempReward;

      // Log Barrier - limit_foot_contact
      for (int i = 0; i < 4; i++) {
          relaxedLogBarrier(0.1, limitFootContact_(0), limitFootContact_(1), footContactDouble_(i), tempReward);
          barrierFootContact += tempReward;
      }
//      // Log Barrier - limit_foot_clearance
//      for (int i = 0; i < 4; i++) {
//          relaxedLogBarrier(0.01, limitFootClearance_(0), limitFootClearance_(1), footClearance_(i), tempReward);
//          barrierFootClearance += tempReward;
//      }

//      if (barrierFootClearance < -40) {
////          std::cout << "barrierJointPos : " <<  barrierJointPos << std::endl;
////          std::cout << "barrierBodyHeight : " <<  barrierBodyHeight << std::endl;
////          std::cout << "barrierBaseMotion : " <<  barrierBaseMotion << std::endl;
////          std::cout << "barrierJointVel : " <<  barrierJointVel << std::endl;
////          std::cout << "barrierTargetVel : " <<  barrierTargetVel << std::endl;
////          std::cout << "barrierFootContact : " <<  barrierFootContact << std::endl;
//          std::cout << "barrierFootClearance : " <<   barrierFootClearance << std::endl;
////                std::cout << "foot clearance : " << footClearance_.transpose() << std::endl;
//      }

      double logClip = -500.0;
      barrierJointPos = fmax(barrierJointPos, logClip);           /// 여기 밖 부분은 gradient 안 받겠다 \
                                                                                   //      barrierBodyHeight = fmax(barrierBodyHeight,logClip);
//      barrierBaseMotion = fmax(barrierBaseMotion,logClip);
//      barrierJointVel = fmax(barrierJointVel,logClip);
//      barrierTargetVel = fmax(barrierTargetVel,logClip);
//      barrierFootContact = fmax(barrierFootContact,logClip);
//      barrierFootClearance = fmax(barrierFootClearance,logClip);
//      rewards_.record("barrierJointPos", barrierJointPos);
//      rewards_.record("barrierBodyHeight", barrierBodyHeight);
//      rewards_.record("barrierBaseMotion", barrierBaseMotion);
//      rewards_.record("barrierJointVel", barrierJointVel);
//      rewards_.record("barrierTargetVel", barrierTargetVel);
      rewards_.record("barrierFootContact", barrierFootContact);
//      rewards_.record("barrierFootClearance", barrierFootClearance);

//      float logBarReward =  (float)(1e-1*(barrierJointPos + barrierBodyHeight + barrierBaseMotion + barrierJointVel + barrierTargetVel + barrierFootContact + barrierFootClearance));
//      rewards_.record("relaxedLog", logBarReward); /// relaxed log barrier
//      return  logBarReward;
  }

  void relaxedLogBarrier(const double& delta,const double& alpha_lower,const double& alpha_upper,const double& x, double& y){
    /// positive reward, boundary 밖에서 gradient 가 큼
    double x_temp = x-alpha_lower;
    // lower bound
    if (x_temp < delta){
            y = 0.5*(pow((x_temp-2*delta)/delta,2)-1) - log(delta);
    }else{
            y = -log(x_temp);
    }
    // upper bound
        x_temp = -(x-alpha_upper);
        if (x_temp < delta){
            y += 0.5*(pow((x_temp-2*delta)/delta,2)-1) - log(delta);
        }else{
            y += -log(x_temp);
        }
        y *= -1;
    }

 private:
  int gcDim_, gvDim_, nJoints_;
  bool visualizable_ = false;
  raisim::ArticulatedSystem* anymal_;
  raisim::Visuals* visual_target;
  raisim::Visuals* visual_EEpos;
  size_t RR_footIndex,RL_footIndex,FR_footIndex,FL_footIndex;
  raisim::Visuals* visual_target2;
  Eigen::VectorXd gc_init_, gv_init_, gc_, gv_, pTarget_, pTarget12_,prevTarget_,prevPrevTarget_, vTarget_,true_contact,phaseSin_,footContactDouble_;
  double terminalRewardCoeff_ = -10.,phase_= 0,bodyOri_;
  raisim::Vec<3> PEEpos_;
  Eigen::VectorXd actionMean_, actionStd_, obDouble_;
  Eigen::Vector3d bodyLinearVel_, bodyAngularVel_, TEEpos_, posError_,baseError_;
  std::set<size_t> footIndices_;

  /// these variables are not in use. They are placed to show you how to create a random number sampler.
  thread_local static std::mt19937 gen_;
  thread_local static std::normal_distribution<double> normDist_;
  thread_local static std::uniform_real_distribution<double> uniDist_;
};
thread_local std::mt19937  raisim::ENVIRONMENT::gen_;
thread_local std::normal_distribution<double> raisim::ENVIRONMENT::normDist_(0., 1.);
thread_local std::uniform_real_distribution<double> raisim::ENVIRONMENT::uniDist_(-1., 1.);
}
