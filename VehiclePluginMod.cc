/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "VehiclePluginMod.hh"
#include <cstring>
//#include <osrf_msgs/JointCommands.h>


using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(VehiclePluginMod)
math::Vector3 _Global_msg_gas;
math::Vector3 _Global_msg_steer;

float steeringSpeed =0.1;
float DesiredAngleL =0;
float DesiredAngleR=0;
float IerL =0;
float IerR=0;
float control_P=1000;
float control_I=0.001;
float control_D=10;
double deltaSimTime=0, timePrevious=0;
#define pi 3.14159265359

/////////////////////////////////////////////////
VehiclePluginMod::VehiclePluginMod()
{
  this->joints.resize(4);

  this->aeroLoad = 0.1;
  this->swayForce = 1000;

  this->maxSpeed = 100;
  this->frontPower = 5000;
  this->rearPower = 5000;
  this->wheelRadius = 0.3;
  this->maxBrake = 0.0;
  this->maxGas = 0.0;
  this->steeringRatio = 1.0;
  this->tireAngleRange = 1.0;
}

/////////////////////////////////////////////////
void VehiclePluginMod::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;
  this->World = this->model->GetWorld();

  this->joints[0] = this->model->GetJoint(_sdf->Get<std::string>("front_left"));
  if (!this->joints[0])
  {
    gzerr << "Unable to find joint: front_left\n";
    return;
  }

  this->joints[1] = this->model->GetJoint(
      _sdf->Get<std::string>("front_right"));

  if (!this->joints[1])
  {
    gzerr << "Unable to find joint: front_right\n";
    return;
  }

  this->joints[2] = this->model->GetJoint(_sdf->Get<std::string>("back_left"));
  if (!this->joints[2])
  {
    gzerr << "Unable to find joint: back_left\n";
    return;
  }


  this->joints[3] = this->model->GetJoint(_sdf->Get<std::string>("back_right"));
  if (!this->joints[3])
  {
    gzerr << "Unable to find joint: back_right\n";
    return;
  }

  this->joints[0]->SetParam("suspension_erp", 0, 0.15);
  this->joints[0]->SetParam("suspension_cfm", 0, 0.04);

  this->joints[1]->SetParam("suspension_erp", 0, 0.15);
  this->joints[1]->SetParam("suspension_cfm", 0, 0.04);

  this->joints[2]->SetParam("suspension_erp", 0, 0.15);
  this->joints[2]->SetParam("suspension_cfm", 0, 0.04);

  this->joints[3]->SetParam("suspension_erp", 0, 0.15);
  this->joints[3]->SetParam("suspension_cfm", 0, 0.04);

  this->gasJoint = this->model->GetJoint(_sdf->Get<std::string>("gas"));
  this->brakeJoint = this->model->GetJoint(_sdf->Get<std::string>("brake"));
  this->steeringJoint = this->model->GetJoint(
      _sdf->Get<std::string>("steering"));

  if (!this->gasJoint)
  {
    gzerr << "Unable to find gas joint["
          << _sdf->Get<std::string>("gas") << "]\n";
    return;
  }

  if (!this->steeringJoint)
  {
    gzerr << "Unable to find steering joint["
          << _sdf->Get<std::string>("steering") << "]\n";
    return;
  }

  if (!this->joints[0])
  {
    gzerr << "Unable to find front_left joint["
          << _sdf->GetElement("front_left") << "]\n";
    return;
  }

  if (!this->joints[1])
  {
    gzerr << "Unable to find front_right joint["
          << _sdf->GetElement("front_right") << "]\n";
    return;
  }

  if (!this->joints[2])
  {
    gzerr << "Unable to find back_left joint["
          << _sdf->GetElement("back_left") << "]\n";
    return;
  }

  if (!this->joints[3])
  {
    gzerr << "Unable to find back_right joint["
          << _sdf->GetElement("back_right") << "]\n";
    return;
  }

  this->maxSpeed = _sdf->Get<double>("max_speed");
  this->aeroLoad = _sdf->Get<double>("aero_load");
  this->tireAngleRange = _sdf->Get<double>("tire_angle_range");
  this->frontPower = _sdf->Get<double>("front_power");
  this->rearPower = _sdf->Get<double>("rear_power");

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          boost::bind(&VehiclePluginMod::OnUpdate, this)));

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());


}

/////////////////////////////////////////////////
void VehiclePluginMod::Init()
{
  this->chassis = this->joints[0]->GetParent();

  // This assumes that the largest dimension of the wheel is the diameter
  physics::EntityPtr parent = boost::dynamic_pointer_cast<physics::Entity>(
      this->joints[0]->GetChild());
  math::Box bb = parent->GetBoundingBox();
  this->wheelRadius = bb.GetSize().GetMax() * 0.5;

  // The total range the steering wheel can rotate
  double steeringRange = this->steeringJoint->GetHighStop(0).Radian() -
                         this->steeringJoint->GetLowStop(0).Radian();

  // Compute the angle ratio between the steering wheel and the tires
  this->steeringRatio = steeringRange / this->tireAngleRange;

  // Maximum gas is the upper limit of the gas joint
  this->maxGas = this->gasJoint->GetHighStop(0).Radian();

  // Maximum brake is the upper limit of the gas joint
  this->maxBrake = this->gasJoint->GetHighStop(0).Radian();
  std::cout << this->model->GetName();

  printf("Version 1154 SssssssssteeringRation[%f] MaxGa[%f]\n", this->steeringRatio, this->maxGas);
     this->GasJointPoseSub = this->node->Subscribe(std::string("/gazebo/default/") +
      this->model->GetName() + "/gas_joint", &VehiclePluginMod::OnGaseJointPoseMsg, this);


    this->SteeringJointPoseSub = this->node->Subscribe(std::string("/gazebo/default/") +
      this->model->GetName() + "/steering_joint", &VehiclePluginMod::OnSteeringJointPoseMsg, this);
}

/////////////////////////////////////////////////
void VehiclePluginMod::OnUpdate()
{

  deltaSimTime =  this->World->GetSimTime().Double() - timePrevious;
  timePrevious = this->World->GetSimTime().Double();
  printf("Gas: %f and Steering: %f \n ",_Global_msg_gas.x, _Global_msg_steer.x);
  this->gasJoint->SetPosition(0,_Global_msg_gas.x);
  this->steeringJoint->SetPosition(0,_Global_msg_steer.x);

  // Get the normalized gas and brake amount
  double gas = this->gasJoint->GetAngle(0).Radian() / this->maxGas;
  double brake = this->brakeJoint->GetAngle(0).Radian() / this->maxBrake;

  // A little force to push back on the pedals
  this->gasJoint->SetForce(0, -0.1);
  this->brakeJoint->SetForce(0, -0.1);

  // Get the steering angle
  double steeringAngle = this->steeringJoint->GetAngle(0).Degree();

  // Compute the angle of the front wheels.
  double wheelAngle = steeringAngle / this->steeringRatio;

  // double idleSpeed = 0.5;

  // Compute the rotational velocity of the wheels
  double jointVel = (std::max(0.0, gas-brake) * this->maxSpeed) /
                    this->wheelRadius;

   // Compute the angle of the front wheels both left and right separately
  double wheelAngleInner =0, wheelAngleInnerDegree=0;
  double wheelAngleOuter =0, wheelAngleOuterDegree=0;
  double WheelBase =1.6;
  double kingpinDistance =0.6;
  double temp = tan(steeringAngle*pi/180);
  wheelAngleInner =WheelBase/((WheelBase/temp)- kingpinDistance);
  wheelAngleOuter =WheelBase/((WheelBase/temp) + kingpinDistance);
  //printf("PRE CONVERSION wheelAngleInner: %f,wheelAngleOuter: %f \n",wheelAngleInner,wheelAngleOuter);
  wheelAngleInnerDegree= (180/pi)*wheelAngleInner;
  wheelAngleOuterDegree= (180/pi)*wheelAngleOuter;
  printf("Steering Angle %f wheelAngleInner: %f,wheelAngleOuter: %f \n",steeringAngle, wheelAngleInnerDegree,wheelAngleOuterDegree);

  if (steeringAngle >0)
  {

	 printf("ActualLeftAngle: %f,DesiredLeftAngle: %f \n", this->joints[0]->GetAngle(0).Radian(),wheelAngleOuter);
     if (fabs(wheelAngleOuter - this->joints[0]->GetAngle(0).Radian())>0.05 ) {steer_controllerL(this->joints[0], wheelAngleOuter);printf("left steering ...\n");}
	 

    printf("ActualRightAngle: %f,DesiredRightAngle: %f \n", this->joints[1]->GetAngle(0).Radian(),wheelAngleInner);
	if (fabs(wheelAngleInner - this->joints[1]->GetAngle(0).Radian())>0.05 ) steer_controllerR(this->joints[1], wheelAngleInner);
}

else 

{
	 printf("ActualLeftAngle: %f,DesiredLeftAngle: %f \n", this->joints[0]->GetAngle(0).Radian(),wheelAngleInner);
     if (fabs(wheelAngleInner - this->joints[0]->GetAngle(0).Radian())>0.05 ) {steer_controllerL(this->joints[0], wheelAngleInner);printf("left steering ...\n");}
     
        printf("ActualRightAngle: %f,DesiredRightAngle: %f \n", this->joints[1]->GetAngle(0).Radian(),wheelAngleOuter);
	if (fabs(wheelAngleOuter - this->joints[1]->GetAngle(0).Radian())>0.05 ) steer_controllerR(this->joints[1], wheelAngleOuter); 
}



  this->joints[0]->SetVelocityLimit(1, -jointVel);
  this->joints[0]->SetForce(1,- (gas + brake) * this->frontPower);

  
  this->joints[1]->SetVelocityLimit(1, -jointVel);
  this->joints[1]->SetForce(1, -(gas + brake) * this->frontPower);
 //this->joints[1]->SetForce(0, -10);

  this->joints[2]->SetVelocityLimit(1, -jointVel);
  this->joints[2]->SetForce(1, -(gas + brake) * this->rearPower);

  this->joints[3]->SetVelocityLimit(1, -jointVel);
  this->joints[3]->SetForce(1, -(gas + brake) * this->rearPower);




}

void VehiclePluginMod:: steer_controllerL(physics::JointPtr steer_joint, double Angle)
  {


    double currentWheelAngle = this->joints[0]->GetAngle(0).Radian();
    double steeringOmega = this->joints[0]->GetVelocity(0);
    if (steer_joint == this->joints[0] && (fabs(Angle - currentWheelAngle)>0.05)) //controlling left wheel
    {
      DesiredAngleL = DesiredAngleL + steeringSpeed * deltaSimTime * (Angle - DesiredAngleL);
      if (fabs(Angle - DesiredAngleL)<0.05)
      {DesiredAngleL=Angle;
	   //IerL =0;
	  }
      IerL+=DesiredAngleL - currentWheelAngle;
      printf( " Pcomp = %f,Icomp =%f, DComp = %f",control_P * (DesiredAngleL - currentWheelAngle),control_I*IerL, control_D * (steeringOmega));
      double jointforce = -(control_P * (DesiredAngleL - currentWheelAngle)+control_I*IerL + control_D * (steeringOmega));
      if (jointforce>1000 && jointforce>0)jointforce =1000;
      if (jointforce<-1000 && jointforce<0)jointforce =-1000;
      printf(" Left joint force %f",jointforce); 
      if (fabs(Angle - currentWheelAngle)>0.05 ) {this->joints[0]->SetForce(0, -jointforce);printf("left applying");}

    }
    
}

void VehiclePluginMod:: steer_controllerR(physics::JointPtr steer_joint, double Angle)
   {     
	   
	double currentWheelAngle = this->joints[1]->GetAngle(0).Radian();
    double steeringOmega = this->joints[1]->GetVelocity(0);
	   
	if (steer_joint == this->joints[1] && (fabs(Angle - currentWheelAngle)>0.05)) //controlling right wheel
    {
      DesiredAngleR = DesiredAngleR + steeringSpeed * deltaSimTime * (Angle - DesiredAngleR);
      if (fabs(Angle - DesiredAngleR)<0.05)
      {DesiredAngleR=Angle;
	  // IerR =0;  
	  }
      IerR+=DesiredAngleR - currentWheelAngle;
      double jointforce =( control_P * (DesiredAngleR - currentWheelAngle)+control_I*IerR - control_D * (steeringOmega));
      if (jointforce>1000 && jointforce>0)jointforce =1000;
      if (jointforce<-1000 && jointforce<0)jointforce =-1000;
      printf("Right joint force %f",jointforce); 
       if (fabs(Angle - currentWheelAngle)>0.05 ){this->joints[1]->SetForce(0, jointforce);printf("Right applying");}
    }

  }

/////////////////////////////////////////////////
void VehiclePluginMod::OnGaseJointPoseMsg(ConstVector3dPtr &_msg)
{	_Global_msg_gas.Set (_msg->x(),0,0);
	printf("Gas %f",_msg->x());
	this->gasJoint->SetPosition(0,_msg->x());
}

void VehiclePluginMod::OnSteeringJointPoseMsg(ConstVector3dPtr &_msg)
{	_Global_msg_steer.Set (_msg->x(),0,0);
	this->steeringJoint->SetPosition(0,_msg->x());
}
