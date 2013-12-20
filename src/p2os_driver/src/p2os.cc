/*
 *  P2OS for ROS
 *  Copyright (C) 2009
 *     David Feil-Seifer, Brian Gerkey, Kasper Stoy,
 *      Richard Vaughan, & Andrew Howard
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <ros/ros.h>
#include "../include/p2os.h"

#include <termios.h>
#include <fcntl.h>
#include <string.h>


P2OSNode::P2OSNode( ros::NodeHandle nh ) :
    n_(nh)/*, gripper_dirty_(false)*/,
    batt_pub_( n_.advertise<p2os_driver::BatteryState>("battery_state",1000),
               diagnostic_,
               diagnostic_updater::FrequencyStatusParam( &desired_freq_, &desired_freq_, 0.1),
               diagnostic_updater::TimeStampStatusParam() )/*,
    ptz_(this)*/
{
  // Use sonar
  ros::NodeHandle n_private("~");
//  n_private.param("use_sonar", use_sonar_, false);

  // read in config options
  // bumpstall
  //n_private.param( "bumpstall", bumpstall, -1 );
  // pulse
  n_private.param( "pulse", pulse_, -1.0 );
  // rot_kp
  n_private.param( "rot_kp", rot_kp_, -1 );
  // rot_kv
  n_private.param( "rot_kv", rot_kv_, -1 );
  // rot_ki
  n_private.param( "rot_ki", rot_ki_, -1 );
  // trans_kp
  n_private.param( "trans_kp", trans_kp_, -1 );
  // trans_kv
  n_private.param( "trans_kv", trans_kv_, -1 );
  // trans_ki
  n_private.param( "trans_ki", trans_ki_, -1 );
  // !!! port !!!
  std::string def = DEFAULT_P2OS_PORT;
  n_private.param( "port", psos_serial_port_, def );
  ROS_INFO( "using serial port: [%s]", psos_serial_port_.c_str() );
  n_private.param( "use_tcp", psos_use_tcp_, false );
  std::string host = DEFAULT_P2OS_TCP_REMOTE_HOST;
  n_private.param( "tcp_remote_host", psos_tcp_host_, host );
  n_private.param( "tcp_remote_port", psos_tcp_port_, DEFAULT_P2OS_TCP_REMOTE_PORT );
  // radio
  n_private.param( "radio", radio_modemp_, 0 );
  // joystick
  n_private.param( "joystick", joystick_, 0 );
  // direct_wheel_vel_control
  n_private.param( "direct_wheel_vel_control", direct_wheel_vel_control_, 0 );
  // max xpeed
  double spd;
  n_private.param( "max_xspeed", spd, MOTOR_DEF_MAX_SPEED);
  motor_max_speed_ = (int)rint(1e3*spd);
  // max_yawspeed
  n_private.param( "max_yawspeed", spd, MOTOR_DEF_MAX_TURNSPEED);
  motor_max_turnspeed_ = (short)rint(RTOD(spd));
  // max_xaccel
  n_private.param( "max_xaccel", spd, 0.0);
  motor_max_trans_accel = (short)rint(1e3*spd);
  // max_xdecel
  n_private.param( "max_xdecel", spd, 0.0);
  motor_max_trans_decel_ = (short)rint(1e3*spd);
  // max_yawaccel
  n_private.param( "max_yawaccel", spd, 0.0);
  motor_max_rot_accel = (short)rint(RTOD(spd));
  // max_yawdecel
  n_private.param( "max_yawdecel", spd, 0.0);
  motor_max_rot_decel_ = (short)rint(RTOD(spd));
  // odom_drift_rotational
  n_private.param( "odom_drift_rotational", odom_drift_rotational_, 0.0);

  desired_freq_ = 10;

  // advertise services
  pose_pub_ = n_.advertise<nav_msgs::Odometry>("pose",1000);
  mstate_pub_ = n_.advertise<p2os_driver::MotorState>("motor_state",1000);
  //grip_state_pub_ = n_.advertise<p2os_driver::GripperState>("gripper_state",1000);
  ptz_state_pub_ = n_.advertise<p2os_driver::PTZState>("ptz_state",1000);
  //sonar_pub_ = n_.advertise<p2os_driver::SonarArray>("sonar", 1000);
  aio_pub_ = n_.advertise<p2os_driver::AIO>("aio", 1000);
  dio_pub_ = n_.advertise<p2os_driver::DIO>("dio", 1000);

  // subscribe to services
  cmdvel_sub_ = n_.subscribe("cmd_vel", 1, &P2OSNode::cmdvel_cb, this);
  cmdmstate_sub_ = n_.subscribe("cmd_motor_state", 1, &P2OSNode::cmdmotor_state,
                               this);
  //gripper_sub_ = n_.subscribe("gripper_control", 1, &P2OSNode::gripperCallback,
  //                           this);
  //ptz_cmd_sub_ = n_.subscribe("ptz_control", 1, &P2OSPtz::callback, &ptz_);

  last_velocity_send_time_ = ros::WallTime::now();

	// add diagnostic functions
	diagnostic_.add("Motor Stall", this, &P2OSNode::check_stall );
	diagnostic_.add("Battery Voltage", this, &P2OSNode::check_voltage );

  // initialize robot parameters (player legacy)
  initialize_robot_params();

  //stuff Tim added
  send_packet_again_ = true;
}

P2OSNode::~P2OSNode()
{
}

void
P2OSNode::cmdmotor_state( const p2os_driver::MotorStateConstPtr &msg)
{
  motor_dirty_ = true;
  cmdmotor_state_ = *msg;
}

//this function sets the motor state (not it's power levels or anything)
void
P2OSNode::check_and_set_motor_state()
{
  if( !motor_dirty_ ) return;
  motor_dirty_ = false;

  unsigned char val = (unsigned char) cmdmotor_state_.state;
  unsigned char command[4];
  P2OSPacket packet;
  command[0] = ENABLE;
  command[1] = ARGINT;
  command[2] = val;
  command[3] = 0;
  packet.Build(command,4);

  // Store the current motor state so that we can set it back
  p2os_data.motors.state = cmdmotor_state_.state;
  SendReceive(&packet,false);
}

void
P2OSNode::cmdvel_cb( const geometry_msgs::TwistConstPtr &msg)
{
	ROS_DEBUG("Got velocity packet at %f",ros::Time::now().toSec());

  //checks if the change in motor velocity since the last used cmd_vel is greater than a threshold
  if( fabs( msg->linear.x - cmdvel_.linear.x ) > 0.01 || fabs( msg->angular.z-cmdvel_.angular.z) > 0.01 )
  {
    ROS_DEBUG( "new speed: [%0.2f,%0.2f](%0.3f)", msg->linear.x*1e3, msg->angular.z, ros::Time::now().toSec() );
    vel_dirty_ = true;
    cmdvel_ = *msg;
    cmdvel_.angular.z+=odom_drift_rotational_;
  }
  else if(ros::WallTime::now().toSec() - last_velocity_send_time_.toSec() > 0.5)
  {
    vel_dirty_ = true;
  }
  vel_dirty_ = true;


}

void
P2OSNode::check_and_set_vel()
{
  //a message must be send every 0.5 seconds, so check when the last message
  //was and send a pulse if necessary
  //it actually checks if the time is greater than 0.4, which should catch
  //the time within 0.5 seconds and send the pulse
  if(ros::WallTime::now().toSec() - last_velocity_send_time_.toSec() > 0.5) {
	  SendPulse();
	  //last_velocity_send_time_ = ros::WallTime::now();
  }
  if( !vel_dirty_ ) return;
  else ROS_INFO("velocity dirty");
  last_velocity_send_time_ = ros::WallTime::now();

  ROS_DEBUG( "setting vel: [%0.2f,%0.2f]",cmdvel_.linear.x,cmdvel_.angular.z);
  vel_dirty_ = false;

  unsigned short absSpeedDemand, absturnRateDemand;
  unsigned char motorcommand[4];
  P2OSPacket motorpacket;

  int vx = (int) (cmdvel_.linear.x*1e3);
  int va = (int)rint(RTOD(cmdvel_.angular.z));

  {
    // non-direct wheel control
    motorcommand[0] = VEL;
    if( vx >= 0 ) motorcommand[1] = ARGINT;
    else motorcommand[1] = ARGNINT;

    absSpeedDemand = (unsigned short)abs(vx);
    if( absSpeedDemand <= this->motor_max_speed_ )
    {
      motorcommand[2] = absSpeedDemand & 0x00FF;
      motorcommand[3] = (absSpeedDemand & 0xFF00) >> 8;
    }
    else
    {
      ROS_WARN( "speed demand thresholded! (true: %u, max: %u)", absSpeedDemand, motor_max_speed_ );
      motorcommand[2] = motor_max_speed_ & 0x00FF;
      motorcommand[3] = (motor_max_speed_ & 0xFF00) >> 8;
    }
    motorpacket.Build(motorcommand, 4);
    SendReceive(&motorpacket);
    ROS_INFO("sent velocity packet");

    motorcommand[0] = RVEL;
    if( va >= 0 ) motorcommand[1] = ARGINT;
    else motorcommand[1] = ARGNINT;

    absturnRateDemand = (unsigned short)abs(va);
    if( absturnRateDemand <= motor_max_turnspeed_ )
    {
      motorcommand[2] = absturnRateDemand & 0x00FF;
      motorcommand[3] = (absturnRateDemand & 0xFF00) >> 8;
    }
    else
    {
      ROS_WARN("Turn rate demand thresholded!");
      motorcommand[2] = this->motor_max_turnspeed_ & 0x00FF;
      motorcommand[3] = (this->motor_max_turnspeed_ & 0xFF00) >> 8;
    }

    motorpacket.Build(motorcommand,4);
    SendReceive(&motorpacket);
  }
}

int
P2OSNode::Setup()
{
  int i;
  int bauds[] = {B9600, B38400, B19200, B115200, B57600};
  int numbauds = sizeof(bauds);
  int currbaud = 0;
  sippacket_ = NULL;
  lastPulseTime_ = 0.0;

  struct termios term;
  unsigned char command;
  P2OSPacket packet, receivedpacket;
  int flags=0;
  bool sent_close = false;
  enum
  {
    NO_SYNC,
    AFTER_FIRST_SYNC,
    AFTER_SECOND_SYNC,
    READY
  } psos_state;

  psos_state = NO_SYNC;

  char name[20], type[20], subtype[20];
  int cnt;


  // use serial port

  ROS_INFO("P2OS connection opening serial port %s...",psos_serial_port_.c_str());

  //this attempts to open the serial port using open().
  //open returns an int which is the system's identifier for the serial port
  //and further calls can use write(that_int,...) to write to the serial port
  if((this->psos_fd_ = open(this->psos_serial_port_.c_str(),
                   O_RDWR | O_SYNC | O_NONBLOCK, S_IRUSR | S_IWUSR )) < 0 )
  {
    ROS_ERROR("P2OS::Setup():open(): failed to open serial port called %s did you chmod 777 %s ?",this->psos_serial_port_.c_str(),this->psos_serial_port_.c_str());
    return(1);
  }

  if(tcgetattr( this->psos_fd_, &term ) < 0 )
  {
    ROS_ERROR("P2OS::Setup():tcgetattr():");
    close(this->psos_fd_);
    this->psos_fd_ = -1;
    return(1);
  }

  cfmakeraw( &term );
  cfsetispeed(&term, bauds[currbaud]);
  cfsetospeed(&term, bauds[currbaud]);

  if(tcsetattr(this->psos_fd_, TCSAFLUSH, &term ) < 0)
  {
    ROS_ERROR("P2OS::Setup():tcsetattr():");
    close(this->psos_fd_);
    this->psos_fd_ = -1;
    return(1);
  }

  if(tcflush(this->psos_fd_, TCIOFLUSH ) < 0)
  {
    ROS_ERROR("P2OS::Setup():tcflush():");
    close(this->psos_fd_);
    this->psos_fd_ = -1;
    return(1);
  }

  if((flags = fcntl(this->psos_fd_, F_GETFL)) < 0)
  {
    ROS_ERROR("P2OS::Setup():fcntl()");
    close(this->psos_fd_);
    this->psos_fd_ = -1;
    return(1);
  }
  // Sync:

  int num_sync_attempts = 3;
  while(psos_state != READY)
  {
    switch(psos_state)
    {
      case NO_SYNC:
        command = SYNC0;
        packet.Build(&command, 1);
        packet.Send(this->psos_fd_);
        usleep(P2OS_CYCLETIME_USEC);
        break;
      case AFTER_FIRST_SYNC:
        ROS_INFO("turning off NONBLOCK mode...");
        if(fcntl(this->psos_fd_, F_SETFL, flags ^ O_NONBLOCK) < 0)
        {
          ROS_ERROR("P2OS::Setup():fcntl()");
          close(this->psos_fd_);
          this->psos_fd_ = -1;
          return(1);
        }
        command = SYNC1;
        packet.Build(&command, 1);
        packet.Send(this->psos_fd_);
        break;
      case AFTER_SECOND_SYNC:
        command = SYNC2;
        packet.Build(&command, 1);
        packet.Send(this->psos_fd_);
        break;
      default:
        ROS_WARN("P2OS::Setup():shouldn't be here...");
        break;
    }
    usleep(P2OS_CYCLETIME_USEC);

    if(receivedpacket.Receive(this->psos_fd_))
    {
      if((psos_state == NO_SYNC) && (num_sync_attempts >= 0))
      {
        num_sync_attempts--;
        usleep(P2OS_CYCLETIME_USEC);
        continue;
      }
      else
      {
        // couldn't connect; try different speed.
        if(++currbaud < numbauds)
        {
          cfsetispeed(&term, bauds[currbaud]);
          cfsetospeed(&term, bauds[currbaud]);
          if( tcsetattr(this->psos_fd_, TCSAFLUSH, &term ) < 0 )
          {
            ROS_ERROR("P2OS::Setup():tcsetattr():");
            close(this->psos_fd_);
            this->psos_fd_ = -1;
            return(1);
          }

          if(tcflush(this->psos_fd_, TCIOFLUSH ) < 0 )
          {
            ROS_ERROR("P2OS::Setup():tcflush():");
            close(this->psos_fd_);
            this->psos_fd_ = -1;
            return(1);
          }
          num_sync_attempts = 3;
          continue;
        }
        else
        {
          // tried all speeds; bail
          break;
        }
      }
    }
    switch(receivedpacket.packet[3])
    {
      case SYNC0:
        ROS_INFO( "SYNC0" );
        psos_state = AFTER_FIRST_SYNC;
        break;
      case SYNC1:
        ROS_INFO( "SYNC1" );
        psos_state = AFTER_SECOND_SYNC;
        break;
      case SYNC2:
        ROS_INFO( "SYNC2" );
        psos_state = READY;
        break;
      default:
        // maybe P2OS is still running from last time.  let's try to CLOSE
        // and reconnect
        if(!sent_close)
        {
          ROS_DEBUG("sending CLOSE");
          command = CLOSE;
          packet.Build( &command, 1);
          packet.Send(this->psos_fd_);
          sent_close = true;
          usleep(2*P2OS_CYCLETIME_USEC);
          tcflush(this->psos_fd_,TCIFLUSH);
          psos_state = NO_SYNC;
        }
        break;
    }
    usleep(P2OS_CYCLETIME_USEC);
  }
  if(psos_state != READY)
  {
    if(this->psos_use_tcp_)
    ROS_INFO("Couldn't synchronize with P2OS.\n"
           "  Most likely because the robot is not connected %s %s",
           this->psos_use_tcp_ ? "to the ethernet-serial bridge device " : "to the serial port",
           this->psos_use_tcp_ ? this->psos_tcp_host_.c_str() : this->psos_serial_port_.c_str());
    close(this->psos_fd_);
    this->psos_fd_ = -1;
    return(1);
  }
  cnt = 4;
  cnt += snprintf(name, sizeof(name), "%s", &receivedpacket.packet[cnt]);
  cnt++;
  cnt += snprintf(type, sizeof(type), "%s", &receivedpacket.packet[cnt]);
  cnt++;
  cnt += snprintf(subtype, sizeof(subtype), "%s", &receivedpacket.packet[cnt]);
  cnt++;

  std::string hwID = std::string( name ) + std::string(": ") + std::string(type) + std::string("/") + std::string( subtype );
  diagnostic_.setHardwareID(hwID);

  command = OPEN;
  packet.Build(&command, 1);
  packet.Send(this->psos_fd_);
  usleep(P2OS_CYCLETIME_USEC);
  command = PULSE;
  packet.Build(&command, 1);
  packet.Send(this->psos_fd_);
  usleep(P2OS_CYCLETIME_USEC);

  ROS_INFO("Done.\n   Connected to %s, a %s %s", name, type, subtype);

  // now, based on robot type, find the right set of parameters
  for(i=0;i<PLAYER_NUM_ROBOT_TYPES;i++)
  {
    if(!strcasecmp(PlayerRobotParams[i].Class.c_str(),type) &&
       !strcasecmp(PlayerRobotParams[i].Subclass.c_str(),subtype))
    {
      param_idx_ = i;
      break;
    }
  }
  if(i == PLAYER_NUM_ROBOT_TYPES)
  {
    ROS_WARN("P2OS: Warning: couldn't find parameters for this robot; "
            "using defaults");
    param_idx_ = 0;
  }

  //sleep(1);

  // first, receive a packet so we know we're connected.
  if(!sippacket_)
  {
    sippacket_ = new SIP(param_idx_);
  }
/*
  sippacket_->x_offset = 0;
  sippacket_->y_offset = 0;
  sippacket_->angle_offset = 0;

  SendReceive((P2OSPacket*)NULL,false);
*/
  // turn off the sonars at first
  this->ToggleSonarPower(0);
  // if requested, set max accel/decel limits
  P2OSPacket accel_packet;
  unsigned char accel_command[4];
  if(this->motor_max_trans_accel > 0)
  {
    accel_command[0] = SETA;
    accel_command[1] = ARGINT;
    accel_command[2] = this->motor_max_trans_accel & 0x00FF;
    accel_command[3] = (this->motor_max_trans_accel & 0xFF00) >> 8;
    accel_packet.Build(accel_command, 4);
    this->SendReceive(&accel_packet,false);
  }

  if(this->motor_max_trans_decel_ < 0)
  {
    accel_command[0] = SETA;
    accel_command[1] = ARGNINT;
    accel_command[2] = abs(this->motor_max_trans_decel_) & 0x00FF;
    accel_command[3] = (abs(this->motor_max_trans_decel_) & 0xFF00) >> 8;
    accel_packet.Build(accel_command, 4);
    this->SendReceive(&accel_packet,false);
  }
  if(this->motor_max_rot_accel > 0)
  {
    accel_command[0] = SETRA;
    accel_command[1] = ARGINT;
    accel_command[2] = this->motor_max_rot_accel & 0x00FF;
    accel_command[3] = (this->motor_max_rot_accel & 0xFF00) >> 8;
    accel_packet.Build(accel_command, 4);
    this->SendReceive(&accel_packet,false);
  }
  if(this->motor_max_rot_decel_ < 0)
  {
    accel_command[0] = SETRA;
    accel_command[1] = ARGNINT;
    accel_command[2] = abs(this->motor_max_rot_decel_) & 0x00FF;
    accel_command[3] = (abs(this->motor_max_rot_decel_) & 0xFF00) >> 8;
    accel_packet.Build(accel_command, 4);
    this->SendReceive(&accel_packet,false);
  }


  // if requested, change PID settings
  P2OSPacket pid_packet;
  unsigned char pid_command[4];
  if(this->rot_kp_ >= 0)
  {
    pid_command[0] = ROTKP;
    pid_command[1] = ARGINT;
    pid_command[2] = this->rot_kp_ & 0x00FF;
    pid_command[3] = (this->rot_kp_ & 0xFF00) >> 8;
    pid_packet.Build(pid_command, 4);
    this->SendReceive(&pid_packet);
  }
  if(this->rot_kv_ >= 0)
  {
    pid_command[0] = ROTKV;
    pid_command[1] = ARGINT;
    pid_command[2] = this->rot_kv_ & 0x00FF;
    pid_command[3] = (this->rot_kv_ & 0xFF00) >> 8;
    pid_packet.Build(pid_command, 4);
    this->SendReceive(&pid_packet);
  }
  if(this->rot_ki_ >= 0)
  {
    pid_command[0] = ROTKI;
    pid_command[1] = ARGINT;
    pid_command[2] = this->rot_ki_ & 0x00FF;
    pid_command[3] = (this->rot_ki_ & 0xFF00) >> 8;
    pid_packet.Build(pid_command, 4);
    this->SendReceive(&pid_packet);
  }
  if(this->trans_kp_ >= 0)
  {
    pid_command[0] = TRANSKP;
    pid_command[1] = ARGINT;
    pid_command[2] = this->trans_kp_ & 0x00FF;
    pid_command[3] = (this->trans_kp_ & 0xFF00) >> 8;
    pid_packet.Build(pid_command, 4);
    this->SendReceive(&pid_packet);
  }
  if(this->trans_kv_ >= 0)
  {
    pid_command[0] = TRANSKV;
    pid_command[1] = ARGINT;
    pid_command[2] = this->trans_kv_ & 0x00FF;
    pid_command[3] = (this->trans_kv_ & 0xFF00) >> 8;
    pid_packet.Build(pid_command, 4);
    this->SendReceive(&pid_packet);
  }
  if(this->trans_ki_ >= 0)
  {
    pid_command[0] = TRANSKI;
    pid_command[1] = ARGINT;
    pid_command[2] = this->trans_ki_ & 0x00FF;
    pid_command[3] = (this->trans_ki_ & 0xFF00) >> 8;
    pid_packet.Build(pid_command, 4);
    this->SendReceive(&pid_packet);
  }


  // if requested, change bumper-stall behavior
  // 0 = don't stall
  // 1 = stall on front bumper contact
  // 2 = stall on rear bumper contact
  // 3 = stall on either bumper contact
/*  if(this->bumpstall >= 0)
  {
    if(this->bumpstall > 3)
      ROS_INFO("ignoring bumpstall value %d; should be 0, 1, 2, or 3",
                    this->bumpstall);
    else
    {
      ROS_INFO("setting bumpstall to %d", this->bumpstall);
      P2OSPacket bumpstall_packet;;
      unsigned char bumpstall_command[4];
      bumpstall_command[0] = BUMP_STALL;
      bumpstall_command[1] = ARGINT;
      bumpstall_command[2] = (unsigned char)this->bumpstall;
      bumpstall_command[3] = 0;
      bumpstall_packet.Build(bumpstall_command, 4);
      this->SendReceive(&bumpstall_packet,false);
    }
  }
*/
  // Turn on the sonar
/*  if(use_sonar_) {
    this->ToggleSonarPower(1);
    ROS_DEBUG("Sonar array powered on.");
  }
  */
//  ptz_.setup();

  return(0);
}

int
P2OSNode::Shutdown()
{
  unsigned char command[20],buffer[20];
  P2OSPacket packet;

/*  if (ptz_.isOn())
  {
    ptz_.shutdown();
  } */

  memset(buffer,0,20);

  if(this->psos_fd_ == -1)
    return -1;

  command[0] = STOP;
  packet.Build(command, 1);
  packet.Send(this->psos_fd_);
  usleep(P2OS_CYCLETIME_USEC);

  command[0] = CLOSE;
  packet.Build(command, 1);
  packet.Send(this->psos_fd_);
  usleep(P2OS_CYCLETIME_USEC);

  close(this->psos_fd_);
  this->psos_fd_ = -1;
  ROS_INFO("P2OS has been shutdown");
  delete this->sippacket_;
  this->sippacket_ = NULL;

  return 0;
}

//this function publishes auxillary information about the pioneer:
//	pose
//	odom (tf)
//	battery level
//	motor state (running/not running)
//	aio??
//	dio??
void
P2OSNode::StandardSIPPutData(ros::Time ts)
{

  p2os_data.position.header.stamp = ts;
  pose_pub_.publish( p2os_data.position );
  p2os_data.odom_trans.header.stamp = ts;
  odom_broadcaster_.sendTransform( p2os_data.odom_trans );

  p2os_data.batt.header.stamp = ts;
  batt_pub_.publish( p2os_data.batt );
  mstate_pub_.publish( p2os_data.motors );

  // put sonar data
//  p2os_data.sonar.header.stamp = ts;
//  sonar_pub_.publish( p2os_data.sonar );

  // put aio data
  aio_pub_.publish( p2os_data.aio);
  // put dio data
  dio_pub_.publish( p2os_data.dio);

  // put gripper and lift data
//  grip_state_pub_.publish( p2os_data.gripper );
//  ptz_state_pub_.publish( ptz_.getCurrentState() );

  // put bumper data
  // put compass data

}

/* send the packet, then receive and parse an SIP */
int
P2OSNode::SendReceive(P2OSPacket* pkt, bool publish_data)
{
  P2OSPacket packet;

  if((this->psos_fd_ >= 0) && this->sippacket_)
  {
	//check if the packet was not NULL
    if(pkt) {
      //ROS_INFO("Sending packet from SendReceive");
      pkt->Send(this->psos_fd_);
      //ROS_INFO("Sent packet  from SendReceive");
    }

    /* receive a packet */
    pthread_testcancel();
    if(packet.Receive(this->psos_fd_))
    {
      ROS_ERROR("RunPsosThread(): Receive errored");
      pthread_exit(NULL);
    }
    //ROS_INFO("Received packet  from SendReceive");

    if(packet.packet[0] == 0xFA && packet.packet[1] == 0xFB &&
       (packet.packet[3] == 0x30 || packet.packet[3] == 0x31 ||
       packet.packet[3] == 0x32 || packet.packet[3] == 0x33 ||
       packet.packet[3] == 0x34))
    {

      // It is a server packet, so process it
      this->sippacket_->ParseStandard( &packet.packet[3] );
      this->sippacket_->FillStandard(&(this->p2os_data));

      if(publish_data)
        this->StandardSIPPutData(packet.timestamp);
    }
    else if(packet.packet[0] == 0xFA && packet.packet[1] == 0xFB &&
            packet.packet[3] == SERAUX)
    {

      // This is an AUX serial packet
 /*     if(ptz_.isOn())
      {
        int len = packet.packet[2] - 3;
        if (ptz_.cb_.gotPacket())
        {
          ROS_ERROR("PTZ got a message, but alread has the complete packet.");
        }
        else
        {
          for (int i=4; i < 4+len; ++i)
          {
            ptz_.cb_.putOnBuf(packet.packet[i]);
          }
        }
      }
      */
    }
    else
    {
      ROS_ERROR("Received other packet!");
      packet.PrintHex();
    }

  }
  return(0);
}

void
P2OSNode::updateDiagnostics()
{
  diagnostic_.update();
}

void
P2OSNode::check_voltage( diagnostic_updater::DiagnosticStatusWrapper &stat )
{
	double voltage = sippacket_->battery / 10.0;
	if( voltage < 11.0 )
	{
		stat.summary( diagnostic_msgs::DiagnosticStatus::ERROR, "battery voltage critically low" );
	}
	else if( voltage < 11.75 )
	{
		stat.summary( diagnostic_msgs::DiagnosticStatus::WARN, "battery voltage getting low" );

	}
	else stat.summary( diagnostic_msgs::DiagnosticStatus::OK, "battery voltage OK" );

	stat.add("voltage", voltage );
}

void
P2OSNode::check_stall( diagnostic_updater::DiagnosticStatusWrapper &stat )
{
	if( sippacket_->lwstall || sippacket_->rwstall )
	{
		stat.summary( diagnostic_msgs::DiagnosticStatus::ERROR, "wheel stalled" );
	}
	else stat.summary( diagnostic_msgs::DiagnosticStatus::OK, "no wheel stall" );

	stat.add("left wheel stall", sippacket_->lwstall );
	stat.add("right wheel stall", sippacket_->rwstall );
}

void
P2OSNode::ResetRawPositions()
{
  P2OSPacket pkt;
  unsigned char p2oscommand[4];

  if(this->sippacket_)
  {
    this->sippacket_->rawxpos = 0;
    this->sippacket_->rawypos = 0;
    this->sippacket_->xpos = 0;
    this->sippacket_->ypos = 0;
    p2oscommand[0] = SETO;
    p2oscommand[1] = ARGINT;
    pkt.Build(p2oscommand, 2);
    this->SendReceive(&pkt,false);
    ROS_INFO("resetting raw positions" );
  }
}

/* toggle sonars on/off, according to val */
void
P2OSNode::ToggleSonarPower(unsigned char val)
{
  unsigned char command[4];
  P2OSPacket packet;

  command[0] = SONAR;
  command[1] = ARGINT;
  command[2] = val;
  command[3] = 0;
  packet.Build(command, 4);
  SendReceive(&packet,false);
}

/* toggle motors on/off, according to val */
void
P2OSNode::ToggleMotorPower(unsigned char val)
{
  unsigned char command[4];
  P2OSPacket packet;
  ROS_INFO( "motor state: %d\n", p2os_data.motors.state );
  p2os_data.motors.state = (int) val;
  command[0] = ENABLE;
  command[1] = ARGINT;
  command[2] = val;
  command[3] = 0;
  packet.Build(command, 4);
  SendReceive(&packet,false);
}

/////////////////////////////////////////////////////
//  Actarray stuff
/////////////////////////////////////////////////////
/*
// Ticks to degrees from the ARIA software
inline double P2OSNode::TicksToDegrees (int joint, unsigned char ticks)
{
  if ((joint < 0) || (joint >= sippacket_->armNumJoints))
    return 0;

  double result;
  int pos = ticks - sippacket_->armJoints[joint].centre;
  result = 90.0 / static_cast<double> (sippacket_->armJoints[joint].ticksPer90);
  result = result * pos;
  if ((joint >= 0) && (joint <= 2))
    result = -result;

  return result;
}

// Degrees to ticks from the ARIA software
inline unsigned char P2OSNode::DegreesToTicks (int joint, double degrees)
{
  double val;

  if ((joint < 0) || (joint >= sippacket_->armNumJoints))
    return 0;

  val = static_cast<double> (sippacket_->armJoints[joint].ticksPer90) * degrees / 90.0;
  val = round (val);
  if ((joint >= 0) && (joint <= 2))
    val = -val;
  val += sippacket_->armJoints[joint].centre;

  if (val < sippacket_->armJoints[joint].min)
    return sippacket_->armJoints[joint].min;
  else if (val > sippacket_->armJoints[joint].max)
    return sippacket_->armJoints[joint].max;
  else
    return static_cast<int> (round (val));
}

inline double P2OSNode::TicksToRadians (int joint, unsigned char ticks)
{
  double result = DTOR (TicksToDegrees (joint, ticks));
  return result;
}

inline unsigned char P2OSNode::RadiansToTicks (int joint, double rads)
{
  unsigned char result = static_cast<unsigned char> (DegreesToTicks (joint, RTOD (rads)));
  return result;
}

inline double P2OSNode::RadsPerSectoSecsPerTick (int joint, double speed)
{
  double degs = RTOD (speed);
  double ticksPerDeg = static_cast<double> (sippacket_->armJoints[joint].ticksPer90) / 90.0f;
  double ticksPerSec = degs * ticksPerDeg;
  double secsPerTick = 1000.0f / ticksPerSec;

  if (secsPerTick > 127)
    return 127;
  else if (secsPerTick < 1)
    return 1;
  return secsPerTick;
}

inline double P2OSNode::SecsPerTicktoRadsPerSec (int joint, double msecs)
{
  double ticksPerSec = 1.0 / (static_cast<double> (msecs) / 1000.0);
  double ticksPerDeg = static_cast<double> (sippacket_->armJoints[joint].ticksPer90) / 90.0f;
  double degs = ticksPerSec / ticksPerDeg;
  double rads = DTOR (degs);

  return rads;
}
*/
void P2OSNode::SendPulse (void)
{
  //stop the motors first
  unsigned char motorcommand[4];
  P2OSPacket motorpacket;

  int vx = 0;
  int va = 0;

  {
    // non-direct wheel control
    motorcommand[0] = VEL;
    motorcommand[1] = ARGNINT;

      motorcommand[2] = 0 & 0x00FF;
      motorcommand[3] = (0 & 0xFF00) >> 8;

    motorpacket.Build(motorcommand, 4);
    SendReceive(&motorpacket);
    ROS_DEBUG("sent stop");

    motorcommand[0] = RVEL;
    motorcommand[1] = ARGNINT;

      motorcommand[2] = 0 & 0x00FF;
      motorcommand[3] = (0 & 0xFF00) >> 8;

    motorpacket.Build(motorcommand,4);
    SendReceive(&motorpacket);
  }
  unsigned char command;
  P2OSPacket packet;

  command = PULSE;
  packet.Build(&command, 1);
  //SendReceive(&packet);
}

