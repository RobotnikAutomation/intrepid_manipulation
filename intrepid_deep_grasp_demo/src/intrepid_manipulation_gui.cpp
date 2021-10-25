/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, PickNik Consulting
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Consulting nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Rviz display panel for controlling and debugging ROS applications
*/

#include <cstdio>

#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QSpinBox>
#include <QTimer>
#include <QVBoxLayout>

#include "intrepid_deep_grasp_demo/intrepid_manipulation_gui.h"

namespace intrepid_dashboard
{
IntrepidManipulationGui::IntrepidManipulationGui(QWidget* parent) : rviz::Panel(parent)
{
  // Create start action push button
  btn_start_ = new QPushButton(this);
  btn_start_->setText("Start Detection");
  connect(btn_start_, SIGNAL(clicked()), this, SLOT(startAction()));
  QPalette pal_start = btn_start_->palette();
  pal_start.setColor(QPalette::Button, QColor(Qt::green));
  btn_start_->setAutoFillBackground(true);
  btn_start_->setPalette(pal_start);
  btn_start_->update();

/*   // Create cancel action push button
  btn_cancel_ = new QPushButton(this);
  btn_cancel_->setText("Cancel action");
  connect(btn_cancel_, SIGNAL(clicked()), this, SLOT(cancelAction()));  
  QPalette pal_cancel = btn_cancel_->palette();
  pal_cancel.setColor(QPalette::Button, QColor(Qt::yellow));
  btn_cancel_->setAutoFillBackground(true);
  btn_cancel_->setPalette(pal_cancel);
  btn_cancel_->update();
 */
  // Create display traj push button
  btn_display_ = new QPushButton(this);
  btn_display_->setText("Display Plan");
  connect(btn_display_, SIGNAL(clicked()), this, SLOT(displayTrajectory()));

  // Create execute traj push button
  btn_execute_ = new QPushButton(this);
  btn_execute_->setText("Execute Trajectory");
  connect(btn_execute_, SIGNAL(clicked()), this, SLOT(executeTrajectory()));  

  // Create stop motion push button
  btn_stop_ = new QPushButton(this);
  btn_stop_->setText("Stop Motion");
  connect(btn_stop_, SIGNAL(clicked()), this, SLOT(stopTrajectory()));  
  QPalette pal_stop = btn_stop_->palette();
  pal_stop.setColor(QPalette::Button, QColor(Qt::red));
  btn_stop_->setAutoFillBackground(true);
  btn_stop_->setPalette(pal_stop);
  btn_stop_->update();

  QGridLayout *gridLayout = new QGridLayout;

  // 0th row
  gridLayout->addWidget(btn_start_,0,0,1,1);
  gridLayout->addWidget(btn_display_,0,1,1,1);
  gridLayout->addWidget(btn_execute_,0,2,1,1);
  gridLayout->addWidget(btn_stop_,0,3,1,1);
/* 
  // 1rst row
  gridLayout->addWidget(btn_cancel_,1,0,1,1);
 */
  // Pick action box
  QGroupBox *pickActionBox;
  pickActionBox = new QGroupBox(tr("Deep learning pick object action"));  
  pickActionBox->setLayout(gridLayout);

  // Main layout
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(pickActionBox);
  setLayout(mainLayout);



  joy_publisher_ = nh_.advertise<sensor_msgs::Joy>("/intrepid_rviz_dashboard", 1);
  action_status_sub_ = nh_.subscribe("/pickup_action_status",10, &IntrepidManipulationGui::pickup_action_status_callback, this);
}

void IntrepidManipulationGui::pickup_action_status_callback(const std_msgs::Bool& ready_)
{
  btn_start_->setEnabled(ready_.data);
}

void IntrepidManipulationGui::startAction()
{
  ROS_INFO_STREAM_NAMED("intrepid_manipulation_dashboard", "Play trajectory");

  sensor_msgs::Joy msg;
  msg.buttons.resize(5);
  msg.buttons[1] = 1;
  joy_publisher_.publish(msg);
}

void IntrepidManipulationGui::cancelAction()
{
/*   ROS_INFO_STREAM_NAMED("intrepid_manipulation_dashboard", "Play trajectory");

  sensor_msgs::Joy msg;
  msg.buttons.resize(5);
  msg.buttons[1] = 1;
  joy_publisher_.publish(msg); */
}

void IntrepidManipulationGui::displayTrajectory()
{
  ROS_INFO_STREAM_NAMED("intrepid_manipulation_dashboard", "Play trajectory");

  sensor_msgs::Joy msg;
  msg.buttons.resize(5);
  msg.buttons[2] = 1;
  joy_publisher_.publish(msg);
}

void IntrepidManipulationGui::executeTrajectory()
{
  ROS_INFO_STREAM_NAMED("intrepid_manipulation_dashboard", "Play trajectory");

  sensor_msgs::Joy msg;
  msg.buttons.resize(5);
  msg.buttons[3] = 1;
  joy_publisher_.publish(msg);
}

void IntrepidManipulationGui::stopTrajectory()
{
  ROS_INFO_STREAM_NAMED("intrepid_manipulation_dashboard", "Play trajectory");

  sensor_msgs::Joy msg;
  msg.buttons.resize(5);
  msg.buttons[4] = 1;
  joy_publisher_.publish(msg);
}

void IntrepidManipulationGui::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

void IntrepidManipulationGui::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}
}  // end namespace rviz_visual_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(intrepid_dashboard::IntrepidManipulationGui, rviz::Panel)
