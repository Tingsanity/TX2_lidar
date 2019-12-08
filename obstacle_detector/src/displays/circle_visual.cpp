/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#include "obstacle_detector/displays/circle_visual.h"

namespace obstacles_display
{

CircleVisual::CircleVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node) {
  scene_manager_ = scene_manager;
  frame_node_1_ = parent_node->createChildSceneNode();
  frame_node_2_ = parent_node->createChildSceneNode();
  m_sceneNode = parent_node->createChildSceneNode();//chiu
  // m_text = new rviz::MovableText("fuck");//chiu
  // m_text->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_BELOW);//chiu
  // m_sceneNode->attachObject(m_text);//chiu

  obstacle_.reset(new rviz::Shape(rviz::Shape::Cylinder, scene_manager_, frame_node_1_));
  margin_.reset(new rviz::Shape(rviz::Shape::Cylinder, scene_manager_, frame_node_2_));
  // makeText();
  // setText(s);
  // text.reset(new rviz::MovableText("unknown"));
}

CircleVisual::~CircleVisual() {
  scene_manager_->destroySceneNode(frame_node_1_);
  scene_manager_->destroySceneNode(frame_node_2_);
  scene_manager_->destroySceneNode(m_sceneNode->getName());//chiu
}

void CircleVisual::setData(const obstacle_detector::CircleObstacle& circle) {
  Ogre::Vector3 pos(circle.center.x, circle.center.y, 0.25);
  obstacle_->setPosition(pos);

  Ogre::Vector3 true_scale(2.0 * circle.true_radius, 0.1, 2.0 * circle.true_radius);
  obstacle_->setScale(true_scale);

  Ogre::Vector3 pos2(circle.center.x, circle.center.y, 0.1);
  margin_->setPosition(pos2);
  m_sceneNode->setPosition(pos2);
  // setText(circle.id);
   
  Ogre::Vector3 scale(2.0 * circle.radius, 0.2, 2.0 * circle.radius);
  margin_->setScale(scale);

  // Ogre::Vector3 pos3(circle.center.x, circle.center.y, 0.1);
  // // text->setPosition(pos3)
  // text->setCaption("fuck");
  // text->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_BELOW);//chiu
  
  Ogre::Vector3 dir(Ogre::Real(1.0), Ogre::Real(0.0), Ogre::Real(0.0));
  Ogre::Radian angle(Ogre::Real(M_PI_2));
  Ogre::Quaternion q(angle, dir);
  obstacle_->setOrientation(q);
  margin_->setOrientation(q);
}
 
void CircleVisual::setFramePosition(const Ogre::Vector3& position) {
  frame_node_1_->setPosition(position);
  frame_node_2_->setPosition(position);
  // m_sceneNode->setPosition(position);
}

void CircleVisual::setFrameOrientation(const Ogre::Quaternion& orientation) {
  frame_node_1_->setOrientation(orientation);
  frame_node_2_->setOrientation(orientation);
  // m_sceneNode->setOrientation(orientation);
}

void CircleVisual::setMainColor(float r, float g, float b, float a) {
  obstacle_->setColor(r, g, b, a);
}

void CircleVisual::setMarginColor(float r, float g, float b, float a) {
  margin_->setColor(r, g, b, a);
}

void CircleVisual::setText(const std::string& caption){
  movable_text = new rviz::MovableText("TEXT");
  movable_text->setCaption(caption);
  movable_text->setCharacterHeight(1);
  movable_text->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_BELOW);//chiu
  m_sceneNode->attachObject(movable_text);
}

// -------------------------------------------
void CircleVisual::makeText(){
  movable_text_ = std::make_shared<rviz::MovableText>("w");
  movable_text_->setCharacterHeight(1);
  movable_text_->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_BELOW);//chiu
  m_sceneNode->attachObject(movable_text_.get());

}

// void CircleVisual::setCharacterHeight(double characterHeight){
//   m_text->setCharacterHeight(characterHeight);
//   m_text->setSpaceWidth(0.3 * characterHeight);
// }//chiu

// double CircleVisual::getCharacterHeight(){
//   return m_text->getCharacterHeight();
// }//chiu

// void CircleVisual::setCaption(const std::string& caption){
//   m_text->setCaption(caption);
// }//chiu

// void CircleVisual::setPosition(const Ogre::Vector3& position){
//   m_sceneNode->setPosition(position);
// }//chiu

// void CircleVisual::setColor(const Ogre::ColourValue& c){
//   m_text->setColor(c);
// }//chiu

// void CircleVisual::showOnTop(bool onTop = true){
//   m_text->showOnTop(onTop);
// }//chiu

} // end namespace obstacles_display

