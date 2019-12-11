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
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

using namespace std;

namespace gazebo
{
  class AnimatedBox : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&AnimatedBox::OnUpdate, this, _1));
    }

         // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      // Apply a small linear velocity to the model.
      this->model->SetLinearVel(ignition::math::Vector3<double /* var type */ >(-0.5f, 0.0f, 0.0f));
      this->model->SetAngularVel(ignition::math::Vector3<double /* var type */ >(0.0f, 0.0f, 0.0f));
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(AnimatedBox)
}

// std::vector<std::vector<double>> getCSV(string name){
//     ifstream in(name);
//     vector<vector<double>> fields;
//     if (in) {
//         string line;
//         while (getline(in, line)) {
//             stringstream sep(line);
//             string field;
//             fields.push_back(vector<double>());
//             while (getline(sep, field, ',')) {
//                 fields.back().push_back(stod(field));
//             }
//         }
//         std::cout << "ERROR - did not find file:" << name << ".\n";
//         return fields;

//     }
//     std::vector<std::vector<double>> fields_out;
//     return fields_out;
// }
