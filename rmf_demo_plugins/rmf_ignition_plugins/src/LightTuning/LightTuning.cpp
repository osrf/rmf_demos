/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <queue>
#include <string>
#include <iostream>

#include <ignition/plugin/Register.hh>
#include <ignition/gui/qt.h>
#include <ignition/gazebo/gui/GuiSystem.hh>
#include <ignition/gazebo/SdfEntityCreator.hh>
#include <ignition/gazebo/components/Light.hh>
#include <ignition/gazebo/components/Name.hh>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>

using namespace ignition;
//using namespace gui;

class LightTuning : public gazebo::GuiSystem
{
  Q_OBJECT

private:
  sdf::Light light;
  ignition::transport::Node _node;

  enum class Action {REMOVE, CREATE};
  std::unordered_map<std::string, std::queue<Action>> actions;

  void add_light();
  void remove_light();
  std::string lightToString(sdf::Light&);

public slots:
  void OnLightTypeSelect(const QString&);
  void OnShadowSelect(bool);
  void OnCreateLight(bool, const QString&,
    const QString&, const QString&, const QString&, const QString&,
    const QString&, const QString&, const QString&, const QString&, const QString&);

public:
  LightTuning();

  virtual void LoadConfig(const tinyxml2::XMLElement* _pluginElem)
  override;

  void Update(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm) override;
};

LightTuning::LightTuning()
{
  light.SetName("sun");
  light.SetType(sdf::LightType::DIRECTIONAL);
  light.SetCastShadows(true);
  light.SetRawPose(ignition::math::Pose3d(0, 0, 10, 0, 0, 0));
  light.SetDiffuse(ignition::math::Color(1, 1, 1, 1));
  light.SetSpecular(ignition::math::Color(0.2, 0.2, 0.2, 1));
  light.SetAttenuationRange(1000);
  light.SetConstantAttenuationFactor(0.09);
  light.SetQuadraticAttenuationFactor(0.001);
  light.SetDirection(ignition::math::Vector3d(-0.5, 0.1, -0.9));

  actions.insert({light.Name(), std::queue<Action>()});
}

void LightTuning::LoadConfig(const tinyxml2::XMLElement* _pluginElem)
{
  if (this->title.empty())
    this->title = "Light Tuning";
}

void LightTuning::Update(const ignition::gazebo::UpdateInfo &_info,
  ignition::gazebo::EntityComponentManager &_ecm)
{
  auto light_queue_it = actions.begin();
  while (light_queue_it != actions.end())
  {
    bool erase = false;
    if (light_queue_it->second.size()) // Pending actions to complete
    {
      if (light_queue_it->second.front() == Action::CREATE)
      {
        add_light();
        light_queue_it->second.pop();
      }
      else
      {
        remove_light();
        light_queue_it->second.pop();
        if (light_queue_it->second.empty())
        {
          erase = true;
        }
      }
    }

    if (erase)
    {
      light_queue_it = actions.erase(light_queue_it);
    }
    else
    {
      ++light_queue_it;
    }
  }
  return;
}

void addLightCb(const ignition::msgs::Boolean &_rep, const bool result)
{
  std::cout << "added light " << std::endl;
}

void removeLightCb(const ignition::msgs::Boolean &_rep, const bool result)
{
  std::cout << "removed light " << std::endl;
}


void LightTuning::add_light()
{
  ignition::msgs::EntityFactory create_light;
  create_light.set_sdf(lightToString(light));
  _node.Request("/world/sim_world/create", create_light, addLightCb);
}

void LightTuning::remove_light()
{
  ignition::msgs::Entity remove_light;
  remove_light.set_name(light.Name());
  remove_light.set_type(ignition::msgs::Entity_Type_LIGHT);
  _node.Request("/world/sim_world/remove", remove_light, removeLightCb);
}

std::string LightTuning::lightToString(sdf::Light& light)
{
  // to check if name is empty
  std::ostringstream ss;
  ss << "<sdf version=\"1.7\"> \n";
  ss << "<light type=\""
    << (light.Type() == sdf::LightType::POINT ? "point" : 
    (light.Type() == sdf::LightType::DIRECTIONAL ? "directional" : "spot"))
    << "\" name=\"" << light.Name() << "\"> \n";
  ss << "<cast_shadows>"
    << (light.CastShadows() ? "true" : "false") << "</cast_shadows> \n";
  ss << "<pose>" << light.RawPose() << "</pose>\n";
  ss << "<diffuse>" << light.Diffuse() << "</diffuse>\n";
  ss << "<specular>" << light.Specular() << "</specular>\n";
  ss << "<attenuation>\n";
  ss << "<range>" << light.AttenuationRange() << "</range>\n";
  ss << "<constant>" << light.ConstantAttenuationFactor() << "</constant>\n";
  ss << "<linear>" << light.LinearAttenuationFactor() << "</linear>\n";
  ss << "<quadratic>" << light.QuadraticAttenuationFactor() << "</quadratic>\n";
  ss << "</attenuation>\n";
  ss << "<direction>" << light.Direction() << "</direction>\n";
  ss << "</light>\n";
  ss << "</sdf>";
  std::cout << "Result: " << ss.str() << std::endl;
  return ss.str();
}

// to also pass light name
void LightTuning::OnLightTypeSelect(const QString& type)
{
  std::string std_type = type.toStdString();
  if (std_type == "Point")
  {
    light.SetType(sdf::LightType::POINT);
  }
  else if(std_type == "Directional")
  {
    light.SetType(sdf::LightType::DIRECTIONAL);
  }
  else
  {
    light.SetType(sdf::LightType::SPOT);
  }
}

void LightTuning::OnShadowSelect(bool cast_shadow)
{
  light.SetCastShadows(cast_shadow);
}

std::optional<ignition::math::Pose3d> parse_pose(const std::string& pose_str)
{
  std::stringstream ss(pose_str);
  double x, y, z, roll, pitch, yaw;
  ss >> x >> y >> z >> roll >> pitch >> yaw;
  if (!ss.fail())
  {
    return ignition::math::Pose3d(x, y, z, roll, pitch, yaw);
  }
  else
  {
    return std::nullopt;
  }
}

std::optional<ignition::math::Color> parse_color(const std::string& color_str)
{
  std::stringstream ss(color_str);
  float r, g, b, a;
  ss >> r >> g >> b >> a;
  if (!ss.fail())
  {
    return ignition::math::Color(r, g, b, a);
  }
  else
  {
    return std::nullopt;
  }
}

std::optional<double> parse_double(const std::string& double_str)
{
  std::stringstream ss(double_str);
  double d;
  ss >> d;
  if (!ss.fail())
  {
    return d;
  }
  else
  {
    return std::nullopt;
  }
}

std::optional<ignition::math::Vector3d> parse_vector(const std::string& vector_str)
{
  std::stringstream ss(vector_str);
  double x, y, z;
  ss >> x >> y >> z;
  if (!ss.fail())
  {
    return ignition::math::Vector3d(x, y, z);
  }
  else
  {
    return std::nullopt;
  }
}

void LightTuning::OnCreateLight(
  bool cast_shadow, const QString& type,
  const QString& name, const QString& pose_str,
  const QString& diffuse_str, const QString& specular_str,
  const QString& attentuation_range_str,
  const QString& attentuation_constant_str,
  const QString& attentuation_linear_str,
  const QString& attentuation_quadratic_str,
  const QString& direction_str)
{
  OnShadowSelect(cast_shadow);
  OnLightTypeSelect(type);
  light.SetName(name.toStdString());
  std::optional<ignition::math::Pose3d> pose = parse_pose(pose_str.toStdString());
  if (pose)
  {
    light.SetRawPose(*pose);
  }
  std::optional<ignition::math::Color> diffuse = parse_color(diffuse_str.toStdString());
  if (diffuse)
  {
    light.SetDiffuse(*diffuse);
  }
  std::optional<ignition::math::Color> specular = parse_color(specular_str.toStdString());
  if (specular)
  {
    light.SetSpecular(*specular);
  }
  std::optional<double> attentuation_range = parse_double(attentuation_range_str.toStdString());
  if (attentuation_range)
  {
    light.SetAttenuationRange(*attentuation_range);
  }
  std::optional<double> attentuation_constant = parse_double(attentuation_constant_str.toStdString());
  if (attentuation_constant)
  {
    light.SetConstantAttenuationFactor(*attentuation_constant);
  }
  std::optional<double> attentuation_linear = parse_double(attentuation_linear_str.toStdString());
  if (attentuation_range)
  {
    light.SetLinearAttenuationFactor(*attentuation_linear);
  }
  std::optional<double> attentuation_quadratic = parse_double(attentuation_quadratic_str.toStdString());
  if (attentuation_quadratic)
  {
    light.SetQuadraticAttenuationFactor(*attentuation_quadratic);
  }
  std::optional<ignition::math::Vector3d> direction = parse_vector(direction_str.toStdString());
  if (direction)
  {
    light.SetDirection(*direction);
  }

  auto light_queue_it = actions.find(light.Name());
  if (light_queue_it != actions.end())
  {
    if (!(light_queue_it->second.size()
      && light_queue_it->second.back() == Action::REMOVE))
    {
      light_queue_it->second.push(Action::REMOVE);
    }
  }
  else
  {
    light_queue_it = actions.insert({light.Name(), std::queue<Action>()}).first;
  }
  light_queue_it->second.push(Action::CREATE);
}

// Register this plugin
IGNITION_ADD_PLUGIN(LightTuning,
  ignition::gui::Plugin)


#include "LightTuning.moc"
