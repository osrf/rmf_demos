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

// Helper functions to parse various inputs from the GUI
std::optional<sdf::LightType> parse_light_type(const std::string& type)
{
  if (type == "Point")
  {
    return sdf::LightType::POINT;
  }
  else if (type == "Directional")
  {
    return sdf::LightType::DIRECTIONAL;
  }
  else if (type == "Spot")
  {
    return sdf::LightType::SPOT;
  }
  ignwarn << "Unable to parse \"" << type <<
    "\" as a light type. Using previous value." << std::endl;
  return std::nullopt;
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
    ignwarn << "Unable to parse \"" << pose_str <<
      "\" as a pose. Using previous value." << std::endl;
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
    ignwarn << "Unable to parse \"" << color_str <<
      "\" as a color. Using previous value." << std::endl;
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
    ignwarn << "Unable to parse \"" << double_str <<
      "\" as a double. Using previous value." << std::endl;
    return std::nullopt;
  }
}

std::optional<ignition::math::Vector3d> parse_vector(
  const std::string& vector_str)
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
    ignwarn << "Unable to parse \"" << vector_str <<
      "\" as a vector. Using previous value." << std::endl;
    return std::nullopt;
  }
}

class LightsModel : public QAbstractListModel
{
  Q_OBJECT
  Q_ENUMS(Roles)

public:
  enum Roles
  {
    NameRole = Qt::UserRole + 1,
    PoseRole,
    IndexRole,
    DiffuseRole,
    SpecularRole,
    AttenuationRangeRole,
    AttenuationConstantRole,
    AttenuationLinearRole,
    AttenuationQuadraticRole,
    DirectionRole
  };

  using QAbstractListModel::QAbstractListModel;

  QHash<int, QByteArray> roleNames() const override;
  int rowCount(const QModelIndex& parent = QModelIndex()) const override;
  QVariant data(const QModelIndex& index,
    int role = Qt::DisplayRole) const override;
  // Note: We do not need to override and define the setData() function since
  // we only update our model in `OnCreateLight`

  void add_new_light(const QString& name_qstr);
  void remove_light(int idx);

  sdf::Light& get_light(int idx);
  sdf::Light& get_light(const std::string& name);
  const QVector<sdf::Light>& get_lights() const;

private:
  QVector<sdf::Light> _lights;
};

QHash<int, QByteArray> LightsModel::roleNames() const
{
  return {{ NameRole, "name"},
    { PoseRole, "pose"},
    { IndexRole, "idx"},
    { DiffuseRole, "diffuse"},
    { SpecularRole, "specular"},
    { AttenuationRangeRole, "attenuation_range"},
    { AttenuationConstantRole, "attenuation_constant"},
    { AttenuationLinearRole, "attenuation_linear"},
    { AttenuationQuadraticRole, "attenuation_quadratic"},
    { DirectionRole, "direction"}};
}

int LightsModel::rowCount(const QModelIndex& parent) const
{
  if (parent.isValid())
    return 0;
  return _lights.size();
}

QVariant LightsModel::data(const QModelIndex& index, int role) const
{
  if (!hasIndex(index.row(), index.column(), index.parent()))
    return {};

  const sdf::Light& light = _lights.at(index.row());

  switch (role)
  {
    case NameRole:
    {
      return QString(light.Name().c_str());
    }
    case PoseRole:
    {
      std::ostringstream ss;
      ss << light.RawPose();
      return QString(ss.str().c_str());
    }
    case IndexRole:
    {
      return index.row();
    }
    case DiffuseRole:
    {
      std::ostringstream ss;
      ss << light.Diffuse();
      return QString(ss.str().c_str());
    }
    case SpecularRole:
    {
      std::ostringstream ss;
      ss << light.Specular();
      return QString(ss.str().c_str());
    }
    case AttenuationRangeRole:
    {
      return light.AttenuationRange();
    }
    case AttenuationConstantRole:
    {
      return light.ConstantAttenuationFactor();
    }
    case AttenuationLinearRole:
    {
      return light.LinearAttenuationFactor();
    }
    case AttenuationQuadraticRole:
    {
      return light.QuadraticAttenuationFactor();
    }
    case DirectionRole:
    {
      std::ostringstream ss;
      ss << light.Direction();
      return QString(ss.str().c_str());
    }
    default:
      break;
  }
  return {};
}

void LightsModel::add_new_light(const QString& name_qstr)
{
  std::string name = name_qstr.toStdString();
  auto existing_it = std::find_if(
    _lights.begin(), _lights.end(), [&name](const sdf::Light& light)
    {
      return light.Name() == name;
    });
  if (existing_it != _lights.end() || name.size() < 1)
  {
    ignerr << "Light names must be unique and at least 1 character long." <<
      std::endl;
    return;
  }

  beginInsertRows(QModelIndex(), _lights.size(), _lights.size());
  sdf::Light light;
  light.SetName(name);
  _lights.push_back(light);
  endInsertRows();
}

void LightsModel::remove_light(int idx)
{
  if (idx >= _lights.size())
  {
    ignerr << "Light to remove does not exist." << std::endl;
    return;
  }

  beginRemoveRows(QModelIndex(), idx, idx);
  _lights.erase(_lights.begin() + idx);
  endRemoveRows();
}

sdf::Light& LightsModel::get_light(int idx)
{
  return _lights[idx];
}

sdf::Light& LightsModel::get_light(const std::string& name)
{
  auto it = std::find_if(
    _lights.begin(), _lights.end(), [&name](const sdf::Light& light)
    {
      return light.Name() == name;
    });
  return *it;
}

const QVector<sdf::Light>& LightsModel::get_lights() const
{
  return _lights;
}

class LightTuning : public gazebo::GuiSystem
{
  Q_OBJECT

public:
  virtual void LoadConfig(const tinyxml2::XMLElement* _pluginElem)
  override;

  void Update(const ignition::gazebo::UpdateInfo& _info,
    ignition::gazebo::EntityComponentManager& _ecm) override;

public slots:
  void OnLightTypeSelect(sdf::Light& light, const QString& type);
  void OnCreateLightBtnPress(
    int idx, bool cast_shadow, const QString& type,
    const QString& name, const QString& pose_str,
    const QString& diffuse_str, const QString& specular_str,
    const QString& attentuation_range_str,
    const QString& attentuation_constant_str,
    const QString& attentuation_linear_str,
    const QString& attentuation_quadratic_str,
    const QString& direction_str);
  void OnRemoveLightBtnPress(int idx, const QString& name);
  void OnAddLightFormBtnPress(const QString& name);
  void OnSaveLightsBtnPress(const QString& url, bool save_all = true,
    int idx = -1);

private:
  ignition::transport::Node _node;
  LightsModel _model;

  enum class Action {REMOVE, CREATE};
  std::unordered_map<std::string, std::queue<Action>> actions;

  void create_light_service(const std::string& name);
  void remove_light_service(const std::string& name);
  std::string light_to_sdf_string(const sdf::Light&);
};

void LightTuning::LoadConfig(const tinyxml2::XMLElement*)
{
  if (this->title.empty())
    this->title = "Light Tuning";

  // Connect model to view
  this->Context()->setContextProperty(
    "LightsModel", &this->_model);
}

void LightTuning::Update(const ignition::gazebo::UpdateInfo&,
  ignition::gazebo::EntityComponentManager&)
{
  auto light_queue_it = actions.begin();
  while (light_queue_it != actions.end())
  {
    bool erase = false;
    if (light_queue_it->second.size()) // Pending actions to complete
    {
      if (light_queue_it->second.front() == Action::CREATE)
      {
        create_light_service(light_queue_it->first);
        light_queue_it->second.pop();
      }
      else
      {
        remove_light_service(light_queue_it->first);
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

// Necesary to supply callbacks to the service requests in order for the
// requests to execute properly, though they are not used for anything here.
void create_light_service_cb(const ignition::msgs::Boolean&, const bool)
{
}

void remove_light_service_cb(const ignition::msgs::Boolean&, const bool)
{
}

void LightTuning::create_light_service(const std::string& name)
{
  ignition::msgs::EntityFactory create_light;
  const sdf::Light& light = _model.get_light(name);
  create_light.set_sdf(light_to_sdf_string(light));
  _node.Request("/world/sim_world/create", create_light,
    create_light_service_cb);
}

void LightTuning::remove_light_service(const std::string& name)
{
  ignition::msgs::Entity remove_light;
  remove_light.set_name(name);
  remove_light.set_type(ignition::msgs::Entity_Type_LIGHT);
  _node.Request("/world/sim_world/remove", remove_light,
    remove_light_service_cb);
}

std::string LightTuning::light_to_sdf_string(const sdf::Light& light)
{
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
  //std::cout << "Result: " << ss.str() << std::endl;
  return ss.str();
}

// Helper template function to parse GUI input and update the sdf Light element
template<typename T, typename F>
void update_light(std::optional<T>(*parse_fn)(const std::string&),
  F set_fn, sdf::Light& light, const QString& val_str)
{
  std::optional<T> val = parse_fn(val_str.toStdString());
  if (val)
  {
    (light.*set_fn)(*val);
  }
}

void LightTuning::OnCreateLightBtnPress(
  int idx, bool cast_shadow, const QString& type_str,
  const QString& name, const QString& pose_str,
  const QString& diffuse_str, const QString& specular_str,
  const QString& attenuation_range_str,
  const QString& attenuation_constant_str,
  const QString& attenuation_linear_str,
  const QString& attenuation_quadratic_str,
  const QString& direction_str)
{
  sdf::Light& light = _model.get_light(idx);

  light.SetName(name.toStdString());
  light.SetCastShadows(cast_shadow);
  update_light(&parse_light_type, &sdf::Light::SetType, light, type_str);
  update_light(&parse_pose, &sdf::Light::SetRawPose, light, pose_str);
  update_light(&parse_color, &sdf::Light::SetDiffuse, light, diffuse_str);
  update_light(&parse_color, &sdf::Light::SetSpecular, light, specular_str);
  update_light(&parse_double, &sdf::Light::SetAttenuationRange,
    light, attenuation_range_str);
  update_light(&parse_double, &sdf::Light::SetConstantAttenuationFactor,
    light, attenuation_constant_str);
  update_light(&parse_double, &sdf::Light::SetLinearAttenuationFactor,
    light, attenuation_linear_str);
  update_light(&parse_double, &sdf::Light::SetQuadraticAttenuationFactor,
    light, attenuation_quadratic_str);
  update_light(&parse_vector, &sdf::Light::SetDirection, light, direction_str);

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

void LightTuning::OnRemoveLightBtnPress(int idx, const QString& name)
{
  auto light_queue_it = actions.find(name.toStdString());
  if (light_queue_it != actions.end())
  {
    if (!(light_queue_it->second.size()
      && light_queue_it->second.back() == Action::REMOVE))
    {
      light_queue_it->second.push(Action::REMOVE);
    }
  }
  _model.remove_light(idx);
}

void LightTuning::OnAddLightFormBtnPress(const QString& name)
{
  _model.add_new_light(name);
}

void LightTuning::OnSaveLightsBtnPress(const QString& url,
  bool save_all, int idx)
{
  std::string path = QUrl(url).toLocalFile().toStdString();
  std::ofstream file(path);
  if (!file)
  {
    ignerr << "Unable to open file for writing." << std::endl;
    return;
  }

  const QVector<sdf::Light>& lights = _model.get_lights();
  if (save_all)
  {
    for (auto& light : lights)
    {
      file << light_to_sdf_string(light);
    }
  }
  else if (idx >= 0 && idx < (int)lights.size())
  {
    file << light_to_sdf_string(lights[idx]);
  }
  else
  {
    ignerr << "Invalid index given. No light saved to file." << std::endl;
  }
  file.close();
  ignmsg << "File saved to: " << path << std::endl;
}

// Register this plugin
IGNITION_ADD_PLUGIN(LightTuning,
  ignition::gui::Plugin)


#include "LightTuning.moc"
