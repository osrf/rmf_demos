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

#include <ignition/gazebo/gui/GuiSystem.hh>
#include <ignition/gazebo/gui/GuiEvents.hh>
#include <ignition/gazebo/SdfEntityCreator.hh>
#include <ignition/gazebo/components/Light.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/World.hh>

#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/msgs.hh>
#include <ignition/transport.hh>

// Helper function that creates a simple cube sdf model string
std::string create_light_marker_str(const std::string& name,
  const ignition::math::Pose3d& pose)
{
  std::ostringstream ss;
  ss << std::string(
      "<?xml version=\"1.0\"?>"
      "<sdf version=\"1.7\">");
  ss << "<model name=\"" << name << "\">" << std::endl;
  ss << "<pose>" << pose << "</pose>" << std::endl;
  ss << std::string(
      "<static>true</static>"
      "<link name=\"box_link\">"
      "<visual name=\"box_visual\">"
      "<cast_shadows>false</cast_shadows>"
      "<transparency>0.5</transparency>"
      "<geometry>"
      "<box>"
      "<size>0.5 0.5 0.5</size>"
      "</box>"
      "</geometry>"
      "</visual>"
      "</link>"
      "</model>"
      "</sdf>");
  return ss.str();
}

// Helper functions to parse inputs of various types from the GUI
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

std::optional<ignition::math::Angle> parse_angle(
  const std::string& angle_str)
{
  std::stringstream ss(angle_str);
  double angle;
  ss >> angle;
  if (!ss.fail())
  {
    return ignition::math::Angle(angle);
  }
  else
  {
    ignwarn << "Unable to parse \"" << angle_str <<
      "\" as an angle in radians. Using previous value." << std::endl;
    return std::nullopt;
  }
}

template<typename T>
std::string to_string(const T& value)
{
  std::ostringstream ss;
  ss << value;
  return ss.str();
}

// Helper function that inserts an SDF element representation of the light to the stream `os`
std::ostream& operator<<(std::ostream& os, const sdf::Light& light)
{
  os << "<light type=\""
     << (light.Type() == sdf::LightType::POINT ? "point" :
    (light.Type() == sdf::LightType::DIRECTIONAL ? "directional" : "spot"))
     << "\" name=\"" << light.Name() << "\"> \n";
  os << "<cast_shadows>"
     << (light.CastShadows() ? "true" : "false") << "</cast_shadows> \n";
  os << "<pose>" << light.RawPose() << "</pose>\n";
  os << "<diffuse>" << light.Diffuse() << "</diffuse>\n";
  os << "<specular>" << light.Specular() << "</specular>\n";
  os << "<attenuation>\n";
  os << "<range>" << light.AttenuationRange() << "</range>\n";
  os << "<constant>" << light.ConstantAttenuationFactor() << "</constant>\n";
  os << "<linear>" << light.LinearAttenuationFactor() << "</linear>\n";
  os << "<quadratic>" << light.QuadraticAttenuationFactor() << "</quadratic>\n";
  os << "</attenuation>\n";
  os << "<direction>" << light.Direction() << "</direction>\n";
  os << "<spot>\n";
  os << "<inner_angle>" << light.SpotInnerAngle() << "</inner_angle>\n";
  os << "<outer_angle>" << light.SpotOuterAngle() << "</outer_angle>\n";
  os << "<falloff>" << light.SpotFalloff() << "</falloff>\n";
  os << "</spot>\n";
  os << "</light>\n";
  return os;
}

// Data model representing a list of lights. Provides data for delegates to display
// in QML via a series of roles which the delegates bind to.
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
    DirectionRole,
    SpotInnerAngleRole,
    SpotOuterAngleRole,
    SpotFalloffRole
  };

  QHash<int, QByteArray> roleNames() const override;
  int rowCount(const QModelIndex& parent = QModelIndex()) const override;
  QVariant data(const QModelIndex& index,
    int role = Qt::DisplayRole) const override;
  // Note: We do not need to override and define the setData() function since
  // we only update our model from the GUI in `LightTuning::OnCreateLight`

  // Inserts a default light with the name specified in `name_qstr`,
  // if it does not exist
  void add_new_light(const QString& name_qstr);
  // Deletes the light from LightsModel at index `idx` if it exists
  void remove_light(int idx);

  // Returns a reference to the light at index `idx` in `_lights`.
  // Assumes `idx` is valid.
  sdf::Light& get_light(int idx);
  // Returns a reference to the light with name `name` in `_lights`.
  // Assumes `name` is a valid name belonging to some light in `_lights`.
  sdf::Light& get_light(const std::string& name);
  // Returns a const reference to a QVector of all lights in the LightsModel
  // object
  const QVector<sdf::Light>& get_lights() const;

private:
  // Collection of lights, each with a unique name (enforced by
  // add_new_light)
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
    { DirectionRole, "direction"},
    { SpotInnerAngleRole, "spot_inner_angle"},
    { SpotOuterAngleRole, "spot_outer_angle"},
    { SpotFalloffRole, "spot_falloff"}};
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
  // Returns displayable value corresponding to the light property requested
  switch (role)
  {
    case NameRole:
    {
      return QString(light.Name().c_str());
    }
    case PoseRole:
    {
      return QString(to_string(light.RawPose()).c_str());
    }
    case IndexRole:
    {
      return index.row();
    }
    case DiffuseRole:
    {
      return QString(to_string(light.Diffuse()).c_str());
    }
    case SpecularRole:
    {
      return QString(to_string(light.Specular()).c_str());
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
      return QString(to_string(light.Direction()).c_str());
    }
    case SpotInnerAngleRole:
    {
      return QString(to_string(light.SpotInnerAngle()).c_str());
    }
    case SpotOuterAngleRole:
    {
      return QString(to_string(light.SpotOuterAngle()).c_str());
    }
    case SpotFalloffRole:
    {
      return light.SpotFalloff();
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

// Class that handles all GUI interactions and their associated
// light creations/deletions
class LightTuning : public ignition::gazebo::GuiSystem
{
  Q_OBJECT

public:
  virtual void LoadConfig(const tinyxml2::XMLElement* _pluginElem)
  override;

  void Update(const ignition::gazebo::UpdateInfo& _info,
    ignition::gazebo::EntityComponentManager& _ecm) override;

signals:
  void poseChanged(QString nm, QString new_pose);
  void markerSelected(QString nm);

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
    const QString& direction_str,
    const QString& spot_inner_angle_str,
    const QString& spot_outer_angle_str,
    const QString& spot_falloff_str);
  void OnRemoveLightBtnPress(int idx, const QString& name);
  void OnAddLightFormBtnPress(const QString& name);
  void OnSaveLightsBtnPress(const QString& url, bool save_all = true,
    int idx = -1);

protected: bool eventFilter(QObject* _obj, QEvent* _event) override;

private:
  const std::string sdf_open_tag = "<sdf version=\"1.7\"> \n";
  const std::string sdf_close_tag = "</sdf>";

  std::string _world_name;
  ignition::transport::Node _node;
  LightsModel _model;

  // Contains an Entity that serves as a physical representation of
  // the light on the screen, so that a user can move it around to set the
  // light pose
  struct LightMarker
  {
    std::string name; // Name of the LightMarker
    ignition::gazebo::Entity en;
  };
  // List of pairs of light name and corresponding marker name being spawned
  std::vector<std::pair<std::string, std::string>> _markers_spawn_pipeline;
  // Map from light name to its corresponding marker
  std::unordered_map<std::string, LightMarker> _markers;

  enum class Action {REMOVE, CREATE};
  // Map from a light name to a queue of create/remove service requests
  // for that corresponding light
  std::unordered_map<std::string, std::queue<Action>> actions;

  // Returns a string representation of the specified light in the
  // SDF v1.7 format
  std::string light_to_sdf_string(const sdf::Light&);

  // Sends a service request to render the light with name `name`
  // using Ignition transport. Also creates a corresponding LightMarker
  void create_light_service(const ignition::gazebo::EntityComponentManager& ecm,
    const std::string& name);
  // Sends a service request to delete the light with name `name`
  // as well as its corresponding LightMarker
  void remove_light_service(const std::string& name);
  // Sends a service request to render the LightMarker corresponding to the light
  // with name `light_name` using Ignition transport
  void create_marker_service(
    const ignition::gazebo::EntityComponentManager& ecm,
    const std::string& light_name, const ignition::math::Pose3d& pose);
  // Sends a service request to remove the LightMarker corresponding to the light
  // with name `light_name` using Ignition transport
  void remove_marker_service(const std::string& light_name);
};

void LightTuning::LoadConfig(const tinyxml2::XMLElement*)
{
  if (this->title.empty())
    this->title = "Light Tuning";

  // Monitor any Light entity selection in order to open relevant dropdown
  ignition::gui::App()->findChild<
    ignition::gui::MainWindow*>()->installEventFilter(this);

  // Connect data model to view
  this->Context()->setContextProperty(
    "LightsModel", &this->_model);
}

void LightTuning::Update(const ignition::gazebo::UpdateInfo&,
  ignition::gazebo::EntityComponentManager& ecm)
{
  // Get world name for sending create/remove requests later.
  // Assumes there is only 1 world in the simulation
  if (!_world_name.size())
  {
    ecm.Each<ignition::gazebo::components::World,
      ignition::gazebo::components::Name>(
      [&](const ignition::gazebo::Entity&,
      const ignition::gazebo::components::World*,
      const ignition::gazebo::components::Name* name)
      -> bool
      {
        _world_name = name->Data();
        return false; // Only look at first entity found
      });
  }

  // Check if any LightMarkers have been spawned and add the
  // generated Entity to _markers if so
  auto _new_markers_it = _markers_spawn_pipeline.begin();
  while (_new_markers_it != _markers_spawn_pipeline.end())
  {
    std::string& light_name = _new_markers_it->first;
    std::string& marker_name = _new_markers_it->second;

    auto marker_en = ecm.EntityByComponents(
      ignition::gazebo::components::Name(marker_name),
      ignition::gazebo::components::Model());
    if (marker_en != ignition::gazebo::kNullEntity)
    {
      _markers[light_name] = LightMarker {marker_name, marker_en};
      _new_markers_it = _markers_spawn_pipeline.erase(_new_markers_it);
    }
    else
    {
      ++_new_markers_it;
    }
  }

  // Update GUI to show latest poses of LightMarkers
  for (auto it = _markers.begin(); it != _markers.end(); ++it)
  {
    auto pose =
      ecm.Component<ignition::gazebo::components::Pose>(it->second.en);
    if (pose)
    {
      poseChanged(QString(it->first.c_str()),
        QString(to_string(pose->Data()).c_str()));
    }
  }

  // When multiple create/remove requests for the same entity are sent
  // in the same update step, the order of processing may be non-deterministic.
  // To workaround this, in each Update() call, we use a queue to ensure we only
  // issue one create/remove request per light. This ensures that entities are
  // created/removed in Ignition in the order requested.
  auto light_queue_it = actions.begin();
  while (light_queue_it != actions.end())
  {
    bool erase = false;
    if (light_queue_it->second.size()) // Pending actions to complete
    {
      if (light_queue_it->second.front() == Action::CREATE)
      {
        create_light_service(ecm, light_queue_it->first);
        light_queue_it->second.pop();
      }
      else
      {
        remove_light_service(light_queue_it->first);
        light_queue_it->second.pop();
        // Mark light for erasure from map if last request is a remove request
        // and no other requests remain
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

// Monitor and emit signal when a LightMarker is selected, so that
// the relevant menu in GUI can be expanded
bool LightTuning::eventFilter(QObject* _obj, QEvent* _event)
{
  if (_event->type() == ignition::gazebo::gui::events::EntitiesSelected::kType)
  {
    auto event =
      reinterpret_cast<ignition::gazebo::gui::events::EntitiesSelected*>(_event);
    if (event && !event->Data().empty())
    {
      const ignition::gazebo::Entity en = *event->Data().begin();
      auto it = std::find_if(_markers.begin(), _markers.end(),
          [&en](const std::pair<std::string, LightMarker>& marker)
          {
            return marker.second.en == en;
          });
      if (it != _markers.end())
      {
        emit markerSelected(QString(it->first.c_str()));
      }
    }
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

// Could possibly use an XML library instead
std::string LightTuning::light_to_sdf_string(const sdf::Light& light)
{
  std::ostringstream ss;
  ss << sdf_open_tag;
  ss << light;
  ss << sdf_close_tag;
  return ss.str();
}

// Necesary to supply callbacks to the service requests in order for the
// requests to execute properly, though they are not used for anything here.
void light_service_cb(const ignition::msgs::Boolean&, const bool)
{
}

// Assumes any service requests will be successful
void LightTuning::create_light_service(
  const ignition::gazebo::EntityComponentManager& ecm, const std::string& name)
{
  ignition::msgs::EntityFactory create_light_req;
  const sdf::Light& light = _model.get_light(name);
  create_light_req.set_sdf(light_to_sdf_string(light));
  _node.Request("/world/" + _world_name + "/create",
    create_light_req, light_service_cb);

  create_marker_service(ecm, name, light.RawPose());
}

void LightTuning::remove_light_service(const std::string& name)
{
  ignition::msgs::Entity remove_light_req;
  remove_light_req.set_name(name);
  remove_light_req.set_type(ignition::msgs::Entity_Type_LIGHT);
  _node.Request("/world/" + _world_name + "/remove",
    remove_light_req, light_service_cb);

  remove_marker_service(name);
}

void marker_service_cb(const ignition::msgs::Boolean&, const bool)
{
}

void LightTuning::create_marker_service(
  const ignition::gazebo::EntityComponentManager& ecm,
  const std::string& light_name, const ignition::math::Pose3d& pose)
{
  std::string marker_name = light_name + "_marker";
  while (
    ecm.EntityByComponents(ignition::gazebo::components::Name(marker_name))
    != ignition::gazebo::kNullEntity)
  {
    marker_name += "_"; // Avoid name collisions
  }

  ignition::msgs::EntityFactory create_marker_req;
  create_marker_req.set_sdf(create_light_marker_str(marker_name, pose));
  _node.Request("/world/" + _world_name + "/create",
    create_marker_req, marker_service_cb);

  _markers_spawn_pipeline.push_back({light_name, marker_name});
}

void LightTuning::remove_marker_service(const std::string& light_name)
{
  const std::unordered_map<std::string, LightMarker>::iterator it =
    _markers.find(light_name);
  if (it == _markers.end())
  {
    ignwarn << "Unable to remove any marker belonging to light with name " <<
      light_name << std::endl;
    return;
  }
  const std::string& marker_name = it->second.name;

  ignition::msgs::Entity remove_marker_req;
  remove_marker_req.set_name(marker_name);
  remove_marker_req.set_type(ignition::msgs::Entity_Type_MODEL);
  _node.Request("/world/" + _world_name + "/remove",
    remove_marker_req, marker_service_cb);

  _markers.erase(it);
}

// Helper template function to parse GUI input and update the sdf Light's property
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
  const QString& direction_str,
  const QString& spot_inner_angle_str,
  const QString& spot_outer_angle_str,
  const QString& spot_falloff_str)
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
  update_light(&parse_angle, &sdf::Light::SetSpotInnerAngle,
    light, spot_inner_angle_str);
  update_light(&parse_angle, &sdf::Light::SetSpotOuterAngle,
    light, spot_outer_angle_str);
  update_light(&parse_double, &sdf::Light::SetSpotFalloff,
    light, spot_falloff_str);

  // Update the light's queue of actions, or create one if necessary
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
  // Add to queue of requests to remove from simulation
  if (light_queue_it != actions.end())
  {
    if (!(light_queue_it->second.size()
      && light_queue_it->second.back() == Action::REMOVE))
    {
      light_queue_it->second.push(Action::REMOVE);
    }
  }
  // Remove from data model (and GUI)
  _model.remove_light(idx);
}

// Adds a new light to the model, but does not render in simulation yet
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
    file << sdf_open_tag;
    for (auto& light : lights)
    {
      file << light;
    }
    file << sdf_close_tag;
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
