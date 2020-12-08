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

import QtQuick 2.9
import QtQuick.Window 2.2
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.2
import QtQuick.Controls.Material.impl 2.2
import QtQuick.Layouts 1.3
import QtQuick.Controls.Styles 1.4
import QtQuick.Dialogs 1.0

Rectangle {
  id: lightTuningPlugin
  Layout.minimumWidth: 350
  Layout.minimumHeight: 1050
  anchors.fill: parent
  color: lightGrey

  property color lightGrey: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade100) :
    Material.color(Material.Grey, Material.Shade800)

  property color darkGrey: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade200) :
    Material.color(Material.Grey, Material.Shade900)

  property color highlightColor: Qt.rgba(
    Material.accent.r,
    Material.accent.g,
    Material.accent.b, 0.3)

  // Horizontal margins
  property int margin: 15

  // Form to add a new light-edit menu and to save all lights to file
  Rectangle {
    id: addSaveForm
    height: newLightName.height + addButton.height
      + saveButton.height + (addSaveBtnsCol.spacing * 2)
    width: parent.width
    anchors.top: parent.top
    anchors.left: parent.left
    anchors.right: parent.right
    color: lightGrey

    Column {
      id: addSaveBtnsCol
      anchors.fill: parent
      spacing: 5

      RowLayout {
        TextField {
            id: newLightName
            placeholderText: "Enter new unique light name"
            font.pointSize: 13
            Layout.minimumWidth: 320
            Layout.leftMargin: margin
            Layout.rightMargin: margin
            Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
        }
      }

      RowLayout {
        // Creates a form to fill in parameters for a new light
        Button {
            id: addButton
            text: "Add Light"
            onClicked: {
              LightTuning.OnAddLightFormBtnPress(newLightName.text)
            }
            Layout.minimumWidth: 100
            Layout.leftMargin: margin
            Layout.rightMargin: margin
            Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
        }
      }

      RowLayout {
        Button {
            // Saves all current lights to file in the SDF format
            id: saveButton
            text: "Save All"
            onClicked: {
              saveAllDialog.open()
            }
            Layout.minimumWidth: 100
            Layout.leftMargin: margin
            Layout.rightMargin: margin
            Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
        }

        FileDialog {
          id: saveAllDialog
          title: "Save lights as SDF file"
          selectExisting: false

          onAccepted: {
            LightTuning.OnSaveLightsBtnPress(fileUrl, true)
            close()
          }
          onRejected: {
            close()
          }
        }
      }
    }
  }

  // Displays the light parameters for each light and propagates any updates to
  // the LightTuning/LightsModel C++ classes
  ListView {
    anchors.top: addSaveForm.bottom
    anchors.bottom: parent.bottom
    anchors.left: parent.left
    anchors.right: parent.right
    id: listView
    orientation: ListView.Vertical

    model: {
      try {
        return LightsModel;
      }
      catch(e) {
        return null;
      }
    }

    delegate:
      Rectangle {
        height: header.height + content.height
        width: parent.width
        color: lightGrey

        function createLight() {
          LightTuning.OnCreateLightBtnPress(model.idx,
            cast_shadow.checked,
            lightTypeList.get(light_type.currentIndex).text,
            model.name, pose.text, diffuse.text,
            specular.text, attenuation_range.text,
            attenuation_constant.value, attenuation_linear.value,
            attenuation_quadratic.value, direction.text,
            spot_inner_angle.value, spot_outer_angle.value,
            spot_falloff.text)
        }

        Column {
          anchors.fill: parent

          // Header
          Rectangle {
            id: header
            width: parent.width
            height: nameId.height + (margin * 2)
            color: lightGrey

            RowLayout {
              anchors.fill: parent
              Image {
                sourceSize.height: nameId.height - 3
                sourceSize.width: nameId.height - 3
                fillMode: Image.Pad
                Layout.alignment: Qt.AlignVCenter
                Layout.leftMargin: margin
                source: content.show ?
                    "images/minus.png" : "images/plus.png"
              }
              Label {
                  id: nameId
                  width: text.width
                  text: model.name
                  font.pointSize: 13
                  wrapMode: Label.Wrap
                  Layout.alignment: Qt.AlignVCenter | Qt.AlignHLeft
                  Layout.margins: margin
              }
              Item {
                Layout.fillWidth: true
              }
            }

            MouseArea {
              anchors.fill: parent
              hoverEnabled: true
              cursorShape: Qt.PointingHandCursor
              Connections {
                target: LightTuning
                onMarkerSelected: {
                  if (nm == model.name)
                  {
                    content.show = true;
                  }
                  else
                  {
                    content.show = false;
                  }
                }
              }
              onClicked: {
                content.show = !content.show;
              }
              onEntered: {
                header.color = highlightColor
              }
              onExited: {
                header.color = lightGrey
              }
            }
          }

          // Collapsible content
          Rectangle {
            id: content
            property bool show: false
            width: parent.width
            height: show ? grid.height : 0
            clip: true
            color: "transparent"

            Behavior on height {
              NumberAnimation {
                duration: 50;
                easing.type: Easing.InOutQuad
              }
            }

            GridLayout {
              id: grid
              width: parent.width
              columns: 1

              RowLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                spacing: 10
                Text {
                  text: "Type"
                  font.pointSize: 10
                  Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
                  Layout.leftMargin: margin
                  Layout.bottomMargin: 5
                }
                ComboBox {
                  id: light_type
                  currentIndex: 0
                  width: 1
                  Layout.leftMargin: margin
                  Layout.bottomMargin: 5
                  model: ListModel {
                    id: lightTypeList
                    ListElement { text: "Point" }
                    ListElement { text: "Directional" }
                    ListElement { text: "Spot" }
                  }
                  onCurrentIndexChanged: {
                    createLight();
                    if (lightTypeList.get(currentIndex).text == "Spot") {
                      spotlightParams.show = true;
                    } else {
                      spotlightParams.show = false;
                    }
                  }
                  Component.onCompleted: {
                    spotlightParams.show = false; // Set initial show value to trigger height change
                  }
                }
              }

              RowLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                MenuItem {
                    text: 'Cast Shadow'
                    id: cast_shadow
                    font.pointSize: 10
                    checkable: true
                    indicator.anchors.right: right
                    indicator.anchors.rightMargin: rightPadding
                    contentItem.anchors.left: left
                    contentItem.anchors.leftMargin: leftPadding
                    Component.onCompleted: {
                        contentItem.leftPadding = 0
                    }
                    onTriggered: {
                      createLight();
                    }
                }
              }

              RowLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                spacing: 10
                Label {
                    id: bar
                    wrapMode: Label.Wrap
                    text: "Pose"
                    Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
                    Layout.leftMargin: margin
                    Layout.bottomMargin: 5
                }

                TextField {
                    id: pose
                    text: model.pose
                    placeholderText: "0 0 0 0 0 0"
                    horizontalAlignment: Qt.AlignHCenter
                    Connections {
                      target: LightTuning
                      onPoseChanged: {
                        pose.text = (nm == model.name && !pose_manual_toggle.checked) ?
                          new_pose : pose.text;
                        createLight();
                      }
                    }
                    onEditingFinished : {
                      if (pose_manual_toggle.checked) {
                        createLight();
                      }
                    }
                }

                Switch {
                    id: pose_manual_toggle
                    text: qsTr("Manual")
                }
              }

              RowLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Label {
                    wrapMode: Label.Wrap
                    text: "Diffuse"
                    Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
                    Layout.leftMargin: margin
                    Layout.bottomMargin: 5
                }

                TextField {
                    id: diffuse
                    placeholderText: "1 1 1 1"
                    text: model.diffuse
                    horizontalAlignment: Qt.AlignHCenter
                    onEditingFinished : {
                      createLight()
                    }
                }

                // Pick diffuse color with GUI instead
                Button {
                  id: pickDiffuseColorButton
                  text: "Pick"
                  icon.source: "images/Colorwheel.png"
                  icon.color: "transparent"
                  onClicked: {
                    diffuseColorDialog.open()
                  }
                  Layout.leftMargin: margin
                  Layout.bottomMargin: 5

                  ColorDialog {
                      id: diffuseColorDialog
                      title: "Please choose a color"
                      showAlphaChannel: true
                      onAccepted: {
                          diffuse.text = diffuseColorDialog.color.r.toFixed(2)
                            + ' ' + diffuseColorDialog.color.g.toFixed(2)
                            + ' ' + diffuseColorDialog.color.b.toFixed(2)
                            + ' ' + diffuseColorDialog.color.a.toFixed(2);
                          createLight();
                          close();
                      }
                      onRejected: {
                          close()
                      }
                  }
                }
              }

              RowLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Label {
                    wrapMode: Label.Wrap
                    text: "Specular"
                    Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
                    Layout.leftMargin: margin
                    Layout.bottomMargin: 5
                }

                TextField {
                    id: specular
                    text: model.specular
                    placeholderText: "1 1 1 1"
                    horizontalAlignment: Qt.AlignHCenter
                    onEditingFinished : {
                      createLight()
                    }
                }
                // Pick specular color with GUI instead
                Button {
                  id: pickSpecularColorButton
                  text: "Pick"
                  icon.source: "images/Colorwheel.png"
                  icon.color: "transparent"
                  onClicked: {
                    specularColorDialog.open()
                  }
                  Layout.leftMargin: margin
                  Layout.bottomMargin: 5

                  ColorDialog {
                      id: specularColorDialog
                      title: "Please choose a color"
                      showAlphaChannel: true
                      onAccepted: {
                          specular.text = specularColorDialog.color.r.toFixed(2)
                            + ' ' + specularColorDialog.color.g.toFixed(2)
                            + ' ' + specularColorDialog.color.b.toFixed(2)
                            + ' ' + specularColorDialog.color.a.toFixed(2);
                          createLight();
                          close()
                      }
                      onRejected: {
                          close()
                      }
                  }
                }
              }

              RowLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Label {
                    wrapMode: Label.Wrap
                    text: "Attenuation Range"
                    Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
                    Layout.leftMargin: margin
                    Layout.bottomMargin: 5
                }

                TextField {
                    id: attenuation_range
                    text: model.attenuation_range
                    validator: DoubleValidator {notation: DoubleValidator.StandardNotation;}
                    placeholderText: "10"
                    horizontalAlignment: Qt.AlignHCenter
                    onEditingFinished : {
                      createLight()
                    }
                }
              }

              RowLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Label {
                    wrapMode: Label.Wrap
                    text: "Constant Attenuation"
                    Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
                    Layout.leftMargin: margin
                    Layout.bottomMargin: 5
                }

                Slider {
                  id: attenuation_constant
                  from: 0.0
                  to: 1.0
                  value: model.attenuation_constant
                  Layout.preferredWidth: 100
                  onPressedChanged: {
                    if (!pressed) {
                      createLight();
                    }
                  }
                }

                Label {
                    wrapMode: Label.Wrap
                    text: attenuation_constant.value.toFixed(2)
                    Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
                    Layout.leftMargin: margin
                    Layout.bottomMargin: 5
                }
              }

              RowLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Label {
                    wrapMode: Label.Wrap
                    text: "Linear Attenuation"
                    Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
                    Layout.leftMargin: margin
                    Layout.bottomMargin: 5
                }

                Slider {
                  id: attenuation_linear
                  from: 0.0
                  to: 1.0
                  value: model.attenuation_linear
                  Layout.preferredWidth: 100
                  onPressedChanged: {
                    if (!pressed) {
                      createLight();
                    }
                  }
                }

                Label {
                    wrapMode: Label.Wrap
                    text: attenuation_linear.value.toFixed(2)
                    Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
                    Layout.leftMargin: margin
                    Layout.bottomMargin: 5
                }
              }

              RowLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Label {
                    wrapMode: Label.Wrap
                    text: "Quadratic Attenuation"
                    Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
                    Layout.leftMargin: margin
                    Layout.bottomMargin: 5
                }

                Slider {
                  id: attenuation_quadratic
                  from: 0.0
                  to: 1.0
                  value: model.attenuation_quadratic
                  Layout.preferredWidth: 100
                  onPressedChanged: {
                    if (!pressed) {
                      createLight();
                    }
                  }
                }

                Label {
                    wrapMode: Label.Wrap
                    text: attenuation_quadratic.value.toFixed(2)
                    Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
                    Layout.leftMargin: margin
                    Layout.bottomMargin: 5
                }
              }

              RowLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Label {
                    wrapMode: Label.Wrap
                    text: "Direction"
                    Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
                    Layout.leftMargin: margin
                    Layout.bottomMargin: 5
                }

                TextField {
                    id: direction
                    text: model.direction
                    placeholderText: "0 0 -1"
                    horizontalAlignment: Qt.AlignHCenter
                    onEditingFinished : {
                      createLight()
                    }
                }
              }

              Column {
                id: spotlightParams

                property bool show: true
                height: { show ? Layout.implicitHeight : 0 }
                clip: true
                Behavior on height {
                  NumberAnimation {
                    duration: 50;
                    easing.type: Easing.InOutQuad
                  }
                }

                RowLayout {
                  Layout.fillWidth: true
                  Layout.fillHeight: true
                  Label {
                      wrapMode: Label.Wrap
                      text: "Spot Inner Angle"
                      Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
                      Layout.leftMargin: margin
                      Layout.bottomMargin: 5
                  }

                  Slider {
                    id: spot_inner_angle
                    from: 0.0
                    to: Math.PI
                    value: model.spot_inner_angle
                    Layout.preferredWidth: 100
                    onPressedChanged: {
                      if (!pressed) {
                        createLight();
                      }
                    }
                  }

                  Label {
                      wrapMode: Label.Wrap
                      text: spot_inner_angle.value.toFixed(2)
                      Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
                      Layout.leftMargin: margin
                      Layout.bottomMargin: 5
                  }
                }

                RowLayout {
                  Layout.fillWidth: true
                  Layout.fillHeight: true
                  Label {
                      wrapMode: Label.Wrap
                      text: "Spot Outer Angle"
                      Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
                      Layout.leftMargin: margin
                      Layout.bottomMargin: 5
                  }

                  Slider {
                    id: spot_outer_angle
                    from: 0.0
                    to: Math.PI
                    value: model.spot_outer_angle
                    Layout.preferredWidth: 100
                    onPressedChanged: {
                      if (!pressed) {
                        createLight();
                      }
                    }
                  }

                  Label {
                      wrapMode: Label.Wrap
                      text: spot_outer_angle.value.toFixed(2)
                      Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
                      Layout.leftMargin: margin
                      Layout.bottomMargin: 5
                  }
                }

                RowLayout {
                  Layout.fillWidth: true
                  Layout.fillHeight: true
                  Label {
                      wrapMode: Label.Wrap
                      text: "Spot Falloff"
                      Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
                      Layout.leftMargin: margin
                      Layout.bottomMargin: 5
                  }

                  TextField {
                      id: spot_falloff
                      text: model.spot_falloff
                      validator: DoubleValidator {notation: DoubleValidator.StandardNotation;}
                      placeholderText: "0"
                      horizontalAlignment: Qt.AlignHCenter
                      onEditingFinished : {
                        createLight();
                      }
                  }
                }
              }

              RowLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                // Renders light in the Ignition Gazebo simulation as well as a
                // marker at its pose
                Button {
                  text: "Create"
                  onClicked: {
                    createLight()
                  }
                  Layout.leftMargin: margin
                  Layout.bottomMargin: 5
                }

                // Deletes the light and its associated form and light marker
                Button {
                  text: "Remove"
                  onClicked: {
                    LightTuning.OnRemoveLightBtnPress(model.idx, model.name)
                  }
                  Layout.leftMargin: margin
                  Layout.bottomMargin: 5
                }

                // Saves the individual light as a file in SDF form
                Button {
                  id: saveButton
                  text: "Save"
                  onClicked: {
                    saveIndividualDialog.open()
                  }
                  Layout.leftMargin: margin
                  Layout.bottomMargin: 5

                  FileDialog {
                    id: saveIndividualDialog
                    title: "Save light as SDF file"
                    selectExisting: false

                    onAccepted: {
                      LightTuning.OnSaveLightsBtnPress(fileUrl, false, model.index)
                      close()
                    }
                    onRejected: {
                      close()
                    }
                  }
                }
              }
            }
          }
        }
      }
  }
}