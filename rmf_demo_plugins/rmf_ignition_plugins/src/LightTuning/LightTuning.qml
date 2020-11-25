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
  Layout.minimumWidth: 280
  Layout.minimumHeight: 1200
  anchors.fill: parent

  property color lightGrey: (Material.theme == Material.Light) ?
    Material.color(Material.Grey, Material.Shade100) :
    Material.color(Material.Grey, Material.Shade800)

  property color highlightColor: Qt.rgba(
    Material.accent.r,
    Material.accent.g,
    Material.accent.b, 0.3)

  // Horizontal margins
  property int margin: 15

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
            Layout.minimumWidth: 250
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
          //folder: shortcuts.home
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

  ListView {
    anchors.bottom: parent.bottom
    anchors.top: addSaveForm.bottom
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
                    "minus.png" : "plus.png"
              }
              Label {
                  id: nameId
                  text: model.name
                  font.weight: Font.Bold
                  font.pointSize: 13
                  wrapMode: Label.Wrap
                  Layout.alignment: Qt.AlignVCenter | Qt.AlignHLeft
                  Layout.margins: margin
              }
            }

            MouseArea {
              anchors.fill: parent
              hoverEnabled: true
              cursorShape: Qt.PointingHandCursor
              onClicked: {
                content.show = !content.show
              }
              onEntered: {
                header.color = highlightColor
              }
              onExited: {
                header.color = lightGrey
              }
            }
          }

          // Content
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
                  text: "Type:"
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
                }
              }

              RowLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                CheckBox {
                  id: cast_shadow
                  text: "Cast Shadow"
                  Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
                  Layout.leftMargin: margin
                  Layout.bottomMargin: 5
                  checked: false
                }
              }

              RowLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                spacing: 10
                Label {
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
                    id: attentuation_range
                    text: model.attenuation_range
                    validator: DoubleValidator {notation: DoubleValidator.StandardNotation;}
                    placeholderText: "10"
                }
              }

              RowLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Label {
                    wrapMode: Label.Wrap
                    text: "Constant Attenuation Factor"
                    Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
                    Layout.leftMargin: margin
                    Layout.bottomMargin: 5
                }

                TextField {
                    id: attentuation_constant
                    text: model.attenuation_constant
                    validator: DoubleValidator {notation: DoubleValidator.StandardNotation; bottom:0.0; top:1.0;}
                    placeholderText: "1.0"
                }
              }

              RowLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Label {
                    wrapMode: Label.Wrap
                    text: "Linear Attenuation Factor"
                    Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
                    Layout.leftMargin: margin
                    Layout.bottomMargin: 5
                }

                TextField {
                    id: attentuation_linear
                    text: model.attenuation_linear
                    validator: DoubleValidator {notation: DoubleValidator.StandardNotation; bottom:0.0; top:1.0;}
                    placeholderText: "1.0"
                }
              }

              RowLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Label {
                    wrapMode: Label.Wrap
                    text: "Quadratic Attenuation Factor"
                    Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
                    Layout.leftMargin: margin
                    Layout.bottomMargin: 5
                }

                TextField {
                    id: attentuation_quadratic
                    text: model.attenuation_quadratic
                    validator: DoubleValidator {notation: DoubleValidator.StandardNotation; bottom:0.0; top:1.0;}
                    placeholderText: "1.0"
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

                TextField {
                    id: spot_inner_angle
                    text: model.spot_inner_angle
                    validator: DoubleValidator {notation: DoubleValidator.StandardNotation;}
                    placeholderText: "1.0"
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

                TextField {
                    id: spot_outer_angle
                    text: model.spot_outer_angle
                    validator: DoubleValidator {notation: DoubleValidator.StandardNotation;}
                    placeholderText: "1.0"
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
                }
              }

              RowLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                // Renders light in the Ignition Gazebo simulation
                Button {
                  text: "Create"
                  onClicked: {
                    LightTuning.OnCreateLightBtnPress(model.idx,
                      cast_shadow.checked,
                      lightTypeList.get(light_type.currentIndex).text,
                      model.name, pose.text, diffuse.text,
                      specular.text, attentuation_range.text,
                      attentuation_constant.text, attentuation_linear.text,
                      attentuation_quadratic.text, direction.text,
                      spot_inner_angle.text, spot_outer_angle.text,
                      spot_falloff.text)
                  }
                  Layout.leftMargin: margin
                  Layout.bottomMargin: 5
                }

                  // Deletes the light and it's associated form
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