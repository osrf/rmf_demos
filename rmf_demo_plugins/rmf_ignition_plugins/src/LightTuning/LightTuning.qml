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

ToolBar {
  Layout.minimumWidth: 280
  Layout.minimumHeight: 900

  background: Rectangle {
    color: "transparent"
  }

  ButtonGroup {
    id: group
  }

  GridLayout {
    rows: 13
    columns: 1
    anchors.fill: parent

    ColumnLayout {
      //anchors.fill: parent
      Label {
          width: parent.width
          wrapMode: Label.Wrap
          horizontalAlignment: Qt.AlignHCenter
          text: "Name"
      }

      TextField {
          id: name
          placeholderText: "sun"
      }
    }

    RowLayout {
      spacing: 10
      Text {
        text: "Type:"
        font.weight: Font.Bold
        Layout.alignment: Qt.AlignVCenter | Qt.AlignHCenter
        Layout.leftMargin: 15
        Layout.bottomMargin: 5
      }
      ComboBox {
        id: light_type
        currentIndex: 0
        width: 1
        Layout.leftMargin: 10
        Layout.bottomMargin: 5
        model: ListModel {
          id: lightTypeList
          ListElement { text: "Point" }
          ListElement { text: "Directional" }
          ListElement { text: "Spot" }
        }
        onCurrentIndexChanged: LightTuning.OnLightTypeSelect(lightTypeList.get(currentIndex).text)
      }
    }

    CheckBox {
      id: cast_shadow
      text: qsTr("Cast Shadow")
      Layout.columnSpan: 1
      Layout.alignment: Qt.AlignVCenter | Qt.AlignLeft
      Layout.leftMargin: 2
      checked: false
      onClicked: {
        LightTuning.OnShadowSelect(checked)
      }
    }

    ColumnLayout {
      //anchors.fill: parent
      Label {
          width: parent.width
          wrapMode: Label.Wrap
          horizontalAlignment: Qt.AlignHCenter
          text: "Pose"
      }

      TextField {
          id: pose
          placeholderText: "0 0 0 0"
      }
    }

    ColumnLayout {
      //anchors.fill: parent
      Label {
          width: parent.width
          wrapMode: Label.Wrap
          horizontalAlignment: Qt.AlignHCenter
          text: "Diffuse"
      }

      TextField {
          id: diffuse
          placeholderText: "0 0 0 0"
      }
    }

    ColumnLayout {
      //anchors.fill: parent
      Label {
          width: parent.width
          wrapMode: Label.Wrap
          horizontalAlignment: Qt.AlignHCenter
          text: "Specular"
      }

      TextField {
          id: specular
          placeholderText: "0 0 0 0"
      }
    }

    ColumnLayout {
      //anchors.fill: parent
      Label {
          width: parent.width
          wrapMode: Label.Wrap
          horizontalAlignment: Qt.AlignHCenter
          text: "Attenuation Range"
      }

      TextField {
          id: attentuation_range
          placeholderText: "1000"
      }
    }

    ColumnLayout {
      //anchors.fill: parent
      Label {
          width: parent.width
          wrapMode: Label.Wrap
          horizontalAlignment: Qt.AlignHCenter
          text: "Attenuation Constant Factor"
      }

      TextField {
          id: attentuation_constant
          placeholderText: "0.09"
      }
    }

    ColumnLayout {
      //anchors.fill: parent
      Label {
          width: parent.width
          wrapMode: Label.Wrap
          horizontalAlignment: Qt.AlignHCenter
          text: "Attenuation Linear Factor"
      }

      TextField {
          id: attentuation_linear
          placeholderText: "0.001"
      }
    }

    ColumnLayout {
      //anchors.fill: parent
      Label {
          width: parent.width
          wrapMode: Label.Wrap
          horizontalAlignment: Qt.AlignHCenter
          text: "Attenuation Quadratic Factor"
      }

      TextField {
          id: attentuation_quadratic
          placeholderText: "0.001"
      }
    }

    ColumnLayout {
      //anchors.fill: parent
      Label {
          width: parent.width
          wrapMode: Label.Wrap
          horizontalAlignment: Qt.AlignHCenter
          text: "Direction"
      }

      TextField {
          id: direction
          placeholderText: "-0.5 0.1 -0.9"
      }
    }

    Button {
        id: createButton
        text: "Create Light"
        onClicked: {
          LightTuning.OnCreateLight(cast_shadow.checked, lightTypeList.get(light_type.currentIndex).text,
            name.text, pose.text, diffuse.text,
            specular.text, attentuation_range.text,
            attentuation_constant.text, attentuation_linear.text,
            attentuation_quadratic.text, direction.text)
        }
    }
  }
}