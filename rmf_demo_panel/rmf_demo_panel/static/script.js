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

// Js script for front end HTML GUI

let selected_robot_name = null;
let selected_task_ids = [];

function submitTaskForm() {
  let start_time = document.getElementById("submitTaskForm").start_time.value;
  let cleaning_zone = document.getElementById("submitTaskForm").cleaning_zone
    .value;
  let evaluator = document.getElementById("submitTaskForm").evaluator.value;
  console.log("submit task: ", start_time, cleaning_zone, evaluator);
  console.log("Submitting Task");
  $.ajax({
    type: "POST",
    url: "/submit_task",
    contentType: "application/json",
    data: JSON.stringify({
      task_type: "Clean",
      start_time: parseInt(start_time),
      evaluator: evaluator,
      description: { cleaning_zone: cleaning_zone },
    }),
    success: function (data) {
      document.getElementById("submitted_task_id").innerHTML = data;
    },
  });
}

function submitTaskListForm() {
  // temp hack to handle task list submission in async
  var global_list_count = 0;
  var global_task_list = [];
  let i = 0; // to set time "delay"
  let res = "Task List Submitted";

  try {
    let json_task_list = document.getElementById("task_list_box").value;
    let task_list = JSON.parse(json_task_list);

    for (task of task_list) {
      if (!("task_type" && "start_time" && "description" in task))
        throw "Incorrect Task Json Format";

      global_task_list.push(task);
      setTimeout(function (i) {
        console.log(" Submit Task!", global_task_list[global_list_count]);
        $.ajax({
          type: "POST",
          url: "/submit_task",
          contentType: "application/json",
          data: JSON.stringify(global_task_list[global_list_count]),
        });
        global_list_count++;
      }, 800 * ++i);
    }
  } catch (err) {
    res = "ERROR! " + err;
  }
  document.getElementById("res_submit_list").innerHTML = res;
}

function cancelTask() {
  let id = document.getElementById("cancelForm").task_id.value;
  console.log("cancel task: ", id);
  $.ajax({
    type: "POST",
    url: "/cancel_task",
    contentType: "application/json",
    data: JSON.stringify({ task_id: id }),
    success: function (data) {
      document.getElementById("delete_result").innerHTML = data;
    },
  });
}

function getTaskStatus() {
  $.ajax({
    type: "GET",
    url: "/get_task",
    success: function (data) {
      populate_task_status(data);
    },
  });
}

function getRobotStates() {
  $.ajax({
    type: "GET",
    url: "/get_robots",
    success: function (data) {
      populate_robot_states(data);
    },
  });
}

//////////////////////////////////////////////////////////////////////////////

function populate_task_status(data) {
  // potential failed get task, ignore
  if (data == null || data.length == 0) return;

  console.log(" Get Task and populate table: ", data);
  $("#task_status_table tbody tr").remove();
  $.each(data, function (i, item) {
    var el = $(
      "<tr class='task_summary_row progress_bar'style='--progress:" +
        item.progress + ";'>" +
        "<td>" + item.task_id + "</td>" +
        "<td>" + item.description + "</td>" +
        "<td>" + item.robot_name + "</td>" +
        "<td>" + item.state + "</td>" +
        "<td>" + item.task_type + "</td>" +
        "<td>" + item.submited_start_time + "s (" +
        (item.submited_start_time / 60.0).toFixed(1) + "mins)</td>" +
        "<td>" + item.start_time + "s (" +
        (item.start_time / 60.0).toFixed(1) + "mins)</td>" +
        "<td>" + item.end_time + "s (" +
        (item.end_time / 60.0).toFixed(1) + "mins)</td>" +
        "<td>" + item.progress + "</td>" +
      "</tr>"
    );
    $("#task_status_table tbody").append(el);
    if (selected_task_ids.includes(item.task_id)) {
      el.addClass("selected");
    }
  });
}

function populate_robot_states(data) {
  console.log(" Get robot states and populate table: ", data);
  $("#robot_states_table tbody tr").remove();
  $.each(data, function (i, item) {
    chging_icon = "%  ";
    if (item.mode == "Charging-1") {
      chging_icon += "<i class='fa fa-charging-station icons'></i>";
    }
    var el = $(
      "<tr class='robot_summary_row'>" +
        "<td>" + item.robot_name + "</td>" +
        "<td>" + item.fleet_name + "</td>" +
        "<td>" + item.assignments + "</td>" +
        "<td>" + item.mode + "</td>" +
        "<td>" + item.battery_percent.toFixed(2) +
        chging_icon + "</td>" +
        "<td>" + item.level_name + " - {" + 
        item.location_x.toFixed(2) + ", " +
        item.location_y.toFixed(2) + ", " + 
        item.location_yaw.toFixed(2) +"}</td>" +
        "</tr>"
    );
    $("#robot_states_table tbody").append(el);
    if (item.robot_name === selected_robot_name) {
      el.addClass("selected");
    }
  });
}

//////////////////////////////////////////////////////////////////////////////

// Handle table box selection while hovering on robot and summary
$(document).ready(function () {
  $(document).on("mouseenter", ".robot_summary_row", function () {
    $(this).addClass("selected");
    selected_robot_name = $(this).children(":nth-child(1)").text();
    selected_task_ids = [];
    $(".task_summary_row").each(function () {
      var task_id = $(this).children(":nth-child(1)").text();
      var assigned_robot_name = $(this).children(":nth-child(3)").text();
      if (assigned_robot_name === selected_robot_name) {
        $(this).addClass("selected");
        selected_task_ids.push(task_id);
      }
    });
  });
  $(document).on("mouseenter", ".task_summary_row", function () {
    $(this).addClass("selected");
    selected_robot_name = $(this).children(":nth-child(3)").text();
    selected_task_ids = [$(this).children(":nth-child(1)").text()];
    $(".robot_summary_row").each(function () {
      var robot_states_table_robot_name = $(this)
        .children(":nth-child(1)")
        .text();
      if (robot_states_table_robot_name === selected_robot_name) {
        $(this).addClass("selected");
      }
    });
  });
  $(document).on(
    "mouseleave",
    ".robot_summary_row, .task_summary_row",
    function () {
      selected_robot_name = null;
      selected_task_ids = [];
      $(".robot_summary_row").each(function () {
        $(this).removeClass("selected");
      });
      $(".task_summary_row").each(function () {
        $(this).removeClass("selected");
      });
    }
  );
});

// socket broadcast receiver
$(document).ready(function () {
  let namespace = "/status_updates";
  let socket = io(namespace);
  socket.on("task_status", function (msg, cb) {
    populate_task_status(msg);
  });
  socket.on("robot_states", function (msg, cb) {
    populate_robot_states(msg);
  });
  socket.on("ros_time", function (msg, cb) {
    document.getElementById("ros_time").innerHTML =
      Math.floor(msg / 3600) +
      ":" +
      (Math.floor(msg / 60) % 60) +
      ":" +
      (msg % 60);
  });
});

// Handle loading of task list from file input
$(document).ready(function () {
  $(document).on("change", "#task_file", function () {
    var fr = new FileReader();
    fr.onload = function () {
      document.getElementById("task_list_box").value = fr.result;
    };
    fr.readAsText(this.files[0]);
  });
});
