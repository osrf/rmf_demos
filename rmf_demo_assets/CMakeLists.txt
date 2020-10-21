cmake_minimum_required(VERSION 3.5)

project(rmf_demo_assets)

find_package(ament_cmake REQUIRED)

# download models (MaleVisitorPhone for now)
set(model_name MaleVisitorPhone)
set(models_path models)
execute_process(COMMAND wget https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/${model_name}.zip -P ${CMAKE_CURRENT_LIST_DIR}/models/${model_name})

execute_process(COMMAND bsdtar xC ${CMAKE_CURRENT_LIST_DIR}/models/${model_name}  -f ${CMAKE_CURRENT_LIST_DIR}/models/${model_name}/${model_name}.zip)

execute_process(COMMAND rm ${CMAKE_CURRENT_LIST_DIR}/models/${model_name}/${model_name}.zip)


install(
  DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
