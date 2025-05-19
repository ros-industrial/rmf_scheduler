# Copyright 2025 ROS Industrial Consortium Asia Pacific
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Generate a Python Client based on Swagger JSON.
#
# :param OPENAPI_PYTHON_CLIENT_VERSION: the tool version to use
# :type OPENAPI_PYTHON_CLIENT_VERSION: string
# :param OPENAPI_JSON: the openapi.json file
# :type OPENAPI_JSON: string
# :param DESTINATION: the base package to install the module to
# :type DESTINATION: string
#

macro(generate_openapi_python_client)
  _rmf2_scheduler_py_setup_pyenv()
  _rmf2_scheduler_py_generate_openapi_python_client(${ARGN})
endmacro()

function(_rmf2_scheduler_py_setup_pyenv)
  if(NOT DEFINED Python3_EXECUTABLE)
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
  endif()

  # Setup venv
  set(_pyenv_dir ${CMAKE_CURRENT_BINARY_DIR}/py_env)
  execute_process(
    COMMAND "${Python3_EXECUTABLE}" -m venv "${_pyenv_dir}"
    RESULT_VARIABLE ret
    OUTPUT_VARIABLE stdout
  )

  if(NOT ret EQUAL 0)
    message(FATAL_ERROR "generate_openapi_python_client() unable to create Python venv, "
      "is venv installed?\n${stdout}")
  endif()

  set(RMF2_SCHEDULER_PY_PYENV_Python_EXECUTABLE ${_pyenv_dir}/bin/python PARENT_SCOPE)
  set(RMF2_SCHEDULER_PY_PYENV_Python3_EXECUTABLE ${_pyenv_dir}/bin/python3 PARENT_SCOPE)
  set(RMF2_SCHEDULER_PY_PYENV_Pip_EXECUTABLE ${_pyenv_dir}/bin/pip PARENT_SCOPE)
  set(RMF2_SCHEDULER_PY_PYENV_Pip3_EXECUTABLE ${_pyenv_dir}/bin/pip3 PARENT_SCOPE)
  set(RMF2_SCHEDULER_PY_PYENV_BINARY_DIR ${_pyenv_dir}/bin PARENT_SCOPE)
endfunction()

function(_rmf2_scheduler_py_generate_openapi_python_client)
  cmake_parse_arguments(
    ARG "" "OPENAPI_PYTHON_CLIENT_VERSION;OPENAPI_JSON;MODULE_NAME;DESTINATION" "" ${ARGN})

  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "generate_openapi_python_client() called with unused "
      "arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  if(NOT ARG_OPENAPI_PYTHON_CLIENT_VERSION)
    message(FATAL_ERROR "generate_openapi_python_client() called with VERSION unset.")
  endif()

  if(NOT IS_ABSOLUTE "${ARG_OPENAPI_JSON}")
    set(ARG_OPENAPI_JSON "${CMAKE_CURRENT_LIST_DIR}/${ARG_OPENAPI_JSON}")
  endif()

  if(NOT EXISTS "${ARG_OPENAPI_JSON}")
    message(FATAL_ERROR "generate_openapi_python_client() the openapi.json "
      "file '${ARG_OPENAPI_JSON}' doesn't exist")
  endif()

  # Install the openapi-python-client
  execute_process(
    COMMAND
      "${RMF2_SCHEDULER_PY_PYENV_Pip3_EXECUTABLE}" install
        openapi-python-client==${ARG_OPENAPI_PYTHON_CLIENT_VERSION}
    RESULT_VARIABLE ret
    OUTPUT_VARIABLE stdout
  )

  if(NOT ret EQUAL 0)
    message(FATAL_ERROR "generate_openapi_python_client() unable to install "
      "openapi-python-client.\n${stdout}")
  endif()

  set(_build_dir ${CMAKE_CURRENT_BINARY_DIR}/openapi_python_client)
  file(MAKE_DIRECTORY ${_build_dir})

  # Setup module name override
  if(NOT ARG_MODULE_NAME)
    set(ARG_MODULE_NAME "api_client")  # default module name to api_client
  endif()

  set(_config_path "${_build_dir}/openapi_config.yaml")
  string(CONFIGURE "\
project_name_override: ${ARG_MODULE_NAME}
package_name_override: ${ARG_MODULE_NAME}
" openapi_config_content)
  file(GENERATE
    OUTPUT "${_config_path}"
    CONTENT "${openapi_config_content}"
  )

  # Generate the client scripts
  set(_out_dir "${_build_dir}/out")
  add_custom_target(
    open_api_client ALL
    DEPENDS
      ${_out_dir}
  )
  add_custom_command(
    OUTPUT
      ${_out_dir}
    COMMAND "${RMF2_SCHEDULER_PY_PYENV_BINARY_DIR}/openapi-python-client" generate
      --path ${ARG_OPENAPI_JSON}
      --output-path ${_out_dir}
      --config ${_config_path}
      --overwrite
    DEPENDS
      ${ARG_OPENAPI_JSON}
      ${_config_path}
      ${RMF2_SCHEDULER_PY_PYENV_BINARY_DIR}/openapi-python-client
    VERBATIM
  )

  # Install the generated python modules
  install(
    DIRECTORY ${_out_dir}/${ARG_MODULE_NAME}
    DESTINATION "${ARG_DESTINATION}"
      PATTERN "*.pyc" EXCLUDE
      PATTERN "__pycache__" EXCLUDE
  )
endfunction()
