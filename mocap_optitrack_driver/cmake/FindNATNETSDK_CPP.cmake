# Copyright 2021 Georgia Institute of Technology, USA
# Copyright 2020 National Institute of Advanced Industrial Science
# and Technology, Japan
# Copyright 2019 Intelligent Robotics Lab
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
# Author: Christian Llanes <christian.llanes@gatech.edu>

include(GNUInstallDirs)

find_library(
    LIBNATNET_SDK_LIBRARY
    NAMES NatNet)

find_path(
  LIBNATNET_SDK_INCLUDE_DIR
  NAMES NatNetClient.h NatNetCAPI.h NatNetTypes.h
  PATHS /usr/local/include/NatNetSDK)

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(NATNETSDK_CPP DEFAULT_MSG
                                  LIBNATNET_SDK_LIBRARY
                                  LIBNATNET_SDK_INCLUDE_DIR)

mark_as_advanced(LIBNATNET_SDK_LIBRARY LIBNATNET_SDK_INCLUDE_DIR)

if(LIBNATNET_SDK_FOUND AND NOT TARGET NatNet::NatNet)
  add_library(NatNet::NatNet SHARED IMPORTED)
  set_target_properties(
    NatNet::NatNet
    PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${LIBNATNET_SDK_INCLUDE_DIR}"
      IMPORTED_LOCATION ${LIBNATNET_SDK_LIBRARY})
endif()