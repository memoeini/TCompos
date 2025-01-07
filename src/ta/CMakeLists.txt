# This file is a part of the TChecker project.
#
# See files AUTHORS and LICENSE for copyright details.

set(TA_SRC
${CMAKE_CURRENT_SOURCE_DIR}/state.cc
${CMAKE_CURRENT_SOURCE_DIR}/static_analysis.cc
${CMAKE_CURRENT_SOURCE_DIR}/system.cc
${CMAKE_CURRENT_SOURCE_DIR}/system_ha.cc
${CMAKE_CURRENT_SOURCE_DIR}/ta.cc
${CMAKE_CURRENT_SOURCE_DIR}/ta_ha.cc
${CMAKE_CURRENT_SOURCE_DIR}/transition.cc
${CMAKE_CURRENT_SOURCE_DIR}/transition_ha.cc
${TCHECKER_INCLUDE_DIR}/tchecker/ta/allocators.hh
${TCHECKER_INCLUDE_DIR}/tchecker/ta/allocators_ha.hh
${TCHECKER_INCLUDE_DIR}/tchecker/ta/edges_iterators.hh
${TCHECKER_INCLUDE_DIR}/tchecker/ta/state.hh
${TCHECKER_INCLUDE_DIR}/tchecker/ta/static_analysis.hh
${TCHECKER_INCLUDE_DIR}/tchecker/ta/system.hh
${TCHECKER_INCLUDE_DIR}/tchecker/ta/system_ha.hh
${TCHECKER_INCLUDE_DIR}/tchecker/ta/ta.hh
${TCHECKER_INCLUDE_DIR}/tchecker/ta/ta_ha.hh
${TCHECKER_INCLUDE_DIR}/tchecker/ta/transition.hh
${TCHECKER_INCLUDE_DIR}/tchecker/ta/transition_ha.hh
PARENT_SCOPE)
