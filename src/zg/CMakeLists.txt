# This file is a part of the TChecker project.
#
# See files AUTHORS and LICENSE for copyright details.

set(ZG_SRC
${CMAKE_CURRENT_SOURCE_DIR}/extrapolation.cc
${CMAKE_CURRENT_SOURCE_DIR}/extrapolation_compos.cc
${CMAKE_CURRENT_SOURCE_DIR}/extrapolation_ha.cc
${CMAKE_CURRENT_SOURCE_DIR}/path.cc
${CMAKE_CURRENT_SOURCE_DIR}/path_ha.cc
${CMAKE_CURRENT_SOURCE_DIR}/semantics.cc
${CMAKE_CURRENT_SOURCE_DIR}/state.cc
${CMAKE_CURRENT_SOURCE_DIR}/transition.cc
${CMAKE_CURRENT_SOURCE_DIR}/transition_ha.cc
${CMAKE_CURRENT_SOURCE_DIR}/zg.cc
${CMAKE_CURRENT_SOURCE_DIR}/zg_compos.cc
${CMAKE_CURRENT_SOURCE_DIR}/zg_ha.cc
${CMAKE_CURRENT_SOURCE_DIR}/zone.cc
${TCHECKER_INCLUDE_DIR}/tchecker/zg/allocators.hh
${TCHECKER_INCLUDE_DIR}/tchecker/zg/allocators_ha.hh
${TCHECKER_INCLUDE_DIR}/tchecker/zg/extrapolation.hh
${TCHECKER_INCLUDE_DIR}/tchecker/zg/extrapolation_compos.hh
${TCHECKER_INCLUDE_DIR}/tchecker/zg/extrapolation_ha.hh
${TCHECKER_INCLUDE_DIR}/tchecker/zg/path.hh
${TCHECKER_INCLUDE_DIR}/tchecker/zg/path_ha.hh
${TCHECKER_INCLUDE_DIR}/tchecker/zg/semantics.hh
${TCHECKER_INCLUDE_DIR}/tchecker/zg/state.hh
${TCHECKER_INCLUDE_DIR}/tchecker/zg/transition.hh
${TCHECKER_INCLUDE_DIR}/tchecker/zg/transition_ha.hh
${TCHECKER_INCLUDE_DIR}/tchecker/zg/zg.hh
${TCHECKER_INCLUDE_DIR}/tchecker/zg/zg_compos.hh           # TODO: NEWLY ADDED
${TCHECKER_INCLUDE_DIR}/tchecker/zg/zg_ha.hh           # TODO: NEWLY ADDED
${TCHECKER_INCLUDE_DIR}/tchecker/zg/zone.hh
PARENT_SCOPE)
