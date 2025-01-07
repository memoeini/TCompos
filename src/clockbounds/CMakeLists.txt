# This file is a part of the TChecker project.
#
# See files AUTHORS and LICENSE for copyright details.

set(CLOCKBOUNDS_SRC
${CMAKE_CURRENT_SOURCE_DIR}/clockbounds.cc
${CMAKE_CURRENT_SOURCE_DIR}/solver.cc
${CMAKE_CURRENT_SOURCE_DIR}/solver_ha.cc
${TCHECKER_INCLUDE_DIR}/tchecker/clockbounds/clockbounds.hh
${TCHECKER_INCLUDE_DIR}/tchecker/clockbounds/solver.hh
${TCHECKER_INCLUDE_DIR}/tchecker/clockbounds/solver_ha.hh
PARENT_SCOPE)
