# This file is part of the TCompos tool, which builds upon the TChecker framework.
#
# See files AUTHORS and LICENSE for copyright details.

set(CLOCKBOUNDS_SRC
${CMAKE_CURRENT_SOURCE_DIR}/clockbounds.cc
${CMAKE_CURRENT_SOURCE_DIR}/solver.cc
${TCHECKER_INCLUDE_DIR}/tchecker/clockbounds/clockbounds.hh
${TCHECKER_INCLUDE_DIR}/tchecker/clockbounds/solver.hh
PARENT_SCOPE)
