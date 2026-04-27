# This file is part of the TCompos tool, which builds upon the TChecker framework.
#
# See files AUTHORS and LICENSE for copyright details.

set(COUVREUR_SCC_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/graph.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/stats.cc
    ${TCHECKER_INCLUDE_DIR}/tchecker/algorithms/ndfs/graph.hh
    ${TCHECKER_INCLUDE_DIR}/tchecker/algorithms/ndfs/stats.hh
    PARENT_SCOPE)
