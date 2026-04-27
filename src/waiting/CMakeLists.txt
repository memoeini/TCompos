# This file is part of the TCompos tool, which builds upon the TChecker framework.
#
# See files AUTHORS and LICENSE for copyright details.

set(WAITING_SRC
${CMAKE_CURRENT_SOURCE_DIR}/waiting.cc
${TCHECKER_INCLUDE_DIR}/tchecker/waiting/pqueue.hh
${TCHECKER_INCLUDE_DIR}/tchecker/waiting/queue.hh
${TCHECKER_INCLUDE_DIR}/tchecker/waiting/stack.hh
${TCHECKER_INCLUDE_DIR}/tchecker/waiting/waiting.hh
PARENT_SCOPE)
