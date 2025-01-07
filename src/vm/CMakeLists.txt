# This file is a part of the TChecker project.
#
# See files AUTHORS and LICENSE for copyright details.

set(VM_SRC
${CMAKE_CURRENT_SOURCE_DIR}/compilers.cc
${CMAKE_CURRENT_SOURCE_DIR}/compilers_ha.cc
${CMAKE_CURRENT_SOURCE_DIR}/vm.cc
${CMAKE_CURRENT_SOURCE_DIR}/vm_ha.cc
${TCHECKER_INCLUDE_DIR}/tchecker/vm/compilers.hh
${TCHECKER_INCLUDE_DIR}/tchecker/vm/compilers_ha.hh
${TCHECKER_INCLUDE_DIR}/tchecker/vm/vm.hh
${TCHECKER_INCLUDE_DIR}/tchecker/vm/vm_ha.hh
PARENT_SCOPE)
