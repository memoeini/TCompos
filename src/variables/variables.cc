/*
 * This file is part of the TCompos tool, which builds upon the TChecker framework.
 *
 * See files AUTHORS and LICENSE for copyright details.
 *
 */

#include "tchecker/variables/variables.hh"

namespace tchecker {

/* size_info_t */

size_info_t::size_info_t(unsigned int size) : _size(size)
{
  if (_size == 0)
    throw std::invalid_argument("Size should be greater than 0");
}

} // end of namespace tchecker
