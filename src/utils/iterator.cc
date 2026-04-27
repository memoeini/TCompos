/*
 * This file is part of the TCompos tool, which builds upon the TChecker framework.
 *
 * See files AUTHORS and LICENSE for copyright details.
 *
 */

#include <limits>

#include "tchecker/utils/iterator.hh"

namespace tchecker {

/* end_iterator_t */

bool operator==(tchecker::end_iterator_t const & it1, tchecker::end_iterator_t const & it2) { return (&it1 == &it2); }

bool operator!=(tchecker::end_iterator_t const & it1, tchecker::end_iterator_t const & it2) { return !(it1 == it2); }

} // namespace tchecker