/*
 * See files AUTHORS and LICENSE for copyright details.
 */

#if BOOST_VERSION <= 106600
#include <boost/functional/hash.hpp>
#else
#include <boost/container_hash/hash.hpp>
#endif

#include "tchecker/zg/transition_ha.hh"

namespace tchecker {

namespace zg_ha {

bool operator==(tchecker::zg_ha::transition_t const & t1, tchecker::zg_ha::transition_t const & t2)
{
  return tchecker::ta_ha::operator==(t1, t2);
}

bool shared_equal_to(tchecker::zg_ha::transition_t const & t1, tchecker::zg_ha::transition_t const & t2)
{
  return tchecker::ta_ha::shared_equal_to(t1, t2);
}

std::size_t hash_value(tchecker::zg_ha::transition_t const & t) { return tchecker::ta_ha::hash_value(t); }

std::size_t shared_hash_value(tchecker::zg_ha::transition_t const & t) { return tchecker::ta_ha::shared_hash_value(t); }

int lexical_cmp(tchecker::zg_ha::transition_t const & t1, tchecker::zg_ha::transition_t const & t2)
{
  return tchecker::ta_ha::lexical_cmp(t1, t2);
}

} // end of namespace zg_ha

} // end of namespace tchecker
