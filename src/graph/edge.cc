/*
 * See files AUTHORS and LICENSE for copyright details.
 */

#include "tchecker/graph/edge.hh"

namespace tchecker {

namespace graph {

edge_vedge_t::edge_vedge_t(tchecker::vedge_sptr_t const & vedge) : _vedge(vedge) {}

edge_vedge_t::edge_vedge_t(tchecker::const_vedge_sptr_t const & vedge) : _vedge(vedge) {}

edge_rel_transition::edge_rel_transition (zg_ha::transition_t const & transition) : _transition(transition) {}

} // namespace graph

} // namespace tchecker