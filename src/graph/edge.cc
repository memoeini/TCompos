/*
 * This file is part of the TCompos tool, which builds upon the TChecker framework.
 *
 * See files AUTHORS and LICENSE for copyright details.
 *
 */

#include "tchecker/graph/edge.hh"

namespace tchecker {

namespace graph {

edge_vedge_t::edge_vedge_t(tchecker::vedge_sptr_t const & vedge) : _vedge(vedge) {}

edge_vedge_t::edge_vedge_t(tchecker::const_vedge_sptr_t const & vedge) : _vedge(vedge) {}

// edge_rel_transition::edge_rel_transition (zg::transition_t const & transition, tchecker::clock_constraint_container_t  guard_cont) : _transition(transition), _guard_cont(guard_cont)  {}
edge_rel_transition::edge_rel_transition (tchecker::clock_constraint_container_t  guard_cont) : _guard_cont(guard_cont)  {}

} // namespace graph

} // namespace tchecker