/*
 * See files AUTHORS and LICENSE for copyright details.
 */

#include "tchecker/zg/semantics.hh"
#include "tchecker/dbm/dbm.hh"

namespace tchecker {

namespace zg {

/* standard_semantics_t */

tchecker::state_status_t standard_semantics_t::initial(tchecker::dbm::db_t * dbm, tchecker::clock_id_t dim, bool delay_allowed,
                                                       tchecker::clock_constraint_container_t const & invariant)
{
  tchecker::dbm::zero(dbm, dim);

  if (tchecker::dbm::constrain(dbm, dim, invariant) == tchecker::dbm::EMPTY)
    return tchecker::STATE_CLOCKS_SRC_INVARIANT_VIOLATED;

  return tchecker::STATE_OK;
}

tchecker::state_status_t standard_semantics_t::final(tchecker::dbm::db_t * dbm, tchecker::clock_id_t dim, bool delay_allowed,
                                                     tchecker::clock_constraint_container_t const & invariant)
{
  tchecker::dbm::universal_positive(dbm, dim);

  if (tchecker::dbm::constrain(dbm, dim, invariant) == tchecker::dbm::EMPTY)
    return tchecker::STATE_CLOCKS_TGT_INVARIANT_VIOLATED;

  return tchecker::STATE_OK;
}

tchecker::state_status_t standard_semantics_t::next(tchecker::dbm::db_t * dbm, tchecker::clock_id_t dim, bool src_delay_allowed,
                                                    tchecker::clock_constraint_container_t const & src_invariant,
                                                    tchecker::clock_constraint_container_t const & guard,
                                                    tchecker::clock_reset_container_t const & clkreset, bool tgt_delay_allowed,
                                                    tchecker::clock_constraint_container_t const & tgt_invariant)
{
  if (tchecker::dbm::constrain(dbm, dim, src_invariant) == tchecker::dbm::EMPTY)
    return tchecker::STATE_CLOCKS_SRC_INVARIANT_VIOLATED;

  if (src_delay_allowed) {
    tchecker::dbm::open_up(dbm, dim);

    if (tchecker::dbm::constrain(dbm, dim, src_invariant) == tchecker::dbm::EMPTY)
      return tchecker::STATE_CLOCKS_SRC_INVARIANT_VIOLATED; // should never occur
  }

  if (tchecker::dbm::constrain(dbm, dim, guard) == tchecker::dbm::EMPTY)
    return tchecker::STATE_CLOCKS_GUARD_VIOLATED;

  tchecker::dbm::reset(dbm, dim, clkreset);

  if (tchecker::dbm::constrain(dbm, dim, tgt_invariant) == tchecker::dbm::EMPTY)
    return tchecker::STATE_CLOCKS_TGT_INVARIANT_VIOLATED;

  return tchecker::STATE_OK;
}

tchecker::state_status_t standard_semantics_t::prev(tchecker::dbm::db_t * dbm, tchecker::clock_id_t dim, bool src_delay_allowed,
                                                    tchecker::clock_constraint_container_t const & src_invariant,
                                                    tchecker::clock_constraint_container_t const & guard,
                                                    tchecker::clock_reset_container_t const & clkreset, bool tgt_delay_allowed,
                                                    tchecker::clock_constraint_container_t const & tgt_invariant)
{
  // prev(dbm) = free_clocks(dbm & tgt_invariant & constraints(clkreset), left_clocks(clkreset)) & guard & src_invariant
  // finally, if src_delay_allowed: opened down and intersected with src_invariant again
  if (tchecker::dbm::constrain(dbm, dim, tgt_invariant) == tchecker::dbm::EMPTY)
    return tchecker::STATE_CLOCKS_TGT_INVARIANT_VIOLATED;

  tchecker::clock_constraint_container_t resets_as_constraints;
  tchecker::clock_resets_to_constraints(clkreset, resets_as_constraints);
  if (tchecker::dbm::constrain(dbm, dim, resets_as_constraints) == tchecker::dbm::EMPTY)
    return tchecker::STATE_CLOCKS_RESET_FAILED;

  tchecker::dbm::free_clock(dbm, dim, clkreset);

  if (tchecker::dbm::constrain(dbm, dim, guard) == tchecker::dbm::EMPTY)
    return tchecker::STATE_CLOCKS_GUARD_VIOLATED;

  if (tchecker::dbm::constrain(dbm, dim, src_invariant) == tchecker::dbm::EMPTY)
    return tchecker::STATE_CLOCKS_SRC_INVARIANT_VIOLATED;

  if (src_delay_allowed) {
    tchecker::dbm::open_down(dbm, dim);

    if (tchecker::dbm::constrain(dbm, dim, src_invariant) == tchecker::dbm::EMPTY)
      return tchecker::STATE_CLOCKS_SRC_INVARIANT_VIOLATED;
  }

  return tchecker::STATE_OK;
}

/* elapsed_semantics_t */

tchecker::state_status_t elapsed_semantics_t::initial(tchecker::dbm::db_t * dbm, tchecker::clock_id_t dim, bool delay_allowed,
                                                      tchecker::clock_constraint_container_t const & invariant)
{
  tchecker::dbm::zero(dbm, dim);

  if (tchecker::dbm::constrain(dbm, dim, invariant) == tchecker::dbm::EMPTY)
    return tchecker::STATE_CLOCKS_SRC_INVARIANT_VIOLATED;

  if (delay_allowed) {
    tchecker::dbm::open_up(dbm, dim);

    if (tchecker::dbm::constrain(dbm, dim, invariant) == tchecker::dbm::EMPTY)
      return tchecker::STATE_CLOCKS_SRC_INVARIANT_VIOLATED;
  }

  return tchecker::STATE_OK;
}

tchecker::state_status_t elapsed_semantics_t::final(tchecker::dbm::db_t * dbm, tchecker::clock_id_t dim, bool delay_allowed,
                                                    tchecker::clock_constraint_container_t const & invariant)
{
  tchecker::dbm::universal_positive(dbm, dim);

  if (tchecker::dbm::constrain(dbm, dim, invariant) == tchecker::dbm::EMPTY)
    return tchecker::STATE_CLOCKS_TGT_INVARIANT_VIOLATED;

  return tchecker::STATE_OK;
}

tchecker::state_status_t elapsed_semantics_t::next(tchecker::dbm::db_t * dbm, tchecker::clock_id_t dim, bool src_delay_allowed,
                                                   tchecker::clock_constraint_container_t const & src_invariant,
                                                   tchecker::clock_constraint_container_t const & guard,
                                                   tchecker::clock_reset_container_t const & clkreset, bool tgt_delay_allowed,
                                                   tchecker::clock_constraint_container_t const & tgt_invariant)
{
  if (tchecker::dbm::constrain(dbm, dim, src_invariant) == tchecker::dbm::EMPTY)
    return tchecker::STATE_CLOCKS_SRC_INVARIANT_VIOLATED;

  if (tchecker::dbm::constrain(dbm, dim, guard) == tchecker::dbm::EMPTY)
    return tchecker::STATE_CLOCKS_GUARD_VIOLATED;

  tchecker::dbm::reset(dbm, dim, clkreset);

  if (tchecker::dbm::constrain(dbm, dim, tgt_invariant) == tchecker::dbm::EMPTY)
    return tchecker::STATE_CLOCKS_TGT_INVARIANT_VIOLATED;

  if (tgt_delay_allowed) {
    tchecker::dbm::open_up(dbm, dim);

    if (tchecker::dbm::constrain(dbm, dim, tgt_invariant) == tchecker::dbm::EMPTY)
      return tchecker::STATE_CLOCKS_TGT_INVARIANT_VIOLATED;
  }

  return tchecker::STATE_OK;
}

tchecker::state_status_t elapsed_semantics_t::prev(tchecker::dbm::db_t * dbm, tchecker::clock_id_t dim, bool src_delay_allowed,
                                                   tchecker::clock_constraint_container_t const & src_invariant,
                                                   tchecker::clock_constraint_container_t const & guard,
                                                   tchecker::clock_reset_container_t const & clkreset, bool tgt_delay_allowed,
                                                   tchecker::clock_constraint_container_t const & tgt_invariant)
{
  if (tchecker::dbm::constrain(dbm, dim, tgt_invariant) == tchecker::dbm::EMPTY)
    return tchecker::STATE_CLOCKS_TGT_INVARIANT_VIOLATED;

  if (tgt_delay_allowed) {
    tchecker::dbm::open_down(dbm, dim);

    if (tchecker::dbm::constrain(dbm, dim, tgt_invariant) == tchecker::dbm::EMPTY)
      return tchecker::STATE_CLOCKS_TGT_INVARIANT_VIOLATED;
  }

  tchecker::clock_constraint_container_t resets_as_constraints;
  tchecker::clock_resets_to_constraints(clkreset, resets_as_constraints);
  if (tchecker::dbm::constrain(dbm, dim, resets_as_constraints) == tchecker::dbm::EMPTY)
    return tchecker::STATE_CLOCKS_RESET_FAILED;

  tchecker::dbm::free_clock(dbm, dim, clkreset);

  if (tchecker::dbm::constrain(dbm, dim, guard) == tchecker::dbm::EMPTY)
    return tchecker::STATE_CLOCKS_GUARD_VIOLATED;

  if (tchecker::dbm::constrain(dbm, dim, src_invariant) == tchecker::dbm::EMPTY)
    return tchecker::STATE_CLOCKS_SRC_INVARIANT_VIOLATED;

  return tchecker::STATE_OK;
}

/* factory */

tchecker::zg::semantics_t * semantics_factory(enum semantics_type_t semantics)
{
  switch (semantics) {
  case tchecker::zg::STANDARD_SEMANTICS:
    return new tchecker::zg::standard_semantics_t{};
  case tchecker::zg::ELAPSED_SEMANTICS:
    return new tchecker::zg::elapsed_semantics_t{};
  default:
    throw std::invalid_argument("Unknown zone semantics");
  }
}

} // end of namespace zg

} // end of namespace tchecker