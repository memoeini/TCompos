/*
 * See files AUTHORS and LICENSE for copyright details.
 */

#ifndef TCHECKER_SYSTEM_OUTPUT_HH
#define TCHECKER_SYSTEM_OUTPUT_HH

#include <iostream>
#include <string>

#include "tchecker/basictypes.hh"
#include "tchecker/system/attribute.hh"
#include "tchecker/system/clock.hh"
#include "tchecker/system/edge.hh"
#include "tchecker/system/system.hh"

/*!
 \file output.hh
 \brief System output functions
 */

namespace tchecker {

namespace system {

/*!
 \brief Output a system following the TChecker syntax
 \param os : output stream
 \param s : system
 \post s has been output to os following the TChecker syntax
*/
void output_tck(std::ostream & os, tchecker::system::system_t const & s);

/*!
 \brief Output a system following the dot graphviz syntax
 \param os : output stream
 \param s : system
 \param delimiter : separator between process name and location name for node
 names
 \post s has been output to os following the dot graphviz syntax
 \throw std::runtime_error : if a location or an edge in s has an attribute "label"
*/
void output_dot(std::ostream & os, tchecker::system::system_t const & s, std::string const & delimiter);

/*!
 \brief Output a system following the JSON syntax
 \param os : output stream
 \param s : system
 \param delimiter : separator between process name and location name for node
 names
 \post s has been output to os following the dot graphviz syntax
 \throw std::runtime_error : if a location or an edge in s has an attribute "label"
*/
void output_json(std::ostream & os, tchecker::system::system_t const & s, std::string const & delimiter);

} // end of namespace system

} // end of namespace tchecker

#endif // TCHECKER_SYSTEM_OUTPUT_HH
