/*
 * MoUT.hpp
 * Copyright 2013 University of Massachusetts Lowell
 * The Museum of Useful Things contains code that does useful, but not terribly specific, things.
 */

#ifndef MOUT_HPP_
#define MOUT_HPP_

#include <string>
#include <sstream>
#include <iostream>

/*!
 * \brief Convert a type into a string in a base
 * For example: to_string<long>(123456, std::oct) returns the string "361100"
 */
template <class T>
std::string toString(T t, std::ios_base & (*f)(std::ios_base&))
{
  std::ostringstream oss;
  oss << f << t;
  return oss.str();
}

/*!
 * \brief Debugging/logging messages
 */
void debug(std::string message)
{
#ifdef DEBUG
	std::cout << message << std::endl;
#endif
}

#endif /* MOUT_HPP_ */
