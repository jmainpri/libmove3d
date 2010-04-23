/**
 * @defgroup string_func Functions for C++ STL strings.
 * Function library to improve functionality of C++ STL strings.
 * The library includes type conversion functions and additional functions like trim.
 *
 * @author Ron Alterovitz
 * @version 2007/03/13 Ron Alterovitz Initial version
 * @{ // Start of defgroup
 */

#ifndef STRING_FUNC_H
#define STRING_FUNC_H

#include <string>
#include <cctype>
#include <algorithm>
#include <sstream>

/**
 * Convert a string to upper case.
 * @param s The input string.
 * @return The string in upper case.
 */
inline std::string to_upper(const std::string& s) {

	std::string s2 = s;
	std::transform (s2.begin(), s2.end(), s2.begin(), (int(*)(int)) std::toupper);
	return s2;
}


/**
 * Convert a string to lower case.
 * @param s The input string.
 * @return The string in lower case.
 */
inline std::string to_lower(const std::string& s) {

	std::string s2 = s;
	std::transform (s2.begin(), s2.end(), s2.begin(), (int(*)(int))std::tolower);
	return s2;
}


/**
 * Test if an input string starts with a particular suffix.
 * @param s The input string.
 * @param prefix The prefix.
 * @return True iff the string s starts with the string prefix.
 */
inline bool starts_with(const std::string& s, const std::string& prefix) {

	if( s.length() < prefix.length() )
		return false;
	return ( s.substr(0, prefix.length()) == prefix );
}


/**
 * Test if an input string ends with a particular suffix.
 * @param s The input string.
 * @param suffix The suffix.
 * @return True iff the string s ends with the string suffix.
 */
inline bool ends_with(const std::string& s, const std::string& suffix) {

	if( s.length() < suffix.length() )
		return false;
	return ( s.substr(s.length()-suffix.length(), suffix.length()) == suffix );
}


/**
 * Return a string representation of a C-style character array string.
 * @param s The input character array.
 * @return A string representation of the character array.
 */
inline std::string to_string(const char *s) {

	std::stringstream s2;
	s2 << s;
	return s2.str();
}


/**
 * Return a string representation of a character.
 * @param c The input character.
 * @return A string representation of the character.
 */
inline std::string to_string(char c) {

	std::stringstream s;
	s << c;
	return s.str();
}


/**
 * Return a string representation of a double.
 * @param f The input double.
 * @return A string representation of the input double.
 */
inline std::string to_string(double f) {

	std::stringstream s;
	s << f;
	return s.str();
}


/**
 * Return a string representation of a float.
 * @param f The input float.
 * @return A string representation of the input float.
 */
inline std::string to_string(float f) {

	std::stringstream s;
	s << f;
	return s.str();
}


/**
 * Return a string representation of an unigned integer.
 * @param i The input unsigned integer.
 * @return A string representation of the input unsigned integer.
 */
inline std::string to_string(unsigned int i) {

	std::stringstream s;
	s << i;
	return s.str();
}


/**
 * Return a string representation of an integer.
 * @param i The input integer.
 * @return A string representation of the input integer.
 */
inline std::string to_string(int i) {

	std::stringstream s;
	s << i;
	return s.str();
}


/**
 * Return a string representation of an unigned long integer.
 * @param i The input unsigned long integer.
 * @return A string representation of the input unsigned long integer.
 */
inline std::string to_string(unsigned long i) {

	std::stringstream s;
	s << i;
	return s.str();
}


/**
 * Return a string representation of a long integer.
 * @param i The input long integer.
 * @return A string representation of the input long integer.
 */
inline std::string to_string(long i) {

	std::stringstream s;
	s << i;
	return s.str();
}


/**
 * Remove trailing and preceeding spaces from a string.
 * @param s The input string.
 * @return The input string with spaces removed from the beginning and end.
 */
inline std::string trim(const std::string& s) {
	std::string str = s;
	std::string::size_type pos = str.find_last_not_of(' ');
	if(pos != std::string::npos) {
		str.erase(pos + 1);
		pos = str.find_first_not_of(' ');
		if(pos != std::string::npos)
			str.erase(0, pos);
	}
	else
		str.erase(str.begin(), str.end());
	return str;
}

#endif

/** \} */ // End of defgroup
