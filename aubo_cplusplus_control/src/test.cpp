#include <iostream>
#include <string>
#include <regex>

int main()
{
    // decimal number in fixed format (99 -99. +99.999 -.999 -3 etc.)
    // prefixed by beginning of string or white space
    // with an optional + or -
    // suffixed by end of string or white space
    std::regex num_re( "(?:^|\\s)[+-]?(?:\\d+\\.?\\d*|\\d*\\.?\\d+)(?:\\s|$)" ) ;

    const char* eva = "I a.m ++7.8 6nu+m-7ber+8 -1000.235 abcd" ;
    
    std::cmatch match ;
    if( std::regex_search( eva, match, num_re ) )
    {
        const double value = std::stod( match[0] ) ;
        std::cout << "found " << std::fixed << value << '\n' ;
    }
    else std::cout << "regex_search failed\n" ;
}
