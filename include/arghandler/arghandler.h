#ifndef ARGHANDLER_HPP
#define ARGHANDLER_HPP

#include <cstdlib>
#include <iostream>
#include <vector>

using namespace std;


class arghandler {
  public:

    const string identifierPrefix = "--";

    arghandler();

    bool processArgs(int argc, char** argv);
    void coutArgs();
    bool hasIdentifier(string ident);
    template<typename T> T getValue(string ident);
    vector<pair<string, string>> getValuesByPrefix(string prefix);

    bool isInteger(string s);
    bool isDouble(string s);
    bool isString(string s);
    bool isBoolean(string s);

    int toInteger(string val);
    double toDouble(string val);
    bool toBoolean(string val);
    string getValueAsString(string ident);

  private:

    bool isIdentifier(string str);
    bool isIdentifierSet = false;
    bool isValueSet = false;
    std::vector<std::pair<string, string>> args{};


};

#endif  /* ARGHANDLER_HPP */

