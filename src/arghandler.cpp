#include <arghandler/arghandler.h>
#include <exception>
#include <utility>
#include <string>
#include <vector>

arghandler::arghandler() {
}

bool arghandler::processArgs(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        string arg(argv[i]);

        if (isIdentifier(arg)) {
            if (isIdentifierSet && !isValueSet) {
                this->args.back().second = "";
            }
            this->args.push_back(make_pair(arg.substr(this->identifierPrefix.
                                           length()), ""));
            isIdentifierSet = true;
            isValueSet = false;

        } else {
            if (isIdentifierSet) {
                args.back().second = arg;
                isIdentifierSet = false;
                isValueSet = false;
            } else {
                return false;
            }
        }
    }
    if (isIdentifierSet && !isValueSet) this->args.back().second = "";
}

void arghandler::coutArgs() {
    cout << "Arguments" << endl << endl;
    for (int i = 0; i < args.size(); ++i) {
        cout << args[i].first << ": " << args[i].second << endl;
    }
    cout << endl << endl;
}

bool arghandler::isInteger(std::string s) {
    if (s.empty() || ((!isdigit(s[0])) && (s[0] != '-') && (s[0] != '+')))
        return false;

    char * p;
    strtol(s.c_str(), &p, 10);

    return (*p == 0);
}

bool arghandler::isDouble(std::string s) {
    if (isInteger(s))return false;
    if (s.empty() || ((!isdigit(s[0])) && (s[0] != '-') && (s[0] != '+')))
        return false;

    char * p;
    strtod(s.c_str(), &p);

    return (*p == 0);
}

bool arghandler::isBoolean(std::string s) {
    return (s == "true" || s == "false");
}

bool arghandler::isString(std::string s) {
    return !(isBoolean(s) || isInteger(s) || isDouble(s));
}

template<typename T> T arghandler::getValue(string ident) {
    string val = this->getValueAsString(ident);
    char * p;

    if (val == "true" || val == "false") {
        return toBoolean(val);
    } else if (isInteger(val)) {
        return toInteger(val);
    } else if (isDouble(val)) {
        return toDouble(val);
    } else {
        return val;
    }
}

bool arghandler::hasIdentifier(string ident) {
    for (vector<pair < string, string>>::iterator it = args.begin(); it
            != args.end(); ++it) {
        if (it->first == ident) {
            return true;
        }
    }
    return false;
}

string arghandler::getValueAsString(string ident) {
    for (vector<pair < string, string>>::iterator it = args.begin(); it
            != args.end(); ++it) {
        if (it->first == ident) {
            return it->second;
        }
    }
    return NULL;
}

bool arghandler::toBoolean(string ident) {
    if (ident == "true") {
        return true;
    } else {
        return false;
    }
}

double arghandler::toDouble(string val) {
    char* p;
    return strtod(val.c_str(), &p);
}

int arghandler::toInteger(string val) {
    char* p;
    return strtol(val.c_str(), &p, 10);
}

vector<pair<string, string>> arghandler::getValuesByPrefix(string prefix) {
    vector<pair<string, string>> rtn;
    for (std::vector<pair < string, string>>::iterator it = args.begin();
            it != args.end(); ++it) {
        if (it->first.substr(0, prefix.length()) == prefix) {
            rtn.push_back(make_pair(it->first.substr(prefix.length()),
                                    it->second));
        }
    }
    return rtn;
}

bool arghandler::isIdentifier(string arg) {
    return (arg.substr(0, this->identifierPrefix.length()) ==
            this->identifierPrefix);
}
