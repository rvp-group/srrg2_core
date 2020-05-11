#pragma once
#include <string>
#include <sstream>
#include <iostream>
#include <map>
#include <vector>

namespace srrg2_core {

  class ParseCommandLine;
  
  struct ArgumentFlag {
    friend class ParseCommandLine;
    ArgumentFlag(ParseCommandLine* cmd_line,
                 const std::string& option_,
                 const std::string& long_option_,
                 const std::string& explanation_,
                 int num_fields_=0);
      
    virtual char** parseArgument(char** argv);
    virtual std::string stringValue() const;
    inline bool isSet() const {return _is_set;}
    inline const std::string& option() const {return _option;}
    inline const std::string& longOption() const {return _long_option;}
    inline const std::string& explanation() const {return _explanation;}
    inline const int numFields() const {return _num_fields;}
    
  protected:
    std::string _option;
    std::string _long_option;
    std::string _explanation;
    int _num_fields;
    bool _is_set=false;
  };

  struct ParseCommandLine{

    ParseCommandLine(char** argv_, const char** what_does_=0);
    void addArgument(ArgumentFlag* arg);
    std::string options();
    std::string summary();
    inline const std::vector<std::string>& lastParsedArgs() const {return _last_parsed_args;}
    void parse();
  protected:
    std::string prog_name;
    char** _argv;
    const char** _what_does;
    std::map<std::string,ArgumentFlag*> _arg_map;
    std::map<std::string,ArgumentFlag*> _long_arg_map;
    std::vector<std::string> _last_parsed_args;

    ArgumentFlag _help_arg;
  };

  template <typename ArgumentType_>
  struct Argument_: public ArgumentFlag {
      
    using ArgumentType=ArgumentType_;
    const ArgumentType_& value() {return _value;}
    Argument_(ParseCommandLine* cmd_line,
              const std::string& option,
              const std::string& long_option,
              const std::string& explanation,
              const ArgumentType_& default_value,
              int num_fields=1):
      ArgumentFlag(cmd_line,option,long_option, explanation, num_fields),
      _value(default_value)
    {}

    char** parseArgument(char** argv) override {
      if (!*argv)
        return argv;
      std::istringstream is(*argv);
      is >> _value;
      return argv+1;
    }

    std::string stringValue() const override {
      std::ostringstream os;
      os << _value;
      return os.str();
    }
    
    inline const ArgumentType& value() const {return _value;}
  protected:
    ArgumentType _value;
  };

  using ArgumentInt=Argument_<int> ;
  using ArgumentFloat=Argument_<float> ;
  using ArgumentDouble=Argument_<double> ;
  using ArgumentString=Argument_<std::string>;
}
