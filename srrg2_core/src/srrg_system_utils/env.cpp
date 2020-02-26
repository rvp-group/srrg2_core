#include <string>
#include <map>
#include <iostream>

extern char** environ;

namespace srrg2_core {

  std::map<std::string,std::string> _env_map;

  void initEnvMap(){
    _env_map.clear();
    char** e=environ;
    while(*e) {
      std::string key_value(*e);
      ++e;
      auto pos=key_value.find_first_of('=');
      if (pos==std::string::npos)
        continue;
      std::string key=key_value.substr(0,pos);
      std::string value=key_value.substr(pos+1);
      _env_map.insert(std::make_pair(key,value));
    }
  }

  const std::map<std::string, std::string>& envMap() {return _env_map;}

    
  void replaceEnvTags(std::string& str) {
    initEnvMap();
    using namespace std;
    string::iterator begin_tag=str.end();
    for (string::iterator it=str.begin();it!=str.end();it++) {
      if (*it=='<') {
        begin_tag=it;
        continue;
      }
      if (*it=='>'&&begin_tag<it) {
        std::string key=str.substr(begin_tag+1-str.begin(),it-begin_tag-1);
        auto k_it=_env_map.find(key);
        std::string replacement("");
        if (k_it!=_env_map.end()) {
          replacement=k_it->second;
        }
        size_t newpos=begin_tag-str.begin()+replacement.length()-1;
        str.replace(begin_tag,it+1,replacement);
        it=str.begin()+newpos;
      }
    }
  }
  
}
