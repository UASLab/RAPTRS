/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#ifndef DEFINITION_TREE_HXX_
#define DEFINITION_TREE_HXX_

#include <Arduino.h>
#include <vector>
#include <variant>
#include <map>

class DefinitionTree {
  public:
    // variable definition
    struct VariableDefinition {
      std::variant<uint64_t*,uint32_t*,uint16_t*,uint8_t*,int64_t*,int32_t*,int16_t*,int8_t*,float*, double*> ValuePtr;
    };
    void DefineMember(std::string Name,struct VariableDefinition &VariableDefinitionRef);
    void InitMember(std::string Name);
    void InitMember(std::string Name,std::variant<uint64_t*,uint32_t*,uint16_t*,uint8_t*,int64_t*,int32_t*,int16_t*,int8_t*,float*, double*> ValuePtr);
    void SetValuePtr(std::string Name,std::variant<uint64_t*,uint32_t*,uint16_t*,uint8_t*,int64_t*,int32_t*,int16_t*,int8_t*,float*, double*> ValuePtr);
    /* Gets pointer to value for a definition tree member */
    template <typename T> T GetValuePtr(std::string Name) {
      if(auto val = std::get_if<T>(&Data_[Name].ValuePtr)) {
        return *val;
      } else {
        return NULL;
      }
    }
    void GetMember(std::string Name,struct VariableDefinition *VariableDefinitionPtr);
    size_t Size(std::string Name);
    void GetKeys(std::string Name,std::vector<std::string> *KeysPtr);
    void Erase(std::string Name);
    void Clear();
  private:
    std::map<std::string,VariableDefinition> Data_;
};

#endif
