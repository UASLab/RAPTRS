/*
definition-tree.hxx
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2018 Bolder Flight Systems
Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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
