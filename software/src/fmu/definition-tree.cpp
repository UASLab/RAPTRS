/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor
*/

#include "definition-tree.h"

/* Needed for std::map */
extern "C"{
  int _getpid(){ return -1;}
  int _kill(int pid, int sig){ return -1; }
}

/* Defines new definition tree member given a variable definition struct */
void DefinitionTree::DefineMember(std::string Name,struct VariableDefinition &VariableDefinitionRef) {
  Data_[Name] = VariableDefinitionRef;
}

/* Initializes new empty definition tree member given a name */
void DefinitionTree::InitMember(std::string Name) {
  struct VariableDefinition TempDef;
  Data_[Name] = TempDef;
}

/* Defines new definition tree member given a name, value, description, datalog, and telemetry */
void DefinitionTree::InitMember(std::string Name,std::variant<uint64_t*,uint32_t*,uint16_t*,uint8_t*,int64_t*,int32_t*,int16_t*,int8_t*,float*, double*> ValuePtr) {
  struct VariableDefinition TempDef;
  Data_[Name] = TempDef;
  Data_[Name].ValuePtr = ValuePtr;
}

/* Sets value for an existing definition tree member */
void DefinitionTree::SetValuePtr(std::string Name,std::variant<uint64_t*,uint32_t*,uint16_t*,uint8_t*,int64_t*,int32_t*,int16_t*,int8_t*,float*, double*> ValuePtr) {
  Data_[Name].ValuePtr = ValuePtr;
}

/* Gets variable struct for a definition tree member */
void DefinitionTree::GetMember(std::string Name,struct VariableDefinition *VariableDefinitionPtr) {
  *VariableDefinitionPtr = Data_[Name];
}

/* Gets number of definition tree members at a given tree level */
size_t DefinitionTree::Size(std::string Name) {
  size_t retval = 0;
  for (auto const& element : Data_) {
    if (element.first.find(Name)!=std::string::npos) {
      retval++;
    }
  }
  return retval;
}

/* Gets list of definition tree member keys at a given tree level */
void DefinitionTree::GetKeys(std::string Name,std::vector<std::string> *KeysPtr) {
  KeysPtr->clear();
  for (auto const& element : Data_) {
    if (element.first.find(Name)!=std::string::npos) {
      KeysPtr->push_back(element.first);
    }
  }
}

/* Erases definition tree members at a given tree level */
void DefinitionTree::Erase(std::string Name) {
  for (auto const& element : Data_) {
    if (element.first.find(Name)!=std::string::npos) {
      Data_.erase(element.first);
    }
  }
}

/* Clears the full definition tree */
void DefinitionTree::Clear() {
  Data_.clear();
}
