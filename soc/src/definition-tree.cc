/*
definition-tree.cxx
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

#include <iostream>

#include "definition-tree.hxx"

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
void DefinitionTree::InitMember(std::string Name,std::variant<uint64_t*,uint32_t*,uint16_t*,uint8_t*,int64_t*,int32_t*,int16_t*,int8_t*,float*, double*> Value,std::string Description,bool Datalog,bool Telemetry) {
  struct VariableDefinition TempDef;
  Data_[Name] = TempDef;
  Data_[Name].Value = Value;
  Data_[Name].Description = Description;
  Data_[Name].Datalog = Datalog;
  Data_[Name].Telemetry = Telemetry;
}

/* Sets value for an existing definition tree member */
void DefinitionTree::SetValuePtr(std::string Name,std::variant<uint64_t*,uint32_t*,uint16_t*,uint8_t*,int64_t*,int32_t*,int16_t*,int8_t*,float*, double*> Value) {
  Data_[Name].Value = Value;
}

/* Sets description for an existing definition tree member */
void DefinitionTree::SetDescription(std::string Name,std::string Description) {
  Data_[Name].Description = Description;
}

/* Sets datalog status for an existing definition tree member */
void DefinitionTree::SetDatalog(std::string Name,bool Datalog) {
  Data_[Name].Datalog = Datalog;
}

/* Sets telemetry status for an existing definition tree member */
void DefinitionTree::SetTelemetry(std::string Name,bool Telemetry) {
  Data_[Name].Telemetry = Telemetry;
}

/* Gets description for a definition tree member */
std::string DefinitionTree::GetDescription(std::string Name) {
  return Data_[Name].Description;
}

/* Gets datalog status for a definition tree member */
bool DefinitionTree::GetDatalog(std::string Name) {
  return Data_[Name].Datalog;
}

/* Gets telemetry status for a definition tree member */
bool DefinitionTree::GetTelemetry(std::string Name) {
  return Data_[Name].Telemetry;
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

/* Gets list of definition tree member keys at a given tree level */
void DefinitionTree::GetChildren(std::string Name,std::vector<std::string> *KeysPtr) {
    KeysPtr->clear();
    for (auto const& element : Data_) {
        std::size_t pos = element.first.find(Name);
        if ( pos != std::string::npos) {
            std::string tail = element.first.substr(pos + 1);
            KeysPtr->push_back(tail);
        }
    }
}

/* Print list of definition tree member keys at a given tree level */
void DefinitionTree::PrettyPrint(std::string Prefix) {
    std::cout << Prefix << std::endl;
    for (auto const& element : Data_) {
        std::size_t pos = element.first.find(Prefix);
        if ( pos != std::string::npos) {
            std::string tail = element.first.substr(pos + 1);
            std::cout << "  " << tail << " (" << GetType(&element.second)
                      << ") = " << GetValue(&element.second) << std::endl;
        }
    }
}

std::string DefinitionTree::GetType(const struct VariableDefinition *VariableDefinitionPtr) {
    if ( auto val = std::get_if<uint64_t*>(&VariableDefinitionPtr->Value) ) {
        return "uint64_t";
    } else if ( auto val = std::get_if<uint32_t*>(&VariableDefinitionPtr->Value) ) {
        return "uint32_t";
    } else if ( auto val = std::get_if<uint16_t*>(&VariableDefinitionPtr->Value) ) {
        return "uint16_t";
    } else if ( auto val = std::get_if<uint8_t*>(&VariableDefinitionPtr->Value) ) {
        return "uint8_t";
    } else if ( auto val = std::get_if<int64_t*>(&VariableDefinitionPtr->Value) ) {
        return "int64_t";
    } else if ( auto val = std::get_if<int32_t*>(&VariableDefinitionPtr->Value) ) {
        return "int32_t";
    } else if ( auto val = std::get_if<int16_t*>(&VariableDefinitionPtr->Value) ) {
        return "int16_t";
    } else if ( auto val = std::get_if<int8_t*>(&VariableDefinitionPtr->Value) ) {
        return "int8_t";
    } else if ( auto val = std::get_if<float*>(&VariableDefinitionPtr->Value) ) {
        return "float";
    } else if ( auto val = std::get_if<double*>(&VariableDefinitionPtr->Value) ) {
        return "double";
    } else {
        return "unknown";
    }
}

std::string DefinitionTree::GetValue(const struct VariableDefinition *VariableDefinitionPtr) {
    if ( auto val = std::get_if<uint64_t*>(&VariableDefinitionPtr->Value) ) {
        return std::to_string(**val);
    } else if ( auto val = std::get_if<uint32_t*>(&VariableDefinitionPtr->Value) ) {
        return std::to_string(**val);
    } else if ( auto val = std::get_if<uint16_t*>(&VariableDefinitionPtr->Value) ) {
        return std::to_string(**val);
    } else if ( auto val = std::get_if<uint8_t*>(&VariableDefinitionPtr->Value) ) {
        return std::to_string(**val);
    } else if ( auto val = std::get_if<int64_t*>(&VariableDefinitionPtr->Value) ) {
        return std::to_string(**val);
    } else if ( auto val = std::get_if<int32_t*>(&VariableDefinitionPtr->Value) ) {
        return std::to_string(**val);
    } else if ( auto val = std::get_if<int16_t*>(&VariableDefinitionPtr->Value) ) {
        return std::to_string(**val);
    } else if ( auto val = std::get_if<int8_t*>(&VariableDefinitionPtr->Value) ) {
        return std::to_string(**val);
    } else if ( auto val = std::get_if<float*>(&VariableDefinitionPtr->Value) ) {
        return std::to_string(**val);
    } else if ( auto val = std::get_if<double*>(&VariableDefinitionPtr->Value) ) {
        return std::to_string(**val);
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
