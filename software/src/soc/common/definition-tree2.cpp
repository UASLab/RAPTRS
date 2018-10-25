// definition-tree2.hxx - Curtis Olson

#include <iostream>
#include "definition-tree2.h"

using std::cout;
using std::endl;

// create a global instance of the deftree
DefinitionTree2 deftree;

Element *DefinitionTree2::initElement(string name, string desc,
                                      Element::log_tag_t datalog,
                                      Element::log_tag_t telemetry) {
    def_tree_t::iterator it;
    it = data.find(name);
    if ( it != data.end() ) {
        cout << "NOTICE: publisher found existing def-tree element: " << name << endl;
        it->second->description = desc;
        it->second->datalog = datalog;
        it->second->telemetry = telemetry;
        return it->second;
    } else {
        Element *ele = new Element;
        ele->description = desc;
        ele->datalog = datalog;
        ele->telemetry = telemetry;
        data[name] = ele;
        return ele;
    }
}

Element *DefinitionTree2::makeAlias(string orig_name, string alias_name) {
    def_tree_t::iterator it;
    it = data.find(orig_name);
    if ( it != data.end() ) {
        data[alias_name] = it->second;
        return it->second;
    } else {
        cout << "Notice: attempt to alias to non-existent entry" << endl;
        return NULL;
    }
}

Element *DefinitionTree2::getElement(string name, bool create) {
    def_tree_t::iterator it;
    it = data.find(name);
    if ( it != data.end() ) {
        return it->second;
    } else if ( create ) {
        cout << "NOTICE: subscriber created def-tree element: " << name << endl;
        Element *ele = new Element;
        data[name] = ele;
        return ele;
    } else {
        return NULL;
    }
}

/* Gets list of definition tree member keys at a given tree level */
void DefinitionTree2::GetKeys(string Name, vector<string> *KeysPtr) {
    KeysPtr->clear();
    for (auto const& element : data) {
        if (element.first.find(Name) != std::string::npos) {
            KeysPtr->push_back(element.first);
        }
    }
}

/* Gets number of definition tree members at a given tree level */
size_t DefinitionTree2::Size(std::string Name) {
  size_t retval = 0;
  for (auto const& element : data) {
    if (element.first.find(Name) != std::string::npos) {
      retval++;
    }
  }
  return retval;
}

void DefinitionTree2::Erase(string name) {
    def_tree_t::iterator it;
    it = data.find(name);
    if ( it != data.end() ) {
        data.erase(it);
    } else {
        cout << "NOTICE: attempting to erase non-existent element: " << name << endl;
    }
}

/* Defines new definition tree member given a variable definition struct */
void DefinitionTreeOld::DefineMember(std::string Name,struct VariableDefinition &VariableDefinitionRef) {
  Data_[Name] = VariableDefinitionRef;
}

/* Initializes new empty definition tree member given a name */
void DefinitionTreeOld::InitMember(std::string Name) {
  struct VariableDefinition TempDef;
  Data_[Name] = TempDef;
}

/* Defines new definition tree member given a name, value, description, datalog, and telemetry */
void DefinitionTreeOld::InitMember(std::string Name,std::variant<uint64_t*,uint32_t*,uint16_t*,uint8_t*,int64_t*,int32_t*,int16_t*,int8_t*,float*, double*> Value,std::string Description,bool Datalog,bool Telemetry) {
  struct VariableDefinition TempDef;
  Data_[Name] = TempDef;
  Data_[Name].Value = Value;
  Data_[Name].Description = Description;
  Data_[Name].Datalog = Datalog;
  Data_[Name].Telemetry = Telemetry;
}

/* Sets value for an existing definition tree member */
void DefinitionTreeOld::SetValuePtr(std::string Name,std::variant<uint64_t*,uint32_t*,uint16_t*,uint8_t*,int64_t*,int32_t*,int16_t*,int8_t*,float*, double*> Value) {
  Data_[Name].Value = Value;
}

/* Sets description for an existing definition tree member */
void DefinitionTreeOld::SetDescription(std::string Name,std::string Description) {
  Data_[Name].Description = Description;
}

/* Sets datalog status for an existing definition tree member */
void DefinitionTreeOld::SetDatalog(std::string Name,bool Datalog) {
  Data_[Name].Datalog = Datalog;
}

/* Sets telemetry status for an existing definition tree member */
void DefinitionTreeOld::SetTelemetry(std::string Name,bool Telemetry) {
  Data_[Name].Telemetry = Telemetry;
}

/* Gets description for a definition tree member */
std::string DefinitionTreeOld::GetDescription(std::string Name) {
  return Data_[Name].Description;
}

/* Gets datalog status for a definition tree member */
bool DefinitionTreeOld::GetDatalog(std::string Name) {
  return Data_[Name].Datalog;
}

/* Gets telemetry status for a definition tree member */
bool DefinitionTreeOld::GetTelemetry(std::string Name) {
  return Data_[Name].Telemetry;
}

/* Gets variable struct for a definition tree member */
void DefinitionTreeOld::GetMember(std::string Name,struct VariableDefinition *VariableDefinitionPtr) {
  *VariableDefinitionPtr = Data_[Name];
}

/* Gets number of definition tree members at a given tree level */
size_t DefinitionTreeOld::Size(std::string Name) {
  size_t retval = 0;
  for (auto const& element : Data_) {
    if (element.first.find(Name)!=std::string::npos) {
      retval++;
    }
  }
  return retval;
}

/* Gets list of definition tree member keys at a given tree level */
void DefinitionTreeOld::GetKeys(std::string Name,std::vector<std::string> *KeysPtr) {
  KeysPtr->clear();
  for (auto const& element : Data_) {
    if (element.first.find(Name)!=std::string::npos) {
      KeysPtr->push_back(element.first);
    }
  }
}

/* print definition tree member keys at a given tree level */
void DefinitionTreeOld::PrettyPrint(std::string Prefix) {
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

std::string DefinitionTreeOld::GetType(const struct VariableDefinition *VariableDefinitionPtr) {
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

std::string DefinitionTreeOld::GetValue(const struct VariableDefinition *VariableDefinitionPtr) {
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
    } else {
        return "";
    }
}

/* Erases definition tree members at a given tree level */
void DefinitionTreeOld::Erase(std::string Name) {
  for (auto const& element : Data_) {
    if (element.first.find(Name)!=std::string::npos) {
      Data_.erase(element.first);
    }
  }
}

/* Clears the full definition tree */
void DefinitionTreeOld::Clear() {
  Data_.clear();
}
