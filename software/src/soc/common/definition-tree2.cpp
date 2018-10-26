// definition-tree2.hxx - Curtis Olson

#include <iostream>
#include "definition-tree2.h"

using std::cout;
using std::endl;

// create a global instance of the deftree
DefinitionTree2 deftree;

Element *DefinitionTree2::initElement(string name, string desc,
                                      log_tag_t datalog,
                                      log_tag_t telemetry)
{
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
  for ( auto const& element : data ) {
    if (element.first.find(Name) != std::string::npos) {
      retval++;
    }
  }
  return retval;
}

/* print definition tree member keys at a given tree level */
void DefinitionTree2::PrettyPrint(std::string Prefix) {
  std::cout << Prefix << std::endl;
  for ( auto const& it : data ) {
    std::size_t pos = it.first.find(Prefix);
    if ( pos != string::npos) {
      string tail = it.first.substr(pos + 1);
      Element *ele = it.second;
      cout << "  " << tail << " (" << ele->getType()
           << ") = " << ele->getValueAsString() << endl;
    }
  }
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
