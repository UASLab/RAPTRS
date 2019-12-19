/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Curtis Olson
*/

#include <iostream>
#include "definition-tree2.h"

using std::cout;
using std::endl;
using std::make_shared;

// create a global instance of the deftree
DefinitionTree2 deftree;

ElementPtr DefinitionTree2::initElement(string name, string desc,
                                        log_tag_t datalog,
                                        log_tag_t telemetry)
{
  def_tree_t::iterator it;
  it = data.find(name);
  if ( it != data.end() ) {
    // cout << "NOTICE: publisher found existing def-tree element: " << name << endl;
    it->second->description = desc;
    it->second->datalog = datalog;
    it->second->telemetry = telemetry;
    return it->second;
  } else {
    ElementPtr ele = make_shared<Element>();
    ele->description = desc;
    ele->datalog = datalog;
    ele->telemetry = telemetry;
    data[name] = ele;
    return ele;
  }
}

ElementPtr DefinitionTree2::getElement(string name, bool create) {
  def_tree_t::iterator it;
  it = data.find(name);
  if ( it != data.end() ) {
    return it->second;
  } else if ( create ) {
    cout << "NOTICE: subscriber created def-tree element: " << name << endl;
    ElementPtr ele = make_shared<Element>();
    data[name] = ele;
    return ele;
  } else {
    cout << "NOTICE: subscriber FAILED TO GET def-tree element: " << name << endl;
    return NULL;
  }
}

/* Gets list of definition tree member keys at a given tree level */
void DefinitionTree2::GetKeys(string Name, vector<string> *KeysPtr) {
  KeysPtr->clear();
  for (auto const& element : data) {
    if (element.first.find(Name) != string::npos) {
      KeysPtr->push_back(element.first);
    }
  }
}

/* Gets number of definition tree members at a given tree level */
size_t DefinitionTree2::Size(string Name) {
  size_t retval = 0;
  for ( auto const& element : data ) {
    if (element.first.find(Name) != string::npos) {
      retval++;
    }
  }
  return retval;
}

/* print definition tree member keys at a given tree level */
void DefinitionTree2::PrettyPrint(string Prefix) {
  cout << "Base path: " << Prefix << endl;
  for ( auto const& it : data ) {
    size_t pos = it.first.find(Prefix);
    if ( pos != string::npos) {
      string tail = it.first.substr(pos + 1);
      ElementPtr ele = it.second;
      cout << "    " << tail << " (" << ele->getType()
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
