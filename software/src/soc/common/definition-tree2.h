// definition-tree2.hxx - Curtis Olson

#pragma once

#include <stdint.h>
#include <vector>
#include <map>
#include <iostream>
#include <string>

using std::map;
using std::string;
using std::to_string;
using std::vector;

// minimal types for logging -- log as this type.  put these in the
// global name space otherwise the notation becomes crushing.
enum log_tag_t {
  LOG_NONE,
  LOG_BOOL, LOG_INT8, LOG_UINT8,
  LOG_INT16, LOG_UINT16,
  LOG_INT32, LOG_UINT32,
  LOG_INT64, LOG_UINT64, LOG_LONG,
  LOG_FLOAT, LOG_DOUBLE
};
    
class Element {

 private:

  // supported types
  enum { NONE, BOOL, INT, LONG, FLOAT, DOUBLE } tag;

  union {
    bool b;
    int i;
    long l;
    float f;
    double d;
  } x = {0};

 public:
    
  string description;
  log_tag_t datalog{LOG_NONE};
  log_tag_t telemetry{LOG_NONE};
    
  Element() {}
  ~Element() {}

  void setBool( bool val ) { x.b = val; tag = BOOL; }
  void setInt( int val ) { x.i = val; tag = INT; }
  void setLong( long val ) { x.l = val; tag = LONG; }
  void setFloat( float val ) { x.f = val; tag = FLOAT; }
  void setDouble( double val ) { x.d = val; tag = DOUBLE; }

  bool getBool() {
    switch(tag) {
    case BOOL: return x.b;
    case INT: return x.i;
    case LONG: return x.l;
    case FLOAT: return x.f;
    case DOUBLE: return x.d;
    default: return false;
    }
  }
  int getInt() {
    switch(tag) {
    case BOOL: return x.b;
    case INT: return x.i;
    case LONG: return x.l;
    case FLOAT: return x.f;
    case DOUBLE: return x.d;
    default: return 0;
    }
  }
  long getLong() {
    switch(tag) {
    case BOOL: return x.b;
    case INT: return x.i;
    case LONG: return x.l;
    case FLOAT: return x.f;
    case DOUBLE: return x.d;
    default: return 0;
    }
  }
  float getFloat() {
    switch(tag) {
    case BOOL: return x.b;
    case INT: return x.i;
    case LONG: return x.l;
    case FLOAT: return x.f;
    case DOUBLE: return x.d;
    default: return 0.0;
    }
  }
  double getDouble() {
    switch(tag) {
    case BOOL: return x.b;
    case INT: return x.i;
    case LONG: return x.l;
    case FLOAT: return x.f;
    case DOUBLE: return x.d;
    default: return 0.0;
    }
  }

  string getType() {
    switch(tag) {
    case BOOL: return "bool";
    case INT: return "int";
    case LONG: return "long";
    case FLOAT: return "float";
    case DOUBLE: return "double";
    default: return "no type";
    }
  }
    
  string getValueAsString() {
    switch(tag) {
    case BOOL: return to_string(x.b);
    case INT: return to_string(x.i);
    case LONG: return to_string(x.l);
    case FLOAT: return to_string(x.f);
    case DOUBLE: return to_string(x.d);
    default: return "no value";
    }
  }
    
  log_tag_t getLoggingType() {
    return datalog;
  }

  log_tag_t getTelemetryType() {
    return telemetry;
  }
};
    
typedef map<string, Element *> def_tree_t ;

class DefinitionTree2 {
    
 public:
    

  DefinitionTree2() {}
  ~DefinitionTree2() {}

  Element *initElement(string name, string desc,
                       log_tag_t datalog,
                       log_tag_t telemetry);
  Element *makeAlias(string orig_name, string alias_name);
  Element *getElement(string name, bool create=true);

  void GetKeys(string Name, vector<string> *KeysPtr);
  size_t Size(string Name);
  void PrettyPrint(string Prefix);

  void Erase(string name);
    
 private:
    
  def_tree_t data;
};

// reference a global instance of the deftree
extern DefinitionTree2 deftree;
