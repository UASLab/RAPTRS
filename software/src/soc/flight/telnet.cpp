/*
Copyright (c) 2016 - 2019 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Author: Brian Taylor, Chris Regan, Curt Olson (original author)
*/

#include <unistd.h>
#include <sstream>
#include <string>

#include "definition-tree2.h"

#include "strutils.h"
#include "netChat.h"
#include "telnet.h"

//using std::stringstream;
using std::ends;
using std::to_string;

static bool debug_on = true;

static string normalize_path(string raw_path) {
  vector<string> tokens = split( raw_path, "/" );
  vector<string> tmp;
  for ( unsigned int i = 1; i < tokens.size(); i++ ) {
    if ( tokens[i] == ".." ) {
      if ( tmp.size() ) {
        tmp.pop_back();
      }
    } else if ( tokens[i] == "." ) {
      // do nothing
    } else {
      tmp.push_back(tokens[i]);
    }
  }
  string result = "";
  for ( unsigned int i = 0; i < tmp.size(); i++ ) {
    result += "/" + tmp[i];
  }
  if ( result == "" ) {
    result = "/";
  }
  printf("Original path = %s\n", raw_path.c_str());
  printf("new      path = %s\n", result.c_str());
  return result;
}

/**
 * Props connection class.
 * This class represents a connection to props client.
 */
class PropsChannel : public netChat
{
  netBuffer buffer;

  /**
   * Current property node name.
   */
  string path;

  enum Mode {
    PROMPT,
    DATA
  };
  Mode mode;

public:

  /**
   * Constructor.
   */
  PropsChannel();
    
  /**
   * Append incoming data to our request buffer.
   *
   * @param s Character string to append to buffer
   * @param n Number of characters to append.
   */
  void collectIncomingData( const char* s, int n );

  /**
   * Process a complete request from the props client.
   */
  void foundTerminator();

private:
  /**
   * Return a "Node no found" error message to the client.
   */
  void node_not_found_error( const string& node_name );
};

/**
 * 
 */
PropsChannel::PropsChannel()
  : buffer(512),
    path("/"),
    mode(PROMPT)
{
  // setTerminator( "\r\n" );
  setTerminator( "\n" );
}

/**
 * 
 */
void
PropsChannel::collectIncomingData( const char* s, int n )
{
  buffer.append( s, n );
}

/**
 * 
 */
void
PropsChannel::node_not_found_error( const string& node_name )
{
  string error = "-ERR Node \"";
  error += node_name;
  error += "\" not found.";
  push( error.c_str() );
  push( getTerminator() );
}


/**
 * We have a command.
 * 
 */
void
PropsChannel::foundTerminator()
{
  const char* cmd = buffer.getData();
  vector<string> tokens = split( cmd );

  if ( debug_on ) {
    printf( "processing command: " );
    for ( size_t i = 0; i < tokens.size(); i++ ) {
      printf("%s ", tokens[i].c_str());
    }
    printf("\n");
  }

  if (!tokens.empty()) {
    string command = tokens[0];

    if ( command == "null" ) {
      // do nothing!
    } else if ( command == "ls" ) {
      vector<string> keys;
      string dir = "";
      if (tokens.size() == 2) {
        if ( tokens[1][0] == '/' ) {
          dir = tokens[1];
        } else {
          dir = path + "/" + tokens[1];
        }
      } else {
        dir = path;
      }

      if ( dir.length() ) {
        string line = "path: " + dir + getTerminator();
        push( line.c_str() );
        vector<string> children;
        deftree.GetKeys(dir, &children);
        string last_child = "";
        for ( unsigned int i = 0; i < children.size(); i++ ) {
          string tail = "";
          if ( dir.length() == 1 ) {
            // special case root node for now
            tail = children[i].substr(1);
          } else {
            tail = children[i].substr(dir.length()+1);
          }
          string child = "";
          bool is_leaf = true;
          size_t pos = tail.find("/");
          if ( pos != string::npos ) {
            child = tail.substr(0, pos);
            is_leaf = false;
          } else {
            child = tail;
            is_leaf = true;
          }
          string line = "";
          if ( is_leaf ) {
            ElementPtr ele = deftree.getElement(children[i]);
            string type = ele->getType();
            string value = ele->getValueAsString();
            if ( mode == PROMPT ) {
              // string value = dir.getString(children[i].c_str());
              line += "  " + child + " (" + type + ") = " + value;
            } else {
              line += children[i];
            }
          } else {              // !is_leaf
            if ( child != last_child ) {
              if ( mode == PROMPT ) {
                line += "  " + child + "/";
              } else {
                line += dir + child;
              }
              last_child = child;
            }
          }
          if ( line.length() ) {
            line += getTerminator();
            push( line.c_str() );
          }
        }
      } else {
        node_not_found_error( tokens[1] );
      }
    } else if ( command == "cd" ) {
      // FIXME: should handle ".." (and maybe even .)
      if (tokens.size() == 2) {
        string newpath = "";
        if ( tokens[1][0] == '/' ) {
          // absolute path specified
          newpath = tokens[1];
        } else {
          // relative path specified
          if ( path == "/" ) {
            newpath = path + tokens[1];
          } else {
            newpath = path + "/" + tokens[1];
          }
        }
        newpath = normalize_path(newpath);
        printf("newpath before = %s\n", newpath.c_str());
        // validate path
        vector<string> tmp_children;
        deftree.GetKeys(newpath, &tmp_children);
        ElementPtr tmp_ele = deftree.getElement(newpath, false);
        if ( tmp_children.size() > 0 && tmp_ele == NULL ) {
          // path matches stuff, but not an element
          printf("path ok = %s\n", newpath.c_str());
          path = newpath;
        } else {
          node_not_found_error( tokens[1] );
        }
      }
    } else if ( command == "pwd" ) {
      push( path.c_str() );
      push( getTerminator() );
    } else if ( command == "get" || command == "show" ) {
      if ( tokens.size() == 2 ) {
        string newpath;
        if ( tokens[1][0] == '/' ) {
          // absolute path specified
          newpath = tokens[1];
        } else {
          // relative path specified
          if ( path == "/" ) {
            newpath = path + tokens[1];
          } else {
            newpath = path + "/" + tokens[1];
          }
        }
                
        string tmp;
        ElementPtr ele = deftree.getElement(newpath);
        string type = ele->getType();
        string value = ele->getValueAsString();
        if ( mode == PROMPT ) {
          tmp = tokens[1];
          tmp += "(" + type + ") = " + value;
        } else {
          tmp = value;
        }
        push( tmp.c_str() );
        push( getTerminator() );
      }
    } else if ( command == "set" ) {
      // an adventure for a later time ...
    } else if ( command == "data" ) {
      mode = DATA;
    } else if ( command == "prompt" ) {
      mode = PROMPT;
    } else {
      const char* msg = "\
Valid commands are:\r\n\
\r\n\
help               show this help message\r\n\
cd <dir>           cd to a directory, '..' to move back\r\n\
get <var>          show the value of a parameter\r\n\
ls [<dir>]         list directory\r\n\
data               switch to raw data mode\r\n\
prompt             switch to interactive mode (default)\r\n\
pwd                display your current path\r\n\
# set <var> <val>    set <var> to a new <val>\r\n";
      push( msg );
    }
  }

  if (mode == PROMPT) {
    string prompt = "> ";
    push( prompt.c_str() );
  }

  buffer.remove();
}

/**
 * 
 */
UGTelnet::UGTelnet( const int port_num ):
  enabled(false)
{
  port = port_num;
}

/**
 * 
 */
UGTelnet::~UGTelnet()
{
}

/**
 * 
 */
bool
UGTelnet::open()
{
  if (enabled ) {
    printf("This shouldn't happen, but the telnet channel is already in use, ignoring\n" );
    return false;
  }

  netChannel::open();
  netChannel::bind( "", port );
  netChannel::listen( 5 );
  printf("Telnet server started on port %d\n", port );

  enabled = true;

  return true;
}

/**
 * 
 */
bool
UGTelnet::close()
{
  if ( debug_on ) {
    printf("closing UGTelnet\n" );
  }

  return true;
}

/**
 * 
 */
bool
UGTelnet::process()
{
  netChannel::poll();
  return true;
}

/**
 * 
 */
void
UGTelnet::handleAccept()
{
  netAddress addr;
  int handle = netChannel::accept( &addr );
  printf("Telnet server accepted connection from %s:%d\n",
         addr.getHost(), addr.getPort() );
  PropsChannel* channel = new PropsChannel();
  channel->setHandle( handle );
}
