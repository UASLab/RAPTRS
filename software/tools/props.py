"""

props.py: a property tree system for python

Provides a hierarchical tree of shared data values.
 - Modules can use this tree as a way to share data
   (i.e. communicate) in a loosly structure / flexible way.
 - Both reader and writer can create properties in the shared tree as they
   need them so there is less worry about initialization order dependence.
 - Tree values can be accessed in native python code as nested class
   members: a = root; a.b.c.var1 = 42
 - Children nodes can be enumerated: /sensors/gps[0], /sensors/gps[1], etc.
 - C++ interface allows complex but flexible data sharing between mixed
   C++ and python modules.
 - Maps well to xml or json data storage (i.e. xml/json config files can
   be loaded into a subtree (child) in the root shared property tree.

Notes:
 - getChild(path, True) will create 'path' as a tree of PropertyNodes() if
   it doens't exist (including any intermediate nodes.)   If the final
   component of the path is intended to be a leaf node, don't include it
   in the path or it will be created as a branch.
 - To create /path/to/variable and assign if a value, call:
   node = getNode("/path/to", create=True)
   node.variable = value

"""

from __future__ import print_function
import re

class PropertyNode:
    def hasChild(self, name):
        return name in self.__dict__

    def getChild(self, path, create=False):
        # print("getChild():", path, "create:", create)
        if path.startswith('/'):
            # require relative paths
            print("Error: attempt to get child with absolute path name")
            return None
        if path.endswith('/'):
            # we will strip this, but let's complain loudly because the
            # caller is being sloppy.
            print("WARNING: a sloppy coder has used a trailing / in a path:", path)
            path = path[:-1]
        if re.match('-', path):
            # require valid python variable names in path
            print("Error: attempt to use '-' in property name")
            return None
        tokens = path.split('/');
        #print "tokens:", tokens
        node = self
        for i, token in enumerate(tokens):
            # test for enumerated form: ident[index]
            parts = re.split('([\w-]+)\[(\d+)\]', token)
            if len(parts) == 4:
                token = parts[1]
                index = int(parts[2])
            else:
                index = None
            if token in node.__dict__:
                #print "node exists:", token
                # node exists
                child = node.__dict__[token]
                child_type = type(child)
                #print "type =", str(child_type)
                if index == None:
                    if not child_type is list:
                        # requested non-indexed node, and node is not indexed
                        node = node.__dict__[token]
                    else:
                        # node is indexed use the first element
                        node = node.__dict__[token][0]
                else:
                    #print "requesting enumerated node"
                    # enumerated (list) node
                    if child_type is list and len(child) > index:
                        node = child[index]
                    elif create:
                        if child_type is list:
                            # list is not large enough and create flag
                            # requested: extend the list
                            self.extendEnumeratedNode(child, index)
                        else:
                            # create on enumerated node, but not a
                            # list yet
                            save = child
                            node.__dict__[token] = [save]
                            child = node.__dict__[token]
                            self.extendEnumeratedNode(child, index)
                        node = child[index]
                    else:
                        return None
                if isinstance(node, PropertyNode) or type(node) is list:
                    # ok
                    pass
                else:
                    print("path:", token, "includes leaf nodes, sorry")
                    return None
            elif create:
                # print('node not found and create flag is true')
                if index == None:
                    node.__dict__[token] = PropertyNode()
                    node = node.__dict__[token]
                else:
                    # create node list and extend size as needed
                    node.__dict__[token] = []
                    tmp = node.__dict__[token]
                    self.extendEnumeratedNode(tmp, index)
                    node = tmp[index]
            else:
                # requested node not found
                return None
        # return the last child node in the path
        return node

    def isEnum(self, child):
        if child in self.__dict__:
            if type(self.__dict__[child]) is list:
                return True
        return False

    def getLen(self, child):
        if child in self.__dict__:
            if type(self.__dict__[child]) is list:
                return len(self.__dict__[child])
            else:
                print("WARNING in getLen() path =", child, " is not enumerated")
                return 1
        else:
            print("WARNING: request length of non-existant attribute:", child)
        return 0

    # make the specified node enumerated (if needed) and expand the
    # length (if needed)
    def setLen(self, child, size, init_val=None):
        #print "called setLen()", child, size
        if child in self.__dict__:
            if not type(self.__dict__[child]) is list:
                # convert existing element to element[0]
                print("converting:", child, "to enumerated")
                save = self.__dict__[child]
                self.__dict__[child] = [save]
        else:
            #print "creating:", child
            self.__dict__[child] = []
        if init_val == None:
            #print "extending branch nodes:", size
            self.extendEnumeratedNode(self.__dict__[child], size-1)
        else:
            #print "extending leaf nodes:", size
            self.extendEnumeratedLeaf(self.__dict__[child], size-1, init_val)
        
    # return a list of children (attributes)
    def getChildren(self, expand=True):
        # constructed the unexpanded list
        pass1 = []
        for child in self.__dict__:
            pass1.append(child)
        # sort the pass1 list and expand if requested
        result = []
        for child in sorted(pass1):
            if expand and type(self.__dict__[child]) is list:
                for i in range(0, len(self.__dict__[child])):
                    name = child + '[' + str(i) + ']'
                    result.append(name)
            else:
                result.append(child)    
        return result
    
    def isLeaf(self, path):
        node = self.getChild(path)
        return not isinstance(node, PropertyNode)

    def getFloat(self, name):
        if name in self.__dict__:
            try:
                return float(self.__dict__[name])
            except Exception as e:
                print(str(e))
        return 0.0
            
    def getInt(self, name):
        if name in self.__dict__:
            try:
                return int(self.__dict__[name])
            except Exception as e:
                print(str(e))
        return 0

    def getBool(self, name):
        if name in self.__dict__:
            try:
                return bool(self.__dict__[name])
            except Exception as e:
                print(str(e))
        return False

    def getString(self, name):
        if name in self.__dict__:
            try:
                return str(self.__dict__[name])
            except Exception as e:
                print(str(e))
        return ""

    def getFloatEnum(self, name, index):
        if name in self.__dict__:
            try:
                self.extendEnumeratedNode(self.__dict__[name], index)
                return float(self.__dict__[name][index])
            except Exception as e:
                print(str(e))
        return 0.0
            
    def getIntEnum(self, name, index):
        if name in self.__dict__:
            try:
                self.extendEnumeratedNode(self.__dict__[name], index)
                return int(self.__dict__[name][index])
            except Exception as e:
                print(str(e))
        return 0
            
    def getStringEnum(self, name, index):
        if name in self.__dict__:
            try:
                self.extendEnumeratedNode(self.__dict__[name], index)
                return str(self.__dict__[name][index])
            except:
                print(str(e))
        return ""
            
    def setFloat(self, name, val):
        self.__dict__[name] = float(val)
            
    def setInt(self, name, val):
        self.__dict__[name] = int(val)
            
    def setBool(self, name, val):
        self.__dict__[name] = bool(val)
            
    def setString(self, name, val):
        self.__dict__[name] = str(val)

    def setFloatEnum(self, name, index, val):
        if not name in self.__dict__:
            self.setLen(name, index, 0.0)            
        self.extendEnumeratedNode(self.__dict__[name], index)
        self.__dict__[name][index] = val
        
    def setIntEnum(self, name, index, val):
        if not name in self.__dict__:
            self.setLen(name, index, 0)            
        self.extendEnumeratedNode(self.__dict__[name], index)
        self.__dict__[name][index] = int(val)
        
    def setBoolEnum(self, name, index, val):
        if not name in self.__dict__:
            self.setLen(name, index, 0)            
        self.extendEnumeratedNode(self.__dict__[name], index)
        self.__dict__[name][index] = bool(val)
        
    def setStringEnum(self, name, index, val):
        if not name in self.__dict__:
            self.setLen(name, index, 0)            
        self.extendEnumeratedNode(self.__dict__[name], index)
        self.__dict__[name][index] = str(val)
        
    def pretty_print(self, indent=""):
        for child in self.__dict__:
            node = self.__dict__[child]
            if isinstance(node, PropertyNode):
                print(indent + "/" + child)
                node.pretty_print(indent + "  ")
            elif type(node) is list:
                #print "child is list:", str(node)
                for i, ele in enumerate(node):
                    # print i, str(ele)
                    if isinstance(ele, PropertyNode):
                        print(indent + "/" + child + "[" + str(i) + "]:")
                        ele.pretty_print(indent + "  ")
                    else:
                        print(indent + str(child) + "[" + str(i) + "]: ", end='')
                        print(str(ele))
            else:
                print(indent + str(child) + ": ", end='')
                print(str(node))
        
    def extendEnumeratedNode(self, node, index):
        for i in range(len(node), index+1):
            # print "branch appending:", i
            node.append( PropertyNode() )
            
    def extendEnumeratedLeaf(self, node, index, init_val):
        for i in range(len(node), index+1):
            # print "leaf appending:", i, "=", init_val
            node.append( init_val )
            
        
root = PropertyNode()

# return/create a node relative to the shared root property node
def getNode(path, create=False):
    #print("getNode():", path, "create:", str(create))
    if path[:1] != '/':
        # require leading /
        return None
    elif path == "/":
        # catch trivial case
        return root
    return root.getChild(path[1:], create)
