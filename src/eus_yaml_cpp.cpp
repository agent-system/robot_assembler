#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <setjmp.h>
#include <errno.h>

#include <list>
#include <vector>
#include <set>
#include <string>
#include <map>
#include <sstream>
#include <cstdio>
#include <iostream>
#include <cctype>

#include <yaml-cpp/yaml.h>
// for eus.h
#define class   eus_class
#define throw   eus_throw
#define export  eus_export
#define vector  eus_vector
#define string  eus_string
#define iostream eus_iostream
#define complex  eus_complex

#include "eus.h"
extern "C" {
  pointer ___eus_yaml_cpp(register context *ctx, int n, pointer *argv, pointer env);
  void register_eus_yaml_cpp(){
    char modname[] = "___eus_yaml_cpp";
    return add_module_initializer(modname, (pointer (*)())___eus_yaml_cpp);}
}

#undef class
#undef throw
#undef export
#undef vector
#undef string
#undef iostream
#undef complex

static std::string &str_toupper(std::string &s) {
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c){ return std::toupper(c); }
                 );
  return s;
}

static pointer parse_node(register context *ctx, YAML::Node &node) {
  //
  if (!node.IsDefined()) return NIL;
  //
  if (node.IsNull()) {
    return NIL;
  }
  //
  if (node.IsScalar()) {
    // integer
    try {
      int i = node.as<int>();
      //std::cout << " " << i;
      return makeint(i);
    } catch (const std::exception&) {
    }
    // float
    try {
      double d = node.as<double>();
      numunion nu;
      //std::cout << " " << d;
      return makeflt(d);
    } catch (const std::exception&) {
    }
    // boolean
    try {
      bool b = node.as<bool>();
      if (b) {
        //std::cout << " t";
        return T;
      } else {
        //std::cout << " nil";
        return NIL;
      }
    } catch (const std::exception&) {
    }
    // string
    //std::cout << " \"" << node.Scalar() << "\"";
    std::string str = node.Scalar();
    pointer res = makestring ((char *)str.c_str(), str.length());
    return res;
  }
  //
  if (node.IsSequence()) {
    //std::cout << "(";
    volatile pointer res = NIL;
    vpush(res);
    int size = node.size();
    for(int i = size - 1; i >= 0; i--) { // reverse iterate
      YAML::Node nd(node[i]);
      pointer pp = parse_node(ctx, nd);
      vpush(pp);
      res = rawcons(ctx, pp, res);
      vpop(); vpop();
      vpush(res);
    }
    //std::cout << ")";
    vpop();
    return res;
  }
  //
  if (node.IsMap()) {
    //std::cout << "(";
    volatile pointer res = NIL;
    vpush(res);
    for(YAML::Node::iterator it = node.begin(); it != node.end(); it++) {
      YAML::Node key(it->first);
      YAML::Node val(it->second);
      if ( key.IsDefined() && key.IsScalar() && val.IsDefined()) {
        //std::cout << "(:" << key.Scalar() << " ";
        std::string str_key(key.Scalar());
        str_key = str_toupper(str_key);
        pointer pk = defkeyword(ctx, (char *)str_key.c_str());
        vpush(pk);
        pointer pp = parse_node(ctx, val);
        //std::cout << ")";
        vpush(pp);
        pp = rawcons(ctx, pp, NIL);
        vpop(); vpush(pp);
        pp = rawcons(ctx, pk, pp);
        vpop(); vpush(pp);
        res = rawcons(ctx, pp, res);
        vpop(); vpop();
        vpush(res);
      }
    }
    //std::cout << ")";
    vpop();
    return res;
  }
  // Error?
  return NIL;
}

pointer PARSE_YAML_FILE(register context *ctx,int n,pointer *argv)
{
  ckarg(1);
  if (!isstring(argv[0])) error (E_NOSTRING);

  std::string fname((char *)get_string(argv[0]));

  YAML::Node config = YAML::LoadFile(fname);

  return parse_node(ctx, config);
}

pointer ___eus_yaml_cpp(register context *ctx, int n, pointer *argv, pointer env)
{
  defun(ctx,"C-PARSE-YAML-FILE", argv[0], (pointer (*)())PARSE_YAML_FILE, NULL);

  return 0;
}
