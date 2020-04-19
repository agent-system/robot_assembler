#!/usr/bin/python

from __future__ import print_function
import os
import sys

def write_python_dict(instance_of_dict, strm=sys.stdout):
    print('(', end="", file=strm)
    for key in instance_of_dict.keys():
        elem = instance_of_dict[key]
        print( '(:%s '%(key), end="", file=strm)
        _write_python_elem(elem, strm)
        print( ')', end="", file=strm)
    print( ')', end="", file=strm)

def _write_python_elem (elem, strm=sys.stdout):
    if type(elem) == dict:
        write_python_dict(elem,strm)
    elif type(elem) == list:
        _write_python_list(elem,strm)
    elif type(elem) == str:
        print( '"%s" '%(elem), end="", file=strm)
    else:
        print( '%s '%(elem), end="", file=strm)

def _write_python_list(instance_of_list, strm=sys.stdout):
    print ( '(', end="", file=strm)
    for elem in instance_of_list:
        _write_python_elem(elem,strm)
    print ( ')', end="", file=strm)

import yaml

if __name__ == '__main__':
    output_f = None
    if len(sys.argv) > 2:
        output_f = sys.argv[2]
    yfile = sys.argv[1]

    if not os.path.isfile(yfile):
        exit(1)

    f = open(yfile)
    data = yaml.load(f)
    f.close()
    ##

    if output_f:
        strm = open(output_f)
    else:
        strm = sys.stdout

    write_python_dict(data, strm=strm)
