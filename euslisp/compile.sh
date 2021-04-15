#!/bin/bash

set -e

irteusgl "(progn (setq *error-handler* #'(lambda (&rest args) (unix::exit -1))) (comp::compile-file \"robot-assembler.l\") (exit))"
irteusgl "(progn (setq *error-handler* #'(lambda (&rest args) (unix::exit -1))) (comp::compile-file \"robot-assembler-utils.l\") (exit))"
irteusgl "(progn (setq *error-handler* #'(lambda (&rest args) (unix::exit -1))) (comp::compile-file \"robot-assembler-viewer.l\") (exit))"

#rm -f robot-assembler.{o,c,h,so} robot-assembler-utils.{o,c,h,so} robot-assembler-viewer.{o,c,h,so}
#rm -f robot-assembler.so robot-assembler-utils.so robot-assembler-viewer.so
rm -f robot-assembler.{o,c,h} robot-assembler-utils.{o,c,h} robot-assembler-viewer.{o,c,h}
