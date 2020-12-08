#!/usr/bin/env roseus

;; for noVNC
(push (list x::*visual*  24) x::*visuals*)

(load "package://robot_assembler/euslisp/robot-assembler-viewer.l")

(robot-assembler-viewer-main)
