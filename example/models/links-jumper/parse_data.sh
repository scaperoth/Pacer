#!/bin/bash

grep "jumper_xd_final = " out.log > xd_final.mat
grep "jumper_x = " out.log > x.mat
grep "jumper_xd = " out.log > xd.mat
grep "jumper_xdd = " out.log > xdd.mat
grep "jumper_coefs_1 = " out.log > splinex.mat
grep "jumper_coefs_2 = " out.log > spliney.mat
grep "jumper_coefs_3 = " out.log > splinez.mat

rpl -q "jumper_xd_final = [" "" xd_final.mat
rpl -q "jumper_x = [" "" x.mat
rpl -q "jumper_xd = [" "" xd.mat
rpl -q "jumper_xdd = [" "" xdd.mat
rpl -q "jumper_coefs_1 = [" "" splinex.mat
rpl -q "jumper_coefs_2 = [" "" spliney.mat
rpl -q "jumper_coefs_3 = [" "" splinez.mat

rpl -q " ]';" "" *.mat
rpl -q " ]';" "" *.mat
rpl -q "]';" "" *.mat
rpl -q ";" "" *.mat
