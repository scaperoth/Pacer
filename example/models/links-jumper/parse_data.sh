#!/bin/bash

grep "jumper_xd_final = " out.log > xd_final.mat
grep "jumper_x = " out.log > x.mat
grep "jumper_xd = " out.log > xd.mat
grep "jumper_xdd = " out.log > xdd.mat
grep "jumper_real_x = " out.log > realx.mat
grep "jumper_real_xd = " out.log > realxd.mat
grep "jumper_real_xdd = " out.log > realxdd.mat

rpl -q "jumper_xd_final = [" "" xd_final.mat
rpl -q "jumper_x = [" "" x.mat
rpl -q "jumper_xd = [" "" xd.mat
rpl -q "jumper_xdd = [" "" xdd.mat
rpl -q "jumper_real_x = [" "" realx.mat
rpl -q "jumper_real_xd = [" "" realxd.mat
rpl -q "jumper_real_xdd = [" "" realxdd.mat

rpl -q " ]';" "" *.mat
rpl -q " ]';" "" *.mat
rpl -q "]';" "" *.mat
rpl -q ";" "" *.mat
