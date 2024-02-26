#!/bin/bash
for ((i=4; i<150; i++)); 
do
    ./bp_main tests/example${i}.trc > tests/example${i}.act
    echo "Comparing actual and expected output for test $i:"
    diff -s -q tests/example${i}.out tests/example${i}.act
    ./bp_main tests/n_example${i}.trc > tests/n_example${i}.act
    diff -s -q tests/n_example${i}.out tests/n_example${i}.act
    ./bp_main tests/small_btb_example${i}.trc > tests/small_btb_example${i}.act
    diff -s -q tests/small_btb_example${i}.out tests/small_btb_example${i}.act
done
echo "done1"	
for ((i=1; i<21; i++)); 
do
    ./bp_main tests/segel/example${i}.trc > tests/segel/segel_example${i}.act
    diff -s -q tests/segel/example${i}.out tests/segel/segel_example${i}.act
done
echo "done2"	

for ((i=1; i<150; i++)); 
do
    ./bp_main 150_tests/test${i}.trc > 150_tests/test${i}.act
    diff -s -q 150_tests/test${i}.out 150_tests/test${i}.act 
done    
echo "done3"	

