# MRPP-MH: Multi Robot Path Planning based on Astar - MATLAB

---------------------------------------------------------

## MRPP_simple

no cooperation at all: many collisions
no time in Open list

#### functions

- selectTopNode_simple
- optimalPath_simple

---------------------------------------------------------

## MRPP_1

expand Open list by collision consideration
with time in Open list
stall node is added

#### functions

- selectTopNode_1
- optimalPath_1

---------------------------------------------------------

## MRPP_2

same as MRPP_1, only difference:
calculates path and update Open for each robot that reaches target

#### functions

- selectTopNode_1
- optimalPath_1

---------------------------------------------------------

## in all

adding targetNodes to ClosedList
