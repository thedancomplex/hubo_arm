      Subroutine hubo_arm_kin (q, R, v, J) 
      Double Precision q(6) 
      Double Precision R(3,3) 
      Double Precision v(3,1) 
      Double Precision J(6,6) 
      R(1,1) = ((sin(q(1))*sin(q(2))*cos(q(3))-cos(q(1))*sin(q(3)))*sin(
     1   q(5))+((sin(q(1))*sin(q(2))*sin(q(3))+cos(q(1))*cos(q(3)))*cos(
     2   q(4))-1.d+0*sin(q(1))*cos(q(2))*sin(q(4)))*cos(q(5)))*cos(q(6))
     3   -1.d+0*((sin(q(1))*sin(q(2))*sin(q(3))+cos(q(1))*cos(q(3)))*sin
     4   (q(4))+1.d+0*sin(q(1))*cos(q(2))*cos(q(4)))*sin(q(6))
      R(1,2) = (sin(q(1))*sin(q(2))*cos(q(3))-cos(q(1))*sin(q(3)))*cos(q
     1   (5))-((sin(q(1))*sin(q(2))*sin(q(3))+cos(q(1))*cos(q(3)))*cos(q
     2   (4))-1.d+0*sin(q(1))*cos(q(2))*sin(q(4)))*sin(q(5))
      R(1,3) = ((sin(q(1))*sin(q(2))*cos(q(3))-cos(q(1))*sin(q(3)))*sin(
     1   q(5))+((sin(q(1))*sin(q(2))*sin(q(3))+cos(q(1))*cos(q(3)))*cos(
     2   q(4))-1.d+0*sin(q(1))*cos(q(2))*sin(q(4)))*cos(q(5)))*sin(q(6))
     3   +1.d+0*((sin(q(1))*sin(q(2))*sin(q(3))+cos(q(1))*cos(q(3)))*sin
     4   (q(4))+1.d+0*sin(q(1))*cos(q(2))*cos(q(4)))*cos(q(6))
      R(2,1) = (cos(q(2))*cos(q(3))*sin(q(5))+(1.d+0*sin(q(2))*sin(q(4))
     1   +cos(q(2))*sin(q(3))*cos(q(4)))*cos(q(5)))*cos(q(6))-1.d+0*(cos
     2   (q(2))*sin(q(3))*sin(q(4))-1.d+0*sin(q(2))*cos(q(4)))*sin(q(6))
      R(2,2) = cos(q(2))*cos(q(3))*cos(q(5))-(1.d+0*sin(q(2))*sin(q(4))+
     1   cos(q(2))*sin(q(3))*cos(q(4)))*sin(q(5))
      R(2,3) = (cos(q(2))*cos(q(3))*sin(q(5))+(1.d+0*sin(q(2))*sin(q(4))
     1   +cos(q(2))*sin(q(3))*cos(q(4)))*cos(q(5)))*sin(q(6))+1.d+0*(cos
     2   (q(2))*sin(q(3))*sin(q(4))-1.d+0*sin(q(2))*cos(q(4)))*cos(q(6))
      R(3,1) = ((sin(q(1))*sin(q(3))+cos(q(1))*sin(q(2))*cos(q(3)))*sin(
     1   q(5))+((cos(q(1))*sin(q(2))*sin(q(3))-sin(q(1))*cos(q(3)))*cos(
     2   q(4))-1.d+0*cos(q(1))*cos(q(2))*sin(q(4)))*cos(q(5)))*cos(q(6))
     3   -1.d+0*((cos(q(1))*sin(q(2))*sin(q(3))-sin(q(1))*cos(q(3)))*sin
     4   (q(4))+1.d+0*cos(q(1))*cos(q(2))*cos(q(4)))*sin(q(6))
      R(3,2) = (sin(q(1))*sin(q(3))+cos(q(1))*sin(q(2))*cos(q(3)))*cos(q
     1   (5))-((cos(q(1))*sin(q(2))*sin(q(3))-sin(q(1))*cos(q(3)))*cos(q
     2   (4))-1.d+0*cos(q(1))*cos(q(2))*sin(q(4)))*sin(q(5))
      R(3,3) = ((sin(q(1))*sin(q(3))+cos(q(1))*sin(q(2))*cos(q(3)))*sin(
     1   q(5))+((cos(q(1))*sin(q(2))*sin(q(3))-sin(q(1))*cos(q(3)))*cos(
     2   q(4))-1.d+0*cos(q(1))*cos(q(2))*sin(q(4)))*cos(q(5)))*sin(q(6))
     3   +1.d+0*((cos(q(1))*sin(q(2))*sin(q(3))-sin(q(1))*cos(q(3)))*sin
     4   (q(4))+1.d+0*cos(q(1))*cos(q(2))*cos(q(4)))*cos(q(6))
      v(1,1) = -1.8159d-1*((sin(q(1))*sin(q(2))*sin(q(3))+cos(q(1))*cos(
     1   q(3)))*sin(q(4))+1.d+0*sin(q(1))*cos(q(2))*cos(q(4)))-1.7914d-1
     2   *sin(q(1))*cos(q(2))
      v(2,1) = 1.7914d-1*sin(q(2))-1.8159d-1*(cos(q(2))*sin(q(3))*sin(q(
     1   4))-1.d+0*sin(q(2))*cos(q(4)))
      v(3,1) = -1.8159d-1*((cos(q(1))*sin(q(2))*sin(q(3))-sin(q(1))*cos(
     1   q(3)))*sin(q(4))+1.d+0*cos(q(1))*cos(q(2))*cos(q(4)))-1.7914d-1
     2   *cos(q(1))*cos(q(2))
      J(1,1) = -1.8159d-1*((cos(q(1))*sin(q(2))*sin(q(3))-sin(q(1))*cos(
     1   q(3)))*sin(q(4))+1.d+0*cos(q(1))*cos(q(2))*cos(q(4)))-1.7914d-1
     2   *cos(q(1))*cos(q(2))
      J(1,2) = sin(q(1))*(1.7914d-1*sin(q(2))-1.8159d-1*(cos(q(2))*sin(q
     1   (3))*sin(q(4))-1.d+0*sin(q(2))*cos(q(4))))
      J(1,3) = 1.8159d-1*sin(q(2))*((cos(q(1))*sin(q(2))*sin(q(3))-sin(q
     1   (1))*cos(q(3)))*sin(q(4))+1.d+0*cos(q(1))*cos(q(2))*cos(q(4)))+
     2   1.8159d-1*cos(q(1))*cos(q(2))*(cos(q(2))*sin(q(3))*sin(q(4))-1.
     3   d+0*sin(q(2))*cos(q(4)))
      J(1,4) = 1.8159d-1*(sin(q(1))*sin(q(3))+cos(q(1))*sin(q(2))*cos(q(
     1   3)))*(cos(q(2))*sin(q(3))*sin(q(4))-1.d+0*sin(q(2))*cos(q(4)))-
     2   1.8159d-1*cos(q(2))*cos(q(3))*((cos(q(1))*sin(q(2))*sin(q(3))-s
     3   in(q(1))*cos(q(3)))*sin(q(4))+1.d+0*cos(q(1))*cos(q(2))*cos(q(4
     4   )))
      J(1,5) = 0
      J(1,6) = 0
      J(2,1) = 0
      J(2,2) = -sin(q(1))*(-1.8159d-1*((sin(q(1))*sin(q(2))*sin(q(3))+co
     1   s(q(1))*cos(q(3)))*sin(q(4))+1.d+0*sin(q(1))*cos(q(2))*cos(q(4)
     2   ))-1.7914d-1*sin(q(1))*cos(q(2)))-cos(q(1))*(-1.8159d-1*((cos(q
     3   (1))*sin(q(2))*sin(q(3))-sin(q(1))*cos(q(3)))*sin(q(4))+1.d+0*c
     4   os(q(1))*cos(q(2))*cos(q(4)))-1.7914d-1*cos(q(1))*cos(q(2)))
      J(2,3) = 1.8159d-1*sin(q(1))*cos(q(2))*((cos(q(1))*sin(q(2))*sin(q
     1   (3))-sin(q(1))*cos(q(3)))*sin(q(4))+1.d+0*cos(q(1))*cos(q(2))*c
     2   os(q(4)))-1.8159d-1*cos(q(1))*cos(q(2))*((sin(q(1))*sin(q(2))*s
     3   in(q(3))+cos(q(1))*cos(q(3)))*sin(q(4))+1.d+0*sin(q(1))*cos(q(2
     4   ))*cos(q(4)))
      J(2,4) = 1.8159d-1*(sin(q(1))*sin(q(2))*cos(q(3))-cos(q(1))*sin(q(
     1   3)))*((cos(q(1))*sin(q(2))*sin(q(3))-sin(q(1))*cos(q(3)))*sin(q
     2   (4))+1.d+0*cos(q(1))*cos(q(2))*cos(q(4)))-1.8159d-1*(sin(q(1))*
     3   sin(q(3))+cos(q(1))*sin(q(2))*cos(q(3)))*((sin(q(1))*sin(q(2))*
     4   sin(q(3))+cos(q(1))*cos(q(3)))*sin(q(4))+1.d+0*sin(q(1))*cos(q(
     5   2))*cos(q(4)))
      J(2,5) = 0
      J(2,6) = 0
      J(3,1) = 1.8159d-1*((sin(q(1))*sin(q(2))*sin(q(3))+cos(q(1))*cos(q
     1   (3)))*sin(q(4))+1.d+0*sin(q(1))*cos(q(2))*cos(q(4)))+1.7914d-1*
     2   sin(q(1))*cos(q(2))
      J(3,2) = cos(q(1))*(1.7914d-1*sin(q(2))-1.8159d-1*(cos(q(2))*sin(q
     1   (3))*sin(q(4))-1.d+0*sin(q(2))*cos(q(4))))
      J(3,3) = -1.8159d-1*sin(q(2))*((sin(q(1))*sin(q(2))*sin(q(3))+cos(
     1   q(1))*cos(q(3)))*sin(q(4))+1.d+0*sin(q(1))*cos(q(2))*cos(q(4)))
     2   -1.8159d-1*sin(q(1))*cos(q(2))*(cos(q(2))*sin(q(3))*sin(q(4))-1
     3   .d+0*sin(q(2))*cos(q(4)))
      J(3,4) = 1.8159d-1*cos(q(2))*cos(q(3))*((sin(q(1))*sin(q(2))*sin(q
     1   (3))+cos(q(1))*cos(q(3)))*sin(q(4))+1.d+0*sin(q(1))*cos(q(2))*c
     2   os(q(4)))-1.8159d-1*(sin(q(1))*sin(q(2))*cos(q(3))-cos(q(1))*si
     3   n(q(3)))*(cos(q(2))*sin(q(3))*sin(q(4))-1.d+0*sin(q(2))*cos(q(4
     4   )))
      J(3,5) = 0
      J(3,6) = 0
      J(4,1) = 0
      J(4,2) = cos(q(1))
      J(4,3) = 1.d+0*sin(q(1))*cos(q(2))
      J(4,4) = sin(q(1))*sin(q(2))*cos(q(3))-cos(q(1))*sin(q(3))
      J(4,5) = 1.d+0*((sin(q(1))*sin(q(2))*sin(q(3))+cos(q(1))*cos(q(3))
     1   )*sin(q(4))+1.d+0*sin(q(1))*cos(q(2))*cos(q(4)))
      J(4,6) = (sin(q(1))*sin(q(2))*cos(q(3))-cos(q(1))*sin(q(3)))*cos(q
     1   (5))-((sin(q(1))*sin(q(2))*sin(q(3))+cos(q(1))*cos(q(3)))*cos(q
     2   (4))-1.d+0*sin(q(1))*cos(q(2))*sin(q(4)))*sin(q(5))
      J(5,1) = 1
      J(5,2) = 0
      J(5,3) = -1.d+0*sin(q(2))
      J(5,4) = cos(q(2))*cos(q(3))
      J(5,5) = 1.d+0*(cos(q(2))*sin(q(3))*sin(q(4))-1.d+0*sin(q(2))*cos(
     1   q(4)))
      J(5,6) = cos(q(2))*cos(q(3))*cos(q(5))-(1.d+0*sin(q(2))*sin(q(4))+
     1   cos(q(2))*sin(q(3))*cos(q(4)))*sin(q(5))
      J(6,1) = 0
      J(6,2) = -sin(q(1))
      J(6,3) = 1.d+0*cos(q(1))*cos(q(2))
      J(6,4) = sin(q(1))*sin(q(3))+cos(q(1))*sin(q(2))*cos(q(3))
      J(6,5) = 1.d+0*((cos(q(1))*sin(q(2))*sin(q(3))-sin(q(1))*cos(q(3))
     1   )*sin(q(4))+1.d+0*cos(q(1))*cos(q(2))*cos(q(4)))
      J(6,6) = (sin(q(1))*sin(q(3))+cos(q(1))*sin(q(2))*cos(q(3)))*cos(q
     1   (5))-((cos(q(1))*sin(q(2))*sin(q(3))-sin(q(1))*cos(q(3)))*cos(q
     2   (4))-1.d+0*cos(q(1))*cos(q(2))*sin(q(4)))*sin(q(5))
      Return 
      End 
