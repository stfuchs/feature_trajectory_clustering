from numpy import *

x1 = mat([2.,2.,3.,3.]).T
x2 = mat([3.,4.,4.,3.]).T
x3 = mat([1.,5.,0.,4.]).T
x = mat([1.,0,0,1.]).T
X = mat(hstack([x,x1,x2,x3]))
print "X:\n",X

E = mat(diag([.1,.1,.1,.1]))
R = mat(diag([.01,.01,.01,.01]))

z11 = mat([.5,.5]).T
z12 = mat([2.5,2.5]).T
z13 = mat([3.5,3.5]).T
z14 = mat([.5,4.5]).T
C1 = (linalg.inv(X.T)* mat(hstack([z11,z12,z13,z14])).T).T
C1 = mat([[.5,0,.5,0],[0,.5,0,.5]])
print "C1:\n",C1

z21 = mat([-1.,0]).T
z22 = mat([2.,4.]).T
z23 = mat([4.,5.]).T
z24 = mat([1.,3.]).T
C2 = (linalg.inv(X.T)* mat(hstack([z21,z22,z23,z24])).T).T
print "C2:\n",C2

Q = mat(diag([.001,.001]))

Ep = E + R
K1 = Ep*C1.T*linalg.inv(C1*Ep*C1.T + Q)
print "K1:\n",K1
x1 = x + K1*(z12 - C1*x)
print "x1:\n",x1
E1 = (identity(4) - K1*C1)*Ep
print "E1:\n",E1

Ep = E1 #+ R
K12 = Ep*C2.T*linalg.inv(C2*Ep*C2.T + Q)
print "K12:\n",K12
x12 = x1 + K12*(z22 - C2*x)
print "x12:\n",x12
E12 = (identity(4) - K12*C2)*Ep
print "E12:\n",E12

