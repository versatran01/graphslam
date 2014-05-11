function v = t2v(A)
% T2V homogeneous transformation to vector
v(1:2,1) = A(1:2,3);
v(3,1) = atan2(A(2,1), A(1,1));

end