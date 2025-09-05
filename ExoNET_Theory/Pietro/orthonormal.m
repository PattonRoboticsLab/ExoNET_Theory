function y = orthonormal(v1, v2, v3) % three vectors as input

u1 = v1/norm(v1,2);
v2n = v2 - dot(v2, u1) * u1;
u2 = v2n/norm(v2n,2);
v3n = v3 - dot(v3, u1)*u1 - dot(v3, u2)*u2; 
u3 = v3n/norm(v3n,2);
y(:,1) = u1; y(:,2) = u2; y(:,3) = u3;  

%{
u1 = v1/norm(v1,2);
p2 = v2 - dot(v2,v1)/dot(v1,v1)*v1; 
u2 = p2/norm(p2,2);
p3 = v3 - dot(v3,v1)/dot(v1,v1)*v1 - dot(v3,v2)/dot(v2,v2)*v2; 
u3 = p3/norm(p3,2);
y(1,:) = u1; y(2,:) = u2; y(3,:) = u3;  
%}
