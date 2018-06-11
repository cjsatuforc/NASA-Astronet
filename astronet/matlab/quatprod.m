function f = quatprod(a,b)
% Returns the quaternion product of quaternions a and b
f(1)=a(1)*b(1)-a(2)*b(2)-a(3)*b(3)-a(4)*b(4);
f(2)=a(1)*b(2)+a(2)*b(1)+a(3)*b(4)-a(4)*b(3);
f(3)=a(1)*b(3)+a(3)*b(1)+a(4)*b(2)-a(2)*b(4);
f(4)=a(1)*b(4)+a(4)*b(1)+a(2)*b(3)-a(3)*b(2);

