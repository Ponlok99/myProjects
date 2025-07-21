function q = scara_ikine_RRP(dh,T)
        q = zeros(3,1);
        wcp = tform2trvec(T)';
        wcp(3,1) = dh.d(1) - wcp(3,1);
        D = (wcp(1,1)^2+wcp(2,1)^2 - dh.a(1)^2-dh.a(2)^2)/(2*dh.a(1)*dh.a(2));
        q(2,1) = atan2(-sqrt(abs(1-D^2)),D);
        h1 = atan2(dh.a(2)*sin(q(2,1)),(dh.a(1)+(dh.a(2)*cos(q(2,1)))));
        h2 = atan2(wcp(2,1), wcp(1,1));
        q(1,1) = h2-h1;
        q(3,1) = wcp(3,1); % prismatic
end