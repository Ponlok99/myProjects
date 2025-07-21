classdef trajMode < int32
    enumeration
        trapz(0),
        cubic(1),
        quintic(2),
        minjerk(3),
        minsnap(4),
        bspline(5),
        scurve(6),
        transtraj(7),
        rottraj(8),
    end
end