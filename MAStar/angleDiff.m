function ad = angleDiff(v, ref_ang)

    ad = mod(((v - ref_ang) + pi), 2*pi) - pi;

end
