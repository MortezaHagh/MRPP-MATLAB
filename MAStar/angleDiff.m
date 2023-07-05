function ad = angleDiff(v,ref_ang)


ad = mod(((v-ref_ang)+180),360)-180;


end