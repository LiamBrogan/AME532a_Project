function C_NE = DCM_ECEF2NED(lat,long)

C_NE = [-sin(lat)*cos(long) -sin(lat)*sin(long)  cos(lat)
        -sin(long)           cos(long)           0
        -cos(lat)*cos(long) -cos(lat)*sin(long) -sin(lat)];

end