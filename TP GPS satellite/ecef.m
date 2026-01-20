function M = ecef(lat, lon)

sl = sin(lat); cl = cos(lat);
sb = sin(lon); cb = cos(lon);

M = [ -sl*cb, -sb,  -cl*cb;
      -sl*sb,  cb,  -cl*sb;
         cl,    0,  -sl ];
end