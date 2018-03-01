function [pos, line, battery] = gpg_read(s)
%GPG_READ    Read remote GoPiGo3 status.
%   POS = GPG_READ(S) reads the position (in rad) of the wheels of remote
%   GoPiGo3 connected to socket S.
%   [POS, LINE] = GPG_READ(S) additionally returns the status of
%   the line sensors as a 10-bit value.
%   [POS, LINE, BATTERY] = GPG_READ(S) additionally returns the status of
%   the battery in V.
%
%   EXAMPLE:
%       s = gpg_open('192.168.0.205');
%       [pos, line, battery] = gpg_read(s)
%       gpg_close(s);
%
%   AUTHOR:
%       Wouter Caarls <wouter@caarls.org>

    data = uint8(fread(s, 36, 'uint8'));
    
    size = typecast(data(1:4), 'uint32');

    if size ~= 32
        error('Protocol mismatch error');
    end
    
    pos = double(typecast(data(5:12), 'int32'))*pi/180;
    line = double(typecast(data(13:32), 'int32'));
    battery = double(typecast(data(33:36), 'single'));

end
