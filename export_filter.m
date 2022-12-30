% EXPORT_FILTER(h, F, fname)
%  Export a low pass filter for the EQ2300 lab to a text file
%  to be inluded in the Arduino Sketch
%
%  h - impulse responce of the filter to be exported
%  F - number of fractional bits to use in the fixed point implementation
%  fname - output file name
%
%  example usage: export_filter(h, 10, 'tmp.txt')
%
%  The function does som basic checking to hopefully avoid some bugs

function export_filter(h,F,fname)

    h = h(:);
    N = length(h);
    
    assert(N < 60, 'Filter should be less than 60 taps');
    assert(0 < N <= 16, 'You should use between 1 and 16 fractional bits');
    DC = abs(sum(h));
    assert((DC < 2) && (DC > 1/2), 'The DC gain of your filter should be around 1. You are more than 3dB off');
    
    % Convert to integer values
    h = round(h*2^F);
    
    fid = fopen(fname,'w');
    fprintf(fid,'#define FILTERLENGTH %d\n',N);
    fprintf(fid,'#define FRACTIONALBITS %d\n',F);
    fprintf(fid,'int Filter[] = {');
    for n=1:N
        fprintf(fid,'%d',h(n));
        if(n < N)
            fprintf(fid,',');
        else
            fprintf(fid,'};\n');
        end;
    end;
    fclose(fid);
end
